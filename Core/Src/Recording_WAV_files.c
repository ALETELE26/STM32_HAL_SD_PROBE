/*
 * wav_recorder.c
 *
 *  Created on: 8 dic. 2024
 *      Author: Cuba
 */


#include <main.h>
#include <exti.h>
#include <gpio.h>
#include <uart.h>
#include <spi.h>
#include <opamp.h>
#include <tim.h>
#include <adc.h>
#include "fatfs.h"
#include "wav_recorder.h"

SPI_HandleTypeDef hspi1;
uint32_t systemClock=72000000;
bool exti_flag = 0;
bool isFileCreated=false;
//FatFs variables
FRESULT fresult;
FATFS fs;
DWORD freeClusters;
FATFS *pFatFs;
uint32_t total_size,free_space;;
FIL file;
UINT readBytes=0;
UINT writeBytes=0;
//#define BUFFER_SIZE 128
//char buffer[BUFFER_SIZE];  // to store strings..
//int bufsize (char *buf)
//{
//	int i=0;
//	while (*buf++ != '\0') i++;
//	return i;
//}
//
//void clear_buffer (void)
//{
//	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
//}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_SPI1_Init(void);


int main (void)
{
	//----------Setup------------------------------//
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	spi_GPIO_config();
	tim_TIM6_MIC_config();//4x44.1kHz Sample Rate
	opamp_config();//MIC is connected to PA0(A0), the OPAMP non-inverting input
	adc_MIC_config();
	gpio_LED_config();
	exti_buttonConfig();//Push bottom as interrupt source configuration (PA12->D2)
	uart_UART2_GPIO_config();
	uart_UART2_config(systemClock);
	MX_SPI1_Init();
	MX_FATFS_Init();
	//Mount SD Card
	fresult=f_mount(&fs, "/", 1);
	if (fresult != FR_OK)
	{
		printf("Failed to mount SD card to FatFs\r\n");
	}
	else
	{
		printf("Successfully Mounted SD card\r\n");
	}
	//Print status
	printf("SD card result = %d\r\n",fresult);



	//----------Loop forever-----------------------//
	while(1)
	{
		gpio_LED_toggleGreen();
		HAL_Delay(250);
		//WAV Recorder
		if (exti_flag)
		{
			if(wav_recorder_fileCreate("88.2kHz_OVS.wav"))
			{
				printf("Audio File Created Successfully\r\n");
				isFileCreated=true;
			}
			else
			{
				printf("Audio File Creation Failed\r\n");
				isFileCreated=false;
			}
			if(isFileCreated)
			{
				gpio_LED_writeGreen(true);
				HAL_Delay(1000);
				exti_flag=false;
				printf("Starting Recording...\r\n");
				wav_recorder_record();
				while(!wav_recorder_isFinished())
				{
					wav_recorder_process();
					if(exti_flag)
					{
						wav_recorder_stop();
						wav_recorder_process();
						printf("Recording Stopped\r\n");
					}
				}
				gpio_LED_writeGreen(false);
			}

			HAL_Delay(1000);
			exti_flag=false;

		}

	}
	return 0;
}


/* Interrupt Routines -----------------------------------------------*/
//Push Button Interrupt
void EXTI15_10_IRQHandler(void)
{
	//Clearing the pending interrupt field
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	EXTI->PR1 |=(EXTI_PR1_PIF12);
	//Application
	exti_flag =true;

}
//DMA ADC Channel Interrupt
void DMA1_Channel1_IRQHandler(void)
{
	//Half Transfer
	if (DMA1->ISR & DMA_ISR_HTIF1)
	{
		//Clear half transfer complete flag
		DMA1->IFCR |= (DMA_IFCR_CHTIF1);
		////Ready for the application
		wavRecorder_halfTransfer_Callback();

	}
	//Full Transfer
	if (DMA1->ISR & DMA_ISR_TCIF1)
	{
		//Clear transfer complete flag
		DMA1->IFCR |= (DMA_IFCR_CTCIF1);
		////Ready for the application
		wavRecorder_fullTransfer_Callback();

	}
	//Clear interrupt pending flag
	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
}






/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  RCC_OscInitStruct.MSICalibrationValue = 0;
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM = 1;
	  RCC_OscInitStruct.PLL.PLLN = 36;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */


void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

