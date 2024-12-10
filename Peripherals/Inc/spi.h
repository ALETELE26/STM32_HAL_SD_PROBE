/*
 * spi.h
 *
 *  Created on: 6 nov. 2024
 *      Author: Cuba
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include <main.h>
#include <gpio.h>


/*
 * Configuration structure for SPI peripheral
 *
 * @spi_devicemode:
 * Mode Master or Slave
 *
 * @spi_BusConfig:
 * Full-Duplex, Half-Duplex or Simplex_RXONLY
 *
 * @spi_SclkSpeed:
 * Divide PCLK by 2,4,8,16,...,256
 *
 * @spi_DataSize:
 * Data word of 4,5,6,7,8,...,16bits
 * --------------------
 * SPI_MODE CPOL  CPHA
 * 0        0     0
 * 1        0     1
 * 2        1     0
 * 3        1     1
 * -------------------
 * @spi_SSM:
 * Software Slave Management-> ENABLE or DISABLE
 *
 * @spi_LSBFIRST:
 * Select MSB/LSB first bit in data frame
 *
 * */
typedef struct{

	uint8_t	spi_DeviceMode;	// Refer @spi_devicemode
	uint8_t	spi_BusConfig;	// Refer @spi_BusConfiguration
	uint8_t spi_SclkSpeed;	// Refer @spi_ClockSpeed
	uint8_t spi_DataSize;   // Refer @spi_DataSize
	uint8_t spi_CPOL;		// Refer @spi_CPOL
	uint8_t spi_CPHA;		// Refer @spi_CPHA
	uint8_t spi_SSM;		// Refer @spi_SSM
	uint8_t spi_LSBFIRST;   // Refer @spi_LSBFIRST
    bool    spi_ErrorInterrupt;

}SPI_Config_t ;

/*
 * Handle structure for SPIx peripheral
 * */
typedef struct{
	        SPI_TypeDef    *pSPIx 	 ;	// this holds the base address of spix(1 ,2,3,4) peripheral
	        SPI_Config_t    SPIConfig;
volatile	uint8_t		   *pTxBuffer;	// To store the app tx buffer address
volatile	uint8_t		   *pRxBuffer;	// To store the app rx buffer address
volatile    uint32_t	    TxLen    ;	// To store the tx len
volatile	uint32_t	    RxLen    ;	// To store the Rx len
volatile    uint8_t	    	TxState  ;	// To store the Tx state
volatile    uint8_t	    	RxState  ;	// To store the rx state


}SPI_Handle_t;

/**
 * @Brief GPIO Peripheral CLK control
 * STM32L432KC have only SPI1 and SPI3
 */
void spi_PeripheralClockControl(SPI_TypeDef *pSPIx , uint8_t EnorDi) ;

/**
 * @brief SPI Initialization API
 */
void spi_Init(SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPI Reset Peripheral Registers
 */
void spi_DeInit(SPI_TypeDef *pSPIx);

/**
 * @Brief SPI (NVIC)Interrupt Configuration
 */
void SPI_IRQInterruptConfig(SPI_TypeDef *pSPIx,uint8_t ENorDIS,uint8_t interrupt_Priority);

/**
 * @Brief SPI Interrupt-Based Transmit
 */
uint8_t spi_SendIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t length);

/**
 * @Brief SPI Interrupt-Based Transmit
 */
uint8_t spi_ReadIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer , uint32_t length);

/**
 * @Brief SPI Interrupt End of TX, stop TX and reset TX parameters
 * !Have to be call when length==0 in SPI ISR routine!
 */
void spi_CloseTransmissionIT(SPI_Handle_t *pSPIHandle );

/**
 * @Brief SPI Interrupt End of RX, stop RX and reset RX parameters
 * !Have to be call when length==0 in SPI ISR routine!
 */
void spi_CloseReceptionIT(SPI_Handle_t *pSPIHandle );

/**
 * @Brief SPI TXE Interrupt Handler
 * !Must be call in SPIx_IRQn routine when TXE==1 and TXEIE==1!
 */
void spi_TXE_Interrupt_Handler(SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPI RXNE Interrupt Handler
 * !Must be call in SPIx_IRQn routine when RXNE==1 and RXNEIE==1!
 */
void spi_RXNE_Interrupt_Handler(SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPI Error Interrupt Handler
 */
void spi_Error_Interrupt_Handler(SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPIx Multi-Cause IRQ Handling
 */
void spi_IRQHandling (SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPI Get Flag Status
 * @Parameter FlagName attempts for:
 * SPI:RXNE,TXE,CRCERR,BSY,MODF,OVR flags
 */
uint8_t spi_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName );


/**
 * @Brief SPI1 GPIO pins configuration
 * (PA5(A4)->SP1_SCK,PA6(A5)->SP1_MISO,PA7(A6)->SP1_MOSI)
 * (PA4(A3)->CS)
 */

void spi_GPIO_config(void);

/**
 * @Brief SPI1 peripheral configuration
 */
void spi_config(uint32_t system_Clock,SPI_Handle_t *pSPIHandle);

/**
 * @Brief SPI1 transmit
 */
bool spi_transmit(uint8_t *pointer_data,uint8_t len,uint32_t timeout);

/**
 * @Brief SPI1 receive
 */
bool spi_receive(uint8_t *pointer_data,uint8_t len,uint32_t timeout);

/**
 * @Brief SPI1 transmit/receive
 */
bool spi_transmitReceive(uint8_t *RX_buffer,uint8_t *TX_buffer,uint8_t len,uint32_t timeout);

/**
 * @Brief SD Chip Select Set/Reset
 */
void spi_cs_sd_write(bool state);

void spi_ApplicationEventCallBack(uint8_t eventcode);

//////Macros//////

// @SPI_devicemode

#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

// @SPI_busconfig

#define SPI_BUS_CONFIG_FULL_DUPLEX			1
#define SPI_BUS_CONFIG_HALF_DUPLEX		    2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	    3

//@SPI_ClockConfig

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

//@SPI_DataSize

#define SPI_DS_4BITS						0x3
#define SPI_DS_5BITS						0x4
#define SPI_DS_6BITS						0x5
#define SPI_DS_7BITS						0x6
#define SPI_DS_8BITS						0x7
#define SPI_DS_9BITS						0x8
#define SPI_DS_10BITS						0x9
#define SPI_DS_11BITS						0xA
#define SPI_DS_12BITS						0xB
#define SPI_DS_13BITS						0xC
#define SPI_DS_14BITS						0xD
#define SPI_DS_15BITS						0xE
#define SPI_DS_16BITS						0xF

//@SPI_CPOL

#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

//@SPI_CPHA

#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

//@SPI_SSM

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

//@SPI_LSBFIRST

#define SPI_MSBFIRST                    0
#define SPI_LSBFIRST                    1

// SPI flag status shifts

#define SPI_RXNE_FLAG					(SPI_SR_RXNE)
#define SPI_TXE_FLAG					(SPI_SR_TXE)
#define SPI_CRCERR_FLAG					(SPI_SR_CRCERR)
#define SPI_MODF_FLAG					(SPI_SR_MODF)
#define SPI_OVR_FLAG					(SPI_SR_OVR)
#define SPI_BSY_FLAG					(SPI_SR_BSY)
#define SPI_FRE_FLAG					(SPI_SR_FRE)

// SPI application states

#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

// SPI Application events

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

// clock enable macros for SPIx

#define SPI1_CLOCK_ENABLE()			(RCC->APB2ENR  |=  (1 << 12) )
#define SPI3_CLOCK_ENABLE()			(RCC->APB1ENR1 |=  (1 << 15) )

// clock disable macros for SPIx
#define SPI1_CLOCK_DISABLE()		(RCC->APB2ENR  &= ~(1 << 12) )
#define SPI3_CLOCK_DISABLE()		(RCC->APB1ENR1 &= ~(1 << 15) )

//SPI peripheral reset macro

#define SPI1_REG_RESET()		   do {(RCC->APB2RSTR |=(1 <<  12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI3_REG_RESET()           do {(RCC->APB1ENR1 |=(1 <<  15)); (RCC->APB1ENR1 &= ~(1 << 15));}while(0)

#endif /* INC_SPI_H_ */
