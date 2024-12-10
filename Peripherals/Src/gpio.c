/*
 * gpio.c
 *
 *  Created on: 1 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#include <gpio.h>

// clock control api for GPIO
void gpioPeriphClockControl(GPIO_TypeDef *pGPIOx , uint8_t EnorDi) {

	if(EnorDi == ENABLE){

		if (pGPIOx == GPIOA) {
			GPIOA_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOB) {
			GPIOB_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_ENABLE();
		}


	}else{
		if (pGPIOx == GPIOA) {
			GPIOA_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOB) {
			GPIOB_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_DISABLE();
		}
	}
}

// GPIO Initialization
void gpio_Init(GPIO_Handle_t* pNewGPIOPin)
{

	uint32_t temp = 0 ;

	//enable the peripheral clock
	gpioPeriphClockControl(pNewGPIOPin->pGPIOx, ENABLE) ;
	// configure the modes of a GPIO pin
	temp = (pNewGPIOPin->pinMode << (2 * pNewGPIOPin->pinNumber)) ;
	pNewGPIOPin->pGPIOx->MODER &= ~(0x3 << (2 * pNewGPIOPin->pinNumber) );
	pNewGPIOPin->pGPIOx->MODER |= temp ;

	temp = 0;

	// configure the speed
	temp = (pNewGPIOPin->pinSpeed << (2 * pNewGPIOPin->pinNumber) ) ;
	pNewGPIOPin->pGPIOx->OSPEEDR  &= ~(0x3 << (2 * pNewGPIOPin->pinNumber)) ;
	pNewGPIOPin->pGPIOx->OSPEEDR  |= temp ;

	temp = 0 ;

	// configure the pull up/down control
	temp = (pNewGPIOPin->pinPUPDControl << (2 * pNewGPIOPin->pinNumber) ) ;
	pNewGPIOPin->pGPIOx->PUPDR  &= ~(0x3 << (2 * pNewGPIOPin->pinNumber));
	pNewGPIOPin->pGPIOx->PUPDR  |= temp ;

	temp = 0 ;

	// configure the output type
	temp = (pNewGPIOPin->pinOutputType << ( pNewGPIOPin->pinNumber) ) ;
	pNewGPIOPin->pGPIOx->OTYPER &= ~(0x1 << pNewGPIOPin->pinNumber) ;
	pNewGPIOPin->pGPIOx->OTYPER |= temp ;

	temp = 0 ;

	// configure the alternate functionality
	if (pNewGPIOPin->pinMode == PIN_MODE_ALTFN)
	{
		// configure the alternate function registers
		uint8_t temp1;
		uint8_t temp2;
		temp1 = pNewGPIOPin->pinNumber  / 8 ;
		temp2 = pNewGPIOPin->pinNumber % 8 ;
		pNewGPIOPin->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)) ;
		pNewGPIOPin->pGPIOx->AFR[temp1] |= (pNewGPIOPin->pinAltFunMode << (4 * temp2)) ;
	}

}
void gpio_PortReset(GPIO_TypeDef *pGPIOx ) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET() ;
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET() ;
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET() ;
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET() ;
	}
}

// GPIO read and write to port ot pin
bool gpio_ReadFromInputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber) {

	return (bool)(pGPIOx->IDR & (0x1UL << (pinNumber)));

}
uint16_t gpio_ReadFromInputPort(GPIO_TypeDef *pGPIOx ) {

	uint16_t value ;
	value = (uint16_t)(pGPIOx->IDR ) ;
	return value ;

}
void gpio_WriteToOutputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber , uint8_t Value) {

	if (Value == PIN_SET) {
		// write 1 to the bit field corresponding pin number
		pGPIOx->ODR |= (1 << pinNumber) ;

	} else {
		// write 0  to the bit field corresponding pin number
		pGPIOx->ODR &= ~(1 << pinNumber) ;
	}
}
void gpio_WriteToOutputPort(GPIO_TypeDef *pGPIOx ,uint16_t Value ) {

	pGPIOx->ODR = Value ;

}
void gpio_ToggleOutputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber ) {

	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber) ;

}


/*
 * Brief LEDs GPIO Configuration PB3(On-board Led 3(Green))
 */
void gpio_LED_config(void)
{
  //Enable GPIO port B clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
  //Pin mode to output
  GPIOB->MODER &= ~(GPIO_MODER_MODE3);
  GPIOB->MODER |= (GPIO_MODER_MODE3_0);
  //Seting output as PushPull
  GPIOB->OTYPER &= ~(GPIO_OTYPER_ODR_3);
  //Setting as High Speed Output
  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
  GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3_1);
}

/*
 * Brief Write/Toggle functions
 */

void gpio_LED_writeGreen(bool state)
{
    if(state)
    {
      GPIOB->ODR |= (GPIO_ODR_OD3);
    }
    else
    {
      GPIOB->ODR &= ~(GPIO_ODR_OD3);
    }
}

void gpio_LED_toggleGreen(void)
{
    GPIOB->ODR ^= (GPIO_ODR_OD3);
}

/*
 * @Brief PA8(D9) as TIM1_CH1 input alternate function
 */
void gpio_TIM1_CH1_input_config(void)
{
	//Enable GPIO port A clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	//Pin mode to alternate function mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER8);
	GPIOA->MODER |= (GPIO_MODER_MODER8_1);
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk);
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL8_0);
}
