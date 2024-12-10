/*
 * gpio.h
 *
 *  Created on: 1 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "main.h"
/*
 * @brief GPIO Driver class
 */
typedef struct {

	// pointer to hold the base address of the gpio peripheral
	GPIO_TypeDef *pGPIOx  ; // this holds the base address of the gpio port to which the pin belongs
	uint8_t pinNumber		;
	uint8_t pinMode 		;
	uint8_t pinSpeed	;
	uint8_t pinPUPDControl ;
	uint8_t pinOutputType  ;
	uint8_t pinAltFunMode ;

}GPIO_Handle_t;

/*
 * @Brief GPIO Peripheral CLK control
 */
void gpioPeriphClockControl(GPIO_TypeDef *pGPIOx , uint8_t EnorDi);

/**
 * @Brief GPIO Initialization Driver
 */
void gpio_Init(GPIO_Handle_t *newGPIOPin);

/**
 * @Brief GPIOx Port Registers Reset
 */
void portReset(GPIO_TypeDef *pGPIOx );

/**
 * @Brief GPIO r/w to pin or port functions
 */
bool gpio_ReadFromInputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber) ;
uint16_t gpio_ReadFromInputPort(GPIO_TypeDef *pGPIOx ) ;
void gpio_WriteToOutputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber , uint8_t Value) ;
void gpio_WriteToOutputPort(GPIO_TypeDef *pGPIOx ,uint16_t Value ) ;
void gpio_ToggleOutputPin(GPIO_TypeDef *pGPIOx , uint8_t pinNumber ) ;

/*
 * Brief LEDs GPIO Configuration PB3(On-board Led 3(Green))
 */
void gpio_LED_config(void);
/*
 * Brief Write/Toggle functions
 */
void gpio_LED_writeGreen(bool state);
void gpio_LED_toggleGreen(void);
/*
 * @Brief PA8(D9) as TIM1_CH1 input alternate function
 */
void gpio_TIM1_CH1_input_config(void);

//////Macros///////
#define PIN_SET 	1
#define PIN_RESET 	0

//GPIO PIN numbers
#define PIN_NO_0		0
#define PIN_NO_1		1
#define PIN_NO_2		2
#define PIN_NO_3		3
#define PIN_NO_4		4
#define PIN_NO_5		5
#define PIN_NO_6		6
#define PIN_NO_7		7
#define PIN_NO_8		8
#define PIN_NO_9		9
#define PIN_NO_10		10
#define PIN_NO_11		11
#define PIN_NO_12		12
#define PIN_NO_13		13
#define PIN_NO_14		14
#define PIN_NO_15		15

//GPIO AF Modes
#define AF0		0
#define AF1		1
#define AF2		2
#define AF3		3
#define AF4		4
#define AF5		5
#define AF6		6
#define AF7		7
#define AF8		8
#define AF9		9
#define AF10	10
#define AF11	11
#define AF12	12
#define AF13	13
#define AF14	14
#define AF15	15

//  GPIO PIN modes

#define PIN_MODE_INPUT 	       0
#define PIN_MODE_OUTPUT	       1
#define PIN_MODE_ALTFN		   2
#define PIN_MODE_ANALOGE	   3

// GPIO output types
#define PIN_OP_TYPE_PP		   0
#define PIN_OP_TYPE_OD		   1

// GPIO PIN Speeds

#define PIN_SPEED_LOW		    0
#define PIN_SPEED_MEDIUM		1
#define PIN_SPEED_FAST			2
#define PIN_SPEED_HIGH			3

//GPIO PIN pull up pull down
#define PIN_NO_PUPD			0
#define PIN_PULL_UP			1
#define PIN_PULL_DOWN	    2

// clock enable macros for GPIOx
#define GPIOA_CLOCK_ENABLE()	(RCC->AHB2ENR  |= (1 << 0) )
#define GPIOB_CLOCK_ENABLE()	(RCC->AHB2ENR  |= (1 << 1) )
#define GPIOC_CLOCK_ENABLE()	(RCC->AHB2ENR  |= (1 << 2) )
#define GPIOH_CLOCK_ENABLE()	(RCC->AHB2ENR  |= (1 << 7) )


// clock disable macros for GPIOx
#define GPIOA_CLOCK_DISABLE()	(RCC->AHB2ENR  &= ~(1 << 0) )
#define GPIOB_CLOCK_DISABLE()	(RCC->AHB2ENR  &= ~(1 << 1) )
#define GPIOC_CLOCK_DISABLE()	(RCC->AHB2ENR  &= ~(1 << 2) )
#define GPIOH_CLOCK_DISABLE()	(RCC->AHB2ENR  &= ~(1 << 7) )

// GPIOx peripheral reset
#define GPIOA_REG_RESET()			do {(RCC->AHB2RSTR |=(1 <<  0)); (RCC->AHB2RSTR &= ~(1 <<  0));}while(0)
#define GPIOB_REG_RESET()           do {(RCC->AHB2RSTR |=(1 <<  1)); (RCC->AHB2RSTR &= ~(1 <<  1));}while(0)
#define GPIOC_REG_RESET()           do {(RCC->AHB2RSTR |=(1 <<  2)); (RCC->AHB2RSTR &= ~(1 <<  2));}while(0)
#define GPIOH_REG_RESET()           do {(RCC->AHB2RSTR |=(1 <<  7)); (RCC->AHB2RSTR &= ~(1 <<  7));}while(0)



#endif /* INC_GPIO_H_ */
