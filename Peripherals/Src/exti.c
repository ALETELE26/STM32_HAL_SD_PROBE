/*
 * exti.c
 *
 *  Created on: 6 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#include <exti.h>

/*
 * @brief Push Button EXTI configuration (PA12->D2)
 */
void exti_buttonConfig(void)
{
  //Enable GPIO port A clock
  RCC->AHB2ENR |=(RCC_AHB2ENR_GPIOAEN);
  //Enable mode Input with Internal Pull-Up
  GPIOA->MODER &= ~(GPIO_MODER_MODE12);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD12);
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPD12_0);
  //System configuration controller clock enable
  RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN);
  //PA12 maps EXTI12
  SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
  //Unblock EXTI0 interrupt
  EXTI->IMR1 |= (EXTI_IMR1_IM12);
  //Enable Rising Edge Only
  EXTI->RTSR1 |= (EXTI_RTSR1_RT12);
  EXTI->FTSR1 &= ~(EXTI_FTSR1_FT12);
  //Minimum Priority Level
  NVIC_SetPriority(EXTI15_10_IRQn,15);
  //Enable interrupt
  NVIC_EnableIRQ(EXTI15_10_IRQn);


}


