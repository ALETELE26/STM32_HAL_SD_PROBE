/*
 * tim.c
 *
 *  Created on: 12 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#include <tim.h>
/**
 * @Brief Delay Timer Configuration - TIM2
 */
void tim_TIM2_delayConfig(void)
{
	//Enable TIM 2 Timer CLOCK
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	//Count Up Mode
	TIM2->CR1 &= ~(TIM_CR1_DIR);
	//Prescaler to 1MHz TCLOCK
	// CK_CNT=fCK_PSC / (PSC[15:0] + 1)
	TIM2->PSC = 80-1;//Ya que 80MHz/80 es 1MHz
	//Enable TIM2 Update Interrupt
	TIM2->DIER |=(TIM_DIER_UIE);
	//Set the interrupt priority
	NVIC_SetPriority(TIM2_IRQn,0);
	//Enable the interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
}
/*
 * @Brief Microsecond Delay - TIM2
 */
void tim_TIM2_usDelay(uint32_t us)
{
	//Setting autoreload value
	//The Counter is overflow when exceed this value
	TIM2->ARR = us - 1;
	//Clock trigger
	TIM2->CR1 |= (TIM_CR1_CEN);
	//	while(!(TIM2->SR & (TIM_SR_UIF)));
	//	//Clock stop
	//	TIM2->CR1 &= ~(TIM_CR1_CEN);
	//	//Clear update interrupt flag
	//	TIM2->SR &= ~(TIM_SR_UIF);
}
/*
 * @Brief Millisecond Delay - TIM2
 */
void tim_TIM2_msDelay(uint32_t ms)
{
	for (uint32_t i=0;i<ms;i++)
	{
		tim_TIM2_usDelay(1000);//Delay of 1ms
	}

}
/*
 * @Brief Input Capture Timer Configuration - TIM3
 */
void tim_TIM1_InputCaptureConfig(void)
{
	//Enable TIM 1 Timer CLOCK
	RCC->APB2ENR |= (0x1UL << (11U));
	//Prescaler to 1MHz TCLOCK
	// CK_CNT=fCK_PSC / (PSC[15:0] + 1)
	TIM1->PSC = 80-1;//Ya que 80MHz/80 es 1MHz
	//Map TI1 on GPIO
	TIM1->OR1  &= ~(TIM1_OR1_TI1_RMP);
	//Map TIMx_CH1 to TI1
	TIM1->CR2 &= ~(TIM_CR2_TI1S);
	//Map IC1 on TI1
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);
	TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0);
	//Validate a transition on TI1 when 8 consecutive
	//samples with the new level have been detected
	TIM1->CCMR1 &= ~(TIM_CCMR1_IC1F);
	TIM1->CCMR1 |= (0x3UL << (4U));
	//Trigger on signal rise
	TIM1->CCER &= ~(TIM_CCER_CC1NP);
	TIM1->CCER &= ~(TIM_CCER_CC1P);
	//Disable capture prescaler
	TIM1->CCMR1 &= ~(TIM_CCMR1_IC1PSC);
	//Enable capture
	TIM1->CCER |= (TIM_CCER_CC1E);
	//Enable capture/compare interrupt
	TIM1->DIER |= (TIM_DIER_CC1IE);
	//Enable update timer interrupt
	TIM1->DIER |= (TIM_DIER_UIE);
	//Set interrupt priority
	NVIC_SetPriority(TIM1_CC_IRQn,1);
	//Enable interrupt
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	//Set interrupt priority
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn ,0);
	//Enable interrupt
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	//Enable counter
	TIM1->CR1 |=(TIM_CR1_CEN);
}

/*
 * @Brief PWM TIM15_CH1 GPIO Configuration(PA2->A7) - TIM15
 */
void tim_TIM15_PWM_GPIO_config(void)
{
	//Enable GPIO port A clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	//Setting as High Speed Output
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2_1);
	//Pin mode to alternate function mode(AF14)
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);
	GPIOA->MODER |= (GPIO_MODER_MODER2_1);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2);
	GPIOA->AFR[0] |= (0xEUL << GPIO_AFRL_AFSEL2_Pos);
}
/*
 * @Brief Timer PWM Mode Configuration - TIM15
 * 10Hz fixed frequency
 */
void tim_TIM15_PWM_config(void)
{
	//Enable TIM15 clock
	RCC->APB2ENR|=(RCC_APB2ENR_TIM15EN);
	//Periodic
	TIM15->CR1 &=~ (TIM_CR1_OPM);
	//Mode-->Reset
	TIM15->CR2 &=~ (TIM_CR2_MMS);
	//Prescaler
	// CK_CNT=fCK_PSC / (PSC[15:0] + 1)
	TIM15->PSC =4-1;//20MHz TIM15 CLK frequency
	//Period
	TIM15->ARR = 4000-1;//5kHz PWM signal frequency
	//Clear update interrupt flag
	TIM15->SR &=~ (0x1UL<< (0U));
	//Disable Output Compare
	TIM15->CCER=0;
	//Compare and Capture as Output Mode
	TIM15->CCMR1 &=~ (TIM_CCMR1_CC1S);
	////CH1 PWM output mode 1
	//CH1 is HIGH as long as TIMx_CNT<TIMx_CCR1 else is LOW
	TIM15->CCMR1 &=~ (TIM_CCMR1_OC1M);
	TIM15->CCMR1 |= (0x6UL << TIM_CCMR1_OC1M_Pos);
	//Enable preload and fast CC mode
	TIM15->CCMR1 |= (TIM_CCMR1_OC1PE);
	TIM15->CCMR1 |= (TIM_CCMR1_OC1FE);
	//Enable auto-reload preload register
	TIM15->CR1 |= (TIM_CR1_ARPE);
	//CH1 HIGH polarity
	TIM15->CCER &=~ (TIM_CCER_CC1P);
	//PWM Duty Cycle(Initially)
	TIM15->CCR1=0;
	//Enable OC CH1(not it's complementary channel)
	TIM15->BDTR |= (TIM_BDTR_MOE);
	TIM15->CCER |= (TIM_CCER_CC1E);
	//Trigger on timer
	TIM15->CR1 |= (TIM_CR1_CEN);
}
/**
 * @Brief PWM Duty Cycle  - Brightness(0-4096)
 */
void tim_PWM_setBrightness(uint16_t bright)
{
	TIM15->CCR1= (uint16_t)((TIM15->ARR+0.0f)*(bright/4096.0f));

}
/**
 * @Brief TIM6 as Trigger Output Configuration
 */
void tim_TIM6_TRGO_config(uint16_t msPeriod)
{
	//Get a 40MHz CLK for TIM6
	//We want a division/2 of the clock but
	//when the APBCLK isn't 1 then TIM CLK= APBCLK x 2
	//therefore we must select a x4 factor
	RCC->CFGR &=~(RCC_CFGR_PPRE1);
	RCC->CFGR |= (0x5UL << RCC_CFGR_PPRE1_Pos );
	//Enable TIM6 clock
	RCC->APB1ENR1 |=(RCC_APB1ENR1_TIM6EN);
	//Periodic
	TIM6->CR1 &=~ (TIM_CR1_OPM);
	//Mode-->TRGO
	TIM6->CR2 &=~ (TIM_CR2_MMS);
	TIM6->CR2 |= (TIM_CR2_MMS_1);
	//Prescaler CK_CNT=fCK_PSC / (PSC[15:0] + 1)
	TIM6->PSC=40000-1;//40MHz/40k= 1kHz CLK
	//Period
	TIM6->ARR=msPeriod-1;
	//Update register on event
	TIM6->EGR=1;
	//Start timer
	TIM6->CR1 |= (TIM_CR1_CEN);
}
/*
 * @Brief RGB PWM Channel GPIO Configuration - TIM1
 * PA8(D9)->TIM1_CH1  PA9(D1)->TIM1_CH2 PA10(D0)->TIM1_CH3
 */
void tim_TIM1_RGB_PWM_GPIO_config(void)
{
	//Enable GPIO port A clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	//Setting as High Speed Output
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_1);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_1);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR10);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10_1);
	//Pin mode to alternate function mode(AF1)
	GPIOA->MODER &= ~(GPIO_MODER_MODER8);
	GPIOA->MODER |= (GPIO_MODER_MODER8_1);
	GPIOA->MODER &= ~(GPIO_MODER_MODER9);
	GPIOA->MODER |= (GPIO_MODER_MODER9_1);
	GPIOA->MODER &= ~(GPIO_MODER_MODER10);
	GPIOA->MODER |= (GPIO_MODER_MODER10_1);
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8);
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL8_0);
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL9_0);
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL10);
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL10_0);
}

/*
 * @Brief RGB Timer PWM Mode Configuration - TIM11
 */
void tim_TIM1_RGB_PWM_config(void)
{
	//Enable TIM1 clock
	RCC->APB2ENR|=(RCC_APB2ENR_TIM1EN);
	//Periodic
	TIM1->CR1 &=~ (TIM_CR1_OPM);
	//Mode-->Reset
	TIM1->CR2 &=~ (TIM_CR2_MMS);
	//Prescaler
	// CK_CNT=fCK_PSC / (PSC[15:0] + 1)
	TIM1->PSC =4-1;//20MHz TIM1 CLK frequency
	//Period
	TIM1->ARR = 4000-1;//5kHz PWM signal frequency
	//Clear update interrupt flag
	TIM1->SR &=~ (0x1UL<< (0U));
	//Disable Output Compare
	TIM1->CCER=0;
	//Compare and Capture as Output Mode
	TIM1->CCMR1 &=~ (TIM_CCMR1_CC1S);
	TIM1->CCMR1 &=~ (TIM_CCMR1_CC2S);
	TIM1->CCMR2 &=~ (TIM_CCMR2_CC3S);
	////CH1,CH2,CH3 as PWM output mode 1
	//CH1 is HIGH as long as TIMx_CNT<TIMx_CCR1 else is LOW
	TIM1->CCMR1 &=~ (0x7UL << (4U));
	TIM1->CCMR1 |= (0x6UL << TIM_CCMR1_OC1M_Pos);
	TIM1->CCMR1 &=~ (0x7UL << (12U));
	TIM1->CCMR1 |= (0x6UL << TIM_CCMR1_OC2M_Pos);
	TIM1->CCMR2 &=~ (0x7UL << (4U));
	TIM1->CCMR2 |= (0x6UL << TIM_CCMR2_OC3M_Pos);
	//Enable preload and fast CC mode
	TIM1->CCMR1 |= (TIM_CCMR1_OC1PE);
	TIM1->CCMR1 |= (TIM_CCMR1_OC1FE);
	TIM1->CCMR1 |= (TIM_CCMR1_OC2PE);
	TIM1->CCMR1 |= (TIM_CCMR1_OC2FE);
	TIM1->CCMR2 |= (TIM_CCMR2_OC3PE);
	TIM1->CCMR2 |= (TIM_CCMR2_OC3FE);
	//Enable auto-reload preload register
	TIM1->CR1 |= (TIM_CR1_ARPE);
	//CH1 HIGH polarity
	TIM1->CCER &=~ (TIM_CCER_CC1P);
	TIM1->CCER &=~ (TIM_CCER_CC2P);
	TIM1->CCER &=~ (TIM_CCER_CC3P);
	//PWM Duty Cycle(Initially)
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	//Enable OC CH1,CH2,CH3(not it's complementary channel)
	TIM1->BDTR |= (TIM_BDTR_MOE);
	TIM1->CCER |= (TIM_CCER_CC1E);
	TIM1->CCER |= (TIM_CCER_CC2E);
	TIM1->CCER |= (TIM_CCER_CC3E);
	//Trigger on timer
	TIM1->CR1 |= (TIM_CR1_CEN);
}
/*
 * @Brief RGB PWM Duty Cycle- Color Levels(0-4096) - TIM1
 */
void tim_TIM1_RGB_PWM_setColor(uint16_t color,uint8_t channel)
{
	// Calcular la dirección de CCR usando el canal
	uint32_t *CCR_address = (uint32_t *)((uint32_t)TIM1 + 0x34 + (channel - 1) * 0x04);
	*CCR_address = (uint16_t)((TIM1->ARR + 0.0f) * (color / 4096.0f));
}

/**
 * @Brief TIM6 as ADC MIC
 * Generate a 44.1kHz Sample Rate
 */
void tim_TIM6_MIC_config(void)
{
	//We are working with a SYSTEM Source of 16MHz
	//Enable TIM6 clock
	RCC->APB1ENR1 |=(RCC_APB1ENR1_TIM6EN);
	//Periodic
	TIM6->CR1 &=~ (TIM_CR1_OPM);
	//Mode-->TRGO
	TIM6->CR2 &=~ (TIM_CR2_MMS);
	TIM6->CR2 |= (TIM_CR2_MMS_1);
	//f=44.1kHz
	TIM6->PSC = 0;//Keep 16MHz of clock
	//Period
	TIM6->ARR=363-1;//16M/363= 44.077kHz
	//Update register on event
	TIM6->EGR=1;
//	//Start timer
//	TIM6->CR1 |= (TIM_CR1_CEN);
}
