/*
 * adc.c
 *
 *  Created on: 18 oct. 2024
 *      Author: Cuba
 */
#include <adc.h>

/*
 * @brief ADC GPIO Pin PA1(A1)->Potentiometer
 * PA3(A2)->Joystick X PA4(A3)->Joystick Y
 */
void adc_GPIO_config(void)
{
	//Enable GPIO port A clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	//Pin mode as analog mode
	GPIOA->MODER |= (GPIO_MODER_MODE1);
	GPIOA->MODER |= (GPIO_MODER_MODE3);
	GPIOA->MODER |= (GPIO_MODER_MODE4);

}
/*
 * @brief delay for ADC voltage regulator start-up time(20us)
 * A 80Mhz->CM=12.5ns->CM x 2000=25us>20us
 */
void adc_ADCVREG_STUP_delay(void)
{
	volatile uint32_t i;
	for (i=0;i<2000;i++) {}
}

/*
 * @brief ADC Single Channel Auto-delayed Continuous Conversion Mode
 */
void adc_single_channel_auto_delayed_continuous_conv_config(void)
{
	//Enable ADC clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN);
	//Select HCLK/1 as ADC clock source
	ADC1_COMMON->CCR &=~(ADC_CCR_CKMODE);
	ADC1_COMMON->CCR |= (ADC_CCR_CKMODE_0);
	//Exit the Deep-power-down mode
	ADC1->CR &=~ (ADC_CR_DEEPPWD);
	//Enable the ADC internal voltage regulator
	ADC1->CR |= (ADC_CR_ADVREGEN);
	//Wait for the startup time of the ADC voltage regulator
	adc_ADCVREG_STUP_delay();
	////ADC Calibration
	//Single-Ended Input mode calibration
	ADC1->CR &=~ (ADC_CR_ADCALDIF);
	//Calibration start
	ADC1->CR |= (ADC_CR_ADCAL);
	//Wait until calibration is stopped
	while (ADC1->CR & ADC_CR_ADCAL);
	//Right Align Mode
	ADC1->CFGR &=~ (ADC_CFGR_ALIGN);
	//Continuous Conversion Mode
	ADC1->CFGR |= (ADC_CFGR_CONT);
	//Auto-delayed Mode
	ADC1->CFGR |= (ADC_CFGR_AUTDLY);
	//Sample time to 24.5 ADC CLK Cycles
	//Tconv= (24.5+12.5) ADC CLK Cycles = 37 ADC clock cycles = 462.5ns
	//Max Sample Frequency = 2.16MHz
	ADC1->SMPR1 &=~ (ADC_SMPR1_SMP6);
	ADC1->SMPR1 |= (0x3UL << ADC_SMPR1_SMP6_Pos);
	//Single Channel Mode
	ADC1->SQR1 &=~(ADC_SQR1_L);
	//Select channel to convert: PA1(A1)->ADC12_IN6
	ADC1->SQR1  &=~ (ADC_SQR1_SQ1);
	ADC1->SQR1  |= (0x6UL << (6U));
	//Software Trigger
	ADC1->CFGR &=~ (ADC_CFGR_EXTEN);
	//Clear the ADC ready flag(It is cleared by writing 1 to it.)
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//Enable ADC
	ADC1->CR |=(ADC_CR_ADEN);
	//Wait until ADC is ready
	while (!(ADC1->CR & ADC_CR_ADEN));
	//Clear the ADC ready flag again
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//Start the first conversion
	ADC1->CR |=(ADC_CR_ADSTART);
}
/*
 * @Brief ADC Multi-Channel configuration
 * 3 Channels this time:1,3,4\\(Single conversion)
 */
void adc_multiChannel_config(void)
{
	//Enable ADC clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN);
	//Select HCLK/1 as ADC clock source
	ADC1_COMMON->CCR &=~(ADC_CCR_CKMODE);
	ADC1_COMMON->CCR |= (ADC_CCR_CKMODE_0);
	//Exit the Deep-power-down mode
	ADC1->CR &=~ (ADC_CR_DEEPPWD);
	//Enable the ADC internal voltage regulator
	ADC1->CR |= (ADC_CR_ADVREGEN);
	//Wait for the startup time of the ADC voltage regulator
	adc_ADCVREG_STUP_delay();
	////ADC Calibration
	//Single-Ended Input mode calibration
	ADC1->CR &=~ (ADC_CR_ADCALDIF);
	//Calibration start
	ADC1->CR |= (ADC_CR_ADCAL);
	//Wait until calibration is stopped
	while (ADC1->CR & ADC_CR_ADCAL);
	//Right Align Mode
	ADC1->CFGR &=~ (ADC_CFGR_ALIGN);
	//Single Conversion Mode
	ADC1->CFGR &=~ (ADC_CFGR_CONT);
	//Sample time to 24.5 ADC CLK Cycles
	//Tconv= (24.5+12.5) ADC CLK Cycles = 37 ADC clock cycles = 462.5ns
	//Max Sample Frequency = 2.16MHz
	ADC1->SMPR1 &=~ (ADC_SMPR1_SMP6);
	ADC1->SMPR1 |= (0x3UL << ADC_SMPR1_SMP6_Pos);
	ADC1->SMPR1 &=~ (ADC_SMPR1_SMP8);
	ADC1->SMPR1 |= (0x3UL << ADC_SMPR1_SMP8_Pos);
	ADC1->SMPR1 &=~ (ADC_SMPR1_SMP9);
	ADC1->SMPR1 |= (0x3UL << ADC_SMPR1_SMP9_Pos);
	//Multi-channel mode(3 conversions)
	ADC1->SQR1 &=~(ADC_SQR1_L);
	ADC1->SQR1 |=(ADC_SQR1_L_1);
	//Conversion sequence:
	/*
	 * 1st->PA1(A1)->ADC12_IN6
	 * 2nd->PA3(A2)->ADC12_IN8
	 * 3rd->PA4(A3)->ADC12_IN9
	 */
	ADC1->SQR1  &=~ (ADC_SQR1_SQ1);
	ADC1->SQR1  |= (0x6UL << (6U));
	ADC1->SQR1  &=~ (ADC_SQR1_SQ2);
	ADC1->SQR1  |= (0x8UL << (12U));
	ADC1->SQR1  &=~ (ADC_SQR1_SQ3);
	ADC1->SQR1  |= (0x9UL << (18U));
	//TIM6 TRGO Trigger
	ADC1->CFGR &=~ (ADC_CFGR_EXTEN);
	ADC1->CFGR |= (ADC_CFGR_EXTEN_0);
	ADC1->CFGR &=~ (ADC_CFGR_EXTSEL);
	ADC1->CFGR |= (0xDUL << ADC_CFGR_EXTSEL_Pos);//Event 13->TIM6_TRGO selected
	//DMA Circular Mode
	ADC1->CFGR |= (ADC_CFGR_DMACFG);
	//Enable DMA-ADC registers
	ADC1->CFGR |= (ADC_CFGR_DMAEN);
	//Clear the ADC ready flag(It is cleared by writing 1 to it.)
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//Enable ADC
	ADC1->CR |=(ADC_CR_ADEN);
	//Wait until ADC is ready
	while (!(ADC1->CR & ADC_CR_ADEN));
	//Clear the ADC ready flag again
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//ADC begin
	ADC1->CR |=(ADC_CR_ADSTART);
}
/*
 * @Brief ADC DMA Configuration
 */
void adc_multiChannel_DMA_config(volatile uint16_t * pointer_ADC_buffer)
{
	//DMA1->Channel1-->ADC1
	//Enable DMA1 clock
	RCC->AHB1ENR |=(RCC_AHB1ENR_DMA1EN);
	//Clear DMA1-CH1 status flags
	DMA1->IFCR |=(DMA_IFCR_CGIF1);
	//Peripheral address (Source is ADC_DR)
	DMA1_Channel1->CPAR =(uint32_t)(&(ADC1->DR));
	//Memory address (Destination is uint16_t adc_samples[3])
	DMA1_Channel1->CMAR =(uint32_t)(pointer_ADC_buffer);
	//Set No. of transfers to 3
	DMA1_Channel1->CNDTR=3;
	//Normal/Circular DMA operation
	DMA1_Channel1->CCR |= (DMA_CCR_CIRC);
	//Enable memory increment
	DMA1_Channel1->CCR |= (DMA_CCR_MINC);
	//Disable peripheral increment
	DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);
	//ADC data->12bits, therefore Peripheral->16bits
	DMA1_Channel1->CCR &= ~(DMA_CCR_PSIZE);
	DMA1_Channel1->CCR |= (DMA_CCR_PSIZE_0);
	//Memory 16 bits(src and dst have to match in data size)
	DMA1_Channel1->CCR &= ~(DMA_CCR_MSIZE);
	DMA1_Channel1->CCR |= (DMA_CCR_MSIZE_0);
	//Direction is Peripheral to memory
	DMA1_Channel1->CCR &= ~(DMA_CCR_DIR);
	//Enable DMA
	DMA1_Channel1->CCR |= (DMA_CCR_EN);
	//Enable Transfer complete interrupt - DMA
	DMA1_Channel1->CCR |= (DMA_CCR_TCIE);
	NVIC_SetPriority(DMA1_Channel1_IRQn,0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @Brief ADC MIC(with OPAMP output as input) configuration
 * PA3(A2)->ADC12_IN8 is the OPAMP output
 */
void adc_MIC_config(void)
{
	//Enable ADC clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN);
	//Select HCLK/1 as ADC clock source
	ADC1_COMMON->CCR &=~(ADC_CCR_CKMODE);
	ADC1_COMMON->CCR |= (ADC_CCR_CKMODE_0);
	//Exit the Deep-power-down mode
	ADC1->CR &=~ (ADC_CR_DEEPPWD);
	//Enable the ADC internal voltage regulator
	ADC1->CR |= (ADC_CR_ADVREGEN);
	//Wait for the startup time of the ADC voltage regulator
	adc_ADCVREG_STUP_delay();
	////ADC Calibration
	//Single-Ended Input mode calibration
	ADC1->CR &=~ (ADC_CR_ADCALDIF);
	//Calibration start
	ADC1->CR |= (ADC_CR_ADCAL);
	//Wait until calibration is stopped
	while (ADC1->CR & ADC_CR_ADCAL);
	//Right Align Mode
	ADC1->CFGR &=~ (ADC_CFGR_ALIGN);
	//Single Conversion Mode
	ADC1->CFGR &=~ (ADC_CFGR_CONT);
	//-----Sample time to 24.5 ADC CLK Cycles-----//
	//Tconv= (24.5+12.5) ADC CLK Cycles = 37 ADC clock cycles = 513.88ns
	//Total Tconv= Tconv *4= 2.055us(with N=4 oversampling)
	//Oversampled max frequency=486.49kHz
	ADC1->SMPR1 &=~ (ADC_SMPR1_SMP8);
	ADC1->SMPR1 |= (0x3UL << ADC_SMPR1_SMP8_Pos);
	//Single-channel mode
	ADC1->SQR1 &=~(ADC_SQR1_L);
	ADC1->SQR1  &=~ (ADC_SQR1_SQ1);
	ADC1->SQR1  |= (0x8UL << (6U));
	//----------------Oversampling Code-----------------------------//
	//Enable Regular Oversampled Mode
	ADC1->CFGR2 |= (ADC_CFGR2_ROVSE);
	//Oversampling Ratio to 4 (no bit shifting)
	ADC1->CFGR2 &=~ (ADC_CFGR2_OVSR);
	ADC1->CFGR2 |= (ADC_CFGR2_OVSR_0);
	//-------------------End of Oversampling Code------------------//
	//TIM6 TRGO Trigger
	ADC1->CFGR &=~ (ADC_CFGR_EXTEN);
	ADC1->CFGR |= (ADC_CFGR_EXTEN_0);
	ADC1->CFGR &=~ (ADC_CFGR_EXTSEL);
	ADC1->CFGR |= (0xDUL << ADC_CFGR_EXTSEL_Pos);//Event 13->TIM6_TRGO selected
	//DMA Circular Mode
	ADC1->CFGR |= (ADC_CFGR_DMACFG);
	//Enable DMA-ADC registers
	ADC1->CFGR |= (ADC_CFGR_DMAEN);
	//Clear the ADC ready flag(It is cleared by writing 1 to it.)
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//Enable ADC
	ADC1->CR |=(ADC_CR_ADEN);
	//Wait until ADC is ready
	while (!(ADC1->CR & ADC_CR_ADEN));
	//Clear the ADC ready flag again
	ADC1->ISR |=(ADC_ISR_ADRDY);
	//ADC begin
	ADC1->CR |=(ADC_CR_ADSTART);
}

/**
 * @Brief ADC MIC DMA configuration
 */
void adc_MIC_DMA_config(uint16_t * pADC_Buffer,uint16_t size)
{
	//DMA1->Channel1-->ADC1
	//Enable DMA1 clock
	RCC->AHB1ENR |=(RCC_AHB1ENR_DMA1EN);
	//Clear DMA1-CH1 status flags
	DMA1->IFCR |=(DMA_IFCR_CGIF1);
	//Peripheral address (Source is ADC_DR)
	DMA1_Channel1->CPAR =(uint32_t)(&(ADC1->DR));
	//Memory address (Destination is uint16_t adc_samples[3])
	DMA1_Channel1->CMAR =(uint32_t)(pADC_Buffer);
	//Set No. of transfers to 4096
	DMA1_Channel1->CNDTR=4096;
	//Normal/Circular DMA operation
	DMA1_Channel1->CCR |= (DMA_CCR_CIRC);
	//Enable memory increment
	DMA1_Channel1->CCR |= (DMA_CCR_MINC);
	//Disable peripheral increment
	DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);
	//ADC data->12bits, therefore Peripheral->16bits
	DMA1_Channel1->CCR &= ~(DMA_CCR_PSIZE);
	DMA1_Channel1->CCR |= (DMA_CCR_PSIZE_0);
	//Memory 16 bits(src and dst have to match in data size)
	DMA1_Channel1->CCR &= ~(DMA_CCR_MSIZE);
	DMA1_Channel1->CCR |= (DMA_CCR_MSIZE_0);
	//Direction is Peripheral to memory
	DMA1_Channel1->CCR &= ~(DMA_CCR_DIR);
	//Enable DMA
	DMA1_Channel1->CCR |= (DMA_CCR_EN);
	//Enable Transfer complete interrupt - DMA
	DMA1_Channel1->CCR |= (DMA_CCR_TCIE);
	NVIC_SetPriority(DMA1_Channel1_IRQn,0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	//Enable Half-Transfer complete interrupt - DMA
	DMA1_Channel1->CCR |= (DMA_CCR_HTIE);


}

