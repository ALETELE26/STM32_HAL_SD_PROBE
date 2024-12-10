  /*
 * adc.h
 *
 *  Created on: 17 oct. 2024
 *      Author: Cuba
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <main.h>

/*
 * @brief ADC GPIO Pin PA1(A1)->Potentiometer
 * PA3(A2)->Joystick X PA4(A3)->Joystick Y
 */
void adc_GPIO_config(void);
/*
 * @brief ADC Single Channel Auto-delayed Continuous Conversion Mode
 */
void adc_single_channel_auto_delayed_continuous_conv_config(void);
/*
 * @brief delay for ADC voltage regulator start-up time(20us)
 * A 80Mhz->CM=12.5ns->CM x 2000=25us>20us
 */
void adc_ADCVREG_STUP_delay(void);
/*
 * @Brief ADC Multi-Channel configuration
 * 3 Channels this time:1,3,4\\(Single conversion)
 */
void adc_multiChannel_config(void);
/*
 * @Brief ADC DMA Configuration
 */
void adc_multiChannel_DMA_config(volatile uint16_t * pointer_ADC_buffer);

/**
 * @Brief ADC MIC(with OPAMP output as input) configuration
 */
void adc_MIC_config(void);

/**
 * @Brief ADC MIC DMA configuration
 */
void adc_MIC_DMA_config(uint16_t * pADC_Buffer,uint16_t size);


#endif /* INC_ADC_H_ */



