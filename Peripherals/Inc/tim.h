/*
 * tim.h
 *
 *  Created on: 12 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include <main.h>
/*
 * @Brief Delay Timer Configuration - TIM2
 */
void tim_TIM2_delayConfig(void);
/*
 * @Brief Millisecond Delay - TIM2
 */
void tim_TIM2_msDelay(uint32_t ms);
/*
 * @Brief Microsecond Delay - TIM2
 */
void tim_TIM2_usDelay(uint32_t us);
/*
 * @Brief Input Capture Timer Configuration - TIM1
 */
void tim_TIM1_InputCaptureConfig(void);
/*
 * @Brief PWM Channel GPIO Configuration - TIM15
 */
void tim_TIM15_PWM_GPIO_config(void);
/*
 * @Brief Timer PWM Mode Configuration - TIM15
 */
void tim_TIM15_PWM_config(void);
/**
 * @Brief PWM Duty Cycle  - Brightness(0-4096)
 */
void tim_PWM_setBrightness(uint16_t bright);
/**
 * @Brief TIM6 as Trigger Output Configuration
 */
void tim_TIM6_TRGO_config(uint16_t msPeriod);
/*
 * @Brief RGB PWM Channel GPIO Configuration - TIM1
 */
void tim_TIM1_RGB_PWM_GPIO_config(void);
/*
 * @Brief RGB Timer PWM Mode Configuration - TIM11
 */
void tim_TIM1_RGB_PWM_config(void);
/*
 * @Brief RGB PWM Duty Cycle- Color Levels(0-4096) - TIM1
 */
void tim_TIM1_RGB_PWM_setColor(uint16_t color,uint8_t channel);
/**
 * @Brief TIM6 as ADC MIC
 * Generate a 44.1kHz Sample Rate
 */
void tim_TIM6_MIC_config(void);





#endif /* INC_TIM_H_ */
