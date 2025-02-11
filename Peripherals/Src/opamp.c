/*
 * opamp.c
 *
 *  Created on: 8 dic. 2024
 *      Author: Cuba
 */

#include <opamp.h>
/**
 * @Brief OPAMP configuration
 */
void opamp_config(void)
{
	//Enable GPIO port A clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	//Pin mode as analog mode
	GPIOA->MODER |= (GPIO_MODER_MODE0);
	GPIOA->MODER |= (GPIO_MODER_MODE3);
	//Enable OPAMP clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_OPAMPEN);
	//Setting OPAMP voltage Range (VDDA>2.4V ->1, else 0);
	OPAMP->CSR |=(OPAMP1_CSR_OPARANGE);
	//Setting OPAMP low power mode
	OPAMP->CSR |=(OPAMP1_CSR_OPALPM);
	//Enable Programmable Gain Amplifier mode
	OPAMP->CSR &=~(OPAMP1_CSR_OPAMODE);
	OPAMP->CSR |=(OPAMP1_CSR_OPAMODE_1);
	//Setting OP Gain of 4
	OPAMP->CSR &=~(OPAMP1_CSR_PGAGAIN);
	OPAMP->CSR |=(OPAMP1_CSR_PGAGAIN_0);
	//Inverting input not externally connected
	OPAMP->CSR &=~(OPAMP1_CSR_VMSEL);
	OPAMP->CSR |=(OPAMP1_CSR_VMSEL_1);
	//GPIO (PA0) connected to Non-Inverting Input
	OPAMP->CSR &=~(OPAMP1_CSR_VPSEL);
	//OPAMP Switch ON
	OPAMP->CSR |=(OPAMP1_CSR_OPAEN);
    //--------Calibration Procedure-----------//
//	//Activate user trim
//	OPAMP->CSR |=(OPAMP1_CSR_USERTRIM);
//	//Start Calibration Mode
//	OPAMP->CSR |=(OPAMP1_CSR_CALON);
//	//Offset cal high for low-power mode(N differential pair)
//	//Starting from cero
//
//	//Wait until CALOUT is HIGH
//	while(!(OPAMP->CSR & (OPAMP1_CSR_CALOUT)))
//	{
//
//	}
}
