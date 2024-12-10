/*
 * spi.c
 *
 *  Created on: 6 nov. 2024
 *      Author: Cuba
 */
#include <spi.h>

/**
 * @Brief GPIO Peripheral CLK control
 * STM32L432KC have only SPI1 and SPI3
 */
void spi_PeripheralClockControl(SPI_TypeDef *pSPIx , uint8_t EnorDi)
{

	if (EnorDi == ENABLE) {

		if (pSPIx == SPI1) {

			SPI1_CLOCK_ENABLE();

		}else if (pSPIx == SPI3) {

			SPI3_CLOCK_ENABLE();

		}

	}else{

		if (pSPIx == SPI1) {

			SPI1_CLOCK_DISABLE();

		}else if (pSPIx == SPI3) {

			SPI3_CLOCK_DISABLE();

		}
	}
}

/**
 * @brief SPI Initialization API
 */
void spi_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable the clock
	spi_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE ) ;
	//// Configure the SPIx_CR1 register
	// Configure the the SPI device mode
	pSPIHandle->pSPIx->CR1 &=~ (0x1 << SPI_CR1_MSTR_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_DeviceMode << SPI_CR1_MSTR_Pos);

	//Bus configuration
	if (pSPIHandle->SPIConfig.spi_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX) {

		// BIDIMODE should be cleared
		pSPIHandle->pSPIx->CR1 &=~ (0x1 << SPI_CR1_BIDIMODE_Pos);

	}else if (pSPIHandle->SPIConfig.spi_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX) {

		// BIDIMODE should be set
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_BIDIMODE_Pos);
		// Also we must set BIDIOE for TX only
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_BIDIOE_Pos);

	}else if (pSPIHandle->SPIConfig.spi_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {

		// BIDIMODE should be cleared ;
		pSPIHandle->pSPIx->CR1 &=~ (0x1 << SPI_CR1_BIDIMODE_Pos);

		// RXOONLY bit should be set
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_RXONLY_Pos);

	}
	// Configure SPI clock speed
	pSPIHandle->pSPIx->CR1 &=~ (0x7 << SPI_CR1_BR_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_SclkSpeed << SPI_CR1_BR_Pos);

	// Configure MSB/LSBFirst
	pSPIHandle->pSPIx->CR1 &=~ (0x1 <<  SPI_CR1_LSBFIRST_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_LSBFIRST << SPI_CR1_LSBFIRST_Pos);

	//Configure SPI CPOL/CPHA mode
	pSPIHandle->pSPIx->CR1 &=~ (0x1 <<  SPI_CR1_CPOL_Pos);
	pSPIHandle->pSPIx->CR1 &=~ (0x1 <<  SPI_CR1_CPHA_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_CPOL << SPI_CR1_CPOL_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_CPHA << SPI_CR1_CPHA_Pos);

	//Configure Software Slave Management(SSM)
	pSPIHandle->pSPIx->CR1 &=~ (0x1 <<  SPI_CR1_SSM_Pos);
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPIConfig.spi_SSM <<  SPI_CR1_SSM_Pos);
	//// Configure the SPIx_CR2 register

	//Configure SPI data size
	pSPIHandle->pSPIx->CR2 &=~ (0xF << SPI_CR2_DS_Pos);
	pSPIHandle->pSPIx->CR2 |= (pSPIHandle->SPIConfig.spi_DataSize << SPI_CR2_DS_Pos);

	//Enable error interrupt
	if(pSPIHandle->SPIConfig.spi_ErrorInterrupt)
	{
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_ERRIE);
	}

	//Finally Enable Peripheral
	pSPIHandle->pSPIx->CR1 |= (0x1 <<  SPI_CR1_SPE_Pos);
}

/**
 * @Brief SPI Reset Peripheral Registers
 */
void spi_DeInit(SPI_TypeDef *pSPIx)
{
	if (pSPIx == SPI1) {
		SPI1_REG_RESET() ;
	}else if (pSPIx == SPI3) {
		SPI3_REG_RESET() ;
	}
}

/**
 * @Brief SPI (NVIC)Interrupt Configuration
 */
void spi_IRQInterruptConfig(SPI_TypeDef *pSPIx,uint8_t ENorDIS,uint8_t interrupt_Priority)
{
	if (pSPIx == SPI1) {
		if (ENorDIS) {
			NVIC_SetPriority(SPI1_IRQn,interrupt_Priority);
			NVIC_EnableIRQ(SPI1_IRQn);
		} else {
			NVIC_DisableIRQ(SPI1_IRQn);
		}

	} else {
		if (ENorDIS) {
			NVIC_SetPriority(SPI3_IRQn,interrupt_Priority);
			NVIC_EnableIRQ(SPI3_IRQn);
		} else {
			NVIC_DisableIRQ(SPI3_IRQn);
		}

	}
}

/**
 * @Brief SPI Get Flag Status
 * @Parameter FlagName attempts for:
 * SPI:RXNE,TXE,CRCERR,BSY,MODF,OVR flags
 */
uint8_t spi_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t FlagName )
{
	if ((pSPIx->SR &  (FlagName))) {

		return FLAG_SET ;
	}

	return FLAG_RESET ;
}

/**
 * @Brief SPI Interrupt-Based Transmit
 */
uint8_t spi_SendIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t length)
{
	uint8_t state = pSPIHandle->TxState ;

	if (state != SPI_BUSY_IN_TX) {

		// save the buffer address and length in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer ;	// saving the buffer address
		pSPIHandle->TxLen = length        ;	// saving the buffer length

		// Set the SPI state is busy in transmission
		pSPIHandle->TxState = SPI_BUSY_IN_TX ;

		// enable the TXEI bit to get an interrupt when the TXE flag is set in SR register
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_TXEIE) ;
		// data transmission will happen in ISR handler
	}
	return state ;
}
/**
 * @Brief SPI Interrupt-Based Transmit
 */
uint8_t spi_ReadIT(SPI_Handle_t *pSPIHandle , uint8_t *pRxBuffer , uint32_t length)
{
	uint8_t state = pSPIHandle->RxState ;

	if (state != SPI_BUSY_IN_RX) {

		// save the buffer address and length in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer ;	// saving the buffer address
		pSPIHandle->RxLen     = length    ;	// saving the buffer length

		// Set the SPI state is busy in transmission
		pSPIHandle->RxState = SPI_BUSY_IN_RX ;

		// enable the TXEIE bit to get an interrupt when the TXE flag is set in  register
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_RXNEIE);
		// data transmission will happen in ISR handler
	}

	return state ;

}


/**
 * @Brief SPI Interrupt End of TX, stop TX and reset TX parameters
 */
void spi_CloseTransmissionIT(SPI_Handle_t *pSPIHandle )
{

	// Disable TXE interrupt
	//(Avoid to reenter in ISR Routine after TX complete)
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_TXEIE) ;

	//  Reset the TX buffer
	pSPIHandle->pTxBuffer = NULL ;

	// Reset the TX length to zero
	pSPIHandle->TxLen = 0 ;

	// Reset the TX state to spi_ready
	pSPIHandle->TxState = SPI_READY ;

}

/**
 * @Brief SPI Interrupt End of RX, stop RX and reset RX parameters
 * !Have to be call when length==0 in SPI ISR routine!
 */
void spi_CloseReceptionIT(SPI_Handle_t *pSPIHandle )
{
	// Disable the RXNE interrupt
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE) ;

	//  Reset the RX buffer
	pSPIHandle->pRxBuffer= NULL ;

	// Reset the RX length to zero
	pSPIHandle->RxLen = 0 ;

	// Reset the RX state to spi_ready
	pSPIHandle->RxState = SPI_READY ;

}

/**
 * @Brief SPI TXE Interrupt Handler
 * !Must be call in SPIx_IRQn routine when TXE==1 and TXEIE==1!
 */
void spi_TXE_Interrupt_Handler(SPI_Handle_t *pSPIHandle)
{
	//Recharges Data Register(DR)for send one or two bytes
	if ((pSPIHandle->pSPIx->CR2 & (SPI_CR2_DS))==(SPI_CR2_DS)) {
		//Data frame is 16bits
		pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen -- ;
		pSPIHandle->TxLen -- ;
		((uint16_t*)(pSPIHandle->pTxBuffer)++) ;
	}  else if((pSPIHandle->pSPIx->CR2 & (SPI_CR2_DS))==(SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0)) {
		//Data frame is 8bits
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer) ;
		pSPIHandle->TxLen --;
		(pSPIHandle->pTxBuffer)++ ;
	}
	//Validate if it's the end of TX to reset parameters
	if (pSPIHandle->TxLen == 0) {
		spi_CloseTransmissionIT(pSPIHandle);
		// Inform the application about the data transfer event completion
		spi_ApplicationEventCallBack(SPI_EVENT_TX_CMPLT) ;
	}}

/**
 * @Brief SPI RXNE Interrupt Handler
 * !Must be call in SPIx_IRQn routine when RXNE==1 and RXNEIE==1!
 */
void spi_RXNE_Interrupt_Handler(SPI_Handle_t *pSPIHandle)
{
	if ((pSPIHandle->pSPIx->CR2 & (SPI_CR2_DS))==(SPI_CR2_DS)) {
		//Data frame is 16bits
		// read 16 bit of  data from DR
		*((uint16_t*)(pSPIHandle->pRxBuffer)) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -- ;
		pSPIHandle->RxLen -- ;
		((uint16_t*)(pSPIHandle->pRxBuffer)++) ;

	}  else if((pSPIHandle->pSPIx->CR2 & (SPI_CR2_DS))==(SPI_CR2_DS_2|SPI_CR2_DS_1|SPI_CR2_DS_0)){
		//Data frame is 8bits
		// Read 8 bit data from DR
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen --;
		(pSPIHandle->pRxBuffer)++ ;
	}
	//Validate if it's the end of RX to reset parameters
	if (pSPIHandle->RxLen == 0) {
		spi_CloseReceptionIT(pSPIHandle);
		// Inform the application about the data transfer event completion
		spi_ApplicationEventCallBack(SPI_EVENT_RX_CMPLT) ;
	}
}

/**
 * @Brief SPI Error Interrupt Handler
 */
void spi_Error_Interrupt_Handler(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp ;
	// clear the over flag

	/* directly reading the DR register causes the OVR flag to reset but the value in the DR may be required
	 * by the application thats why we read the DR to clear the OVR flag only when SPI is not transmitting because
	 *  we receive data only when there is clock and clock only happens when master is transmitting */
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {

		// read operation to dr register
		temp = pSPIHandle->pSPIx->DR ;

		// read operation to sr register
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp ;

	// inform the application about the error
	spi_ApplicationEventCallBack(SPI_EVENT_OVR_ERR) ;
}

/**
 * @Brief SPIx Multi-Cause IRQ Handling
 */
void spi_IRQHandling (SPI_Handle_t *pSPIHandle)
{
	//Check for a RXNE interrupt request
	if ((pSPIHandle->pSPIx->SR & (SPI_SR_RXNE)) && (pSPIHandle->pSPIx->CR2 & (SPI_CR2_RXNEIE)))
	{
		// Handling RXNE interrupt
		spi_RXNE_Interrupt_Handler(pSPIHandle);
	}
	//Check for a TXE interrupt request
	if ((pSPIHandle->pSPIx->SR & (SPI_SR_TXE)) && (pSPIHandle->pSPIx->CR2 & (SPI_CR2_TXEIE)))
	{
		// Handling TXE interrupt
		spi_TXE_Interrupt_Handler(pSPIHandle);
	}
	//Check for an error interrupt request(OVR in this case)
	if ((pSPIHandle->pSPIx->SR & (SPI_SR_OVR)) && (pSPIHandle->pSPIx->CR2 & (SPI_CR2_ERRIE)))
	{
		// Handling error interrupt
		spi_Error_Interrupt_Handler(pSPIHandle);
	}
}

/**
 * @Brief SPI1 GPIO pins configuration
 * (PA5(A4)->SP1_SCK,PA6(A5)->SP1_MISO,PA7(A6)->SP1_MOSI)
 * (PA4(A3)->CS)
 */
void spi_GPIO_config(void)
{
	//SPI AF GPIO pins configuration
	GPIO_Handle_t spiPins;
	spiPins.pGPIOx=GPIOA;
	spiPins.pinMode=PIN_MODE_ALTFN;
	spiPins.pinOutputType=PIN_OP_TYPE_PP;
	spiPins.pinPUPDControl=PIN_NO_PUPD;
	spiPins.pinSpeed=PIN_SPEED_HIGH;
	spiPins.pinAltFunMode=AF5;
	spiPins.pinNumber=PIN_NO_7;//MOSI
	gpio_Init(&spiPins);
	spiPins.pinNumber=PIN_NO_6;//MISO
	spiPins.pinPUPDControl=PIN_PULL_UP;
	gpio_Init(&spiPins);
	spiPins.pinNumber=PIN_NO_5;//SCLK
	spiPins.pinPUPDControl=PIN_NO_PUPD;
	gpio_Init(&spiPins);
	spiPins.pinNumber=PIN_NO_4;//CS
	spiPins.pinMode=PIN_MODE_OUTPUT;
	gpio_Init(&spiPins);
	//Default value of CS is HIGH
	GPIOA->BSRR |= (GPIO_BSRR_BS4);
}

/**
 * @Brief SPI1 peripheral configuration
 *--------------------
 * SPI_MODE CPOL  CPHA
 * 0        0     0
 * 1        0     1
 * 2        1     0
 * 3        1     1
 * -------------------
 *
 */
void spi_config(uint32_t system_Clock,SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx=SPI1;
	pSPIHandle->SPIConfig.spi_BusConfig=SPI_BUS_CONFIG_FULL_DUPLEX;
	pSPIHandle->SPIConfig.spi_CPHA=SPI_CPHA_LOW;
	pSPIHandle->SPIConfig.spi_CPOL=SPI_CPOL_LOW;
	pSPIHandle->SPIConfig.spi_DataSize=SPI_DS_8BITS;
	pSPIHandle->SPIConfig.spi_DeviceMode=SPI_DEVICE_MODE_MASTER;
	pSPIHandle->SPIConfig.spi_ErrorInterrupt=DISABLE;
	pSPIHandle->SPIConfig.spi_LSBFIRST=SPI_MSBFIRST;
	pSPIHandle->SPIConfig.spi_SSM=SPI_SSM_EN;
	if (system_Clock==16000000)//To generate a 250kHz baud rate
	{
		pSPIHandle->SPIConfig.spi_SclkSpeed=SPI_SCLK_SPEED_DIV64;
	}
	else//To generate a 10MHz baud rate(with fPCLK=80MHz)
	{
		pSPIHandle->SPIConfig.spi_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	}
	spi_Init(pSPIHandle);
}

/**
 * @Brief SPI1 transmit
 */
bool spi_transmit(uint8_t *pointer_data,uint8_t len,uint32_t timeout)
{
	//Enable SPI1(if not)
	SPI1->CR1 |= (SPI_CR1_SPE);
	//Timeout initial ticks
	uint8_t dataIdx = 0;
	uint32_t startTick = HAL_GetTick();
	//While loop,TX data managing timeout
	while(dataIdx<len)
	{
		if(SPI1->SR & SPI_SR_TXE) //Tx buffer empty
		{
			SPI1->DR = pointer_data[dataIdx];
			dataIdx++;
		}
		else //Manage timeout
		{
			if((HAL_GetTick() - startTick)>= timeout) return false;
		}
	}
	//Wait for busy flag
	while((SPI1->SR & SPI_SR_BSY))
	{
		if((HAL_GetTick() - startTick)>= timeout) return false;
	}
	//Clear overrun conditions
	volatile uint32_t tempRead = SPI1->DR;
	tempRead = SPI1->SR;
	(void)tempRead ;
	return true;

}

/**
 * @Brief SPI1 receive
 */
bool spi_receive(uint8_t *pointer_data,uint8_t len,uint32_t timeout)
{
	//Enable SPI1(if not)
	SPI1->CR1 |= (SPI_CR1_SPE);
	//Timeout initial ticks
	uint8_t dataIdx = 0;
	uint32_t startTick = HAL_GetTick();
	bool isTransmit=1;
	//While loop: TX first, then RX, managing timeout
	while(dataIdx<len)
	{
		//TX dummy data
		if((SPI1->SR & SPI_SR_TXE) && isTransmit) //Tx buffer empty
		{
			SPI1->DR = 0xFF;
			isTransmit=false;
		}
		if (SPI1->SR & SPI_SR_RXNE)
		{
			pointer_data[dataIdx] = SPI1->DR;
			dataIdx++;
			isTransmit=true;

		}
		else //Manage timeout
		{
			if((HAL_GetTick() - startTick)>= timeout) return false;
		}
	}
	//Wait for busy flag
	while((SPI1->SR & SPI_SR_BSY))
	{
		if((HAL_GetTick() - startTick)>= timeout) return false;
	}
	//Clear overrun conditions
	volatile uint32_t tempRead = SPI1->DR;
	tempRead = SPI1->SR;
	(void)tempRead ;
	return true;

}

/**
 * @Brief SPI1 transmit/receive
 */
bool spi_transmitReceive(uint8_t *RX_buffer,uint8_t *TX_buffer,uint8_t len,uint32_t timeout)
{
	//Enable SPI1(if not)
	SPI1->CR1 |= (SPI_CR1_SPE);
	//Timeout initial ticks
	uint8_t dataIdx = 0;
	uint32_t startTick = HAL_GetTick();
	uint32_t actualTick;
	bool isTransmit=1;
	//While loop: TX first, then RX, managing timeout
	while(dataIdx<len)
	{
		//TX data
		if((SPI1->SR & SPI_SR_TXE) && isTransmit) //Tx buffer empty
		{
			SPI1->DR = TX_buffer[dataIdx];
			isTransmit=false;
		}
		//RX data
		if (SPI1->SR & SPI_SR_RXNE)
		{
			RX_buffer[dataIdx] = SPI1->DR;
			dataIdx++;
			isTransmit=true;

		}
		else //Manage timeout
		{
			actualTick=HAL_GetTick();
			if((actualTick - startTick)>= timeout) return false;
		}
	}
	//Wait for busy flag
	while((SPI1->SR & SPI_SR_BSY))
	{
		if((HAL_GetTick() - startTick)>= timeout) return false;
	}
	//Clear overrun conditions
	volatile uint32_t tempRead = SPI1->DR;
	tempRead = SPI1->SR;
	(void)tempRead ;
	return true;

}


/**
 * @Brief SD Chip Select Set/Reset
 */
void spi_cs_sd_write(bool state)
{
	if (state)
	{
		GPIOA->BSRR |= (GPIO_BSRR_BS4);
	}
	else
	{
		GPIOA->BSRR |= (GPIO_BSRR_BR4);
	}
}

void spi_ApplicationEventCallBack(uint8_t eventcode)
{
	if (eventcode == SPI_EVENT_TX_CMPLT) {
		printf("Transmission completed \n") ;
	} else if(eventcode == SPI_EVENT_RX_CMPLT){
		printf("Reception completed \n") ;
	}else if (eventcode == SPI_EVENT_OVR_ERR) {
		printf("OVR error has occurred clearing now \n") ;
	}else if (eventcode == SPI_EVENT_CRC_ERR) {
		printf("CRC Error has occurred \n ") ;
	}

}




