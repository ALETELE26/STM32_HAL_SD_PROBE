/*
 * uart.c
 *
 *  Created on: 11 oct. 2024
 *      Author: Alejandro Iglesias Gutiérrez
 */

#include <uart.h>

/*
 * @Brief UART1 GPIO Config(PA9(Tx)->D1;PA10(Rx)->D0)
 */
void uart_UART1_GPIO_config(void)
{
  // Enable PortA Clock
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
  // Mode to AF (UART1)
  GPIOA->MODER &= ~(GPIO_MODER_MODER9);
  GPIOA->MODER &= ~(GPIO_MODER_MODER10);
  GPIOA->MODER |= (GPIO_MODER_MODE9_1);
  GPIOA->MODER |= (GPIO_MODER_MODE10_1);
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
  GPIOA->OSPEEDR |= (0x2UL << (18U));
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR10);
  GPIOA->OSPEEDR |= (0x2UL << (20U));
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL10);
  GPIOA->AFR[1] |= (0x7UL << (4U));
  GPIOA->AFR[1] |= (0x7UL << (8U));
}
/*
 * @Brief UART1 Peripheral Config
 */
void uart_UART1_config()
{
  //// Enable floating-point unit.
  //SCB->CPACR    |=  ( 0xFUL << (20U) );
  // Enable UART1 Clock
  RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);
  // Set word length to 8-bits
  USART1->CR1  &= ~(USART_CR1_M1);
  // Set baud rate to 9600 Hz
  USART1->BRR = 8333;//80MHz/8333=9600b/s
  // Parity control disabled
  USART1->CR1  &= ~(USART_CR1_PCE);
  // Stop bits to 1
  USART1->CR2  &= ~(USART_CR2_STOP);
  // Disable HW flow control
  USART1->CR3  &= ~(USART_CR3_CTSE);
  USART1->CR3  &= ~(USART_CR3_RTSE);
  // Clear flags
  USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART1->CR3 &= ~(USART_CR3_HDSEL | USART_CR3_IREN | USART_CR3_SCEN );
  // Enable UART1 peripheral
  USART1->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE );
}
/*
 * @Brief UART1 Transmit
 */
bool uart_UART1_TX(uint8_t *data,uint8_t len,uint32_t timeout)
{
  //Wait on TXE to start transmit
  //Write to DR as TXE flag is HIGH (Tx buffer Empty)
  uint8_t dataIdx = 0;
  uint32_t startTick = HAL_GetTick();
  while(dataIdx<len)
  {
    if(USART1->ISR & USART_ISR_TXE) //Tx buffer empty
    {
      USART1->TDR = data[dataIdx];
      dataIdx++;
    }
    else //Manage timeout
    {
      if((HAL_GetTick() - startTick)>= timeout) return false;
    }
  }
  //Wait for busy flag
  while(!(USART1->ISR & USART_ISR_TC))
  {
    if((HAL_GetTick() - startTick)>= timeout) return false;
  }
  return true;
}
/*
 * @Brief UART2 GPIO Config(PA2(Tx);PA15(Rx))
 */
void uart_UART2_GPIO_config(void)
{
  // Enable PortA Clock
  RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN );
  // Configure pins PA2, PA15 for USART2 .
  GPIOA->MODER    &= ~( ( 0x3 << ( 2 * 2 ) ) |
      ( 0x3 << ( 15 * 2 ) ) );
  GPIOA->MODER    |=  ( ( 0x2 << ( 2 * 2 ) ) |
      ( 0x2 << ( 15 * 2 ) ) );
  GPIOA->OTYPER   &= ~( ( 0x1 << 2 ) |
      ( 0x1 << 15 ) );
  GPIOA->OSPEEDR  &= ~( ( 0x3 << ( 2 * 2 ) ) |
      ( 0x3 << ( 15 * 2 ) ) );
  GPIOA->OSPEEDR  |=  ( ( 0x2 << ( 2 * 2 ) ) |
      ( 0x2 << ( 15 * 2 ) ) );
  GPIOA->AFR[ 0 ] &= ~( ( 0xF << ( 2 * 4 ) ) );
  GPIOA->AFR[ 0 ] |=  ( ( 0x7 << ( 2 * 4 ) ) );
  GPIOA->AFR[ 1 ] &= ~( ( 0xF << ( ( 15 - 8 ) * 4 ) ) );
  GPIOA->AFR[ 1 ] |=  ( ( 0x3 << ( ( 15 - 8 ) * 4 ) ) );
}
/*
 * @Brief UART2 Peripheral Config
 */
void uart_UART2_config(uint32_t SYSTEM_CLK)
{
  //// Enable floating-point unit.
  //SCB->CPACR    |=  ( 0xFUL << (20U) );
  // Enable UART2 Clock
  RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
  // Set word length to 8-bits
  USART2->CR1  &= ~(USART_CR1_M1);
  // Set baud rate to 9600 Hz
  USART2->BRR = SYSTEM_CLK/9600;
  // Parity control disabled
  USART2->CR1  &= ~(USART_CR1_PCE);
  // Stop bits to 1
  USART2->CR2  &= ~(USART_CR2_STOP);
  // Disable HW flow control
  USART2->CR3  &= ~(USART_CR3_CTSE);
  USART2->CR3  &= ~(USART_CR3_RTSE);
  // Clear flags
  USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART2->CR3 &= ~(USART_CR3_HDSEL | USART_CR3_IREN | USART_CR3_SCEN );
  // Enable UART2 peripheral
  USART2->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE );
  // Enable Rx ready to read interruption
  USART2->CR1 |= ( USART_CR1_RXNEIE );
  //All priority bits assigned to the preempt priority group
  NVIC_SetPriorityGrouping(0U);
  //Priority of the interrupt (NVIC)
  NVIC_SetPriority(USART2_IRQn,1);
  //Enable the interrupt
  NVIC_EnableIRQ(USART2_IRQn);

}
/*
 * @Brief UART2 Transmit
 */
bool uart_UART2_TX(uint8_t *data,uint8_t len,uint32_t timeout)
{
  //Write to DR as TXE flag is HIGH (Tx buffer Empty)
  uint8_t dataIdx = 0;
  uint32_t startTick = HAL_GetTick();
  while(dataIdx<len)
  {
    if(USART2->ISR & USART_ISR_TXE) //Tx buffer empty
    {
      USART2->TDR = data[dataIdx];
      dataIdx++;
    }
    else //Manage timeout
    {
      if((HAL_GetTick() - startTick)>= timeout) return false;
    }
  }
  //Wait for busy flag
  while((USART2->ISR & USART_ISR_TC))
  {
    if((HAL_GetTick() - startTick)>= timeout) return false;
  }
  return true;

}
/**
 * @Brief UART2 DMA configuration
 */
void uart_UART2_DMA_config(void)
{
  //Enable DMA-UART continuos communications
  USART2->CR3 |=( USART_CR3_DMAT | USART_CR3_DMAR);
  //Enable DMA1 clock
  RCC->AHB1ENR |=(RCC_AHB1ENR_DMA1EN);

  ////TX Channel DMA configuration(Channel7-->USART2_TX)
  //Clear DMA1-CH7 status flags
  DMA1->IFCR |=(DMA_IFCR_CGIF7);
  //Select UART2_TX as DMA_CH7
  DMA1_CSELR->CSELR &=~(DMA_CSELR_C7S);
  DMA1_CSELR->CSELR |= (0x2UL << (24U));
  //Peripheral address (Dest is USART_TDR)
  DMA1_Channel7->CPAR =(uint32_t)(&(USART2->TDR));
  //Normal/Circular DMA operation
  DMA1_Channel7->CCR &= ~ (DMA_CCR_CIRC);
  //Enable memory increment
  DMA1_Channel7->CCR |= (DMA_CCR_MINC);
  //Disable peripheral increment
  DMA1_Channel7->CCR &= ~(DMA_CCR_PINC);
  //Memory/Peripheral Size to 8bits
  DMA1_Channel7->CCR &= ~(DMA_CCR_MSIZE);
  DMA1_Channel7->CCR &= ~(DMA_CCR_PSIZE);
  //Direction is Memory to Peripheral
  DMA1_Channel7->CCR |= (DMA_CCR_DIR);
  //Disable the channel for now
  DMA1_Channel7->CCR &= ~(DMA_CCR_EN);
  //Enable Transfer complete interrupt - DMA
  DMA1_Channel7->CCR |= (DMA_CCR_TCIE);

  ////RX Channel DMA configuration(Channel6-->USART2_RX)
  //Clear DMA1-CH6 status flags
  DMA1->IFCR |=(DMA_IFCR_CGIF6);
  //Select UART2_RX as DMA_CH6
  DMA1_CSELR->CSELR &=~(DMA_CSELR_C6S);
  DMA1_CSELR->CSELR |= (0x2UL << (20U));
  //Peripheral address (Dest is USART_RDR)
  DMA1_Channel6->CPAR =(uint32_t)(&(USART2->RDR));
  //Normal/Circular DMA operation
  DMA1_Channel6->CCR &= ~ (DMA_CCR_CIRC);
  //Enable memory increment
  DMA1_Channel6->CCR |= (DMA_CCR_MINC);
  //Disable peripheral increment
  DMA1_Channel6->CCR &= ~(DMA_CCR_PINC);
  //Memory/Peripheral Size to 8bits
  DMA1_Channel6->CCR &= ~(DMA_CCR_MSIZE);
  DMA1_Channel6->CCR &= ~(DMA_CCR_PSIZE);
  //Direction is Peripheral to memory
  DMA1_Channel6->CCR &= ~ (DMA_CCR_DIR);
  //Disable the channel for now
  DMA1_Channel6->CCR &= ~(DMA_CCR_EN);
  //Enable Transfer complete interrupt - DMA
  DMA1_Channel6->CCR |= (DMA_CCR_TCIE);

  ////Enable DMA Interrupts
  NVIC_SetPriority(DMA1_Channel7_IRQn,6);
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  NVIC_SetPriority(DMA1_Channel6_IRQn,5);
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}
/**
 * @Brief UART2 - DMA Transmit
 */
void uart_UART2_DMA_TX(uint8_t *data,uint8_t len)
{
  //Clear DMA1-CH7 status flags
  DMA1->IFCR |=(DMA_IFCR_CGIF7);
  //Memory address(Src)
  DMA1_Channel7->CMAR =(uint32_t)data;
  //No. of transfers
  DMA1_Channel7->CNDTR=len;
  //Enable DMA Channel
  DMA1_Channel7->CCR |= (DMA_CCR_EN);

}
/**
 * @Brief UART2 - DMA Receive
 */
void uart_UART2_DMA_RX(uint8_t *data,uint8_t len)
{
  //Clear overrun error, if any
 USART2->ICR |=(USART_ICR_ORECF);
 //Clear DMA1-CH6 status flags
 DMA1->IFCR |=(DMA_IFCR_CGIF6);
 //Memory address(Src)
 DMA1_Channel6->CMAR =(uint32_t)data;
 //No. of transfers
 DMA1_Channel6->CNDTR=len;
 //Enable DMA Channel
 DMA1_Channel6->CCR |= (DMA_CCR_EN);


}
