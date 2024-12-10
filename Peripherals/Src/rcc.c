///*
// * rcc.c
// *
// *  Created on: Oct 8, 2024
// *      Author: DANNY
// */
//
//#include <rcc.h>
//
////miliseconds tick
//static __IO uint32_t msTicks=0;
//
///*
// * Brief MSI Clock Config
// */
//void rcc_MSI_config(void)
//{
//  /**
//   * @brief  System Clock Configuration
//   *         The system Clock is configured as follows :
//   *            System Clock source            = PLL (MSI)
//   *            SYSCLK(Hz)                     = 80000000
//   *            HCLK(Hz)                       = 80000000
//   *            AHB Prescaler                  = 1
//   *            APB1 Prescaler                 = 1
//   *            APB2 Prescaler                 = 1
//   *            MSI Frequency(Hz)              = 4000000
//   *            PLL_M                          = 1
//   *            PLL_N                          = 40
//   *            PLL_R                          = 2
//   *            PLL_P                          = 7
//   *            PLL_Q                          = 4
//   *            Flash Latency(WS)              = 4
//   */
//  ////MSI Configuration
//  //Disable write protection of BDCR
//  RCC->APB1ENR1 |=(RCC_APB1ENR1_PWREN);
//  PWR->CR1 |=(PWR_CR1_DBP);
//  //Power On LSE for HW Automatic Calibration
//  RCC->BDCR |= (RCC_BDCR_LSEON);
//  //Wait until LSE ready
//  while(!(RCC->BDCR & RCC_BDCR_LSERDY ));
//  //Active MSI PLL Mode
//  //(MSI ON by default)
//  RCC->CR |= (RCC_CR_MSIPLLEN);
//
//  ////PLL Configuration
//  //Disable PLL
//  RCC->CR &= ~(RCC_CR_PLLON);
//  //Wait for PLL Ready is cleared
//  while((RCC->CR & RCC_CR_PLLRDY ));
//  //Select MSI as PLL clock source
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_MSI);
//  //PLL_M=1
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
//  //PLL_N=40
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
//  RCC->PLLCFGR |= (0x28FUL << (8U));
//  //PLL_R
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
//  //PLL_Q
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLQ_0);
//  //PLL_P
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
//  //Enable PLL
//  RCC->CR |= (RCC_CR_PLLON);
//  //Enable PLLCLK output(R)
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLREN);
//  //Wait for PLL Ready is set
//  while(!(RCC->CR & RCC_CR_PLLRDY ));
//  //Flash Latency and prefetch
//  FLASH->ACR &= ~(FLASH_ACR_LATENCY);
//  FLASH->ACR |= (0x4UL << (0U));
//  //PLL as clock source of SYSCLK
//  RCC->CFGR &= ~(RCC_CFGR_SW_Msk);
//  RCC->CFGR |= (RCC_CFGR_SW_Msk);
//  ////Wait for clock change be stable
//  while(!(RCC->CFGR & (0x3UL << (2U)) ));
//  //Clear prescalers
//  RCC->CFGR &= ~(RCC_CFGR_PPRE1);
//  RCC->CFGR &= ~(RCC_CFGR_PPRE2);
//  RCC->CFGR &= ~(RCC_CFGR_HPRE);
//}
//
//
///*
// * @Brief MSI Clock Configuration for UART1 baud rate(72MHz)
// */
//void rcc_USART2_for_BR_config(void)
//{
//  /**
//   * @brief  System Clock Configuration
//   *         The system Clock is configured as follows :
//   *            System Clock source            = PLL (MSI)
//   *            SYSCLK(Hz)                     = 72000000
//   *            HCLK(Hz)                       = 72000000
//   *            AHB Prescaler                  = 1
//   *            APB1 Prescaler                 = 1
//   *            APB2 Prescaler                 = 1
//   *            MSI Frequency(Hz)              = 4000000
//   *            PLL_M                          = 1
//   *            PLL_N                          = 36
//   *            PLL_R                          = 2
//   *            PLL_P                          = 7
//   *            PLL_Q                          = 4
//   *            Flash Latency(WS)              = 4
//   */
//  ////MSI Configuration
//  //Disable write protection of BDCR
//  RCC->APB1ENR1 |=(RCC_APB1ENR1_PWREN);
//  PWR->CR1 |=(PWR_CR1_DBP);
//  //Power On LSE for HW Automatic Calibration
//  RCC->BDCR |= (RCC_BDCR_LSEON);
//  //Wait until LSE ready
//  while(!(RCC->BDCR & RCC_BDCR_LSERDY ));
//  //Active MSI PLL Mode
//  //(MSI ON by default)
//  RCC->CR |= (RCC_CR_MSIPLLEN);
//
//
//
//  ////PLL Configuration
//  //Disable PLL
//  RCC->CR &= ~(RCC_CR_PLLON);
//  //Wait for PLL Ready is cleared
//  while((RCC->CR & RCC_CR_PLLRDY ));
//  //Select MSI as PLL clock source
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_MSI);
//  //PLL_M=1
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
//  //PLL_N=32
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
//  RCC->PLLCFGR |= (0x20FUL << (8U));
//  //PLL_R
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
//  //PLL_Q
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLQ_0);
//  //PLL_P
//  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
//  //Enable PLL
//  RCC->CR |= (RCC_CR_PLLON);
//  //Enable PLLCLK output(R)
//  RCC->PLLCFGR |= (RCC_PLLCFGR_PLLREN);
//  //Wait for PLL Ready is set
//  while(!(RCC->CR & RCC_CR_PLLRDY ));
//  //Flash Latency and prefetch
//  FLASH->ACR &= ~(FLASH_ACR_LATENCY);
//  FLASH->ACR |= (0x4UL << (0U));
//  //PLL as clock source of SYSCLK
//  RCC->CFGR &= ~(RCC_CFGR_SW_Msk);
//  RCC->CFGR |= (RCC_CFGR_SW_Msk);
//  ////Wait for clock change be stable
//  while(!(RCC->CFGR & (0x3UL << (2U)) ));
//  //Clear prescalers
//  RCC->CFGR &= ~(RCC_CFGR_PPRE1);
//  RCC->CFGR &= ~(RCC_CFGR_PPRE2);
//  RCC->CFGR &= ~(RCC_CFGR_HPRE);
//
//
//}
//
///*
// * Brief SysTick Configuration
// */
//void rcc_SysTick_config(uint32_t arr)
//{
//  //Reset control register
//  SysTick->CTRL=0;
//  //Set the reload value
//  SysTick->LOAD =arr-1;
//  //Priority SysTick interrupt (NVIC)
//  NVIC_SetPriority(SysTick_IRQn,15);
//  //Reset SysTick value to 0
//  SysTick->VAL=0;
//  //Enable SysTick from Control register
//  SysTick->CTRL &=  ~(SysTick_CTRL_CLKSOURCE_Msk);//Select Processor clock (AHB)
//  SysTick->CTRL |=  SysTick_CTRL_TICKINT_Msk;
//  SysTick->CTRL |=  (0x1UL << (0U));
//
//}
///*
// * Brief increment msTicks
// */
//void rcc_msIncTicks(void)
//{
//  msTicks++;
//}
///*
// * Brief get msTicks
// */
//uint32_t rcc_msGetTicks(void)
//{
//  return msTicks;
//}
///*
// * Brief msDelay
// */
//void rcc_msDelay(uint32_t msDelayTime)
//{
//  uint32_t startTicks= rcc_msGetTicks();
//  while((rcc_msGetTicks() - startTicks )< msDelayTime);
//
//
//}
///*
// * @Brief HSI for UART (16MHz) Clock Configuration
// */
//void rcc_HSI_UART_config(void)
//{
//  // Default clock source is the "multi-speed" internal oscillator.
//  // Switch to the 16MHz HSI oscillator.
//  RCC->CR |=  ( RCC_CR_HSION );
//  while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
//  RCC->CFGR &= ~( RCC_CFGR_SW );
//  RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
//  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
//
//}
///*
// * Brief SysTick Interrupt callback
// */
//void SysTick_Handler(void)
//{
//  NVIC_ClearPendingIRQ(SysTick_IRQn);
//  rcc_msIncTicks();
//}
//
//
//
//
