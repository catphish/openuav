#include <stm32g4xx.h>

// The SystemInit function is called at startup to configure all the STM32 peripherals
void SystemInit(void) {
  /* Set floating point coprosessor access mode. */
  //  SCB->CPACR |= ((3UL << 10 * 2) | /* set CP10 Full Access */
  //                 (3UL << 11 * 2)); /* set CP11 Full Access */

  // Enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // Enable USART2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  // Enable USART3 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  // Enable TIM2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  // Enable I2C2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Enable GPIOC clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

  // Set PB6 and PB7 to alternate function mode
  GPIOB->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE6_1;
  GPIOB->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE7_1;
  // Set PB6 and PB7 to AF7 (USART1)
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL6_2;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_1 | GPIO_AFRL_AFSEL7_2;

  // Set PB3 and PB4 to alternate function mode
  GPIOB->MODER &= ~GPIO_MODER_MODE3_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE3_1;
  GPIOB->MODER &= ~GPIO_MODER_MODE4_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE4_1;
  // Set PB3 and PB4 to AF7 (USART2)
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_1 | GPIO_AFRL_AFSEL4_2;

  // Set PC10 and PC11 to alternate function mode
  GPIOC->MODER &= ~GPIO_MODER_MODE10_Msk;
  GPIOC->MODER |= GPIO_MODER_MODE10_1;
  GPIOC->MODER &= ~GPIO_MODER_MODE11_Msk;
  GPIOC->MODER |= GPIO_MODER_MODE11_1;
  // Set PC10 and PC11 to AF7 (USART3)
  GPIOC->AFR[1] |= GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL10_1 | GPIO_AFRH_AFSEL10_2;
  GPIOC->AFR[1] |= GPIO_AFRH_AFSEL11_0 | GPIO_AFRH_AFSEL11_1 | GPIO_AFRH_AFSEL11_2;

  // Set PA8 and PA9 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE8_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE9_1;
  // Set PA8 and PA9 to AF4 (I2C2)
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_2;
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9_2;
  // Set PA08 and PA09 to open drain
  GPIOA->OTYPER |= GPIO_OTYPER_OT8;
  GPIOA->OTYPER |= GPIO_OTYPER_OT9;
  // Set PA08 and PA09 to very high speed
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED8_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_0 | GPIO_OSPEEDR_OSPEED9_1;

  // Disable USART1
  USART1->CR1 = USART_CR1_UE;
  // Set USART1 baud rate to 9600 (16MHz / 9600 = 1667)
  USART1->BRR = 1667;
  // Enable USART1
  USART1->CR1 = USART_CR1_UE;
  // Enable USART1 transmitter
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

  // Disable USART3
  USART3->CR1 = USART_CR1_UE;
  // Set USART3 baud rate to 420000 (16MHz / 420000 = 38)
  USART3->BRR = 38;
  // Enable USART3
  USART3->CR1 = USART_CR1_UE;
  // Enable USART3 receiver
  USART3->CR1 = USART_CR1_UE | USART_CR1_RE;
  // Enable USART3 interrupt
  USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
  // Enable USART3 interrupt in NVIC
  NVIC_EnableIRQ(USART3_IRQn);

  // Set A0, A1, A2, A3 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE0_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE1_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE2_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE3_1;
  // Set A0, A1, A2, A3 to AF1 (TIM2)
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL0_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL1_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;

  // Set TIM2 period to 2500us (1us = 16, 2500us = 40000)
  TIM2->ARR = 40000;
  // Set TIM2 output compare mode to PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  // Enable TIM2 output compare 1
  TIM2->CCER |= TIM_CCER_CC1E;
  // Enable TIM2 output compare 2
  TIM2->CCER |= TIM_CCER_CC2E;
  // Enable TIM2 output compare 3
  TIM2->CCER |= TIM_CCER_CC3E;
  // Enable TIM2 output compare 4
  TIM2->CCER |= TIM_CCER_CC4E;
  //  Enable TIM2
  TIM2->CR1 = TIM_CR1_OPM;

  // Make sure I2C2 is disabled
  I2C2->CR1 &= ~I2C_CR1_PE;
  // Reset I2C2 Configuration to default values
  I2C2->CR1 = 0x00000000;
  // Set I2C timing to 400kHz
  I2C2->TIMINGR = (1 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (2 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos);
  // Enable I2C2
  I2C2->CR1 |= I2C_CR1_PE;

  // Enable interrupts
  __enable_irq();
}
