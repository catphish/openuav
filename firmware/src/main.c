#include <stm32g4xx.h>

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
  // Set TIM2 output compare registers to 1000us (1us = 16, 1000us = 16000)
  TIM2->CCR1 = 16000;
  TIM2->CCR2 = 16000;
  TIM2->CCR3 = 16000;
  TIM2->CCR4 = 16000;
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
  // Enable TIM2
  TIM2->CR1 = TIM_CR1_CEN;
}

int main(void) {
  while (1) {
    // Copy data from USART3 to USART1
    if (USART3->ISR & USART_ISR_RXNE) {
      USART1->TDR = USART3->RDR;
    }
  }
  return 0;
}

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8

void process_elrs_char(uint8_t received) {
  static uint8_t buffer[64];
  static uint8_t buffer_index = 0;
  // If the buffer is empty, wait for a sync byte or address byte
  if (buffer_index == 0) {
    if (received == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      buffer[buffer_index++] = received;
    }
  } else if (buffer_index == 1) {
    if (received > 62) {
      buffer_index = 0;
    } else {
      buffer[buffer_index++] = received;
    }
  } else if (buffer_index < buffer[1] + 1) {
    buffer[buffer_index++] = received;
  } else {
    // Process the packet
    uint8_t crc8 = received;
    buffer_index = 0;
  }
}
