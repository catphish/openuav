#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "elrs.h"

void uart_init() {
    // Enable UART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    // Enable GPIOB clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    // Configure PB6 as USART1_TX
    gpio_pin_mode(GPIOB, 6, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, 0);
    // Configure PB7 as USART1_RX
    gpio_pin_mode(GPIOB, 7, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, 0);
    // Configure USART1 for 420000 baud for ELRS (160MHz / 420000 = 380.952)
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = 381;
    // Enable USART1 receiver
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE;
    // Enable USART1 interrupt
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    // Enable USART1 interrupt in NVIC
    NVIC_EnableIRQ(USART1_IRQn);
}

// UART1 interrupt handler
void USART1_IRQHandler(void) {
  // If the interrupt was triggered by a received byte
  if (USART1->ISR & USART_ISR_RXNE) {
    // Read the received byte
    volatile uint8_t received = USART1->RDR;
    // Process the received byte
    elrs_process_char(received);
  }
  // Clear the ORE interrupt flag
  USART1->ICR = USART_ICR_ORECF;
}