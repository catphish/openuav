#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "elrs.h"
#include "usb.h"
#include "msp.h"

volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;
volatile uint8_t tx_buffer[256];

// Buffer a string to transmit over UART3.
void uart_tx_string(uint8_t *string, uint32_t length) {
  // Copy each byte into the transmit buffer. If the buffer is full, the byte
  // will be dropped.
  for (uint32_t i = 0; i < length; i++) {
    uint8_t next_head = tx_head + 1;
    if (next_head != tx_tail) {
      tx_buffer[tx_head] = string[i];
      tx_head = next_head;
    }
  }
}

// Transmit bytes over UART3 when bytes are buffered and the transmitter is ready.
void uart_tx(void) {
  // If the transmitter is ready and there are bytes to transmit
  if ((USART3->ISR & USART_ISR_TXE) && (tx_head != tx_tail)) {
    // Transmit the next byte
    uint8_t b = tx_buffer[tx_tail];
    tx_tail++;
    USART3->TDR = b;
  }
}

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
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = 381;
    // Enable USART1 receiver
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE;
    // Enable USART1 interrupt
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    // Enable USART1 interrupt in NVIC
    NVIC_EnableIRQ(USART1_IRQn);

    // Enable USART3 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
    // Enable GPIOC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    // Configure PC10 as USART3_TX
    gpio_pin_mode(GPIOC, 10, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, 0);
    // Configure PC11 as USART3_RX
    gpio_pin_mode(GPIOC, 11, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, 0);
    // Configure USART3 for 115200 baud for MSP (160MHz / 115200 = 1388.888)
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = 1389;
    // Enable USART3 receiver and transmitter
    USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
    // Enable USART3 interrupt
    USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
    // Enable USART3 interrupt in NVIC
    NVIC_EnableIRQ(USART3_IRQn);
}

// UART1 interrupt handler
void USART1_IRQHandler(void) {
  // If the interrupt was triggered by a received byte
  if (USART1->ISR & USART_ISR_RXNE) {
    // Read the received byte
    uint8_t received = USART1->RDR;
    // Process the received byte
    elrs_process_char(received);
  }
  // Clear the ORE interrupt flag
  USART1->ICR = USART_ICR_ORECF;
}

// UART3 interrupt handler
void USART3_IRQHandler(void) {
  // If the interrupt was triggered by a received byte
  if (USART3->ISR & USART_ISR_RXNE) {
    // Read the received byte
    uint8_t received = USART3->RDR;
    // Dump the byte to USB as hex
    msp_process_char(received);
  }
  // Clear the ORE interrupt flag
  USART3->ICR = USART_ICR_ORECF;
}
