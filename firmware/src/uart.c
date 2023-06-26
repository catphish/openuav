#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"

void uart_init() {
    // Configure PA6 as AF7 (USART1_TX)
    gpio_pin_mode(GPIOA, 9, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Configure PA10 as AF7 (USART1_RX)
    gpio_pin_mode(GPIOA, 10, GPIO_MODE_AF, 7, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Configure USART1
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = 0x1A0;
}
