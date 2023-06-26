#include <stm32g4xx.h>
#include "gpio.h"

void spi_init() {
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Enable GPIOB clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // Enable SPI2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
    // Enable SPI3 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;

    // Set PA5 to AF5 (SPI1_SCK)
    gpio_pin_mode(GPIOA, 5, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PA6 to AF5 (SPI1_MISO)
    gpio_pin_mode(GPIOA, 6, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PA7 to AF5 (SPI1_MOSI)
    gpio_pin_mode(GPIOA, 7, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PC4 to output (SPI1_CS)
    gpio_pin_mode(GPIOC, 4, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

    // Configure SPI1 CR1
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;
    // Configure SPI1 CR2
    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}
