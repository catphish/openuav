#include <stm32g4xx.h>

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

}