#include <stm32g4xx.h>
#include "gpio.h"

volatile uint8_t temp;

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

    // Set CS high
    GPIOC->BSRR = GPIO_BSRR_BS_4;
    // Set PA5 to AF5 (SPI1_SCK)
    gpio_pin_mode(GPIOA, 5, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PA6 to AF5 (SPI1_MISO)
    gpio_pin_mode(GPIOA, 6, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PA7 to AF5 (SPI1_MOSI)
    gpio_pin_mode(GPIOA, 7, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Set PC4 to output (SPI1_CS)
    gpio_pin_mode(GPIOC, 4, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

    // Disable SPI1
    SPI1->CR1 = 0;
    // Configure SPI1 CR1
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
    // Configure SPI1 CR2
    SPI1->CR2 =  SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_transmit(uint8_t data)
{
    volatile uint8_t * dr = (uint8_t*)&(SPI1->DR);

    // Wait for TXE
    while(!(SPI1->SR & SPI_SR_TXE));
    // Write data
    *dr = data;
    // Wait for BSY to be cleared
    while(SPI1->SR & SPI_SR_BSY);
    // Wait for RXNE
    while(!(SPI1->SR & SPI_SR_RXNE));
    // Read data
    temp = *dr;
}

uint8_t spi_receive()
{
    volatile uint8_t * dr = (uint8_t*)&(SPI1->DR);

    // Wait for TXE
    while(!(SPI1->SR & SPI_SR_TXE));
    // Write dummy data
    *dr = 0xFF;
    // Wait for BSY to be cleared
    while(SPI1->SR & SPI_SR_BSY);
    // Wait for RXNE
    while(!(SPI1->SR & SPI_SR_RXNE));
    // Read data
    return *dr;
}

void spi_write_register(uint8_t reg, uint8_t value) {
    // Set CS low
    GPIOC->BSRR = GPIO_BSRR_BR_4;
    spi_transmit(reg);
    spi_transmit(value);
    // Set CS high
    GPIOC->BSRR = GPIO_BSRR_BS_4;
}

uint8_t spi_read_register(uint8_t reg) {
    // Set CS low
    GPIOC->BSRR = GPIO_BSRR_BR_4;
    spi_transmit(reg | 0x80);
    uint8_t value = spi_receive();
    // Set CS high
    GPIOC->BSRR = GPIO_BSRR_BS_4;
    return value;
}
