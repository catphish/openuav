#include <stm32g4xx.h>
#include "gpio.h"

volatile uint8_t temp;

static void spi1_init() {
  // Enable SPI1 clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
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

static void spi2_init() {
  // Enable SPI2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
  // Set CS high
  GPIOC->BSRR = GPIO_BSRR_BS_6;
  // Set PB13 to AF5 (SPI2_SCK)
  gpio_pin_mode(GPIOB, 13, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set PB14 to AF5 (SPI2_MISO)
  gpio_pin_mode(GPIOB, 14, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set PB15 to AF5 (SPI2_MOSI)
  gpio_pin_mode(GPIOB, 15, GPIO_MODE_AF, 5, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set PC6 to output (SPI2_CS)
  gpio_pin_mode(GPIOC, 6, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

  // Disable SPI2
  SPI2->CR1 = 0;
  // Configure SPI1 CR1
  SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
  // Configure SPI1 CR2
  SPI2->CR2 =  SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
  // Enable SPI1
  SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_init() {
  // Ensure GPIOs are enabled.
  gpio_init();
  // Configure each SPI peripheral.
  spi1_init();
  spi2_init();
}

void spi_transmit(SPI_TypeDef * SPIx, uint8_t data)
{
    volatile uint8_t * dr = (uint8_t*)&(SPIx->DR);

    // Wait for TXE
    while(!(SPIx->SR & SPI_SR_TXE));
    // Write data
    *dr = data;
    // Wait for BSY to be cleared
    while(SPIx->SR & SPI_SR_BSY);
    // Wait for RXNE
    while(!(SPIx->SR & SPI_SR_RXNE));
    // Read data
    temp = *dr;
}

uint8_t spi_receive(SPI_TypeDef * SPIx)
{
    volatile uint8_t * dr = (uint8_t*)&(SPIx->DR);

    // Wait for TXE
    while(!(SPIx->SR & SPI_SR_TXE));
    // Write dummy data
    *dr = 0xFF;
    // Wait for BSY to be cleared
    while(SPIx->SR & SPI_SR_BSY);
    // Wait for RXNE
    while(!(SPIx->SR & SPI_SR_RXNE));
    // Read data
    return *dr;
}
