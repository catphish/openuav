#include <stm32g4xx.h>
#include <stdint.h>

void SPI1_Write(uint8_t address, uint8_t data) {
  // Set SPI1 CS low
  GPIOA->BSRR = GPIO_BSRR_BR4;
  // Send address with write bit
  while (!(SPI1->SR & SPI_SR_TXE));
  SPI1->DR = address & 0x7f;
  while (!(SPI1->SR & SPI_SR_TXE));
  // Send data
  SPI1->DR = data;
  while (!(SPI1->SR & SPI_SR_TXE));
  // Set SPI1 CS high
  GPIOA->BSRR = GPIO_BSRR_BS4;
}

uint8_t SPI1_Read(uint8_t address) {
  // Set SPI1 CS low
  GPIOA->BSRR = GPIO_BSRR_BR4;
  // Send address with read bit
  while (!(SPI1->SR & SPI_SR_TXE));
  SPI1->DR = address | 0x80;
  while (!(SPI1->SR & SPI_SR_TXE));
  // Send dummy byte
  SPI1->DR = 0;
  while (!(SPI1->SR & SPI_SR_TXE));
  // Wait for dummy response
  while (!(SPI1->SR & SPI_SR_RXNE));
  // Send dummy byte
  SPI1->DR = 0;
  while (!(SPI1->SR & SPI_SR_TXE));
  // Wait for data
  while (!(SPI1->SR & SPI_SR_RXNE));
  // Set SPI1 CS high
  GPIOA->BSRR = GPIO_BSRR_BS4;
  // Return received data
  return SPI1->DR;
}