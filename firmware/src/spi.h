#include <stdint.h>

void spi_init();
void spi_transmit(SPI_TypeDef * SPIx, uint8_t data);
uint8_t spi_receive(SPI_TypeDef * SPIx);
