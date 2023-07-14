#include <stdint.h>

void spi_init();
uint8_t spi_read_register(uint8_t reg);
void spi_write_register(uint8_t reg, uint8_t value);