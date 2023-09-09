#pragma once
#include <stdint.h>

// Reset the flash chip
void flash_init(void);
void write_enable(void);
void flash_erase_block(uint16_t block);
void flash_program_load(uint8_t * buffer, uint32_t offset, uint32_t length);
void flash_program_execute(uint32_t page_address);
void flash_page_read(uint16_t page_address);
void flash_read(uint8_t * data, uint16_t length, uint16_t offset);
