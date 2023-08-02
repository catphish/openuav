#pragma once

void uart_init();
void uart_tx_string(uint8_t *string, uint32_t length);
void uart_tx(void);
