#include <stdint.h>
#include <string.h>
#include "usb.h"
#include "uart.h"

void send_msp(uint8_t command, uint8_t *payload, uint8_t length) {
  uint8_t checksum = 0;
  // Send the address
  uart_tx_string((uint8_t*)"$M>", 3);
  // Send the payload length
  uart_tx_string(&length, 1);
  checksum ^= length;
  // Send the command
  uart_tx_string(&command, 1);
  checksum ^= command;
  // Send the payload
  uart_tx_string(payload, length);
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= payload[i];
  }
  // Send the checksum
  uart_tx_string(&checksum, 1);
}

void send_msp_status_response(void) {
  // Prepare a MSP_STATUS response payload
  uint8_t payload[11];
  memset(payload, 0, 11);
  // Send the arming status (currently hardcoded to 1=armed)
  payload[6] = 1;
  // Send the response
  send_msp(101, payload, 11);
}

void send_msp_displayport_clear() {
  // Prepare a MSP_DISPLAYPORT payload
  uint8_t payload[1];
  // Send the clear subcommand
  payload[0] = 2;
  // Send the payload
  send_msp(182, payload, 1);
}

void send_msp_displayport_write() {
  // Prepare a MSP_DISPLAYPORT payload
  uint8_t payload[10];
  // Send the clear subcommand
  payload[0] = 3;
  // Row
  payload[1] = 2;
  // Column
  payload[2] = 2;
  // Attributes
  payload[3] = 0;
  // String
  payload[4] = 'H';
  payload[5] = 'e';
  payload[6] = 'l';
  payload[7] = 'l';
  payload[8] = 'o';
  payload[9] = 0;
  // Send the payload
  send_msp(182, payload, 10);
}

void send_msp_displayport_draw() {
  // Prepare a MSP_DISPLAYPORT payload
  uint8_t payload[1];
  // Send the draw subcommand
  payload[0] = 4;
  // Send the payload
  send_msp(182, payload, 1);
}

void msp_process_char(uint8_t received) {
  static uint8_t rx_index = 0;
  static uint8_t length = 0;
  static uint8_t command = 0;
  // If the buffer is empty, wait for an address byte
  if (rx_index == 0) {
    if (received == '$') {
      rx_index++;
    }
  } else if (rx_index == 1) {
    if (received == 'M') {
      rx_index++;
    } else {
      rx_index = 0;
    }
  } else if (rx_index == 2) {
    if (received == '<') {
      rx_index++;
    } else {
      rx_index = 0;
    }
  } else if (rx_index == 3) {
    // Read the payload length
    length = received;
    rx_index++;
  } else if (rx_index == 4) {
    // Read the command
    command = received;
    rx_index++;
  } else if (rx_index < length + 5) {
    // Read the payload
    rx_index++;
  } else {
    // Read the checksum, we don't check it yet
    rx_index = 0;
    // Process the command
    if (command == 101) {
      //usb_printf("Received MSP status request\n");
      send_msp_status_response();
      send_msp_displayport_clear();
      send_msp_displayport_write();
      send_msp_displayport_draw();
    }
  }
}
