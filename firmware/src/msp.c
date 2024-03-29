#include <stdint.h>
#include <string.h>
#include "usb.h"   // For debugging purposes only
#include "uart.h"  // MSP messages are sent via UART
#include "adc.h"   // Necessary to get battery voltage
#include "main.h"  // Exposes armed state
#include "settings.h" // Cell count and chemistry
#include "stdio.h"
#include "elrs.h"

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
  // Calculate the checksum
  for (uint8_t i = 0; i < length; i++) checksum ^= payload[i];
  // Send the checksum
  uart_tx_string(&checksum, 1);
}

void send_msp_status_response(void) {
  // Prepare a MSP_STATUS response payload
  uint8_t payload[11];
  memset(payload, 0, 11);
  // Send the arming status
  payload[6] = main_get_armed_state();
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
  // Strings can have payload length minus four characters
  uint8_t payload[20];

  // Prepare a MSP_DISPLAYPORT payload
  memset(payload, 0, 20);
  // Send the write string subcommand
  payload[0] = 3;
  // Bottom row
  payload[1] = 15;
  // Column
  // (left third on older HDZ VRX firmware)
  payload[2] = 6;
  // Attributes
  payload[3] = 0;
  // String (all letters must be uppercase, lowercase letters map to symbols)
  // Indicate flight mode followed by an asterisk if armed
  char mode_chars[7] = "ACRO ";
  if(elrs_channel(5) > 0) snprintf(mode_chars, 6, "ANGL ");
  if(1 == main_get_armed_state()) mode_chars[4] = '*';
  snprintf((char*)payload+4, 10, "%s", mode_chars);
  // Send the payload
  send_msp(182, payload, 14);
  
  // Set cell count, but only once
  volatile struct settings *settings = settings_get();
  // (Anything else would result in meaningless output)
  if (settings->cell_count > 0) {
    // Prepare a MSP_DISPLAYPORT payload
    memset(payload, 0, 20);
    // Send the write string subcommand
    payload[0] = 3;
    // Bottom row
    payload[1] = 15;
    // Column
    // (right third on older HDZ VRX firmware)
    payload[2] = 20;
    // Attributes
    payload[3] = 0;
    // String
    uint16_t vbatt = adc_read_mv() / settings->cell_count;
    uint8_t v = vbatt / 1000;
    uint16_t mv = (vbatt % 1000) / 10; // For 2 decimal places
    snprintf((char*)payload+4, 16, "%iS %d.%02dV", (uint8_t)settings->cell_count, v, mv);
    // Send the payload
    send_msp(182, payload, 20);

    if(vbatt < 3500) {
      // Prepare another MSP_DISPLAYPORT payload
      memset(payload, 0, 20);
      // Send the write string subcommand
      payload[0] = 3;
      // Middle row
      payload[1] = 7;
      // Column
      // (mid-point on older HDZ VRX firmware)
      payload[2] = 13;
      // Attributes
      payload[3] = 0;
      // Another string
      snprintf((char*)payload+4, 10, "BATTERY!");
      // Send the payload again
      send_msp(182, payload, 14);
    }
  }
  // Prepare a MSP_DISPLAYPORT payload
  memset(payload, 0, 20);
  // Send the write string subcommand
  payload[0] = 3;
  // Near bottom
  payload[1] = 14;
  // Column
  // (right third on older HDZ VRX firmware)
  payload[2] = 20;
  // Attributes
  payload[3] = 0;
  // String
  uint16_t current = adc_read_ma();
  uint16_t a = current / 1000;
  uint16_t ma = (current % 1000) / 10; // For 2 decimal places
  snprintf((char*)payload+4, 16, "%d.%02dA", a, ma);
  // Send the payload
  send_msp(182, payload, 20);
}

void send_msp_displayport_draw() {
  // Prepare a MSP_DISPLAYPORT payload
  uint8_t payload[1];
  // Send the draw subcommand
  payload[0] = 4;
  // Send the payload
  send_msp(182, payload, 1);
}

volatile int response_due = 0;
void msp_send_response(void) {
  if(response_due) {
    response_due = 0;
    send_msp_status_response();
    send_msp_displayport_clear();
    send_msp_displayport_write();
    send_msp_displayport_draw();
  }
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
      response_due = 1;
    }
  }
}
