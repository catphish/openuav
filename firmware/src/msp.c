#include <stdint.h>
#include <string.h>
#include "usb.h"   // For debugging purposes only
#include "uart.h"  // MSP messages are sent via UART
#include "adc.h"   // Necessary to get battery voltage
#include "main.h"  // Exposes armed state
#include "settings.h" // Cell count and chemistry
#include "stdio.h"

static int8_t  cell_count = -1; // As-yet unset
static uint8_t chemistry  =  0; // LiPo

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

// Parts of this are borrowed from the QuickSilver firmware's source code
uint8_t guess_battery_cell_count(uint8_t chemistry) {
  // A best-effort guess is best done from the midpoint of the
  // cell's voltage range, which depends on battery chemistry
  float v_cell_min;
  float v_cell_max;
  switch(chemistry) {
    case 1:  v_cell_min = 3.2f; v_cell_max = 4.35f; break; // lihv
    case 2:  v_cell_min = 2.5f; v_cell_max = 4.20f; break; // good-quality li-ion
    default: v_cell_min = 3.2f; v_cell_max = 4.20f;        // lipo
  }
  float v_cell_mid = (v_cell_min+((v_cell_max-v_cell_min)/2));

  // Using average cell voltage and voltage range midpoint,
  // we're now able to make our guess with some confidence
  uint32_t v_batt_curr = adc_read_mV();
  for(int i=6; i>0; i--) {
    if((v_batt_curr / i) > (uint32_t)(v_cell_mid*1000)) {
      return i;
    }
  }

  return 0; // no clue...
}

void send_msp_displayport_write() {
  // Prepare a MSP_DISPLAYPORT payload
  uint8_t payload[20];
  memset(payload, 0, 20);
  // Send the write string subcommand
  payload[0] = 3;
   // Row
  payload[1] = 15;
   // Column
  payload[2] = 20;
   // Attributes
  payload[3] = 0;

  // Set cell count, but only once
  if (cell_count == -1) {
    struct settings *settings = settings_get(); // convenience
    cell_count = settings->cell_count;
    if (cell_count == 0) {
      chemistry = settings->chemistry;
      cell_count = guess_battery_cell_count(chemistry);
    }
  }

  // Anything else would result in meaningless output
  if (cell_count > 0) {
    // String
    int vbatt = adc_read_mV() / cell_count;
    uint8_t v = vbatt / 1000;
    uint16_t mv = (vbatt % 1000) / 10; // For 2 decimal places
    snprintf((char*)payload+4, 16, "%iS %d.%02dV", (uint8_t)cell_count, v, mv);
    // Send the payload
    send_msp(182, payload, 20);
  }

  // Prepare a MSP_DISPLAYPORT payload
  memset(payload, 0, 12);
  // Send the write string subcommand
  payload[0] = 3;
  // Row
  payload[1] = 15;
  // Column
  payload[2] = 10;
   // Attributes
  payload[3] = 0;
  // Either "armed" or "disarmed"
  if(1 == main_get_armed_state()) snprintf((char*)payload+4, 6, "armed");
  else snprintf((char*)payload+4, 9, "disarmed");
  // Send the payload
  send_msp(182, payload, 8);
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
      //usb_printf("Received MSP status request\n");
      response_due = 1;
    }
  }
}
