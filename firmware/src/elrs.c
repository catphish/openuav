#include <stm32g4xx.h>
#include "elrs.h"

int channel[8];
int channels_valid = 0;
uint8_t crc;

// Append a byte to a CRC-8
void crc8_append(uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc <<= 1;
    }
  }
}

void elrs_process_char(uint8_t received) {
  static uint8_t buffer[64];
  static uint8_t buffer_index = 0;
  // If the buffer is empty, wait for an address byte
  if (buffer_index == 0) {
    crc = 0;
    if (received == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
      buffer[buffer_index++] = received;
    }
  } else if (buffer_index == 1) {
    if (received > 62) {
      buffer_index = 0;
    } else {
      buffer[buffer_index++] = received;
    }
  } else if (buffer_index < buffer[1] + 1) {
    crc8_append(received);
    buffer[buffer_index++] = received;
  } else {
    crc8_append(received);
    if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && crc == 0) {
      struct crsf_channels_s *channels = (struct crsf_channels_s *)&buffer[3];
      channel[0] = (int)channels->ch0 - 992;
      channel[1] = (int)channels->ch1 - 992;
      channel[2] = (int)channels->ch2 - 992;
      channel[3] = (int)channels->ch3 - 992;
      channel[4] = (int)channels->ch4 - 992;
      channel[5] = (int)channels->ch5 - 992;
      channel[6] = (int)channels->ch6 - 992;
      channel[7] = (int)channels->ch7 - 992;
      channels_valid = 1000;
    }
    // uint8_t crc8 = received;
    buffer_index = 0;
  }
}

void elrs_tick() {
  if (channels_valid > 0) {
    channels_valid--;
  }
}

uint8_t elrs_valid() {
  return channels_valid > 0;
}

int elrs_channel(int channel_number) {
  return channel[channel_number];
}