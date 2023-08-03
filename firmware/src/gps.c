#include <stdint.h>
#include "usb.h"
#include "led.h"

int32_t lat;
int32_t lon;
int32_t prev_lat;
int32_t prev_lon;
float filtered_lat_d;
float filtered_lon_d;

void gps_process_char(uint8_t received) {
  static uint8_t index = 0;
  static uint8_t class;
  static uint8_t id;
  static uint16_t length;
  static uint8_t data[92];

  // If the buffer is empty, wait for preamble
  if (index == 0) {
    if (received == 0xB5) index++;
  } else if (index == 1) {
    if(received == 0x62) index++;
    else index = 0;
  } else if (index == 2) {
    class = received;
    index++;
  } else if (index == 3) {
    id = received;
    index++;
  } else if (index == 4) {
    length = received;
    index++;
  } else if (index == 5) {
    length |= received << 8;
    index++;
  } else if (index < length + 6) {
    if(class == 1 && id == 7 && length == 92)
      data[index - 6] = received;
    index++;
  } else {
    index = 0;
    if((data[20] == 2 || data[20] == 3) && data[23] > 6) {
      prev_lat = lat;
      prev_lon = lon;
      lon = (int32_t)data[24] | (int32_t)data[25] << 8 | (int32_t)data[26] << 16 | (int32_t)data[27] << 24;
      lat = (int32_t)data[28] | (int32_t)data[29] << 8 | (int32_t)data[30] << 16 | (int32_t)data[31] << 24;
      if(prev_lat == 0 || prev_lon == 0) {
        prev_lat = lat;
        prev_lon = lon;
      }
      led0_on();
    } else {
      lon = 0;
      lat = 0;
      prev_lat = 0;
      prev_lon = 0;
      led0_off();
    }
  }
}

void gps_filter() {
  if(lat == 0 || lon == 0) {
    return;
  }

  filtered_lat_d = filtered_lat_d * 0.99 + (float)(lat - prev_lat) * 0.01f;
  filtered_lon_d = filtered_lon_d * 0.99 + (float)(lon - prev_lon) * 0.01f;
}

int32_t gps_lat() {
  return lat;
}
int32_t gps_lon() {
  return lon;
}

float gps_lat_d() {
  return filtered_lat_d;
}
float gps_lon_d() {
  return filtered_lon_d;
}