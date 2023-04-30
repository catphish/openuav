#include <stm32g4xx.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

struct crsf_channels_s {
  unsigned ch0 : 11;
  unsigned ch1 : 11;
  unsigned ch2 : 11;
  unsigned ch3 : 11;
  unsigned ch4 : 11;
  unsigned ch5 : 11;
  unsigned ch6 : 11;
  unsigned ch7 : 11;
  unsigned ch8 : 11;
  unsigned ch9 : 11;
  unsigned ch10 : 11;
  unsigned ch11 : 11;
  unsigned ch12 : 11;
  unsigned ch13 : 11;
  unsigned ch14 : 11;
  unsigned ch15 : 11;
} __attribute__((packed));

int channel[5];
int channels_valid = 0;

void process_elrs_char(uint8_t received) {
  static uint8_t buffer[64];
  static uint8_t buffer_index = 0;
  // If the buffer is empty, wait for an address byte
  if (buffer_index == 0) {
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
    buffer[buffer_index++] = received;
  } else {
    if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
      struct crsf_channels_s *channels = (struct crsf_channels_s *)&buffer[3];
      channel[0] = (int)channels->ch0 - 992;
      channel[1] = (int)channels->ch1 - 992;
      channel[2] = (int)channels->ch2 - 992;
      channel[3] = (int)channels->ch3 - 992;
      channel[4] = (int)channels->ch4 - 992;
      channels_valid = 100;
    }
    // uint8_t crc8 = received;
    buffer_index = 0;
  }
}

// UART3 interrupt handler
void USART3_IRQHandler(void) {
  // If the interrupt was triggered by a received byte
  if (USART3->ISR & USART_ISR_RXNE) {
    // Read the received byte
    volatile uint8_t received = USART3->RDR;
    // Process the received byte
    process_elrs_char(received);
  }
  // Clear the ORE interrupt flag
  USART3->ICR = USART_ICR_ORECF;
}