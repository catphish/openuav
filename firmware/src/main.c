#include <stm32g4xx.h>
#include <stdint.h>
#include "spi.h"

#define CTRL2_G 0x11
#define OUTX_L_G 0x22
#define STATUS_REG 0x1E

extern int32_t channel[5];
extern int32_t channels_valid;

uint8_t I2C2_Read(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes);
uint8_t I2C2_Write(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes);

int32_t limit_motor_speed(int32_t m) {
  if (m < 16000) m = 16000;
  if (m > 32000) m = 32000;
  return m;
}

int main(void) {
  // Enable gyro
  //SPI1_Write(0x7d, 2);
  // Configure gyro to ODR=800Hz, OSR4, performance optimized
//  SPI1_Write(0x42, 0x0b | (1<<6) | (1<<7));
  // Wait a while

    // for (int i = 0; i < 500000; i++) __asm__("nop");
    // GPIOA->BSRR = GPIO_BSRR_BR10;
    // for (int i = 0; i < 500000; i++) __asm__("nop");
    // GPIOA->BSRR = GPIO_BSRR_BS10;
  while(1) {
    for (int i = 0; i < 500000; i++) __asm__("nop");

    uint8_t data = SPI1_Read(0x0);
    //if (data == 0x24)
    //  GPIOA->BSRR = GPIO_BSRR_BR10;
  }
  // uint8_t rx_data[6];
  // while (1) {
  //   // Set gyro to 416Hz. Do this on every cycle for now because
  //   // it doesn't seem to always work if done once at startup.
  //   I2C2_Write(0b1101010, CTRL2_G, (uint8_t[]){0x60}, 1);
  //   // Fetch status register
  //   I2C2_Read(0b1101010, STATUS_REG, rx_data, 1);
  //   // If gyro data is ready
  //   if (rx_data[0] & 0x02) {
  //     int32_t throttle = (channel[2] + 820) * 10;
  //     // Read all gyro data
  //     I2C2_Read(0b1101010, OUTX_L_G, rx_data, 6);
  //     int16_t gyro_x = rx_data[1] << 8 | rx_data[0];
  //     int16_t gyro_y = rx_data[3] << 8 | rx_data[2];
  //     int16_t gyro_z = rx_data[5] << 8 | rx_data[4];
  //     // Decrement ELRS timeout
  //     if (channels_valid) channels_valid--;
  //     // If ELRS is active and armed
  //     if (channels_valid && channel[4] > 0) {
  //       // Calculate motor speeds
  //       int32_t m;
  //       m = 16000 + throttle - gyro_x + gyro_y - gyro_z - channel[0] * 10 + channel[1] * 10 - channel[3] * 10;
  //       TIM2->CCR1 = limit_motor_speed(m);
  //       m = 16000 + throttle - gyro_x - gyro_y + gyro_z - channel[0] * 10 - channel[1] * 10 + channel[3] * 10;
  //       TIM2->CCR2 = limit_motor_speed(m);
  //       m = 16000 + throttle + gyro_x - gyro_y - gyro_z + channel[0] * 10 - channel[1] * 10 - channel[3] * 10;
  //       TIM2->CCR3 = limit_motor_speed(m);
  //       m = 16000 + throttle + gyro_x + gyro_y + gyro_z + channel[0] * 10 + channel[1] * 10 + channel[3] * 10;
  //       TIM2->CCR4 = limit_motor_speed(m);
  //     } else {
  //       // Stop all motors
  //       TIM2->CCR1 = 16000;
  //       TIM2->CCR2 = 16000;
  //       TIM2->CCR3 = 16000;
  //       TIM2->CCR4 = 16000;
  //     }
  //     // Enable TIM2
  //     TIM2->CNT = 0;
  //     TIM2->CR1 = TIM_CR1_OPM;
  //     TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
  //   }
  // }
    while(1);
    //  {
    // if (channel[4] > 0)
    //   GPIOA->BSRR = GPIO_BSRR_BR10;
    // else
    //   GPIOA->BSRR = GPIO_BSRR_BS10;
    // }
  return 0;
}
