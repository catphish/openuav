#include <stm32g4xx.h>

void SystemInit(void) {
  /* Set floating point coprosessor access mode. */
  //  SCB->CPACR |= ((3UL << 10 * 2) | /* set CP10 Full Access */
  //                 (3UL << 11 * 2)); /* set CP11 Full Access */

  // Enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // Enable USART2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  // Enable USART3 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  // Enable TIM2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  // Enable I2C2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Enable GPIOC clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

  // Set PB6 and PB7 to alternate function mode
  GPIOB->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE6_1;
  GPIOB->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE7_1;
  // Set PB6 and PB7 to AF7 (USART1)
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL6_2;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_1 | GPIO_AFRL_AFSEL7_2;

  // Set PB3 and PB4 to alternate function mode
  GPIOB->MODER &= ~GPIO_MODER_MODE3_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE3_1;
  GPIOB->MODER &= ~GPIO_MODER_MODE4_Msk;
  GPIOB->MODER |= GPIO_MODER_MODE4_1;
  // Set PB3 and PB4 to AF7 (USART2)
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_1 | GPIO_AFRL_AFSEL4_2;

  // Set PC10 and PC11 to alternate function mode
  GPIOC->MODER &= ~GPIO_MODER_MODE10_Msk;
  GPIOC->MODER |= GPIO_MODER_MODE10_1;
  GPIOC->MODER &= ~GPIO_MODER_MODE11_Msk;
  GPIOC->MODER |= GPIO_MODER_MODE11_1;
  // Set PC10 and PC11 to AF7 (USART3)
  GPIOC->AFR[1] |= GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL10_1 | GPIO_AFRH_AFSEL10_2;
  GPIOC->AFR[1] |= GPIO_AFRH_AFSEL11_0 | GPIO_AFRH_AFSEL11_1 | GPIO_AFRH_AFSEL11_2;

  // Set PA8 and PA9 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE8_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE9_1;
  // Set PA8 and PA9 to AF4 (I2C2)
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_2;
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9_2;
  // Set PA08 and PA09 to open drain
  GPIOA->OTYPER |= GPIO_OTYPER_OT8;
  GPIOA->OTYPER |= GPIO_OTYPER_OT9;
  // Set PA08 and PA09 to very high speed
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED8_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_0 | GPIO_OSPEEDR_OSPEED9_1;

  // Disable USART1
  USART1->CR1 = USART_CR1_UE;
  // Set USART1 baud rate to 9600 (16MHz / 9600 = 1667)
  USART1->BRR = 1667;
  // Enable USART1
  USART1->CR1 = USART_CR1_UE;
  // Enable USART1 transmitter
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

  // Disable USART3
  USART3->CR1 = USART_CR1_UE;
  // Set USART3 baud rate to 420000 (16MHz / 420000 = 38)
  USART3->BRR = 38;
  // Enable USART3
  USART3->CR1 = USART_CR1_UE;
  // Enable USART3 receiver
  USART3->CR1 = USART_CR1_UE | USART_CR1_RE;
  // Enable USART3 interrupt
  USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
  // Enable USART3 interrupt in NVIC
  NVIC_EnableIRQ(USART3_IRQn);

  // Set A0, A1, A2, A3 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE0_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE1_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE2_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE3_1;
  // Set A0, A1, A2, A3 to AF1 (TIM2)
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL0_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL1_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;

  // Set TIM2 period to 2500us (1us = 16, 2500us = 40000)
  TIM2->ARR = 40000;
  // Set TIM2 output compare mode to PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  // Enable TIM2 output compare 1
  TIM2->CCER |= TIM_CCER_CC1E;
  // Enable TIM2 output compare 2
  TIM2->CCER |= TIM_CCER_CC2E;
  // Enable TIM2 output compare 3
  TIM2->CCER |= TIM_CCER_CC3E;
  // Enable TIM2 output compare 4
  TIM2->CCER |= TIM_CCER_CC4E;
  //  Enable TIM2
  // TIM2->CR1 = TIM_CR1_OPM;

  // Make sure I2C2 is disabled
  I2C2->CR1 &= ~I2C_CR1_PE;
  // Reset I2C2 Configuration to default values
  I2C2->CR1 = 0x00000000;
  // Set I2C timing to 400kHz
  I2C2->TIMINGR = (1 << I2C_TIMINGR_PRESC_Pos) | (9 << I2C_TIMINGR_SCLL_Pos) | (3 << I2C_TIMINGR_SCLH_Pos) | (2 << I2C_TIMINGR_SDADEL_Pos) | (3 << I2C_TIMINGR_SCLDEL_Pos);
  // Enable I2C2
  I2C2->CR1 |= I2C_CR1_PE;

  // Enable interrupts
  __enable_irq();
}

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

int channel[4];
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

uint8_t I2C2_Read(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes) {
  uint32_t timeout;  // Flag waiting timeout
  uint8_t n;         // Loop counter

  // Set device address
  I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
  I2C2->CR2 |= ((device_address << 1U) << I2C_CR2_SADD_Pos);

  // Set I2C in Write mode
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;

  // Transfer NBYTES = 1, no AUTOEND
  I2C2->CR2 &= ~I2C_CR2_NBYTES;
  I2C2->CR2 |= (1 << 16U);
  I2C2->CR2 &= ~I2C_CR2_AUTOEND;

  // Start I2C transaction
  I2C2->CR2 |= I2C_CR2_START;

  // Wait for TXIS with timeout
  timeout = 100000;
  while (((I2C2->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS) {
    timeout--;
    if (timeout == 0) return 1;
  }

  // Send Register address
  I2C2->TXDR = register_address;

  // Wait for TC with timeout
  timeout = 100000;
  while (((I2C2->ISR) & I2C_ISR_TC) != I2C_ISR_TC) {
    timeout--;
    if (timeout == 0) return 2;
  }

  // Set I2C in Read mode
  I2C2->CR2 |= I2C_CR2_RD_WRN;

  // Transfer NBYTES, no AUTOEND
  I2C2->CR2 &= ~I2C_CR2_NBYTES;
  I2C2->CR2 |= (nbytes << 16U);
  I2C2->CR2 &= ~I2C_CR2_AUTOEND;

  // Re-Start transaction
  I2C2->CR2 |= I2C_CR2_START;

  n = nbytes;

  while (n > 0) {
    // Wait for RXNE with timeout
    timeout = 1000;
    while (((I2C2->ISR) & I2C_ISR_RXNE) != I2C_ISR_RXNE) {
      timeout--;
      if (timeout == 0) return 3;
    }

    // Store data into buffer
    *buffer = I2C2->RXDR;
    buffer++;
    n--;
  }

  // Generate STOP condition
  I2C2->CR2 |= I2C_CR2_STOP;

  // Wait for STOPF with timeout
  timeout = 1000;
  while (((I2C2->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF) {
    timeout--;
    if (timeout == 0) return 4;
  }

  // Return success
  return 0;
}

uint8_t I2C2_Write(uint8_t device_address, uint8_t register_address, uint8_t *buffer, uint8_t nbytes) {
  uint32_t timeout;  // Flag waiting timeout
  uint8_t n;         // Loop counter

  // Set device address
  I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
  I2C2->CR2 |= ((device_address << 1U) << I2C_CR2_SADD_Pos);

  // Set I2C in Write mode
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;

  // Transfer NBYTES, with AUTOEND
  I2C2->CR2 &= ~I2C_CR2_NBYTES;
  I2C2->CR2 |= ((nbytes + 1) << 16U);
  I2C2->CR2 |= I2C_CR2_AUTOEND;

  // Clear STOPF flag
  I2C2->ICR |= I2C_ICR_STOPCF;

  // Start I2C transaction
  I2C2->CR2 |= I2C_CR2_START;

  // Wait for TXIS with timeout
  timeout = 100000;
  while (((I2C2->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS) {
    timeout--;
    if (timeout == 0) return 1;
  }

  // Send register address
  I2C2->TXDR = register_address;

  n = nbytes;

  while (n > 0) {
    // Wait for TXIS with timeout
    timeout = 100000;
    while (((I2C2->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS) {
      timeout--;
      if (timeout == 0) return 2;
    }

    // Send data
    I2C2->TXDR = *buffer;
    buffer++;
    n--;
  }

  // Wait for STOPF with timeout
  timeout = 100000;
  while (((I2C2->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF) {
    timeout--;
    if (timeout == 0) return 3;
  }

  // Return success
  return 0;
}

#define CTRL2_G 0x11
#define OUTX_L_G 0x22
#define STATUS_REG 0x1E

int main(void) {
  uint8_t rx_data[6];
  while (1) {
    // Set gyro to 416Hz
    I2C2_Write(0b1101010, CTRL2_G, (uint8_t[]){0x60}, 1);
    // Fetch status register
    I2C2_Read(0b1101010, STATUS_REG, rx_data, 1);
    // If gyro data is ready
    if (rx_data[0] & 0x02) {
      // Read all gyro data
      I2C2_Read(0b1101010, OUTX_L_G, rx_data, 6);
      int16_t gyro_x = rx_data[1] << 8 | rx_data[0];
      int16_t gyro_y = rx_data[3] << 8 | rx_data[2];
      int16_t gyro_z = rx_data[5] << 8 | rx_data[4];
      TIM2->CCR1 = 16000 + channel[2] * 10 + gyro_x + gyro_y + gyro_z;
      TIM2->CCR2 = 16000 + gyro_x + gyro_y + gyro_z;
      TIM2->CCR3 = 16000 + gyro_x + gyro_y + gyro_z;
      TIM2->CCR4 = 16000 + gyro_x + gyro_y + gyro_z;
      // USART1->TDR = 'B';
      //   Enable TIM2
      TIM2->CNT = 0;
      TIM2->CR1 = TIM_CR1_OPM;
      TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
    }
  }
  return 0;
}
