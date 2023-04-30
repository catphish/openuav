#include <stm32g4xx.h>

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
