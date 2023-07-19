#include <stm32g4xx.h>
#include "gpio.h"

void gpio_init() {
  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Enable GPIOC clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  // Enable GPIOF clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
}

void gpio_pin_mode(GPIO_TypeDef * port, uint32_t pin, uint32_t mode, uint32_t af, uint32_t pupdr, uint32_t otyper) {
  port->MODER &= ~( 3 << (pin * 2));
  port->MODER |= (mode << (pin * 2));
  if(pin > 7) {
    port->AFR[1] &= ~(0xF << ((pin-8) * 4));
    port->AFR[1] |= af << ((pin-8) * 4);
  } else {
    port->AFR[0] &= ~(0xF << (pin * 4));
    port->AFR[0] |= af << (pin * 4);
  }
  if(otyper) {
    port->OTYPER |= (1<<pin);
  } else {
    port->OTYPER &= ~(1<<pin);
  }
  port->PUPDR &= ~(3 << (pin * 2));
  port->PUPDR |= pupdr << (pin * 2);
}

void gpio_set_pin(GPIO_TypeDef * port, uint32_t pin, uint32_t value) {
  if(value) {
    port->BSRR |= (1<<pin);
  } else {
    port->BSRR |= (1<<(pin+16));
  }
}
uint8_t gpio_get_pin(GPIO_TypeDef * port, uint32_t pin) {
  return (port->IDR & (1<<pin)) ? 1 : 0;
}