#include "gpio.h"

void led_init() {
  gpio_set_pin(GPIOA, 10, 1);
  gpio_pin_mode(GPIOA, 10, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
}

void led0_on() {
  gpio_set_pin(GPIOA, 10, 0);
}

void led0_off() {
  gpio_set_pin(GPIOA, 10, 1);
}

void led0_toggle() {
  gpio_set_pin(GPIOA, 10, !gpio_get_pin(GPIOA, 10));
}
