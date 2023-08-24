#include "gpio.h"

void led_init() {
  // LED1 - PA14
  gpio_set_pin(GPIOA, 14, 0);
  gpio_pin_mode(GPIOA, 14, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // LED2 - PA13
  gpio_set_pin(GPIOA, 13, 0);
  gpio_pin_mode(GPIOA, 13, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // 9V Enable - PB9
  gpio_set_pin(GPIOB, 9, 0);
  gpio_pin_mode(GPIOB, 9, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
}

void led1_on() {
  gpio_set_pin(GPIOA, 14, 1);
}

void led1_off() {
  gpio_set_pin(GPIOA, 14, 0);
}

void led1_toggle() {
  gpio_set_pin(GPIOA, 14, !gpio_get_pin(GPIOA, 14));
}

void led2_on() {
  gpio_set_pin(GPIOA, 13, 1);
}

void led2_off() {
  gpio_set_pin(GPIOA, 13, 0);
}

void led2_toggle() {
  gpio_set_pin(GPIOA, 13, !gpio_get_pin(GPIOA, 13));
}

void nine_volt_on() {
  gpio_set_pin(GPIOB, 9, 1);
}

void nine_volt_off() {
  gpio_set_pin(GPIOB, 9, 0);
}
