#include "gpio.h"

#define LED1_PORT GPIOA
#define LED1_PIN 13
#define LED2_PORT GPIOA
#define LED2_PIN 14

void led_init() {
  // LED1
  gpio_set_pin(LED1_PORT, LED1_PIN, 0);
  gpio_pin_mode(LED1_PORT, LED1_PIN, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // LED2
  gpio_set_pin(LED2_PORT, LED2_PIN, 0);
  gpio_pin_mode(LED2_PORT, LED2_PIN, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
}


void led1_on() {
  gpio_set_pin(LED1_PORT, LED1_PIN, 1);
}

void led1_off() {
  gpio_set_pin(LED1_PORT, LED1_PIN, 0);
}

void led1_toggle() {
  gpio_toggle_pin(LED1_PORT, LED1_PIN);
}

void led2_on() {
  gpio_set_pin(LED2_PORT, LED2_PIN, 1);
}

void led2_off() {
  gpio_set_pin(LED2_PORT, LED2_PIN, 0);
}

void led2_toggle() {
  gpio_toggle_pin(LED2_PORT, LED2_PIN);
}
