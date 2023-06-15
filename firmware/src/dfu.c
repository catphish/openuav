#include <stdint.h>
#include "gpio.h"

void dfu() {
  void (*SysMemBootJump)(void);
  SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
  SysMemBootJump();
}

void dfu_init() {
  // Configure PA8 as input
  gpio_pin_mode(GPIOA, 8, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
}

void dfu_main() {
  if(gpio_get_pin(GPIOA, 8)) {
    dfu();
  }
}