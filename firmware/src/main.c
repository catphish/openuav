#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"

void dfu() {
  void (*SysMemBootJump)(void);
  SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
  SysMemBootJump();
}

void SystemInit(void) {
  gpio_init();
  // Set up LED
  gpio_set_pin(GPIOA, 10, 1);
  gpio_pin_mode(GPIOA, 10, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set up button
  gpio_pin_mode(GPIOB, 8, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

  // Initialize USB
  usb_init();
}

int main(void) {
  while(1) {
    if(GPIOB->IDR & (1<<8)) {
     dfu();
    }
    usb_main();
    if(ep_tx_ready(1)) {
      usb_write(1, "Hello World!\n", 13);
    }
  }
  return 0;
}