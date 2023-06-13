#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"

void SystemInit(void) {
  gpio_init();
  gpio_port_mode(GPIOA, 10, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  gpio_port_mode(GPIOB, 8, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  usb_init();
}

void dfu() {
	void (*SysMemBootJump)(void);
	SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
	SysMemBootJump();
}

int main(void) {
  gpio_set_pin(GPIOA, 10, 1);
  while(1) {
    // gpio_set_pin(GPIOA, 10, 1);
    // for(uint32_t i = 0; i < 100000; i++) __asm__("nop");
    // gpio_set_pin(GPIOA, 10, 0);
    // for(uint32_t i = 0; i < 100000; i++) __asm__("nop");
    usb_main_loop();
    // Read the button connected to PB8
    if(GPIOB->IDR & (1<<8)) {
      dfu();
    }
  }
  return 0;
}
