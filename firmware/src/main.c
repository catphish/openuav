#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "usb.h"
#include "dshot.h"

void SystemInit(void) {
  gpio_init();
  gpio_pin_mode(GPIOA, 10, GPIO_MODE_OUTPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  gpio_pin_mode(GPIOB, 8, GPIO_MODE_INPUT, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  //usb_init();
}

void dfu() {
	void (*SysMemBootJump)(void);
	SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
	SysMemBootJump();
}

int main(void) {
  gpio_set_pin(GPIOA, 10, 1);

  dshot_init();
  while(1) {
    // Read the button connected to PB8
    //if(GPIOB->IDR & (1<<8)) {
    //  dfu();
    //}
    // Wait for the DMA to finish
    if(DMA1_Channel1->CNDTR == 0) {
      if(GPIOB->IDR & (1<<8)) {
        dshot_write(100);
      } else {
        dshot_write(0);
      }
    }
  }
  return 0;
}
