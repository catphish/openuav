#include <stm32g4xx.h>

#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_AF 2
#define GPIO_MODE_ANALOG 3

#define GPIO_PUPD_NONE 0
#define GPIO_PUPD_PU 1
#define GPIO_PUPD_PD 2

#define GPIO_OTYPE_PP 0
#define GPIO_OTYPE_OD 1

void gpio_init();
void gpio_pin_mode(GPIO_TypeDef * port, uint32_t pin, uint32_t mode, uint32_t af, uint32_t pupdr, uint32_t otyper);
void gpio_set_pin(GPIO_TypeDef * port, uint32_t pin, uint32_t value);
