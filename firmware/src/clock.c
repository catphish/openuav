#include <stm32g4xx.h>
#include "led.h"

void clock_init(void) {
    // Enable FPU
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */

    // Configure PLL to 160MHz using HSI as source
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    // Wait for HSI to be ready
    while(!(RCC->CR & RCC_CR_HSIRDY));
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    // Wait for PLL to be disabled
    while(RCC->CR & RCC_CR_PLLRDY);
    // Configure PLL
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | RCC_PLLCFGR_PLLM_0 | (40 << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN;
    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    led1_toggle();
    // Wait for PLL to be ready
    while(!(RCC->CR & RCC_CR_PLLRDY));
    led0_toggle();
    // Set flash latency to 4 wait states
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
    // Set SYSCLK to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    // Wait for SYSCLK to be ready
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}