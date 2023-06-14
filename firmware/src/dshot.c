#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"

uint8_t dma_data[17];

void dshot_init(void) {
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Configure PA0 as AF1 (TIM2_CH1)
    gpio_pin_mode(GPIOA, 0, GPIO_MODE_AF, 1, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    // Configure TIM2_CH1 as PWM
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    // Enable TIM2_CH1 preload
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    // Enable TIM2_CH1
    TIM2->CCER |= TIM_CCER_CC1E;
    // Set TIM2 period to 53 (16Mhz / 53 = 301.886792kHz)
    TIM2->ARR = 53;
    // Set TIM2_CH1 duty cycle to 0
    TIM2->CCR1 = 0;
    // Set TIM2 DMA request on update
    TIM2->DIER |= TIM_DIER_UDE;
    // Enable TIM2
    TIM2->CR1 |= TIM_CR1_CEN;

    // Enable DMAMUX1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    // Configure DMAMUX1 channel 1 to TIM2_UP
    DMAMUX1_Channel0->CCR |= 60;

    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    // Set DMA1 channel 1 peripheral address to TIM2_CCR1
    DMA1_Channel1->CPAR = (uint32_t) &TIM2->CCR1;
    // Set DMA1 channel 1 number of data to transfer to 0
    DMA1_Channel1->CNDTR = 0;
    // Disable DMA1 channel 1
    DMA1_Channel1->CCR = 0;
    // Set DMA1 channel 1 to memory to peripheral mode
    DMA1_Channel1->CCR |= DMA_CCR_DIR;
    // Set DMA1 channel 1 to 8-bit memory size
    DMA1_Channel1->CCR |= 0;
    // Set DMA1 channel 1 to 32-bit peripheral size
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_1;
    // Set DMA1 channel 1 to increment memory address
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    // Set DMA1 channel 1 data transfer direction to read from memory
    DMA1_Channel1->CCR |= DMA_CCR_DIR;
}

void dshot_write(uint16_t data) {
    data <<= 1;
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    data <<= 4;
    data |= crc;

    // For each bit in data, write a byte to dma_data, MSB first
    // 0 bits are 20, 1 bits are 40
    for(uint8_t i = 0; i < 16; i++) {
        if(data & (1 << (15 - i))) {
            dma_data[i] = 40;
        } else {
            dma_data[i] = 20;
        }
    }
    // Write 0 to the last byte
    dma_data[16] = 0;

    // Disable DMA1 channel 1
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    // Set DMA1 channel 1 memory address
    DMA1_Channel1->CMAR = (uint32_t)dma_data;
    // Set DMA1 channel 1 number of data to transfer to 17
    DMA1_Channel1->CNDTR = 17;
    // Enable DMA1 channel 1
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}
