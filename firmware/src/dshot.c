#include <stm32g4xx.h>
#include <stdint.h>
#include "gpio.h"
#include "dshot.h"

volatile uint16_t dma_data[17 * 4];

void dshot_init(void) {
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Enable DMAMUX1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Configure PA0 as AF1 (TIM2_CH1)
    gpio_pin_mode(GPIOA, 0, GPIO_MODE_AF, 1, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Configure PA1 as AF1 (TIM2_CH2)
    gpio_pin_mode(GPIOA, 1, GPIO_MODE_AF, 1, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Configure PA2 as AF1 (TIM2_CH3)
    gpio_pin_mode(GPIOA, 2, GPIO_MODE_AF, 1, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
    // Configure PA3 as AF1 (TIM2_CH4)
    gpio_pin_mode(GPIOA, 3, GPIO_MODE_AF, 1, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Configure TIM2_CH1 as PWM
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    // Configure TIM2_CH2 as PWM
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    // Configure TIM2_CH3 as PWM
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    // Configure TIM2_CH4 as PWM
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    // Enable TIM2_CH1 preload
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    // Enable TIM2_CH2 preload
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    // Enable TIM2_CH3 preload
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    // Enable TIM2_CH4 preload
    TIM2->CCMR2 |= TIM_CCMR2_OC4PE;

    // Enable TIM2_CH1
    TIM2->CCER |= TIM_CCER_CC1E;
    // Enable TIM2_CH2
    TIM2->CCER |= TIM_CCER_CC2E;
    // Enable TIM2_CH3
    TIM2->CCER |= TIM_CCER_CC3E;
    // Enable TIM2_CH4
    TIM2->CCER |= TIM_CCER_CC4E;

    // Set TIM2 period to 53 (160Mhz / 300khz = 533.333)
    TIM2->ARR = 533;

    // Set TIM2_CH1 duty cycle to 0
    TIM2->CCR1 = 0;
    // Set TIM2_CH2 duty cycle to 0
    TIM2->CCR2 = 0;
    // Set TIM2_CH3 duty cycle to 0
    TIM2->CCR3 = 0;
    // Set TIM2_CH4 duty cycle to 0
    TIM2->CCR4 = 0;

    // Set TIM2 DMA request on update
    TIM2->DIER |= TIM_DIER_UDE;
    // Set TIM2 to accept 4 transfers per DMA request
    TIM2->DCR |= TIM_DCR_DBL_1 | TIM_DCR_DBL_0;
    // Set TIM2 DMA base address to CCR1
    TIM2->DCR |= 0xD;

    // Configure DMAMUX1 channel 1 to TIM2_UP
    DMAMUX1_Channel0->CCR |= 60;

    // Set DMA1 channel 1 peripheral address to TIM2_CCR1
    DMA1_Channel1->CPAR = (uint32_t) &TIM2->DMAR;
    // Disable DMA1 channel 1
    DMA1_Channel1->CCR = 0;
    // Set DMA1 channel 1 to memory to peripheral mode
    DMA1_Channel1->CCR |= DMA_CCR_DIR;
    // Set DMA1 channel 1 to 16-bit memory size
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
    // Set DMA1 channel 1 to 32-bit peripheral size
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_1;
    // Set DMA1 channel 1 to increment memory address
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    // Set DMA1 channel 1 data transfer direction to read from memory
    DMA1_Channel1->CCR |= DMA_CCR_DIR;
    // Set DMA1 channel 1 priority to very high
    DMA1_Channel1->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0;

}

static void set_dma_data(int n, uint8_t armed, int32_t data) {
    if(data > 2047) {
        data = 2047;
    }
    if(data < 48) {
        data = 48;
    }
    if(!armed) {
        data = 0;
    }

    data <<= 1;
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    data <<= 4;
    data |= crc;

    // For each bit in data, write a byte to dma_data, MSB first
    // 0 bits are 20, 1 bits are 40
    for(uint8_t i = 0; i < 16; i++) {
        if(data & (1 << (15 - i))) {
            dma_data[i*4+n] = 400;
        } else {
            dma_data[i*4+n] = 200;
        }
    }
    dma_data[16*4+n] = 0;
}

void dshot_write(struct dshot_data * data) {
    set_dma_data(0, data->armed, data->motor1);
    set_dma_data(1, data->armed, data->motor2);
    set_dma_data(2, data->armed, data->motor3);
    set_dma_data(3, data->armed, data->motor4);

    // Disable DMA1 channel 1
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    // Set DMA1 channel 1 memory address
    DMA1_Channel1->CMAR = (uint32_t)dma_data;
    // Set DMA1 channel 1 number of data to transfer to 17
    DMA1_Channel1->CNDTR = 17*4;
    // Enable TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
    // Enable DMA1 channel 1
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}
