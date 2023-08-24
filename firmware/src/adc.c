#include <stm32g4xx.h>
#include "gpio.h"
#include "util.h"
#include "led.h"
#include "settings.h"

void adc_init() {
  // Set ADC12 clock source to sysclk
  RCC->CCIPR |= RCC_CCIPR_ADC12SEL_1;
  // Enable ADC12 clock
  RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
  // Set ADC1 clock divider to 32 (5MHz)
  ADC12_COMMON->CCR = ADC_CCR_PRESC_3;

  // Enable GPIOF clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
  // Set PF0 to analog mode
  gpio_pin_mode(GPIOF, 0, GPIO_MODE_ANALOG, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set PF1 to analog mode
  gpio_pin_mode(GPIOF, 1, GPIO_MODE_ANALOG, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

  // Disable the Deep Power Down mode
  ADC2->CR &= ~ADC_CR_DEEPPWD;
  msleep(10);
  // Enable the internal ADC voltage regulator
  ADC2->CR |= ADC_CR_ADVREGEN;
  msleep(10);

  // Ensure ADC2 is disabled
  ADC2->CR &= ~ADC_CR_ADEN;

  // Calibrate ADC2
  ADC2->CR |= ADC_CR_ADCAL;
  while(ADC2->CR & ADC_CR_ADCAL) {}

  // Enable ADC2
  ADC2->ISR |= ADC_ISR_ADRDY;
  ADC2->CR |= ADC_CR_ADEN;
  while(!(ADC2->ISR & ADC_ISR_ADRDY)) {}

  // Set ADC2 to continuous mode and overrun mode
  ADC2->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_CONT;

  // Instruct ADC2 to oversample
  ADC2->CFGR2 = ADC_CFGR2_ROVSE;

  // Set ADC2 to 256x oversampling, this results in 20 bit total resolution
  ADC2->CFGR2 |= ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_2;

  // Set ADC2 to 4-bit right shift, this results in 20-bit output, the lowest 4 bits are discarded
  // Otherwise, at 20 bits of resolution, the output would always be clipped to 65535, because the
  // DR register is only 16 bits wide
  ADC2->CFGR2 |= ADC_CFGR2_OVSS_2;

  // Set ADC2 to 640.5 + 12.5 cycles sampling time
  ADC2->SMPR1 |= ADC_SMPR1_SMP0_0 | ADC_SMPR1_SMP0_1 | ADC_SMPR1_SMP0_2;
  ADC2->SMPR1 |= ADC_SMPR1_SMP1_0 | ADC_SMPR1_SMP1_1 | ADC_SMPR1_SMP1_2;
  // Set the sequence. One conversion, channel 10.
  ADC2->SQR1 = 0 | ADC_SQR1_SQ1_3 | ADC_SQR1_SQ1_1;

  // At 5Mhz, this is 640.5 + 12.5 cycles = 130us per sample
  // With 256x oversampling, this is 33ms per sample

  // Start ADC2
  ADC2->CR |= ADC_CR_ADSTART;
}

uint32_t adc_read_mV() {
  struct settings *settings = settings_get(); // convenience
  uint32_t adc = 0;

  // Wait for ADC to be ready
  // The above message was mindread by a LLaMA, wow!
  do {
    // Read from the ADC data register
    adc = ADC2->DR;
  } while (adc < 1);

  // Divide ADC reading by ADC coefficient, which is itself
  // divided by 1000 since it needs to be stored as an integer
  return adc / (settings->adc_coefficient / 1000.0f);
}
