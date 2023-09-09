#include <stm32g4xx.h>
#include "gpio.h"
#include "util.h"
#include "led.h"
#include "settings.h"

// Configure ADC1 or ADC2
void adc_configure(ADC_TypeDef * adc) {
  // Disable the Deep Power Down mode
  adc->CR &= ~ADC_CR_DEEPPWD;
  msleep(10);
  // Enable the internal ADC voltage regulator
  adc->CR |= ADC_CR_ADVREGEN;
  msleep(10);

  // Ensure adc is disabled
  adc->CR &= ~ADC_CR_ADEN;

  // Calibrate adc
  adc->CR |= ADC_CR_ADCAL;
  while(adc->CR & ADC_CR_ADCAL) {}

  // Enable adc
  adc->ISR |= ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  while(!(adc->ISR & ADC_ISR_ADRDY)) {}

  // Set adc to continuous mode and overrun mode
  adc->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_CONT;

  // Instruct adc to oversample
  adc->CFGR2 = ADC_CFGR2_ROVSE;

  // Set adc to 256x oversampling, this results in 20 bit total resolution
  adc->CFGR2 |= ADC_CFGR2_OVSR_0 | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_2;

  // Set adc to 4-bit right shift, this results in 20-bit output, the lowest 4 bits are discarded
  // Otherwise, at 20 bits of resolution, the output would always be clipped to 65535, because the
  // DR register is only 16 bits wide
  adc->CFGR2 |= ADC_CFGR2_OVSS_2;

  // Set adc to 640.5 + 12.5 cycles sampling time
  adc->SMPR1 |= ADC_SMPR1_SMP0_0 | ADC_SMPR1_SMP0_1 | ADC_SMPR1_SMP0_2;
  adc->SMPR1 |= ADC_SMPR1_SMP1_0 | ADC_SMPR1_SMP1_1 | ADC_SMPR1_SMP1_2;
  // Set the sequence. One conversion, channel 10.
  adc->SQR1 = 0 | ADC_SQR1_SQ1_3 | ADC_SQR1_SQ1_1;

  // At 5Mhz, this is 640.5 + 12.5 cycles = 130us per sample
  // With 256x oversampling, this is 33ms per sample

  // Start adc
  adc->CR |= ADC_CR_ADSTART;
}

// Configure the analog to digital converter
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

  // Configure ADC1 and ADC2
  adc_configure(ADC1);
  adc_configure(ADC2);
}

// Read ADC2 and return the value in millivolts
uint32_t adc_read_mv() {
  volatile struct settings *settings = settings_get();
  uint32_t adc = 0;
  adc = ADC2->DR;
  if(settings->adc_coefficient == 0) return 0;
  return adc * settings->adc_coefficient / 1000;
}

// Read ADC1 and return the raw value
// TODO: This should scale the data to milliamps
uint32_t adc_read_ma() {
 return ADC1->DR;
}
