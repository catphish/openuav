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

  // Enable GPIOF clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
  // Set PF0 to analog mode
  gpio_pin_mode(GPIOF, 0, GPIO_MODE_ANALOG, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);
  // Set PF1 to analog mode
  gpio_pin_mode(GPIOF, 1, GPIO_MODE_ANALOG, 0, GPIO_PUPD_NONE, GPIO_OTYPE_PP);

  ADC1->CR &= ~ADC_CR_DEEPPWD;
  msleep(10);
  ADC1->CR |= ADC_CR_ADVREGEN;
  msleep(10);

  // Disable ADC1
  ADC1->CR &= ~ADC_CR_ADEN;

  // Calibrate ADC1
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR & ADC_CR_ADCAL) {}

  // Enable ADC1
  ADC1->ISR |= ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while(!(ADC1->ISR & ADC_ISR_ADRDY)) {}

  // Set ADC1 to continuous mode and overrun mode
  ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_CONT;
  // Set the sequence. One conversion, channel 10.
  ADC1->SQR1 = 0 | ADC_SQR1_SQ1_3 | ADC_SQR1_SQ1_1;
  // Start ADC1
  ADC1->CR |= ADC_CR_ADSTART;
}

uint16_t adc_read_mV() {
  struct settings *settings = settings_get(); // convenience
  // Multiply ADC reading with ADC coefficient
  // (which is saved at * 100 its required value)
  return ADC1->DR * (settings->adc_coefficient / 100.0f);
}
