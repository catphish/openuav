#include <stm32g4xx.h>

// The SystemInit function is called at startup to configure all the STM32 peripherals
void SystemInit(void) {
  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Enable USART2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  // Enable SPI1 clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  // Enable TIM2 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  // Enable TIM3 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
  
  // Set PA10 to output mode to control status LED
  GPIOA->MODER &= ~GPIO_MODER_MODE10_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE10_0;
  // Set PA10 to open-drain output
  GPIOA->OTYPER |= GPIO_OTYPER_OT10_Msk;
  GPIOA->BSRR = GPIO_BSRR_BS10;


  // Set PA14 and PA15 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE14_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE14_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE15_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE15_1;
  // Set PA14 and PA15 to USART2 AF7
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_1 | GPIO_AFRH_AFSEL14_2;
  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_1 | GPIO_AFRH_AFSEL15_2;

  // Set PA4 to output mode to control SPI1 chip select
  GPIOA->MODER &= ~GPIO_MODER_MODE4_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE4_0;
  // Set PA4 to push-pull output
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
  // Set PA4 to high speed
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_1;
  // Set PA4 (SPI1 CS) high
  GPIOA->BSRR = GPIO_BSRR_BS4;

  // Set PB2 to input mode to read SPI1 data ready
  GPIOB->MODER &= ~GPIO_MODER_MODE2_Msk;

  // Set PA5, PA6, PA7 to alternate function mode
  GPIOA->MODER &= ~GPIO_MODER_MODE5_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE5_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE6_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE6_1;
  GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;
  GPIOA->MODER |= GPIO_MODER_MODE7_1;
  // Set PA5, PA6, PA7 to SPI1 AF5
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_2;
  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2;

  // Disable USART2
  USART2->CR1 = USART_CR1_UE;
  // Set USART2 baud rate to 420000 (16MHz / 420000 = 38)
  USART2->BRR = 38;
  // Enable USART2
  USART2->CR1 = USART_CR1_UE;
  // Enable USART2 receiver
  USART2->CR1 = USART_CR1_UE | USART_CR1_RE;
  // Enable USART2 interrupt
  USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
  // Enable USART2 interrupt in NVIC
  NVIC_EnableIRQ(USART2_IRQn);

  SPI1->CR1 = 0;
  // Set SPI1 speed to 1MHhz (16MHz / 1MHz = 16)
  SPI1->CR1 = SPI_CR1_BR_0 | SPI_CR1_BR_1;
  // Set SPI1 to master mode
  SPI1->CR1 |= SPI_CR1_MSTR;
  // Set SPI1 to software slave management
  SPI1->CR1 |= SPI_CR1_SSM;
  SPI1->CR1 |= SPI_CR1_SSI;
  // Enable SPI1
  SPI1->CR1 |= SPI_CR1_SPE;

  // Enable interrupts
  __enable_irq();
}
