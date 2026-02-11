#include "stm32f767xx.h"
#include "GPIO.h"

void init_GPIO()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable peripheral clock for Port A
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable peripheral clock for Port B
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable peripheral clock for Port C
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; // enable peripheral clock for Port G

  GPIOC->MODER |= (1<<0); // PC0 as output for CS (BME280)
  GPIOB->MODER |= (1<<0); // PB0 as output (LED1)
  GPIOG->MODER |= (1<<0); // PG0 as output (Test-Pin)
  GPIOA->MODER |= (2<<10) | (2<<12) | (2<<14); // Alternate functions for PA5 (SPI1_CLK), PA6 (SP1_MISO) and PA7 (SPI1_MOSI)
  GPIOA->MODER |= (1<<0);  // PA0 as output for DC (LCD)
  GPIOB->MODER |= (1<<12); // PB6 as output for CS (LCD)
  GPIOB->MODER |= (1<<18); // PB9 as output for RST (LCD)
  GPIOB->MODER |= (2<<26) | (2<<30); // Alternate functions for PB13 (SPI2_CLK) and PB15 (SPI2_MOSI)
  GPIOC->MODER |= (2<<4); // Alternate function for PC2 (SPI2_MISO)

  GPIOA->OSPEEDR |= (3<<0) | (3<<10) | (3<<12) | (3<<14); // high speed PA0 (SS), PA5 (SPI1_CLK), PA6 (SPI1_MISO) and PA7 (SPI1_MOSI)
  GPIOB->OSPEEDR |= (3<<26) | (3<<30);
  GPIOC->OSPEEDR |= (3<<4);

  GPIOA->AFR[0] |= (5<<20) | (5<<24) | (5<<28); // Alternate function 5 for SPI1
  GPIOB->AFR[1] |= (5<<20) | (5<<28); // Alternate function 5 for SPI2
  GPIOC->AFR[0] |= (5<<8);
}

void GPIO_toggle_green_LED()
{
  GPIOB->ODR & 1 ? (GPIOB->BSRR = (1<<16)) : (GPIOB->BSRR = (1<<0));
}

void GPIO_toggle_Pin_PG0(void)
{
  if (GPIOG->ODR & (1U << 0)) {
    GPIOG->BSRR = (1U << (0 + 16)); // Reset PG0
  } else {
    GPIOG->BSRR = (1U << 0);        // Set PG0
  }
}
