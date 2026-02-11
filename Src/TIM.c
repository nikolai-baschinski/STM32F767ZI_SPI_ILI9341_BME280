#include "stm32f767xx.h"
#include "TIM.h"

void init_TIM2()
{
  RCC->APB1ENR |= (1<<0);      // Enable clock for timer
  TIM2->PSC = (96-1);          // prescaler
  TIM2->ARR = 10000;           // Automatic reset
  TIM2->DIER |= TIM_DIER_UIE;  // Enable interrupt
  TIM2->CR1 |= (1<<0);         // Start timer
  while(!(TIM2->SR & (1<<0))); // Wait until timer is running
}

void init_TIM3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = 0;
    TIM3->PSC = 960 - 1;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void init_TIM()
{
  init_TIM2();
  init_TIM3();
}

