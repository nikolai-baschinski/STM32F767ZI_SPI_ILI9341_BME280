#include "stm32f767xx.h"
#include "NVIC.h"

void init_NVIC()
{
  NVIC_SetPriority(TIM2_IRQn, 5);
  NVIC_EnableIRQ(TIM2_IRQn);
}
