#include "stm32f767xx.h"
#include "FPU.h"

void init_FPU()
{
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // enable floating point capability
}
