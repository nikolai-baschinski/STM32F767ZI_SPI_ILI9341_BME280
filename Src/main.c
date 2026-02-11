// Nikolai Baschinski
// Reading temperature, air pressure and humidity from BME280
// and showing it on the ILI93411 LCD

#include "main.h"

struct ProcessImage pi={0};

int main(void)
{
  init_FPU();
  init_PLL();
  init_GPIO();
  init_TIM();
  init_SPI();
  init_BME();
  init_LCD();
  init_NVIC();

  for(;;) {
    cyclic_LCD(&pi);
  }
}

void TIM2_IRQHandler(void)
{
  if(TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF; // reset UIF bit in the status register of the timer

    if(pi.cntr_10ms % 100 == 0) {
      GPIO_toggle_green_LED();
      pi.print_on_lcd_flag = 1; // write the data once a second
    }

    cyclic_BME(&pi.bme280); // this takes 83 us with prescaler 3 (measured with the oscilloscope)

    pi.cntr_10ms++;

    //GPIO_toggle_Pin_PG0();
  }
}
