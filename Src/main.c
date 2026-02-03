// Nikolai Baschinski
// Nucleo-F767ZI LD1 blinking 1Hz bare metal
// PG0 pin toggle 50 Hz

#include "stm32f767xx.h"
#include "main.h"
#include "lcd.h"

int main(void)
{
  init_FPU();
  init_PLL();
  init_GPIO();
  init_TIM2();
  init_TIM3();
  init_SPI1();
  init_NVIC();

  lcd_reset();
  lcd_CS_enable(); // chip select
  lcd_init();
  lcd_clear_display(WHITE);

  Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
  Paint_SetClearFuntion(lcd_clear_display);
  Paint_SetDisplayFuntion(lcd_draw_paint);

  Paint_DrawString_EN (5, 34, "Hello World",  &Font24,    BLUE,    CYAN);
  Paint_DrawFloatNum  (5, 150 ,987.654321,2,  &Font20,    BLACK,   WHITE);

  Paint_DrawRectangle (125, 240, 225, 300,    RED     ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  Paint_DrawLine      (125, 240, 225, 300,    MAGENTA ,DOT_PIXEL_2X2,LINE_STYLE_SOLID);
  Paint_DrawLine      (225, 240, 125, 300,    MAGENTA ,DOT_PIXEL_2X2,LINE_STYLE_SOLID);

  Paint_DrawCircle(150,100,  25,        BLUE    ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  Paint_DrawCircle(180,100,  25,        BLACK   ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  Paint_DrawCircle(210,100,  25,        RED     ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  Paint_DrawCircle(165,125,  25,        YELLOW  ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
  Paint_DrawCircle(195,125,  25,        GREEN   ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);

  //Paint_DrawImage(gImage_a,0,0,240,320);
  lcd_CS_disable();

  for(;;);
}

void TIM2_IRQHandler(void)
{
  static unsigned int cntr_10ms = 0;

  if(TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF; // reset UIF bit in the status register of the timer

    if(cntr_10ms % 100 == 0) {
      GPIO_toggle_green_LED();
    }

    cntr_10ms++;

    GPIO_toggle_Pin_PG0();
  }
}

void init_FPU()
{
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // enable floating point capability
}

void init_PLL()
{
    RCC->CR |= RCC_CR_HSEBYP; // Clock control register, Enable bypass, the clock comes from the SWD
    RCC->CR |= RCC_CR_HSEON;  // Clock control register, Enable High Speed External Clock
    while(!(RCC->CR & RCC_CR_HSERDY)); // Clock control register, wait until the HSE is ready

    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable the peripheral clock

    FLASH->ACR |= FLASH_ACR_LATENCY_7WS; // Flash latency has to be set

    // SYSCLK prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // ABH prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 prescaler

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    // PLL Source Mux Prescaler
    #define PLL_M 4
    #define PLL_N 96
    #define PLL_P 0 // means prescaler 2
    RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY)); // check the ready flag of the PLL

    // System Clock Mux
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != (RCC_CFGR_SWS_PLL));
}

void init_GPIO()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable peripheral clock for Port B
	GPIOB->MODER |= (1<<0); // pin 0 port B is output
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; // enable peripheral clock for Port G
	GPIOG->MODER |= (1<<0); // pin 0 port G is output

	// SPI1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable peripheral clock for Port A

	GPIOA->MODER |= (2<<10) | (2<<12) | (2<<14); // Alternate function for PA5 (SPI1_CLK), PA6 (SP1_MISO) and PA7 (SPI1_MOSI)
	GPIOA->MODER |= (1<<0);  // PA0 as output for DC
	GPIOB->MODER |= (1<<12); // PB6 as output for SS
	GPIOB->MODER |= (1<<18); // PB9 as output for RST

	GPIOA->OSPEEDR |= (3<<0) | (3<<10) | (3<<12) | (3<<14); // high speed PA0 (SS), PA5 (SPI1_CLK), PA6 (SPI1_MISO) and PA7 (SPI1_MOSI)

	GPIOA->AFR[0] |= (5<<20) | (5<<24) | (5<<28); // Alternate function 5 for SPI
}

void init_TIM2()
{
	RCC->APB1ENR |= (1<<0);      // Enable clock for timer
	TIM2->PSC = (96-1);          // prescaler
	TIM2->ARR = 10000;           // Automatic reset
	TIM2->DIER |= TIM_DIER_UIE;  // Enable interrupt
	TIM2->CR1 |= (1<<0);         // Start timer
	while(!(TIM2->SR & (1<<0))); // Wait until timer is running
}

void init_NVIC()
{
  NVIC_SetPriority(TIM2_IRQn, 5);
  NVIC_EnableIRQ(TIM2_IRQn);
}

void init_SPI1()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SP1

  // CPOL and CPHA is 0 by default

  SPI1->CR1 |= (3<<3); // divide SYSCLK by

  // MSB comes first, default setting

  SPI1->CR1 |= SPI_CR1_SSM; // Enable software slave management
  SPI1->CR1 |= SPI_CR1_SSI; // Set internal slave select to 1

  // Full-duplex, RXONLY-Bit is default 0

  SPI1->CR1 |= SPI_CR1_MSTR; // Set master mode

  SPI1->CR2 |= (3<<8); // 8 bits data

  SPI1->CR1 |= (1<<6); // Enable SPI
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
