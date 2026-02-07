// Nikolai Baschinski
// Reading temperature, air pressure and humidity from BME280
// and showing it on the ILI93411 LCD

#include <string.h>
#include "stm32f767xx.h"
#include "main.h"
#include "lcd.h"
#include "bme.h"

#define MAX_RECV_BURST 20

uint8_t data_rcv_burst[MAX_RECV_BURST]= {0};
uint16_t cntr_burst = 0;

uint8_t print_on_lcd_flag = 0;

void SPI2_TransmitBurst(uint8_t data)
{
  while(!(SPI2->SR & SPI_SR_TXE));
  SPI2->DR = data;

  while(!(SPI2->SR & SPI_SR_TXE));
  while(SPI2->SR & SPI_SR_BSY);

  while((SPI2->SR & SPI_SR_RXNE) && (cntr_burst < MAX_RECV_BURST)) {
    data_rcv_burst[cntr_burst] = SPI2->DR;
    cntr_burst++;
  }
}

int main(void)
{
  init_FPU();
  init_PLL();
  init_GPIO();
  init_TIM2();
  init_TIM3();
  init_SPI1();
  init_SPI2();
  init_bme();
  init_NVIC();

  lcd_reset();
  lcd_CS_enable();
  lcd_init();
  lcd_clear_display(WHITE);

  Paint_NewImage(LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT, ROTATE_270, WHITE);
  Paint_SetClearFuntion(lcd_clear_display);
  Paint_SetDisplayFuntion(lcd_draw_paint);

  Paint_DrawString_EN(10, 30, "Temperat:      C", &Font24, WHITE, BLACK);
  Paint_DrawString_EN(10, 60, "Pressure:      hPa", &Font24, WHITE, BLACK);
  Paint_DrawString_EN(10, 90, "Humidity:      %", &Font24, WHITE, BLACK);

  lcd_CS_disable();

  struct BME280_for_LCD bme280;
  struct BME280_for_LCD bme280_memory;

  for(;;) {

    memset(&data_rcv_burst, 0, MAX_RECV_BURST);
    cntr_burst = 0;

    bme_CS_enable();
    SPI2_TransmitBurst(0xF7); // start burst
    for(int i=0; i<4; i++) {
      SPI2_TransmitBurst(0xFF); // run the burst with dummy data on MOSI
    }
    bme_CS_disable();

    bme_set_raw_data(data_rcv_burst);
    bme_compensate();
    bme_get_sensor_data(&bme280);

    // update
    if(print_on_lcd_flag==1)  {
      print_on_lcd_flag = 0;

      if(bme280.temperature != bme280_memory.temperature) {
        lcd_CS_enable();
        Paint_ClearWindows(180, 30, 265, 50, WHITE);
        Paint_DrawFloatNum(180, 30, bme280.temperature, 1, &Font24, WHITE, BLACK);
        lcd_CS_disable();
      }

      if(bme280.pressure != bme280_memory.pressure) {
        lcd_CS_enable();
        Paint_ClearWindows(180, 60, 265, 80, WHITE);
        Paint_DrawNum(180, 60, bme280.pressure, &Font24, WHITE, BLACK);
        lcd_CS_disable();
      }

      if(bme280.humidity != bme280_memory.humidity) {
        lcd_CS_enable();
        Paint_ClearWindows(180, 90, 265, 110, WHITE);
        Paint_DrawNum(180, 90, bme280.humidity, &Font24, WHITE, BLACK);
        lcd_CS_disable();
      }

      bme280_memory.temperature = bme280.temperature;
      bme280_memory.pressure = bme280.pressure;
      bme280_memory.humidity = bme280.humidity;
    }

    for (int i = 0;i < 100000;i++); // 34 ms when 100000
  }
}

void TIM2_IRQHandler(void)
{
  static unsigned int cntr_10ms = 0;

  if(TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF; // reset UIF bit in the status register of the timer

    if(cntr_10ms % 100 == 0) {
      GPIO_toggle_green_LED();
      print_on_lcd_flag = 1;
    }

    cntr_10ms++;

    //GPIO_toggle_Pin_PG0();
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

// For the ILI9341 LCD
void init_SPI1()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SP1

  // CPOL and CPHA is 0 by default (SPI mode 0)

  SPI1->CR1 |= (3<<3); // divide SYSCLK by

  // MSB comes first, default setting

  SPI1->CR1 |= SPI_CR1_SSM; // Enable software slave management
  SPI1->CR1 |= SPI_CR1_SSI; // Set internal slave select to 1

  // Full-duplex, RXONLY-Bit is default 0

  SPI1->CR1 |= SPI_CR1_MSTR; // Set master mode

  SPI1->CR2 |= (3<<8); // 8 bits data

  SPI1->CR1 |= (1<<6); // Enable SPI
}

// For the BDM280
void init_SPI2()
{
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // Enable SP2

  // CPOL and CPHA is 0 by default (SPI mode 0)

  SPI2->CR1 &= ~(7 << 3);
  SPI2->CR1 |=  (6 << 3); // divide SYSCLK by 16

  SPI2->CR1 |= SPI_CR1_SSM; // Enable software slave management
  SPI2->CR1 |= SPI_CR1_SSI; // Set internal slave select to 1

  // Full-duplex, RXONLY-Bit is default 0

  SPI2->CR1 |= SPI_CR1_MSTR; // Set master mode

  SPI2->CR2 |= (3<<8); // 8 bits data
  SPI2->CR2 |= SPI_CR2_FRXTH; // Read 8 bits

  SPI2->CR1 |= (1<<6); // Enable SPI
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
