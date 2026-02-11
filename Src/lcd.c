#include "LCD.h"

void init_LCD()
{
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
}

void delay(unsigned int value_ms) {
  TIM3->CNT = 0;
  while(TIM3->CNT < value_ms*100);
}

void SPI1_Enable()
{
  SPI1->CR1 |= (1<<6);
}

void SPI1_Disable()
{
  SPI1->CR1 &= ~(1<<6);
}

void lcd_CS_enable()
{
  GPIOB->BSRR = 1U << (6+16); // Clear pin 6 of port B, which means chip select enable
}

void lcd_CS_disable()
{
  GPIOB->BSRR = 1U << 6; // Set pin 6 of port B, which means chip select disable
}

void lcd_RST_set()
{
  GPIOB->BSRR = 1U << 9;
}

void lcd_RST_reset()
{
  GPIOB->BSRR = 1U << (9+16);
}

void lcd_DC_set_data()
{
  GPIOA->BSRR = 1U << 0;
}

void lcd_DC_set_command()
{
  GPIOA->BSRR = 1U << (0+16);
}

void spi_send(uint8_t byte)
{
  while(!(SPI1->SR & SPI_SR_TXE));
  *((__IO uint8_t*)&SPI1->DR) = byte;
  while(SPI1->SR & SPI_SR_BSY);
}

void lcd_send_c(uint8_t byte)
{
  lcd_DC_set_command();
  spi_send(byte);
}

void lcd_send_d(uint8_t byte)
{
  lcd_DC_set_data();
  spi_send(byte);
}

void lcd_set_window(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
  lcd_send_c(0x2A);
  lcd_send_d(Xstart >> 8);
  lcd_send_d(Xstart & 0xFF);
  lcd_send_d((Xend - 1) >> 8);
  lcd_send_d((Xend - 1) & 0xFF);

  lcd_send_c(0x2B);
  lcd_send_d(Ystart >> 8);
  lcd_send_d(Ystart & 0xFF);
  lcd_send_d((Yend - 1) >> 8);
  lcd_send_d((Yend - 1) & 0xFF);

  lcd_send_c(0x2C);
}

void lcd_send_d_word(uint16_t data)
{
  lcd_DC_set_data();
  spi_send((data>>8) & 0xff);
  spi_send(data);
}

void lcd_clear_display(uint16_t color)
{
  lcd_set_window(0, 0, LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT);
  lcd_DC_set_data();
  for(int i = 0; i < LCD_2IN4_WIDTH; i++){
    for(int j = 0; j < LCD_2IN4_HEIGHT; j++){
      lcd_send_d_word(color);
    }
  }
}

void lcd_display_image(uint8_t *image)
{
  lcd_set_window(0, 0, LCD_2IN4_WIDTH, LCD_2IN4_HEIGHT);

  lcd_DC_set_data();
  for(uint8_t i = 0; i < LCD_2IN4_WIDTH; i++){
    for(uint8_t j = 0; j < LCD_2IN4_HEIGHT; j++){
      lcd_send_d_word(*(image+i*LCD_2IN4_WIDTH+j));
    }
  }
}

void lcd_set_cursor(uint16_t X, uint16_t Y)
{
  lcd_send_c(0x2A);
  lcd_send_d(X >> 8);
  lcd_send_d(X);
  lcd_send_d(X >> 8);
  lcd_send_d(X);

  lcd_send_c(0x2B);
  lcd_send_d(Y >> 8);
  lcd_send_d(Y);
  lcd_send_d(Y >> 8);
  lcd_send_d(Y);

  lcd_send_c(0x2C);
}

void lcd_draw_paint(uint16_t x, uint16_t y, uint16_t color)
{
  lcd_set_cursor(x, y);
  lcd_send_d_word(color);
}

void lcd_clear_window(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t color)
{
  lcd_set_window(Xstart, Ystart, Xend, Yend);
  for(uint8_t i = Ystart; i <= Yend; i++) {
    for(uint8_t j = Xstart; j <= Xend; j++) {
      lcd_send_d_word(color);
    }
  }
}

void lcd_reset()
{
  lcd_RST_set();
  delay(100);
  lcd_RST_reset();
  delay(100);
  lcd_RST_set();
  delay(100);
}

void lcd_init()
{
  lcd_send_c(0x11); // SLEEP OUT: COMMAND
  delay(120);
  lcd_send_c(0xCF); // POWER CONTROL B: COMMAND
  lcd_send_d(0x00); // POWER CONTROL B: DATA
  lcd_send_d(0xC1); // POWER CONTROL B: DATA
  lcd_send_d(0x30); // POWER CONTROL B: DATA
  lcd_send_c(0xED); // POWER-ON SEQUENCE CONTROL: COMMAND
  lcd_send_d(0x64); // SOFT START CONTROL: DATA
  lcd_send_d(0x03); // POWER-ON SEQUENCE CONTROL: DATA
  lcd_send_d(0x12); // POWER-ON SEQUENCE CONTROL: DATA
  lcd_send_d(0x81); // DDVDH ENHANCE MODE: DATA
  lcd_send_c(0xE8);
  lcd_send_d(0x85);
  lcd_send_d(0x00);
  lcd_send_d(0x79);
  lcd_send_c(0xCB);
  lcd_send_d(0x39);
  lcd_send_d(0x2C);
  lcd_send_d(0x00);
  lcd_send_d(0x34);
  lcd_send_d(0x02);
  lcd_send_c(0xF7);
  lcd_send_d(0x20);
  lcd_send_c(0xEA);
  lcd_send_d(0x00);
  lcd_send_d(0x00);
  lcd_send_c(0xC0);
  lcd_send_d(0x1D);
  lcd_send_c(0xC1);
  lcd_send_d(0x12);
  lcd_send_c(0xC5);
  lcd_send_d(0x33);
  lcd_send_d(0x3F);
  lcd_send_c(0xC7);
  lcd_send_d(0x92);
  lcd_send_c(0x3A);
  lcd_send_d(0x55);
  lcd_send_c(0x36);
  lcd_send_d(0x08);
  lcd_send_c(0xB1);
  lcd_send_d(0x00);
  lcd_send_d(0x12);
  lcd_send_c(0xB6);
  lcd_send_d(0x0A);
  lcd_send_d(0xA2);
  lcd_send_c(0x44);
  lcd_send_d(0x02);
  lcd_send_c(0xF2);
  lcd_send_d(0x00);
  lcd_send_c(0x26);
  lcd_send_d(0x01);
  lcd_send_c(0xE0);
  lcd_send_d(0x0F);
  lcd_send_d(0x22);
  lcd_send_d(0x1C);
  lcd_send_d(0x1B);
  lcd_send_d(0x08);
  lcd_send_d(0x0F);
  lcd_send_d(0x48);
  lcd_send_d(0xB8);
  lcd_send_d(0x34);
  lcd_send_d(0x05);
  lcd_send_d(0x0C);
  lcd_send_d(0x09);
  lcd_send_d(0x0F);
  lcd_send_d(0x07);
  lcd_send_d(0x00);
  lcd_send_c(0xE1);
  lcd_send_d(0x00);
  lcd_send_d(0x23);
  lcd_send_d(0x24);
  lcd_send_d(0x07);
  lcd_send_d(0x10);
  lcd_send_d(0x07);
  lcd_send_d(0x38);
  lcd_send_d(0x47);
  lcd_send_d(0x4B);
  lcd_send_d(0x0A);
  lcd_send_d(0x13);
  lcd_send_d(0x06);
  lcd_send_d(0x30);
  lcd_send_d(0x38);
  lcd_send_d(0x0F);
  lcd_send_c(0x29);
}

// write the data to the LCD on request and only if it has changed
void cyclic_LCD(struct ProcessImage* p_pi)
{
  if(p_pi->print_on_lcd_flag == 1) {
    p_pi->print_on_lcd_flag = 0;

    if(p_pi->bme280.temperature != p_pi->bme280_memory.temperature) {
      lcd_CS_enable();
      Paint_ClearWindows(180, 30, 180+17*5, 50, WHITE);
      Paint_DrawFloatNum(180, 30, p_pi->bme280.temperature, 1, &Font24, WHITE, BLACK);
      lcd_CS_disable();
    }

    if(p_pi->bme280.pressure != p_pi->bme280_memory.pressure) {
      lcd_CS_enable();
      Paint_ClearWindows(180, 60, 180+17*4, 80, WHITE);
      Paint_DrawNum(180, 60, p_pi->bme280.pressure, &Font24, WHITE, BLACK);
      lcd_CS_disable();
    }

    if(p_pi->bme280.humidity != p_pi->bme280_memory.humidity) {
      lcd_CS_enable();
      Paint_ClearWindows(180, 90, 180+17*3, 110, WHITE);
      Paint_DrawNum(180, 90, p_pi->bme280.humidity, &Font24, WHITE, BLACK);
      lcd_CS_disable();
    }

    p_pi->bme280_memory.temperature = p_pi->bme280.temperature;
    p_pi->bme280_memory.pressure = p_pi->bme280.pressure;
    p_pi->bme280_memory.humidity = p_pi->bme280.humidity;
  }
}
