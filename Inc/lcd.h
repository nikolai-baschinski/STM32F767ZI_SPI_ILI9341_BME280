#ifndef LCD_H_
#define LCD_H_

#include "stm32f767xx.h"
#include "GUI_Paint.h"
#include "image.h"

void lcd_CS_disable();
void lcd_CS_enable();

void lcd_reset();
void lcd_init();
void lcd_clear_display(uint16_t color);
void lcd_draw_paint(uint16_t x, uint16_t y, uint16_t color);

void init_TIM3();


#endif /* LCD_H_ */
