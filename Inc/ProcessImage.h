// The process image contains data of the process
// There can't exist loose data- Each data belongs logically to a module
#ifndef PROCESSIMAGE_H_
#define PROCESSIMAGE_H_

#include "BME.h"

struct ProcessImage {
  struct BME280_for_LCD bme280;
  struct BME280_for_LCD bme280_memory;
  uint8_t print_on_lcd_flag;
  uint32_t cntr_10ms;
};

#endif /* PROCESSIMAGE_H_ */
