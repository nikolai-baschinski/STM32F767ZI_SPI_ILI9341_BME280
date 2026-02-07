#ifndef BME_H_
#define BME_H_

#include "stm32f767xx.h"

struct BME280_for_LCD {
  float temperature;
  uint16_t pressure;
  uint8_t humidity;
};

void bme_set_raw_data(uint8_t* data_rcv_burst);
void bme_CS_enable();
void bme_CS_disable();
uint8_t SPI2_SendRecv(uint8_t data);
void bme_compensate();
void init_bme();
void bme_get_sensor_data(struct BME280_for_LCD* bme_data);

#endif /* BME_H_ */
