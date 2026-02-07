BME280 data (temperature, air pressure and humidity) displayed on the Waveshare ILI9341 LCD by STM32F767ZI (dev board) bare metal (no HAL) using the CubeIDE.

![BME280 and 2.4" LCD](/Doc/BME280_and_2.4_inch_LCD_controlled_by_STM32F767ZI.bmp)
BME280 and 2.4" LCD

Connections (https://os.mbed.com/platforms/ST-Nucleo-F767ZI/)


| LCD      | Port     | Function     |
|----------|----------|------------- |
| VCC      | 3,3V     | Vcc          |
| GND      | GND      | GND          |
| DIN      | PA7      | SPI1_MOSI    |
| CLK      | PA5      | SPI1_CLK     |
| CS       | PB6      | Chip Select  |
| DC       | PA0      | Data/Command |
| RST      | PB9      | Reset        |
| BL       | -        | Backlight    |


| BME280   | Port     | Function     |
|----------|----------|--------------|
| VCC      | 3,3V     | Vcc          |
| GND      | GND      | GND          |
| SCK      | PB13     | SPI2_CLK     |
| MOSI     | PB15     | SPI2_MOSI    |
| MISO     | PC2      | SPI2_MISO    |
| CS       | PC0      | Chip Select  |


![BME280 raw data burst via SPI](/Doc/BME280_raw_data_burst_via_SPI.bmp)
BME280 raw data burst via SPI

Hardware
BME280: https://seengreat.com/product/207/bme280-environmental-sensor?srsltid=AfmBOorvlymsT9w0Ea-JBnftBbgADYcXMKpadnPHUyHl7X1wOO5TTgUa

LCD: https://www.waveshare.com/wiki/2.4inch_LCD_Module?srsltid=AfmBOoqtv3bq-mZfPtsi2BxiewwQnIkomXrloIzpVwGw_HnrOcmvQZar

Nucleo-STM32767ZI: https://www.st.com/en/evaluation-tools/nucleo-f767zi.html
