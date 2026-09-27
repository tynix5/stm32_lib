#ifndef SSD1306_HAL_H_
#define SSD1306_HAL_H_

#include "ssd1306.h"

#define I2C_TIMEOUT_MS      25
#define I2C_LONG_TIMEOUT_MS 250

/* Generic I2C writer for SSD1306 */
ssd1306_status_t ssd1306_hal_write(ssd1306_handle_t * holed, uint8_t * data, uint32_t len);
/* DMA sender method */
ssd1306_status_t ssd1306_hal_write_i2c_dma(ssd1306_handle_t * holed, uint8_t * data, uint32_t len);
/* Blocking sender method */
ssd1306_status_t ssd1306_hal_write_i2c_blocking(ssd1306_handle_t * holed, uint8_t * data, uint32_t len, uint32_t timeout);
/* Callback function for HAL_I2C_MasterTxCpltCallback */
void ssd1306_hal_i2c_dma_callback(I2C_HandleTypeDef * hi2c);

#endif