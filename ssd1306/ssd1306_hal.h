#ifndef SSD1306_HAL_H_
#define SSD1306_HAL_H_

#include "ssd1306.h"
#include "error_types.h"

#define I2C_TIMEOUT_MS      25
#define I2C_LONG_TIMEOUT_MS 250

/* Generic I2C writer for SSD1306 */
status_t ssd1306_write(ssd1306_handle_t * holed, uint8_t * data, uint32_t len);
/* DMA sender method */
status_t i2c_write_dma(ssd1306_handle_t * holed, uint8_t * data, uint32_t len);
/* Blocking sender method */
status_t i2c_write_blocking(ssd1306_handle_t * holed, uint8_t * data, uint32_t len, uint32_t timeout);
/* Callback function for HAL_I2C_MasterTxCpltCallback */
void i2c_dma_callback(I2C_HandleTypeDef * hi2c);

#endif