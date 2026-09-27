#include "ssd1306_hal.h"

ssd1306_status_t ssd1306_hal_write(ssd1306_handle_t * holed, uint8_t * data, uint32_t len)
{
    if (holed->mode == SSD1306_PHY_MODE_BLOCKING)
    {
        // trash
        if (len < 50)
            return ssd1306_hal_write_i2c_blocking(holed, data, len, I2C_TIMEOUT_MS);
        else
            return ssd1306_hal_write_i2c_blocking(holed, data, len, I2C_LONG_TIMEOUT_MS);
    }
    else if (holed->mode == SSD1306_PHY_MODE_DMA)
        return ssd1306_hal_write_i2c_dma(holed, data, len);

    return SSD1306_INVALID_CFG;
}

ssd1306_status_t ssd1306_hal_write_i2c_dma(ssd1306_handle_t * holed, uint8_t * data, uint32_t len)
{
    // check if DMA is already in use as to not overwrite TX buffer
    if (holed->_dma_busy == SSD1306_DMA_BUSY)
        return SSD1306_BUSY;

    // if DMA starts, note it is in use
    if (HAL_I2C_Master_Transmit_DMA(holed->hi2c, SSD1306_I2C_WRITE, data, len) == HAL_OK)
    {
        holed->_dma_busy = SSD1306_DMA_BUSY;
        return SSD1306_OK;
    }

    return SSD1306_PHY_ERR;
}

ssd1306_status_t ssd1306_hal_write_i2c_blocking(ssd1306_handle_t * holed, uint8_t * data, uint32_t len, uint32_t timeout)
{
    // should never return SSD1306_BUSY, but check if user mixes DMA transactions with non-DMA transactions
    if (holed->_dma_busy == SSD1306_DMA_BUSY)
        return SSD1306_BUSY;

    if (HAL_I2C_Master_Transmit(holed->hi2c, SSD1306_I2C_WRITE, data, len, timeout) == HAL_OK)
        return SSD1306_OK;

    return SSD1306_PHY_ERR;
}

// turn into a callback function and have user register callback 
void ssd1306_hal_i2c_dma_callback(I2C_HandleTypeDef * hi2c)
{
    // once transfer completes, reset DMA busy bit for OLED
    for (uint8_t i = 0; i < ssd1306_device_cnt; i++)
    {
        ssd1306_handle_t * oled = ssd1306_devices[i];

        if (oled == NULL)
            continue;

        // if two OLEDs share one I2C bus, only one of them has DMA active at any time
        // CAUTION: this logic will successfully find the right OLED, but nothing is preventing from two OLEDs on the same bus from trying to start
        // one transaction while another one from a different OLED is running
        // perhaps add some logic to prevent that?
        if (oled->hi2c == hi2c && oled->_dma_busy == SSD1306_DMA_BUSY)
        {
            oled->_dma_busy = SSD1306_DMA_OK;
            return;
        }
    }
}