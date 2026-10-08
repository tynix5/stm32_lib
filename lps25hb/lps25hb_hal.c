#include "lps25hb_hal.h"
#include "lps25hb.h"

lps25hb_status_t lps25hb_hal_select(const lps25hb_handle_t * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_RESET);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_hal_deselect(const lps25hb_handle_t * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_SET);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_hal_write_reg(const lps25hb_handle_t * bar, uint8_t reg, const uint8_t data)
{
    reg |= LPS25HB_WRITE;
    
    lps25hb_hal_select(bar);
    HAL_SPI_Transmit(bar->hspi, &reg, 1, LPS25HB_SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(bar->hspi, &data, 1, LPS25HB_SPI_TIMEOUT_MS);
    lps25hb_hal_deselect(bar);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_hal_read_reg(const lps25hb_handle_t * bar, uint8_t reg, uint8_t * data)
{
    reg |= LPS25HB_READ;

    lps25hb_hal_select(bar);
    HAL_SPI_Transmit(bar->hspi, &reg, 1, LPS25HB_SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, data, 1, LPS25HB_SPI_TIMEOUT_MS);
    lps25hb_hal_deselect(bar);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_hal_read(const lps25hb_handle_t * bar, uint8_t start_reg, const uint8_t dlen, uint8_t * data)
{
    // page 28 of datasheet, bit 6 (MS) must be 1 to increment address
    start_reg |= LPS25HB_READ | LPS25HB_ADDR_INC;

    lps25hb_hal_select(bar);
    HAL_SPI_Transmit(bar->hspi, &start_reg, 1, LPS25HB_SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, data, dlen, LPS25HB_SPI_TIMEOUT_MS);
    lps25hb_hal_deselect(bar);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_hal_verify_id(const lps25hb_handle_t * bar)
{
    // wait until communication is successful, or exit if too many failed attempts occur...
    uint8_t retries = 0;
    uint8_t who_am_i;

    do
    {
        if (retries == LPS25HB_SPI_MAX_RETRIES) // exit after too many failed attempts
            return LPS25HB_TIMEOUT;
        else if (retries)                       // wait a short amount of time after a failed attempt
            bar->delay_ms(10);

        lps25hb_status_t status = lps25hb_hal_read_reg(bar, LPS25HB_REG_WHO_AM_I, &who_am_i);

        if (status != LPS25HB_OK)
            return status;

        retries++;

    } while (who_am_i != LPS25HB_ID);

    return LPS25HB_OK;
}