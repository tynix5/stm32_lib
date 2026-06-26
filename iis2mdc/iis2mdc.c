#include "iis2mdc.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

/******************* SPI Helper Functions ******************/
static inline void mag_select(MagParams * mag);
static inline void mag_deselect(MagParams * mag);
static void mag_write_reg(MagParams * mag, uint8_t addr, uint8_t data);
static uint8_t mag_read_reg(MagParams * mag, uint8_t addr);
static void mag_read(MagParams * mag, uint8_t start_addr, uint8_t len, uint8_t * data);
static uint8_t mag_status(MagParams * mag);
/***********************************************************/

ErrorType mag_init(MagParams * mag)
{

}

ErrorType mag_is_data_ready(MagParams * mag)
{

}

ErrorType mag_read_raw(MagParams * mag, int16_t * magx_lsb, int16_t * magy_lsb, int16_t * magz_lsb)
{

}

ErrorType mag_read_gauss(MagParams * mag, float * magx, float * magy, float * magz)
{

}

ErrorType mag_powerup(MagParams * mag)
{

}

ErrorType mag_powerdown(MagParams * mag)
{

}

static inline void mag_select(MagParams * mag)
{
    HAL_GPIO_WritePin(mag->cs_gpio, mag->cs, GPIO_PIN_RESET);
}

static inline void mag_deselect(MagParams * mag)
{
    HAL_GPIO_WritePin(mag->cs_gpio, mag->cs, GPIO_PIN_SET);
}

static void mag_write_reg(MagParams * mag, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = {addr | MAG_WRITE, data};

    mag_select(mag);
    HAL_SPI_Transmit(mag->hspi, cmd, 2, HAL_TIMEOUT);
    mag_deselect(mag);
}

static uint8_t mag_read_reg(MagParams * mag, uint8_t addr)
{

    addr |= MAG_READ;
    
    uint8_t data;

    mag_select(mag);
    HAL_SPI_Transmit(mag->hspi, &addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(mag->hspi, &data, 1, HAL_TIMEOUT);
    mag_deselect(mag);

    return data;
}

static void mag_read(MagParams * mag, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    start_addr |= MAG_READ;

    mag_select(mag);
    HAL_SPI_Transmit(mag->hspi, &start_addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(mag->hspi, data, len, HAL_TIMEOUT);
    mag_deselect(mag);
}

static uint8_t mag_status(MagParams * mag)
{
    return mag_read_reg(mag, MAG_STATUS_REG);
}