#include "iis2mdc.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

/******************* SPI Helper Functions ******************/
/* CS low */
static inline void mag_select(MagParams * mag);
/* CS high */
static inline void mag_deselect(MagParams * mag);
/* Write one byte to selected magnetometer register */
static void mag_write_reg(MagParams * mag, uint8_t addr, uint8_t data);
/* Read one byte from selected register */
static uint8_t mag_read_reg(MagParams * mag, uint8_t addr);
/* Read multiple bytes starting from selected register and auto-increment address */
static void mag_read(MagParams * mag, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Return status register */
static uint8_t mag_status(MagParams * mag);
/***********************************************************/

ErrorType mag_init(MagParams * mag)
{
    mag_deselect(mag);

    // wait for power up
    mag->delay_ms(10);

    uint8_t retries = 0;
    // wait until connection to magnetometer is present
    while (mag_read_reg(mag, MAG_WHO_AM_I) != MAG_ID)
    {
        // exit if maximum retries reached
        if (retries == SPI_MAX_RETRIES)
            return TIMEOUT;
        
        retries++;
        mag->delay_ms(10);
    }
        

    // set output data rate and mode
    uint8_t cfg_a = mag->odr | mag->mode;
    mag_write_reg(mag, MAG_CFG_REG_A, cfg_a);

    
    // enable/disable lowpass filter
    uint8_t cfg_b = 0;
    
    if (mag->lpf == ENABLE)         cfg_b |= MAG_LPF;
    else                            cfg_b &= ~MAG_LPF;
    
    mag_write_reg(mag, MAG_CFG_REG_B, cfg_b);
    
    // disable I2C; optionally select interrupt modes
    uint8_t cfg_c = MAG_I2C_DIS;
    
    if (mag->int_drdy == ENABLE)    cfg_c |= MAG_DRDY_ON_PIN;
    else                            cfg_c &= ~MAG_DRDY_ON_PIN;
    
    mag_write_reg(mag, MAG_CFG_REG_C, cfg_c);
    
    // enable/disable specific interrupts
    return SUCCESS;
}

ErrorType mag_is_data_ready(MagParams * mag, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    // wait for new data available by polling status register
    while (!(mag_status(mag) & MAG_ZYXDA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return TIMEOUT;
    }

    return SUCCESS;
}

ErrorType mag_read_raw(MagParams * mag, int16_t * magx_lsb, int16_t * magy_lsb, int16_t * magz_lsb)
{
    uint8_t regs[6];
    mag_read(mag, MAG_OUTX_L_REG, 6, regs);

    // OUT_L_REG | (OUT_H_REG << 8)
    *magx_lsb = regs[0] | (regs[1] << 8);
    *magy_lsb = regs[2] | (regs[3] << 8);
    *magz_lsb = regs[4] | (regs[5] << 8);

    return SUCCESS;
}

ErrorType mag_read_gauss(MagParams * mag, float * magx, float * magy, float * magz)
{
    int16_t x, y, z;
    mag_read_raw(mag, &x, &y, &z);

    return SUCCESS;
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
    HAL_SPI_Transmit(mag->hspi, cmd, 2, SPI_TIMEOUT_MS);
    mag_deselect(mag);
}

static uint8_t mag_read_reg(MagParams * mag, uint8_t addr)
{
    uint8_t data;
    addr |= MAG_READ;

    mag_select(mag);
    HAL_SPI_Transmit(mag->hspi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(mag->hspi, &data, 1, SPI_TIMEOUT_MS);
    mag_deselect(mag);

    return data;
}

static void mag_read(MagParams * mag, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    start_addr |= MAG_READ;

    mag_select(mag);
    HAL_SPI_Transmit(mag->hspi, &start_addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(mag->hspi, data, len, SPI_TIMEOUT_MS);
    mag_deselect(mag);
}

static uint8_t mag_status(MagParams * mag)
{
    return mag_read_reg(mag, MAG_STATUS_REG);
}