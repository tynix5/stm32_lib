#include "iis2mdc.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

/********************** Helper Functions *******************/
/* Configure output data rate */
static ErrorType mag_set_odr(MagParams * mag);
/* Configure low pass filter */
static ErrorType mag_set_filt(MagParams * mag);
/* Configure interrupts */
static ErrorType mag_set_it(MagParams * mag);
/* Enable/Disable low-power mode */
static ErrorType mag_set_power(MagParams * mag);
/***********************************************************/

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
    while (mag_read_reg(mag, MAG_REG_WHO_AM_I) != MAG_ID)
    {
        // exit if maximum retries reached
        if (retries == SPI_MAX_RETRIES)
        return RET_TIMEOUT;
        
        retries++;
        mag->delay_ms(10);
    }
    
    if (mag_set_odr(mag) != RET_OK)
        return RET_INVALID_CFG;

    if (mag_set_power(mag) != RET_OK)
        return RET_INVALID_CFG;

    if (mag_set_filt(mag) != RET_OK)
        return RET_INVALID_CFG;

    if (mag_set_it(mag) != RET_OK)
        return RET_INVALID_CFG;

    // enable temperature compensation to access TEMP registers
    // AN5080
    uint8_t cfg_reg_a = mag_read_reg(mag, MAG_REG_CFG_REG_A);
    cfg_reg_a |= MAG_BIT_COMP_TEMP_EN;
    mag_write_reg(mag, MAG_REG_CFG_REG_A, cfg_reg_a);
    
    // block updates and disable I2C interface
    uint8_t cfg_reg_c = mag_read_reg(mag, MAG_REG_CFG_REG_C);
    cfg_reg_c |= MAG_BIT_BDU | MAG_BIT_I2C_DIS;
    mag_write_reg(mag, MAG_REG_CFG_REG_C, cfg_reg_c);
            
    return RET_OK;
}

ErrorType mag_is_data_ready(MagParams * mag, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    // wait for new data available by polling status register
    while (!(mag_status(mag) & MAG_BIT_ZYXDA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
        return RET_TIMEOUT;
    }
    
    return RET_OK;
}

ErrorType mag_read_raw(MagParams * mag, int16_t * magx_lsb, int16_t * magy_lsb, int16_t * magz_lsb)
{
    uint8_t m[6];
    mag_read(mag, MAG_REG_OUTX_L_REG, 6, m);
    
    // OUT_L_REG | (OUT_H_REG << 8)
    *magx_lsb = m[0] | (m[1] << 8);
    *magy_lsb = m[2] | (m[3] << 8);
    *magz_lsb = m[4] | (m[5] << 8);
    
    return RET_OK;
}

ErrorType mag_read_mgauss(MagParams * mag, float * magx, float * magy, float * magz)
{
    int16_t x, y, z;
    mag_read_raw(mag, &x, &y, &z);

    // convert from LSB to mgauss
    *magx = (float) x * MAG_MILLIGAUSS_SCALING_FACTOR;
    *magy = (float) y * MAG_MILLIGAUSS_SCALING_FACTOR;
    *magz = (float) z * MAG_MILLIGAUSS_SCALING_FACTOR;
    
    return RET_OK;
}

ErrorType mag_read_temp(MagParams * mag, float * temp_c)
{
    uint8_t t[2];
    mag_read(mag, MAG_REG_TEMP_OUT_L_REG, 2, t);

    // Figure 8.16 datasheet
    int16_t temp_lsb = (t[1] << 8) | t[0];
    *temp_c = MAG_TEMP_OFFSET + (float) temp_lsb * MAG_TEMP_SCALING_FACTOR;

    return RET_OK;
}

/********************** Helper Functions *******************/
static ErrorType mag_set_odr(MagParams * mag)
{
    MagOdr odr = mag->odr;
    MagMode mode = mag->mode;

    // check for invalid configs
    if (odr > MAG_ODR_100HZ || mode > MAG_MODE_IDLE)
        return RET_INVALID_CFG;

    // set sampling mode and ODR
    uint8_t cfg_reg_a = mag_read_reg(mag, MAG_REG_CFG_REG_A);
    cfg_reg_a = (cfg_reg_a & 0xf0) | (odr << MAG_BIT_ODR_SHIFT) | mode;
    mag_write_reg(mag, MAG_REG_CFG_REG_A, cfg_reg_a);

    return RET_OK;
}

static ErrorType mag_set_filt(MagParams * mag)
{
    uint8_t cfg_reg_b = mag_read_reg(mag, MAG_REG_CFG_REG_B);

    // enable low-pass filter
    // if enabled, BW = ODR/4
    // if disabled BW = ODR/2
    if (mag->lpf == FEAT_ENABLE)
        cfg_reg_b |= MAG_BIT_LPF;
    else
        cfg_reg_b &= ~MAG_BIT_LPF;

    mag_write_reg(mag, MAG_REG_CFG_REG_B, cfg_reg_b);

    return RET_OK;
}

static ErrorType mag_set_it(MagParams * mag)
{
    MagIntDrdy type = mag->IntParams.int_drdy;
    
    if (type > MAG_DRDY_ON_PIN)
        return RET_INVALID_CFG;

    uint8_t cfg_reg_c = mag_read_reg(mag, MAG_REG_CFG_REG_C);

    // select which interface goes to pin
    if (type == MAG_INT_ON_PIN)
    {
        cfg_reg_c |= MAG_BIT_INT_ON_PIN;
        cfg_reg_c &= ~MAG_BIT_DRDY_ON_PIN;

        // select interrupt detections, latched values, and polarity
        mag_write_reg(mag, MAG_REG_INT_CTRL_REG, mag->IntParams.it);
    }
    else if (type == MAG_DRDY_ON_PIN)
    {
        // interrupt on pin when data-ready signal (Zyxda bit is driven on INT/DRDY pin)
        cfg_reg_c &= ~MAG_BIT_INT_ON_PIN;
        cfg_reg_c |= MAG_BIT_DRDY_ON_PIN;
    }
    else
    {
        // disable both interrupt sources
        cfg_reg_c &= ~MAG_BIT_INT_ON_PIN;
        cfg_reg_c &= ~MAG_BIT_DRDY_ON_PIN;
    }

    mag_write_reg(mag, MAG_REG_CFG_REG_C, cfg_reg_c);

    return RET_OK;
}

static ErrorType mag_set_power(MagParams * mag)
{
    if (mag->low_pwr > FEAT_ENABLE)
        return RET_INVALID_CFG;

    uint8_t cfg_reg_a = mag_read_reg(mag, MAG_REG_CFG_REG_A);

    // enable/disable low-power mode
    if (mag->low_pwr == FEAT_ENABLE)
        cfg_reg_a |= MAG_BIT_LP;
    else
        cfg_reg_a &= ~MAG_BIT_LP;
    
    mag_write_reg(mag, MAG_REG_CFG_REG_A, cfg_reg_a);

    return RET_OK;
}
/***********************************************************/

/******************* SPI Helper Functions ******************/
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
    return mag_read_reg(mag, MAG_REG_STATUS_REG);
}
/***********************************************************/