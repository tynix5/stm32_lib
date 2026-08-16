#include <stdint.h>
#include <limits.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr.h"

/**************************************** Helper Functions *******************************************/
/* Software reset */
static status_t imu_sw_reset(lsm6_handle_t * imu);
/* Reboot memory content */
static status_t imu_reboot(lsm6_handle_t * imu);
/* Configure interrupts for accelerometer and gyroscope */
static status_t imu_set_it(lsm6_handle_t * imu); // needs implementation
/* Initialize internal gyroscope in IMU */
static status_t gyro_init(lsm6_handle_t * imu);
/* Initialize internal accelerometer in IMU */
static status_t accel_init(lsm6_handle_t * imu);
/* Configure internal FIFOs */
static status_t imu_set_fifo_mode(lsm6_handle_t * imu); // needs further implementation to support different modes
/* Set ODR for gyroscope */
static status_t gyro_set_odr(lsm6_handle_t * imu);
/* Set ODR for accelerometer */
static status_t accel_set_odr(lsm6_handle_t * imu);
/* Set full-scale range for gyroscope */
static status_t gyro_set_fs(lsm6_handle_t * imu);
/* Set full-scale range for accelerometer */
static status_t accel_set_fs(lsm6_handle_t * imu);
/*****************************************************************************************************/

/************************************* SPI Wrapper Functions *****************************************/
/* CS low */
static inline void imu_select(lsm6_handle_t * imu);
/* CS high */
static inline void imu_deselect(lsm6_handle_t * imu);
/* Write byte to selected register over SPI */
static void imu_write_reg(lsm6_handle_t * imu, uint8_t addr, uint8_t data);
/* Read byte from selected register over SPI */
static uint8_t imu_read_reg(lsm6_handle_t * imu, uint8_t addr);
/* Multi-byte read starting at start_addr with a length of len bytes over SPI */
static void imu_read(lsm6_handle_t * imu, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Read status register */
static uint8_t imu_status(lsm6_handle_t * imu);
/*****************************************************************************************************/

status_t imu_init(lsm6_handle_t * imu)
{
    imu_deselect(imu);

    // clear configurations
    imu_sw_reset(imu);
    imu_reboot(imu);

    uint8_t retries = 0;
    // wait until communicaiton is successful
    while (imu_read_reg(imu, IMU_REG_WHO_AM_I) != IMU_ID)
    {
        if (retries == SPI_MAX_RETRIES)
            return STATUS_TIMEOUT;

        retries++;
        imu->delay_ms(10);
    }

    if (imu_set_fifo_mode(imu) != STATUS_OK)
        return STATUS_INVALID;

    if (gyro_init(imu) != STATUS_OK)
        return STATUS_INVALID;

    if (accel_init(imu) != STATUS_OK)
        return STATUS_INVALID;

    // configure control register 3
    // normal mode, block update, interrupts active high, push-pull interrupts, 4-wire SPI, register addr auto inc, little endian
    imu_write_reg(imu, IMU_REG_CTRL3_C, IMU_BIT_IF_INC | IMU_BIT_BDU);

    if (imu_set_it(imu) != STATUS_OK)
        return STATUS_INVALID;

    return STATUS_OK;
}

status_t imu_is_accel_data_ready(lsm6_handle_t * imu, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (!(imu_status(imu) & IMU_BIT_STATUS_XLDA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}

status_t imu_is_gyro_data_ready(lsm6_handle_t * imu, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (!(imu_status(imu) & IMU_BIT_STATUS_GDA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}

status_t imu_read_accel_mps2(lsm6_handle_t * imu, float * a_x, float * a_y, float * a_z)
{
    uint8_t a[6];
    imu_read(imu, IMU_REG_OUTX_L_XL, 6, a);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = a[0] | (a[1] << 8); // OUT_L_XL | OUT_H_XL << 8
    y_raw = a[2] | (a[3] << 8);
    z_raw = a[4] | (a[5] << 8);

    // convert LSBs to m/s^2
    uint8_t scale_g;

    // determine full scale range (in g's)
    if (imu->accel_handle_t.fs == IMU_FS_XL_16G)
    {
        scale_g = 16;
    }
    else if (imu->accel_handle_t.fs == IMU_FS_XL_8G)
    {
        scale_g = 8;
    }
    else if (imu->accel_handle_t.fs == IMU_FS_XL_4G)
    {
        scale_g = 4;
    }
    else
    {
        scale_g = 2;
    }

    // convert LSBs to G's, then determine acceleration in m/s^2
    float lsb_to_g = (float)scale_g / SHRT_MAX;

    // LSBs -> G -> M/S^2
    *a_x = (float)x_raw * lsb_to_g * IMU_GRAVITY;
    *a_y = (float)y_raw * lsb_to_g * IMU_GRAVITY;
    *a_z = (float)z_raw * lsb_to_g * IMU_GRAVITY;

    return STATUS_OK;
}

status_t imu_read_accel_raw(lsm6_handle_t * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb)
{
    uint8_t a[6];
    imu_read(imu, IMU_REG_OUTX_L_XL, 6, a);

    // convert low and high bytes into single value
    *a_x_lsb = a[0] | (a[1] << 8); // OUT_L_XL | OUT_H_XL << 8
    *a_y_lsb = a[2] | (a[3] << 8);
    *a_z_lsb = a[4] | (a[5] << 8);

    return STATUS_OK;
}

status_t imu_read_gyro_radps(lsm6_handle_t * imu, float * w_x, float * w_y, float * w_z)
{
    uint8_t w[6];
    imu_read(imu, IMU_REG_OUTX_L_G, 6, w);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    if (imu->gyro_handle_t.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->gyro_handle_t.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->gyro_handle_t.fs == IMU_FS_G_500DPS)
    {
        scale_dps = 500;
    }
    else
    {
        scale_dps = 245;
    }

    float lsb_to_dps = (float)scale_dps / SHRT_MAX;

    // LSBs -> DEG/S -> RAD/S
    *w_x = (float)x_raw * lsb_to_dps * IMU_DEG_TO_RAD;
    *w_y = (float)y_raw * lsb_to_dps * IMU_DEG_TO_RAD;
    *w_z = (float)z_raw * lsb_to_dps * IMU_DEG_TO_RAD;

    return STATUS_OK;
}

status_t imu_read_gyro_degps(lsm6_handle_t * imu, float * w_x, float * w_y, float * w_z)
{
    uint8_t w[6];
    imu_read(imu, IMU_REG_OUTX_L_G, 6, w);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    if (imu->gyro_handle_t.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->gyro_handle_t.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->gyro_handle_t.fs == IMU_FS_G_500DPS)
    {
        scale_dps = 500;
    }
    else
    {
        scale_dps = 245;
    }

    float lsb_to_dps = (float)scale_dps / SHRT_MAX;

    // LSBs -> DEG/S
    *w_x = (float)x_raw * lsb_to_dps;
    *w_y = (float)y_raw * lsb_to_dps;
    *w_z = (float)z_raw * lsb_to_dps;

    return STATUS_OK;
}

status_t imu_read_gyro_raw(lsm6_handle_t * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb)
{
    uint8_t w[6];
    imu_read(imu, IMU_REG_OUTX_L_G, 6, w);

    // convert low and high bytes into single value
    *w_x_lsb = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    *w_y_lsb = w[2] | (w[3] << 8);
    *w_z_lsb = w[4] | (w[5] << 8);

    return STATUS_OK;
}

status_t gyro_powerdown(lsm6_handle_t * imu)
{
    // turn off gyroscope by clearing all ODR bits
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_REG_CTRL2_G);
    ctrl2_g &= 0x0f; // turn off all ODR bits
    imu_write_reg(imu, IMU_REG_CTRL2_G, ctrl2_g);

    return STATUS_OK;
}

status_t accel_powerdown(lsm6_handle_t * imu)
{
    // turn off accelerometer by clearing all ODR bits
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_REG_CTRL1_XL);
    ctrl1_xl &= 0x0f; // turn off all ODR bits
    imu_write_reg(imu, IMU_REG_CTRL1_XL, ctrl1_xl);

    return STATUS_OK;
}

status_t gyro_powerup(lsm6_handle_t * imu)
{
    return gyro_set_odr(imu);
}

status_t accel_powerup(lsm6_handle_t * imu)
{
    return accel_set_odr(imu);
}

/**************************************** Helper Functions *******************************************/
static status_t imu_sw_reset(lsm6_handle_t * imu)
{
    // AN5130 section 5.7
    // 1. Power down gyroscope
    gyro_powerdown(imu);

    // 2. Set accelerometer in High Performance mode
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_REG_CTRL6_C);
    ctrl6_c &= ~IMU_BIT_XL_HM_MODE;
    imu_write_reg(imu, IMU_REG_CTRL6_C, ctrl6_c);

    // 3. Set SW_RESET bit
    uint8_t ctrl3_c = imu_read_reg(imu, IMU_REG_CTRL3_C);
    ctrl3_c |= IMU_BIT_SW_RESET;
    imu_write_reg(imu, IMU_REG_CTRL3_C, ctrl3_c);

    // 4. wait for software reset bit to clear
    while (imu_read_reg(imu, IMU_REG_CTRL3_C) & IMU_BIT_SW_RESET)
        ;

    return STATUS_OK;
}

static status_t imu_reboot(lsm6_handle_t * imu)
{
    // AN5130 section 5.7
    // 1. Power down gyroscope
    gyro_powerdown(imu);

    // 2. Set accelerometer in High Performance mode
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_REG_CTRL6_C);
    ctrl6_c &= ~IMU_BIT_XL_HM_MODE;
    imu_write_reg(imu, IMU_REG_CTRL6_C, ctrl6_c);

    // 3. Set boot bit
    uint8_t ctrl3_c = imu_read_reg(imu, IMU_REG_CTRL3_C);
    ctrl3_c |= IMU_BIT_BOOT;
    imu_write_reg(imu, IMU_REG_CTRL3_C, ctrl3_c);

    // 4. Wait 15ms
    imu->delay_ms(15);

    return STATUS_OK;
}

static status_t imu_set_it(lsm6_handle_t * imu)
{
    imu_write_reg(imu, IMU_REG_INT1_CTRL, imu->int_handle_t.int1); // INT1
    imu_write_reg(imu, IMU_REG_INT2_CTRL, imu->int_handle_t.int2); // INT2

    return STATUS_OK;
}

static status_t gyro_init(lsm6_handle_t * imu)
{
    if (gyro_set_odr(imu) != STATUS_OK)
        return STATUS_INVALID;

    if (gyro_set_fs(imu) != STATUS_OK)
        return STATUS_INVALID;

    // configure gyroscope
    uint8_t ctrl4_c = imu_read_reg(imu, IMU_REG_CTRL4_C);
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_REG_CTRL6_C);
    uint8_t ctrl7_g = imu_read_reg(imu, IMU_REG_CTRL7_G);

    // if using filter, high performance mode must be enabled
    if (imu->gyro_handle_t.filt != IMU_NO_FILTER)
        ctrl7_g &= ~IMU_BIT_G_HM_MODE;

    // configure LPF and HPF
    if (imu->gyro_handle_t.filt == IMU_BPF_EN || imu->gyro_handle_t.filt == IMU_LPF_EN)
    {
        ctrl4_c |= IMU_BIT_LPF1_SEL_G;  // enable lowpass filter
        ctrl6_c &= ~IMU_BIT_FTYPE_MASK; // set bandwith to medium value
    }
    else
    {
        ctrl4_c &= ~IMU_BIT_LPF1_SEL_G; // disable lowpass filter
    }

    if (imu->gyro_handle_t.filt == IMU_BPF_EN || imu->gyro_handle_t.filt == IMU_HPF_EN)
    {
        ctrl7_g |= IMU_BIT_HP_EN_G; // enable digital high pass filter

        // select cutoff frequency
        if (imu->gyro_handle_t.cutoff <= IMU_HP_G_1_04HZ)
            ctrl7_g |= imu->gyro_handle_t.cutoff << IMU_BIT_HPM_G_SHIFT;
        else
            return STATUS_INVALID;
    }
    else
    {
        ctrl7_g &= ~IMU_BIT_HP_EN_G; // disable highpass filter
    }

    imu_write_reg(imu, IMU_REG_CTRL4_C, ctrl4_c | IMU_BIT_I2C_DISABLE);
    imu_write_reg(imu, IMU_REG_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_REG_CTRL7_G, ctrl7_g);

    return STATUS_OK;
}

static status_t accel_init(lsm6_handle_t * imu)
{
    if (accel_set_odr(imu) != STATUS_OK)
        return STATUS_INVALID;

    if (accel_set_fs(imu) != STATUS_OK)
        return STATUS_INVALID;

    // configure accelerometer
    // odr should be configured as 2x gyro odr to match odr rates
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_REG_CTRL6_C);
    uint8_t ctrl8_xl = imu_read_reg(imu, IMU_REG_CTRL8_XL);

    // bandpass filter mode not allowed
    if (imu->accel_handle_t.filt == IMU_BPF_EN)
        return STATUS_INVALID;

    // if any filters are enabled, high performance mode must be on
    if (imu->accel_handle_t.filt != IMU_NO_FILTER)
        ctrl6_c &= ~IMU_BIT_XL_HM_MODE;

    if (imu->accel_handle_t.filt == IMU_LPF_EN)
    {
        ctrl8_xl &= ~IMU_BIT_HP_SLOPE_XL_EN;                // disable highpass filter
        ctrl8_xl |= IMU_BIT_LPF2_XL_EN;                     // enable lowpass filter
        ctrl8_xl &= ~(IMU_BIT_HPCF_XL1 | IMU_BIT_HPCF_XL0); // ODR/50
    }
    else if (imu->accel_handle_t.filt == IMU_HPF_EN)
    {
        ctrl8_xl |= IMU_BIT_HP_SLOPE_XL_EN;                 // enable highpass filter
        ctrl8_xl &= ~IMU_BIT_LPF2_XL_EN;                    // disable lowpass filter
        ctrl8_xl &= ~(IMU_BIT_HPCF_XL1 | IMU_BIT_HPCF_XL0); // ODR/4
    }
    else
    {
        ctrl8_xl &= ~IMU_BIT_HP_SLOPE_XL_EN; // disable highpass filter
        ctrl8_xl &= ~IMU_BIT_LPF2_XL_EN;     // disable lowpass filter
    }

    imu_write_reg(imu, IMU_REG_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_REG_CTRL8_XL, ctrl8_xl);
    imu_write_reg(imu, IMU_REG_CTRL9_XL, 0);

    return STATUS_OK;
}

static status_t imu_set_fifo_mode(lsm6_handle_t * imu)
{
    lsm6_fifo_odr_t fifo_odr = imu->fifo_handle_t.odr;
    lsm6_gyro_odr_t gyro_odr = imu->gyro_handle_t.odr;
    lsm6_accel_odr_t xl_odr = imu->accel_handle_t.odr;

    lsm6_fifo_mode_t fifo_mode = imu->fifo_handle_t.mode;

    // check for invalid configs
    if (fifo_odr > IMU_ODR_FIFO_6_66KHZ || xl_odr > IMU_ODR_XL_1_6HZ || gyro_odr > IMU_ODR_G_6_66KHZ)
        return STATUS_INVALID;

    if (fifo_mode > IMU_FIFO_CONT)
        return STATUS_INVALID;

    // FIFO ODR must be less than or equal to accelerometer ODR and gyro ODR
    if (((uint8_t)fifo_odr > (uint8_t)xl_odr || (uint8_t)fifo_odr > (uint8_t)gyro_odr) && (xl_odr != IMU_ODR_XL_1_6HZ))
        return STATUS_INVALID;

    uint8_t fifo_ctrl5 = (fifo_odr << IMU_BIT_FIFO_ODR_SHIFT) | (fifo_mode << IMU_BIT_FIFO_MODE_SHIFT);
    imu_write_reg(imu, IMU_REG_FIFO_CTRL5, fifo_ctrl5);

    return STATUS_OK;
}

static status_t gyro_set_odr(lsm6_handle_t * imu)
{
    lsm6_gyro_odr_t odr = imu->gyro_handle_t.odr;
    lsm6_filter_mode_t filt = imu->gyro_handle_t.filt;

    uint8_t ctrl7_g = imu_read_reg(imu, IMU_REG_CTRL7_G);

    if (odr > IMU_ODR_G_6_66KHZ)
        return STATUS_INVALID;

    // if filter is selected, must be high performance mode
    if (filt != IMU_NO_FILTER)
    {
        ctrl7_g &= ~IMU_BIT_G_HM_MODE; // high performance mode
    }
    else
    {
        if (odr >= IMU_ODR_G_12_5HZ && odr <= IMU_ODR_G_208HZ)
            ctrl7_g |= IMU_BIT_G_HM_MODE;  // disable high performance mode
        else
            ctrl7_g &= ~IMU_BIT_G_HM_MODE; // high performance mode
    }

    imu_write_reg(imu, IMU_REG_CTRL7_G, ctrl7_g);

    // only modify ODR bits
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_REG_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0x0f) | (odr << IMU_BIT_ODR_G_SHIFT);
    imu_write_reg(imu, IMU_REG_CTRL2_G, ctrl2_g);

    return STATUS_OK;
}

static status_t accel_set_odr(lsm6_handle_t * imu)
{
    lsm6_accel_odr_t odr = imu->accel_handle_t.odr;
    lsm6_filter_mode_t filt = imu->gyro_handle_t.filt;

    uint8_t ctrl6_c = imu_read_reg(imu, IMU_REG_CTRL6_C);

    if (odr >= IMU_ODR_XL_1_6HZ)
        return STATUS_INVALID;

    // if filter is selected, must be high performance mode
    if (filt != IMU_NO_FILTER)
    {
        ctrl6_c &= ~IMU_BIT_XL_HM_MODE; // high performance mode
    }
    else
    {
        if ((odr >= IMU_ODR_XL_12_5HZ && odr <= IMU_ODR_XL_208HZ) || odr == IMU_ODR_XL_1_6HZ)
            ctrl6_c |= IMU_BIT_XL_HM_MODE;  // disable high peformance mode
        else
            ctrl6_c &= ~IMU_BIT_XL_HM_MODE; // high performance mode
    }

    imu_write_reg(imu, IMU_REG_CTRL6_C, ctrl6_c);

    // only modify ODR bits
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_REG_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x0f) | (odr << IMU_BIT_ODR_XL_SHIFT);
    imu_write_reg(imu, IMU_REG_CTRL1_XL, ctrl1_xl);

    return STATUS_OK;
}

static status_t gyro_set_fs(lsm6_handle_t * imu)
{
    lsm6_gyro_fullscale_t fs = imu->gyro_handle_t.fs;

    if (fs > IMU_FS_G_2000DPS)
        return STATUS_INVALID;

    uint8_t ctrl2_g = imu_read_reg(imu, IMU_REG_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0xf3) | (fs << IMU_BIT_FS_G_SHIFT);
    imu_write_reg(imu, IMU_REG_CTRL2_G, ctrl2_g);

    return STATUS_OK;
}

static status_t accel_set_fs(lsm6_handle_t * imu)
{
    lsm6_accel_fullscale_t fs = imu->accel_handle_t.fs;

    if (fs > IMU_FS_XL_8G)
        return STATUS_INVALID;

    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_REG_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0xf3) | (fs << IMU_BIT_FS_XL_SHIFT);
    imu_write_reg(imu, IMU_REG_CTRL1_XL, ctrl1_xl);

    return STATUS_OK;
}
/*****************************************************************************************************/

/************************************* SPI Wrapper Functions *****************************************/
static inline void imu_select(lsm6_handle_t * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_RESET);
}

static inline void imu_deselect(lsm6_handle_t * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_SET);
}

static void imu_write_reg(lsm6_handle_t * imu, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = { addr | IMU_WRITE, data };

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, cmd, 2, SPI_TIMEOUT_MS);
    imu_deselect(imu);
}

static uint8_t imu_read_reg(lsm6_handle_t * imu, uint8_t addr)
{
    uint8_t result;
    addr |= IMU_READ;

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, &result, 1, SPI_TIMEOUT_MS);
    imu_deselect(imu);

    return result;
}

static void imu_read(lsm6_handle_t * imu, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    // read multiple byte burst from IMU
    // IF_INC must be enabled in CTRL3_C reg
    start_addr |= IMU_READ;

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &start_addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, data, len, SPI_TIMEOUT_MS);
    imu_deselect(imu);
}

static uint8_t imu_status(lsm6_handle_t * imu)
{
    return imu_read_reg(imu, IMU_REG_STATUS);
}
/*****************************************************************************************************/