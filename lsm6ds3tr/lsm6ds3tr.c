#include <stdint.h>
#include <limits.h>
#include "lsm6ds3tr_hal.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr.h"

/************************************************************* Helper Functions ***************************************************************/
/* Software reset */
/* [input] imu - pointer to LSM6DS3TR structure handle */
static lsm6ds3tr_status_t lsm6ds3tr_sw_reset(const lsm6ds3tr_handle_t * imu);

/* Reboot memory content */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_reboot(const lsm6ds3tr_handle_t * imu);

/* Configure interrupts for accelerometer and gyroscope */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_set_it(const lsm6ds3tr_handle_t * imu);

/* Initialize internal gyroscope in IMU */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_gyro_init(const lsm6ds3tr_handle_t * imu);

/* Initialize internal accelerometer in IMU */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_accel_init(const lsm6ds3tr_handle_t * imu);

/* Configure internal FIFOs */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_set_fifo_mode(const lsm6ds3tr_handle_t * imu);

/* Set ODR for gyroscope */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_gyro_set_odr(const lsm6ds3tr_handle_t * imu);

/* Set ODR for accelerometer */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_accel_set_odr(const lsm6ds3tr_handle_t * imu);

/* Set full-scale range for gyroscope */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_gyro_set_fs(const lsm6ds3tr_handle_t * imu);

/* Set full-scale range for accelerometer */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
static lsm6ds3tr_status_t lsm6ds3tr_accel_set_fs(const lsm6ds3tr_handle_t * imu);
/**********************************************************************************************************************************************/

lsm6ds3tr_status_t lsm6ds3tr_init(lsm6ds3tr_handle_t * imu)
{
    if (imu->delay_ms == NULL || imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    lsm6ds3tr_deselect(imu);

    // clear configurations
    lsm6ds3tr_sw_reset(imu);
    lsm6ds3tr_reboot(imu);

    // wait until communication is successful, or exit
    lsm6ds3tr_status_t verify_id = lsm6ds3tr_hal_verify_id(imu);
    if (verify_id != LSM6DS3TR_OK)
        return verify_id;

    lsm6ds3tr_status_t fifo_status = lsm6ds3tr_set_fifo_mode(imu);
    if (fifo_status != LSM6DS3TR_OK)
        return fifo_status;

    lsm6ds3tr_status_t gyro_status = lsm6ds3tr_gyro_init(imu);
    if (gyro_status != LSM6DS3TR_OK)
        return gyro_status;

    lsm6ds3tr_status_t accel_status = lsm6ds3tr_accel_init(imu);
    if (accel_status != LSM6DS3TR_OK)
        return accel_status;

    // configure control register 3
    // normal mode, block update, interrupts active high, push-pull interrupts, 4-wire SPI, register addr auto inc, little endian
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL3_C, LSM6DS3TR_BIT_IF_INC | LSM6DS3TR_BIT_BDU);

    lsm6ds3tr_status_t it_status = lsm6ds3tr_set_it(imu);
    if (it_status != LSM6DS3TR_OK)
        return it_status;

    imu->_initialized = true;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_is_accel_data_ready(const lsm6ds3tr_handle_t * imu, bool * drdy)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t status_reg;
    lsm6ds3tr_status_t status;

    status = lsm6ds3tr_read_status(imu, &status_reg);

    if (status != LSM6DS3TR_OK)
        return status;

    if (status_reg & LSM6DS3TR_BIT_STATUS_XLDA)
        *drdy = true;
    else
        *drdy = false;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_is_gyro_data_ready(const lsm6ds3tr_handle_t * imu, bool * drdy)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t status_reg;
    lsm6ds3tr_status_t status;

    status = lsm6ds3tr_read_status(imu, &status_reg);

    if (status != LSM6DS3TR_OK)
        return status;

    if (status_reg & LSM6DS3TR_BIT_STATUS_GDA)
        *drdy = true;
    else
        *drdy = false;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_accel_mps2(const lsm6ds3tr_handle_t * imu, float * a_x, float * a_y, float * a_z)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t a[6];
    lsm6ds3tr_status_t status = lsm6ds3tr_hal_read(imu, LSM6DS3TR_REG_OUTX_L_XL, 6, a);

    if (status != LSM6DS3TR_OK)
        return status;

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = a[0] | (a[1] << 8); // OUT_L_XL | OUT_H_XL << 8
    y_raw = a[2] | (a[3] << 8);
    z_raw = a[4] | (a[5] << 8);

    // convert LSBs to m/s^2
    uint8_t scale_g;

    // determine full scale range (in g's)
    switch (imu->accel_handle_t.fs)
    {
    case LSM6DS3TR_FS_XL_2G:
        scale_g = 2;
        break;
    case LSM6DS3TR_FS_XL_4G:
        scale_g = 4;
        break;
    case LSM6DS3TR_FS_XL_8G:
        scale_g = 8;
        break;
    case LSM6DS3TR_FS_XL_16G:
        scale_g = 16;
        break;
    default:
        return LSM6DS3TR_INVALID_CFG;
    }

    // convert LSBs to G's, then determine acceleration in m/s^2
    float lsb_to_g = (float)scale_g / SHRT_MAX;

    // LSBs -> G -> M/S^2
    *a_x = (float)x_raw * lsb_to_g * LSM6DS3TR_CONST_G;
    *a_y = (float)y_raw * lsb_to_g * LSM6DS3TR_CONST_G;
    *a_z = (float)z_raw * lsb_to_g * LSM6DS3TR_CONST_G;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_accel_raw(const lsm6ds3tr_handle_t * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t a[6];
    lsm6ds3tr_status_t status = lsm6ds3tr_hal_read(imu, LSM6DS3TR_REG_OUTX_L_XL, 6, a);

    if (status != LSM6DS3TR_OK)
        return status;

    // convert low and high bytes into single value
    *a_x_lsb = a[0] | (a[1] << 8); // OUT_L_XL | OUT_H_XL << 8
    *a_y_lsb = a[2] | (a[3] << 8);
    *a_z_lsb = a[4] | (a[5] << 8);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_gyro_radps(const lsm6ds3tr_handle_t * imu, float * w_x, float * w_y, float * w_z)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t w[6];
    lsm6ds3tr_status_t status = lsm6ds3tr_hal_read(imu, LSM6DS3TR_REG_OUTX_L_G, 6, w);

    if (status != LSM6DS3TR_OK)
        return status;

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    switch (imu->gyro_handle_t.fs)
    {
    case LSM6DS3TR_FS_G_245DPS:
        scale_dps = 245;
        break;
    case LSM6DS3TR_FS_G_500DPS:
        scale_dps = 500;
        break;
    case LSM6DS3TR_FS_G_1000DPS:
        scale_dps = 1000;
        break;
    case LSM6DS3TR_FS_G_2000DPS:
        scale_dps = 2000;
        break;
    default:
        return LSM6DS3TR_INVALID_CFG;
    }

    float lsb_to_dps = (float)scale_dps / SHRT_MAX;

    // LSBs -> DEG/S -> RAD/S
    *w_x = (float)x_raw * lsb_to_dps * LSM6DS3TR_DEG_TO_RAD;
    *w_y = (float)y_raw * lsb_to_dps * LSM6DS3TR_DEG_TO_RAD;
    *w_z = (float)z_raw * lsb_to_dps * LSM6DS3TR_DEG_TO_RAD;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_gyro_degps(const lsm6ds3tr_handle_t * imu, float * w_x, float * w_y, float * w_z)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t w[6];
    lsm6ds3tr_status_t status = lsm6ds3tr_hal_read(imu, LSM6DS3TR_REG_OUTX_L_G, 6, w);

    if (status != LSM6DS3TR_OK)
        return status;

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    switch (imu->gyro_handle_t.fs)
    {
    case LSM6DS3TR_FS_G_245DPS:
        scale_dps = 245;
        break;
    case LSM6DS3TR_FS_G_500DPS:
        scale_dps = 500;
        break;
    case LSM6DS3TR_FS_G_1000DPS:
        scale_dps = 1000;
        break;
    case LSM6DS3TR_FS_G_2000DPS:
        scale_dps = 2000;
        break;
    default:
        return LSM6DS3TR_INVALID_CFG;
    }

    float lsb_to_dps = (float)scale_dps / SHRT_MAX;

    // LSBs -> DEG/S
    *w_x = (float)x_raw * lsb_to_dps;
    *w_y = (float)y_raw * lsb_to_dps;
    *w_z = (float)z_raw * lsb_to_dps;

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_gyro_raw(const lsm6ds3tr_handle_t * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    if (!imu->_initialized)
        return LSM6DS3TR_NOT_INITIALIZED;

    uint8_t w[6];
    lsm6ds3tr_status_t status = lsm6ds3tr_hal_read(imu, LSM6DS3TR_REG_OUTX_L_G, 6, w);

    if (status != LSM6DS3TR_OK)
        return status;

    // convert low and high bytes into single value
    *w_x_lsb = w[0] | (w[1] << 8); // OUT_L_G | OUT_H_G << 8
    *w_y_lsb = w[2] | (w[3] << 8);
    *w_z_lsb = w[4] | (w[5] << 8);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t gyro_powerdown(const lsm6ds3tr_handle_t * imu)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    // turn off gyroscope by clearing all ODR bits
    uint8_t ctrl2_g;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL2_G, &ctrl2_g);
    ctrl2_g &= 0x0f; // turn off all ODR bits
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL2_G, ctrl2_g);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t accel_powerdown(const lsm6ds3tr_handle_t * imu)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    // turn off accelerometer by clearing all ODR bits
    uint8_t ctrl1_xl;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL1_XL, &ctrl1_xl);
    ctrl1_xl &= 0x0f; // turn off all ODR bits
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL1_XL, ctrl1_xl);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t gyro_powerup(const lsm6ds3tr_handle_t * imu)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;
    return lsm6ds3tr_gyro_set_odr(imu);
}

lsm6ds3tr_status_t accel_powerup(const lsm6ds3tr_handle_t * imu)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;
    return lsm6ds3tr_accel_set_odr(imu);
}

/************************************************************* Helper Functions ***************************************************************/
static lsm6ds3tr_status_t lsm6ds3tr_sw_reset(const lsm6ds3tr_handle_t * imu)
{
    // AN5130 section 5.7
    // 1. Power down gyroscope
    gyro_powerdown(imu);

    // 2. Set accelerometer in High Performance mode
    uint8_t ctrl6_c;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL6_C, &ctrl6_c);
    ctrl6_c &= ~LSM6DS3TR_BIT_XL_HM_MODE;
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL6_C, ctrl6_c);

    // 3. Set SW_RESET bit
    uint8_t ctrl3_c;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL3_C, &ctrl3_c);
    ctrl3_c |= LSM6DS3TR_BIT_SW_RESET;
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL3_C, ctrl3_c);

    // 4. wait for software reset bit to clear
    do
    {
        lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL3_C, &ctrl3_c);
    } while (ctrl3_c & LSM6DS3TR_BIT_SW_RESET);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_reboot(const lsm6ds3tr_handle_t * imu)
{
    // AN5130 section 5.7
    // 1. Power down gyroscope
    gyro_powerdown(imu);

    // 2. Set accelerometer in High Performance mode
    uint8_t ctrl6_c;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL6_C, &ctrl6_c);
    ctrl6_c &= ~LSM6DS3TR_BIT_XL_HM_MODE;
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL6_C, ctrl6_c);

    // 3. Set boot bit
    uint8_t ctrl3_c;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL3_C, &ctrl3_c);
    ctrl3_c |= LSM6DS3TR_BIT_BOOT;
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL3_C, ctrl3_c);

    // 4. Wait 15ms
    imu->delay_ms(15);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_set_it(const lsm6ds3tr_handle_t * imu)
{
    if (imu->hspi == NULL)
        return LSM6DS3TR_NULL_POINTER;

    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_INT1_CTRL, imu->int_handle_t.int1); // INT1
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_INT2_CTRL, imu->int_handle_t.int2); // INT2

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_gyro_init(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_status_t gyro_odr_status = lsm6ds3tr_gyro_set_odr(imu);
    if (gyro_odr_status != LSM6DS3TR_OK)
        return gyro_odr_status;

    lsm6ds3tr_status_t gyro_fs_status = lsm6ds3tr_gyro_set_fs(imu);
    if (gyro_fs_status != LSM6DS3TR_OK)
        return gyro_fs_status;

    // configure gyroscope
    uint8_t ctrl4_c, ctrl6_c, ctrl7_g;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL4_C, &ctrl4_c);
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL6_C, &ctrl6_c);
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL7_G, &ctrl7_g);

    // if using filter, high performance mode must be enabled
    if (imu->gyro_handle_t.filt != LSM6DS3TR_NO_FILTER)
        ctrl7_g &= ~LSM6DS3TR_BIT_G_HM_MODE;

    // configure LPF and HPF
    if (imu->gyro_handle_t.filt == LSM6DS3TR_BPF_EN || imu->gyro_handle_t.filt == LSM6DS3TR_LPF_EN)
    {
        ctrl4_c |= LSM6DS3TR_BIT_LPF1_SEL_G;  // enable lowpass filter
        ctrl6_c &= ~LSM6DS3TR_BIT_FTYPE_MASK; // set bandwith to medium value
    }
    else
    {
        ctrl4_c &= ~LSM6DS3TR_BIT_LPF1_SEL_G; // disable lowpass filter
    }

    if (imu->gyro_handle_t.filt == LSM6DS3TR_BPF_EN || imu->gyro_handle_t.filt == LSM6DS3TR_HPF_EN)
    {
        ctrl7_g |= LSM6DS3TR_BIT_HP_EN_G; // enable digital high pass filter

        // select cutoff frequency
        if (imu->gyro_handle_t.cutoff <= LSM6DS3TR_HP_G_1_04HZ)
            ctrl7_g |= imu->gyro_handle_t.cutoff << LSM6DS3TR_BIT_HPM_G_SHIFT;
        else
            return LSM6DS3TR_INVALID_CFG;
    }
    else
    {
        ctrl7_g &= ~LSM6DS3TR_BIT_HP_EN_G; // disable highpass filter
    }

    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL4_C, ctrl4_c | LSM6DS3TR_BIT_I2C_DISABLE);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL6_C, ctrl6_c);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL7_G, ctrl7_g);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_accel_init(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_status_t accel_odr_status = lsm6ds3tr_accel_set_odr(imu);
    if (accel_odr_status != LSM6DS3TR_OK)
        return accel_odr_status;

    lsm6ds3tr_status_t accel_fs_status = lsm6ds3tr_accel_set_fs(imu);
    if (accel_fs_status != LSM6DS3TR_OK)
        return accel_fs_status;

    // configure accelerometer
    uint8_t ctrl6_c, ctrl8_xl;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL6_C, &ctrl6_c);
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL8_XL, &ctrl8_xl);

    // bandpass filter mode not allowed
    if (imu->accel_handle_t.filt == LSM6DS3TR_BPF_EN)
        return LSM6DS3TR_INVALID_CFG;

    // if any filters are enabled, high performance mode must be on
    if (imu->accel_handle_t.filt != LSM6DS3TR_NO_FILTER)
        ctrl6_c &= ~LSM6DS3TR_BIT_XL_HM_MODE;

    if (imu->accel_handle_t.filt == LSM6DS3TR_LPF_EN)
    {
        ctrl8_xl &= ~LSM6DS3TR_BIT_HP_SLOPE_XL_EN;                      // disable highpass filter
        ctrl8_xl |= LSM6DS3TR_BIT_LPF2_XL_EN;                           // enable lowpass filter
        ctrl8_xl &= ~(LSM6DS3TR_BIT_HPCF_XL1 | LSM6DS3TR_BIT_HPCF_XL0); // ODR/50
    }
    else if (imu->accel_handle_t.filt == LSM6DS3TR_HPF_EN)
    {
        ctrl8_xl |= LSM6DS3TR_BIT_HP_SLOPE_XL_EN;                       // enable highpass filter
        ctrl8_xl &= ~LSM6DS3TR_BIT_LPF2_XL_EN;                          // disable lowpass filter
        ctrl8_xl &= ~(LSM6DS3TR_BIT_HPCF_XL1 | LSM6DS3TR_BIT_HPCF_XL0); // ODR/4
    }
    else
    {
        ctrl8_xl &= ~LSM6DS3TR_BIT_HP_SLOPE_XL_EN; // disable highpass filter
        ctrl8_xl &= ~LSM6DS3TR_BIT_LPF2_XL_EN;     // disable lowpass filter
    }

    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL6_C, ctrl6_c);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL8_XL, ctrl8_xl);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL9_XL, 0);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_set_fifo_mode(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_fifo_odr_t fifo_odr = imu->fifo_handle_t.odr;
    lsm6ds3tr_gyro_odr_t gyro_odr = imu->gyro_handle_t.odr;
    lsm6ds3tr_accel_odr_t xl_odr = imu->accel_handle_t.odr;

    lsm6ds3tr_fifo_mode_t fifo_mode = imu->fifo_handle_t.mode;

    // check for invalid configs
    if (fifo_odr > LSM6DS3TR_ODR_FIFO_6_66KHZ || xl_odr > LSM6DS3TR_ODR_XL_1_6HZ || gyro_odr > LSM6DS3TR_ODR_G_6_66KHZ)
        return LSM6DS3TR_INVALID_CFG;

    if (fifo_mode > LSM6DS3TR_FIFO_CONT)
        return LSM6DS3TR_INVALID_CFG;

    // FIFO ODR must be less than or equal to accelerometer ODR and gyro ODR
    if (((uint8_t)fifo_odr > (uint8_t)xl_odr || (uint8_t)fifo_odr > (uint8_t)gyro_odr) && (xl_odr != LSM6DS3TR_ODR_XL_1_6HZ))
        return LSM6DS3TR_INVALID_CFG;

    uint8_t fifo_ctrl5 = (fifo_odr << LSM6DS3TR_BIT_FIFO_ODR_SHIFT) | (fifo_mode << LSM6DS3TR_BIT_FIFO_MODE_SHIFT);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_FIFO_CTRL5, fifo_ctrl5);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_gyro_set_odr(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_gyro_odr_t odr = imu->gyro_handle_t.odr;
    lsm6ds3tr_filter_mode_t filt = imu->gyro_handle_t.filt;

    uint8_t ctrl7_g;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL7_G, &ctrl7_g);

    if (odr > LSM6DS3TR_ODR_G_6_66KHZ)
        return LSM6DS3TR_INVALID_CFG;

    // if filter is selected, must be high performance mode
    if (filt != LSM6DS3TR_NO_FILTER)
    {
        ctrl7_g &= ~LSM6DS3TR_BIT_G_HM_MODE; // high performance mode
    }
    else
    {
        if (odr >= LSM6DS3TR_ODR_G_12_5HZ && odr <= LSM6DS3TR_ODR_G_208HZ)
            ctrl7_g |= LSM6DS3TR_BIT_G_HM_MODE;  // disable high performance mode
        else
            ctrl7_g &= ~LSM6DS3TR_BIT_G_HM_MODE; // high performance mode
    }

    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL7_G, ctrl7_g);

    // only modify ODR bits
    uint8_t ctrl2_g;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL2_G, &ctrl2_g);
    ctrl2_g = (ctrl2_g & 0x0f) | (odr << LSM6DS3TR_BIT_ODR_G_SHIFT);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL2_G, ctrl2_g);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_accel_set_odr(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_accel_odr_t odr = imu->accel_handle_t.odr;
    lsm6ds3tr_filter_mode_t filt = imu->gyro_handle_t.filt;

    uint8_t ctrl6_c;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL6_C, &ctrl6_c);

    if (odr >= LSM6DS3TR_ODR_XL_1_6HZ)
        return LSM6DS3TR_INVALID_CFG;

    // if filter is selected, must be high performance mode
    if (filt != LSM6DS3TR_NO_FILTER)
    {
        ctrl6_c &= ~LSM6DS3TR_BIT_XL_HM_MODE; // high performance mode
    }
    else
    {
        if ((odr >= LSM6DS3TR_ODR_XL_12_5HZ && odr <= LSM6DS3TR_ODR_XL_208HZ) || odr == LSM6DS3TR_ODR_XL_1_6HZ)
            ctrl6_c |= LSM6DS3TR_BIT_XL_HM_MODE;  // disable high peformance mode
        else
            ctrl6_c &= ~LSM6DS3TR_BIT_XL_HM_MODE; // high performance mode
    }

    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL6_C, ctrl6_c);

    // only modify ODR bits
    uint8_t ctrl1_xl;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL1_XL, &ctrl1_xl);
    ctrl1_xl = (ctrl1_xl & 0x0f) | (odr << LSM6DS3TR_BIT_ODR_XL_SHIFT);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL1_XL, ctrl1_xl);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_gyro_set_fs(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_gyro_fullscale_t fs = imu->gyro_handle_t.fs;

    if (fs > LSM6DS3TR_FS_G_2000DPS)
        return LSM6DS3TR_INVALID_CFG;

    uint8_t ctrl2_g;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL2_G, &ctrl2_g);
    ctrl2_g = (ctrl2_g & 0xf3) | (fs << LSM6DS3TR_BIT_FS_G_SHIFT);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL2_G, ctrl2_g);

    return LSM6DS3TR_OK;
}

static lsm6ds3tr_status_t lsm6ds3tr_accel_set_fs(const lsm6ds3tr_handle_t * imu)
{
    lsm6ds3tr_accel_fullscale_t fs = imu->accel_handle_t.fs;

    if (fs > LSM6DS3TR_FS_XL_8G)
        return LSM6DS3TR_INVALID_CFG;

    uint8_t ctrl1_xl;
    lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_CTRL1_XL, &ctrl1_xl);
    ctrl1_xl = (ctrl1_xl & 0xf3) | (fs << LSM6DS3TR_BIT_FS_XL_SHIFT);
    lsm6ds3tr_hal_write_reg(imu, LSM6DS3TR_REG_CTRL1_XL, ctrl1_xl);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_read_status(const lsm6ds3tr_handle_t * imu, uint8_t * status)
{
    return lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_STATUS, status);
}
/**********************************************************************************************************************************************/
