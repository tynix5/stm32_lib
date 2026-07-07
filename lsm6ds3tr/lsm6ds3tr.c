#include <stdint.h>
#include <limits.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr.h"

/*********************** Helper Functions *********************/
/* Configure interrupts for accelerometer and gyroscope */
static ErrorType imu_set_it(ImuParams * imu);                   // needs implementation
/* Initialize internal gyroscope in IMU */
static ErrorType gyro_init(ImuParams * imu);
/* Initialize internal accelerometer in IMU */
static ErrorType accel_init(ImuParams * imu);
/* Configure internal FIFOs */
static ErrorType imu_set_fifo_mode(ImuParams * imu);            // needs implementation
/* Set ODR for gyroscope */
static ErrorType gyro_set_odr(ImuParams * imu);
/* Set ODR for accelerometer */
static ErrorType accel_set_odr(ImuParams * imu);
/* Set full-scale range for gyroscope */
static ErrorType gyro_set_fs(ImuParams * imu);
/* Set full-scale range for accelerometer */
static ErrorType accel_set_fs(ImuParams * imu);
/**************************************************************/

/********************** SPI Wrapper Functions *****************/
/* CS low */
static inline void imu_select(ImuParams * imu);
/* CS high */
static inline void imu_deselect(ImuParams * imu);
/* Write byte to selected register over SPI */
static void imu_write_reg(ImuParams * imu, uint8_t addr, uint8_t data);
/* Read byte from selected register over SPI */
static uint8_t imu_read_reg(ImuParams * imu, uint8_t addr);
/* Multi-byte read starting at start_addr with a length of len bytes over SPI */
static void imu_read(ImuParams * imu, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Read status register */
static uint8_t imu_status(ImuParams * imu);
/**************************************************************/

ErrorType imu_init(ImuParams * imu)
{
    imu_deselect(imu);

    // wait for power up
    imu->delay_ms(10);

    uint8_t retries = 0;
    // wait until communicaiton is RET_OKful
    while (imu_read_reg(imu, IMU_WHO_AM_I) != IMU_ID)
    {
        if (retries == SPI_MAX_RETRIES)
            return RET_TIMEOUT;

        retries++;
        imu->delay_ms(10);
    }

    imu_set_fifo_mode(imu);

    gyro_init(imu);
    accel_init(imu);

    // configure control register 3
    // normal mode, continuous update, interrupts active high, push-pull interrupts, 4-wire SPI, register addr auto inc, little endian
    imu_write_reg(imu, IMU_CTRL3_C, IMU_IF_INC);

    imu_set_it(imu);

    return RET_OK;
}

ErrorType imu_read_accel_mps2(ImuParams * imu, float * a_x, float * a_y, float * a_z)
{
    while (!(imu_read_reg(imu, IMU_STATUS_REG) & 0x1));

    uint8_t a[6];
    imu_read(imu, IMU_OUTX_L_XL, 6, a);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = a[0] | (a[1] << 8);          // OUT_L_XL | OUT_H_XL << 8
    y_raw = a[2] | (a[3] << 8);
    z_raw = a[4] | (a[5] << 8);

    // convert LSBs to m/s^2
    uint8_t scale_g;

    // determine full scale range (in g's)
    if (imu->AccelParams.fs == IMU_FS_XL_16G)
    {
        scale_g = 16;
    }
    else if (imu->AccelParams.fs == IMU_FS_XL_8G)
    {
        scale_g = 8;
    }
    else if (imu->AccelParams.fs == IMU_FS_XL_4G)
    {
        scale_g = 4;
    }
    else
    {
        scale_g = 2;
    }

    // convert LSBs to G's, then determine acceleration in m/s^2
    float lsb_to_g = (float) scale_g / SHRT_MAX;

    // LSBs -> G -> M/S^2
    *a_x = (float) x_raw * lsb_to_g * IMU_GRAVITY;
    *a_y = (float) y_raw * lsb_to_g * IMU_GRAVITY;
    *a_z = (float) z_raw * lsb_to_g * IMU_GRAVITY;

    return RET_OK;
}

ErrorType imu_read_accel_raw(ImuParams * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb)
{
    uint8_t a[6];
    imu_read(imu, IMU_OUTX_L_XL, 6, a);

    // convert low and high bytes into single value
    *a_x_lsb = a[0] | (a[1] << 8);          // OUT_L_XL | OUT_H_XL << 8
    *a_y_lsb = a[2] | (a[3] << 8);
    *a_z_lsb = a[4] | (a[5] << 8);

    return RET_OK;
}

ErrorType imu_read_gyro_radps(ImuParams * imu, float * w_x, float * w_y, float * w_z)
{
    uint8_t w[6];
    imu_read(imu, IMU_OUTX_L_G, 6, w);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8);          // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    if (imu->GyroParams.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->GyroParams.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->GyroParams.fs == IMU_FS_G_500DPS)
    {
        scale_dps = 500;
    }
    else
    {
        scale_dps = 245;
    }

    float lsb_to_dps = (float) scale_dps / SHRT_MAX;

    // LSBs -> DEG/S -> RAD/S
    *w_x = (float) x_raw * lsb_to_dps * IMU_DEG_TO_RAD;
    *w_y = (float) y_raw * lsb_to_dps * IMU_DEG_TO_RAD;
    *w_z = (float) z_raw * lsb_to_dps * IMU_DEG_TO_RAD;

    return RET_OK;
}

ErrorType imu_read_gyro_degps(ImuParams * imu, float * w_x, float * w_y, float * w_z)
{
    uint8_t w[6];
    imu_read(imu, IMU_OUTX_L_G, 6, w);

    int16_t x_raw, y_raw, z_raw;

    // convert low and high bytes into single value
    x_raw = w[0] | (w[1] << 8);          // OUT_L_G | OUT_H_G << 8
    y_raw = w[2] | (w[3] << 8);
    z_raw = w[4] | (w[5] << 8);

    // convert LSBs to radians per second
    uint16_t scale_dps;

    if (imu->GyroParams.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->GyroParams.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->GyroParams.fs == IMU_FS_G_500DPS)
    {
        scale_dps = 500;
    }
    else
    {
        scale_dps = 245;
    }

    float lsb_to_dps = (float) scale_dps / SHRT_MAX;

    // LSBs -> DEG/S
    *w_x = (float) x_raw * lsb_to_dps;
    *w_y = (float) y_raw * lsb_to_dps;
    *w_z = (float) z_raw * lsb_to_dps;

    return RET_OK;
}

ErrorType imu_read_gyro_raw(ImuParams * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb)
{
    uint8_t w[6];
    imu_read(imu, IMU_OUTX_L_G, 6, w);

    // convert low and high bytes into single value
    *w_x_lsb = w[0] | (w[1] << 8);          // OUT_L_G | OUT_H_G << 8
    *w_y_lsb = w[2] | (w[3] << 8);
    *w_z_lsb = w[4] | (w[5] << 8);
    
    return RET_OK;
}

ErrorType gyro_powerdown(ImuParams * imu)
{
    // turn off gyroscope by clearing all ODR bits
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g &= 0x0f;                                    // turn off all ODR bits
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return RET_OK;
}

ErrorType accel_powerdown(ImuParams * imu)
{
    // turn off accelerometer by clearing all ODR bits
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl &= 0x0f;                                   // turn off all ODR bits
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return RET_OK;
}

ErrorType gyro_powerup(ImuParams * imu)
{
    if (imu->GyroParams.odr > IMU_ODR_G_6_66KHZ)
        return RET_INVALID_CFG;
    
    // turn on gyroscope by selecting nonzero ODR
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0x0f) | (imu->GyroParams.odr << IMU_ODR_G_SHIFT);          // select desired ODR
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return RET_OK;
}

ErrorType accel_powerup(ImuParams * imu)
{
    if (imu->AccelParams.odr > IMU_ODR_XL_1_6HZ)
        return RET_INVALID_CFG;

    // turn on accelerometer by selecting nonzero ODR
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x0f) | (imu->AccelParams.odr << IMU_ODR_XL_SHIFT);          // select desired ODR
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return RET_OK;
}

/*************************** Helper Functions **************************/
static ErrorType imu_set_it(ImuParams * imu)
{
    // INT1 pin
    imu_write_reg(imu, IMU_INT1_CTRL, IMU_INT1_DRDY_G);      // interrupt on gyro data ready
    // INT2 pin
    imu_write_reg(imu, IMU_INT2_CTRL, IMU_INT2_DRDY_XL);     // interrupt on accel data ready

    return RET_OK;
}

static ErrorType gyro_init(ImuParams * imu)
{
    if (gyro_set_odr(imu) != RET_OK)
        return RET_INVALID_CFG;

    if (gyro_set_fs(imu) != RET_OK)
        return RET_INVALID_CFG;

    // configure gyroscope
    uint8_t ctrl4_c = imu_read_reg(imu, IMU_CTRL4_C);
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_CTRL6_C);
    uint8_t ctrl7_g = imu_read_reg(imu, IMU_CTRL7_G);

    // configure LPF and HPF
    if (imu->GyroParams.filt == IMU_BPF_EN || imu->GyroParams.filt == IMU_LPF_EN)
    {
        ctrl4_c |= IMU_LPF1_SEL_G;              // enable lowpass filter
        ctrl6_c &= ~IMU_FTYPE_MASK;             // set bandwith to medium value
    }
    else
    {
        ctrl4_c &= ~IMU_LPF1_SEL_G;             // disable lowpass filter
    }

    if (imu->GyroParams.filt == IMU_BPF_EN || imu->GyroParams.filt == IMU_HPF_EN)
    {
        ctrl7_g |= IMU_HP_EN_G;                 // enable digital high pass filter

        // select cutoff frequency
        if (imu->GyroParams.cutoff <= IMU_HP_G_1_04HZ)
            ctrl7_g |= imu->GyroParams.cutoff << IMU_HPM_G_SHIFT;
        else
            return RET_INVALID_CFG;
    }
    else
    {
        ctrl7_g &= ~IMU_HP_EN_G;                // disable highpass filter
    }

    imu_write_reg(imu, IMU_CTRL4_C, ctrl4_c | IMU_I2C_DISABLE);
    imu_write_reg(imu, IMU_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_CTRL7_G, ctrl7_g);

    return RET_OK;
}

static ErrorType accel_init(ImuParams * imu)
{
    if (accel_set_odr(imu) != RET_OK)
        return RET_INVALID_CFG;

    if (accel_set_fs(imu) != RET_OK)
        return RET_INVALID_CFG;

    // configure accelerometer
    // odr should be configured as 2x gyro odr to match odr rates
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_CTRL6_C);
    uint8_t ctrl8_xl = imu_read_reg(imu, IMU_CTRL8_XL);
    
    // bandpass filter mode not allowed
    if (imu->AccelParams.filt == IMU_BPF_EN)
    {
        return RET_INVALID_CFG;
    }
    else if (imu->AccelParams.filt == IMU_LPF_EN)
    {
        ctrl8_xl &= ~IMU_HP_SLOPE_XL_EN;            // disable highpass filter
        ctrl8_xl |= IMU_LPF2_XL_EN;                 // enable lowpass filter
        ctrl8_xl &= ~(IMU_HPCF_XL1 | IMU_HPCF_XL0);     // ODR/50
    }
    else if (imu->AccelParams.filt == IMU_HPF_EN)
    {
        ctrl8_xl |= IMU_HP_SLOPE_XL_EN;             // enable highpass filter
        ctrl8_xl &= ~IMU_LPF2_XL_EN;                // disable lowpass filter
        ctrl8_xl &= ~(IMU_HPCF_XL1 | IMU_HPCF_XL0);     // ODR/4
    }
    else
    {
        ctrl8_xl &= ~IMU_HP_SLOPE_XL_EN;            // disable highpass filter
        ctrl8_xl &= ~IMU_LPF2_XL_EN;                // disable lowpass filter
    }

    imu_write_reg(imu, IMU_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_CTRL8_XL, ctrl8_xl);
    imu_write_reg(imu, IMU_CTRL9_XL, 0);

    return RET_OK;
}

static ErrorType imu_set_fifo_mode(ImuParams * imu)
{
    imu_write_reg(imu, IMU_FIFO_CTRL5, 0b00000000);     // bypass mode --> FIFO disabled

    return RET_OK;
}

static ErrorType gyro_set_odr(ImuParams * imu)
{
    GyroOdr odr = imu->GyroParams.odr;


    uint8_t ctrl7_g = imu_read_reg(imu, IMU_CTRL7_G);

    if (odr >= IMU_ODR_G_12_5HZ && odr <= IMU_ODR_G_208HZ)
        ctrl7_g |= IMU_G_HM_MODE;                       // disable high performance mode
    else if (odr >= IMU_ODR_G_416HZ && odr <= IMU_ODR_G_6_66KHZ)
        ctrl7_g &= ~IMU_G_HM_MODE;                      // high performance mode
    else 
        return RET_INVALID_CFG;

    imu_write_reg(imu, IMU_CTRL7_G, ctrl7_g);

    // only modify ODR bits
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0x0f) | (odr << IMU_ODR_G_SHIFT);
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return RET_OK;
}

static ErrorType accel_set_odr(ImuParams * imu)
{
    AccelOdr odr = imu->AccelParams.odr;


    uint8_t ctrl6_c = imu_read_reg(imu, IMU_CTRL6_C);

    if ((odr >= IMU_ODR_XL_12_5HZ && odr <= IMU_ODR_XL_208HZ) || odr == IMU_ODR_XL_1_6HZ)
        ctrl6_c |= IMU_XL_HM_MODE;                      // disable high peformance mode
    else if (odr >= IMU_ODR_XL_416HZ && odr <= IMU_ODR_XL_6_66KHZ)
        ctrl6_c &= ~IMU_XL_HM_MODE;                     // high performance mode
    else 
        return RET_INVALID_CFG;

    imu_write_reg(imu, IMU_CTRL6_C, ctrl6_c);


    // only modify ODR bits
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x0f) | (odr << IMU_ODR_XL_SHIFT);
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return RET_OK;
}

static ErrorType gyro_set_fs(ImuParams * imu)
{
    GyroFs fs = imu->GyroParams.fs;

    if (fs >= IMU_FS_G_2000DPS)
        return RET_INVALID_CFG;
    
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0xf3) | (fs << IMU_FS_G_SHIFT);
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return RET_OK;
}

static ErrorType accel_set_fs(ImuParams * imu)
{
    AccelFs fs = imu->AccelParams.fs;

    if (fs >= IMU_FS_XL_8G)
        return RET_INVALID_CFG;

    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0xf3) | (fs << IMU_FS_XL_SHIFT);
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return RET_OK;
}
/***********************************************************************/


/************************* SPI Wrapper Functions ***********************/
static inline void imu_select(ImuParams * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_RESET);
}

static inline void imu_deselect(ImuParams * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_SET);
}

static void imu_write_reg(ImuParams * imu, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = {addr | IMU_WRITE, data};
    
    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, cmd, 2, SPI_TIMEOUT_MS);
    imu_deselect(imu);
}

static uint8_t imu_read_reg(ImuParams * imu, uint8_t addr)
{
    uint8_t result;
    addr |= IMU_READ;
    
    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, &result, 1, SPI_TIMEOUT_MS);
    imu_deselect(imu);
    
    return result;
}

static void imu_read(ImuParams * imu, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    // read multiple byte burst from IMU
    // IF_INC must be enabled in CTRL3_C reg
    start_addr |= IMU_READ;

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &start_addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, data, len, SPI_TIMEOUT_MS);
    imu_deselect(imu);
}

static uint8_t imu_status(ImuParams * imu)
{
    return imu_read_reg(imu, IMU_STATUS_REG);
}
/***********************************************************************/