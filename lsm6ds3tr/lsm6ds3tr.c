#include <stdint.h>
#include <limits.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr.h"

/*********************** Helper Functions *********************/
/* Initialize internal gyroscope in IMU */
static void gyro_init(IMUParams * imu);
/* Initialize internal accelerometer in IMU */
static void accel_init(IMUParams * imu);
/**************************************************************/

/********************** SPI Wrapper Functions *****************/
/* CS low */
static inline void imu_select(IMUParams * imu);
/* CS high */
static inline void imu_deselect(IMUParams * imu);
/* Write byte to selected register over SPI */
static void imu_write_reg(IMUParams * imu, uint8_t addr, uint8_t data);
/* Read byte from selected register over SPI */
static uint8_t imu_read_reg(IMUParams * imu, uint8_t addr);
/* Multi-byte read starting at start_addr with a length of len bytes over SPI */
static void imu_read(IMUParams * imu, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Read status register */
static uint8_t imu_status(IMUParams * imu);
/**************************************************************/

ErrorType imu_init(IMUParams * imu)
{
    imu_deselect(imu);

    // wait for power up
    imu->delay_ms(10);

    uint8_t retries = 0;
    // wait until communicaiton is successful
    while (imu_read_reg(imu, IMU_WHO_AM_I) != IMU_ID)
    {
        if (retries == SPI_MAX_RETRIES)
            return TIMEOUT;

        retries++;
        imu-->delay_ms(10);
    }

    // configure FIFOs
    /*
    imu_write_reg(imu, FIFO_CTRL1, 10);             // 10 words max
    imu_write_reg(imu, FIFO_CTRL2, 0);              // disable pedometer
    imu_write_reg(imu, FIFO_CTRL3, 0b00001001);     // no decimation for gyroscope or accelerometer
    imu_write_reg(imu, FIFO_CTRL4, 0b10000000);     // enable FIFO threshold use
    imu_write_reg(imu, FIFO_CTRL5, 0b00001110);     // FIFO ODR is 12.5Hz and if FIFO is full, overwrite old data
    */
    imu_write_reg(imu, IMU_FIFO_CTRL5, 0b00000000);     // bypass mode --> FIFO disabled

    gyro_init(imu);
    accel_init(imu);

    // configure control register 3
    // normal mode, continuous update, interrupts active high, push-pull interrupts, 4-wire SPI, register addr auto inc, little endian
    imu_write_reg(imu, IMU_CTRL3_C, IMU_IF_INC);

    // INT1 pin
    imu_write_reg(imu, IMU_INT1_CTRL, IMU_INT1_DRDY_G);      // interrupt on gyro data ready
    // INT2 pin
    imu_write_reg(imu, IMU_INT2_CTRL, IMU_INT2_DRDY_XL);     // interrupt on accel data ready

    /************ Apply High pass filter on Gyroscope to negate drift ************/
    /********** Apply Low pass filter on accelerometer to eliminate noise ********/

    return SUCCESS;
}

ErrorType imu_read_accel_mps2(IMUParams * imu, float * a_x, float * a_y, float * a_z)
{
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
    if (imu->xl.fs == IMU_FS_XL_16G)
    {
        scale_g = 16;
    }
    else if (imu->xl.fs == IMU_FS_XL_8G)
    {
        scale_g = 8;
    }
    else if (imu->xl.fs == IMU_FS_XL_4G)
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

    return SUCCESS;
}

ErrorType imu_read_accel_raw(IMUParams * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb)
{
    uint8_t a[6];
    imu_read(imu, IMU_OUTX_L_XL, 6, a);

    // convert low and high bytes into single value
    *a_x_lsb = a[0] | (a[1] << 8);          // OUT_L_XL | OUT_H_XL << 8
    *a_y_lsb = a[2] | (a[3] << 8);
    *a_z_lsb = a[4] | (a[5] << 8);

    return SUCCESS;
}

ErrorType imu_read_gyro_radps(IMUParams * imu, float * w_x, float * w_y, float * w_z)
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

    if (imu->gyro.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->gyro.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->gyro.fs == IMU_FS_G_500DPS)
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

    return SUCCESS;
}

ErrorType imu_read_gyro_degps(IMUParams * imu, float * w_x, float * w_y, float * w_z)
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

    if (imu->gyro.fs == IMU_FS_G_2000DPS)
    {
        scale_dps = 2000;
    }
    else if (imu->gyro.fs == IMU_FS_G_1000DPS)
    {
        scale_dps = 1000;
    }
    else if (imu->gyro.fs == IMU_FS_G_500DPS)
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

    return SUCCESS;
}

ErrorType imu_read_gyro_raw(IMUParams * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb)
{
    uint8_t w[6];
    imu_read(imu, IMU_OUTX_L_G, 6, w);

    // convert low and high bytes into single value
    *w_x_lsb = w[0] | (w[1] << 8);          // OUT_L_G | OUT_H_G << 8
    *w_y_lsb = w[2] | (w[3] << 8);
    *w_z_lsb = w[4] | (w[5] << 8);
    
    return SUCCESS;
}

ErrorType gyro_powerdown(IMUParams * imu)
{
    // turn off gyroscope by clearing all ODR bits
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0x0f) | IMU_ODR_PWR_DOWN;          // turn off all ODR bits
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return SUCCESS;
}

ErrorType accel_powerdown(IMUParams * imu)
{
    // turn off accelerometer by clearing all ODR bits
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x0f) | IMU_ODR_PWR_DOWN;          // turn off all ODR bits
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return SUCCESS;
}

ErrorType gyro_powerup(IMUParams * imu)
{
    // turn on gyroscope by selecting nonzero ODR
    uint8_t ctrl2_g = imu_read_reg(imu, IMU_CTRL2_G);
    ctrl2_g = (ctrl2_g & 0x0f) | imu->gyro.odr;          // select desired ODR
    imu_write_reg(imu, IMU_CTRL2_G, ctrl2_g);

    return SUCCESS;
}

ErrorType accel_powerup(IMUParams * imu)
{
    // turn on accelerometer by selecting nonzero ODR
    uint8_t ctrl1_xl = imu_read_reg(imu, IMU_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x0f) | imu->xl.odr;          // select desired ODR
    imu_write_reg(imu, IMU_CTRL1_XL, ctrl1_xl);

    return SUCCESS;
}

/*************************** Helper Functions **************************/
static void gyro_init(IMUParams * imu)
{
    // configure gyroscope
    uint8_t ctrl4_c = imu_read_reg(imu, IMU_CTRL4_C);
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_CTRL6_C);
    uint8_t ctrl7_g = imu_read_reg(imu, IMU_CTRL7_G);

    if (imu->gyro.odr < IMU_ODR_416HZ)
    {
        ctrl7_g |= IMU_G_HM_MODE;               // disable high performance mode
    }
    else
    {
        ctrl7_g &= ~IMU_G_HM_MODE;              // enable high performance mode
    }

    if (imu->gyro.lpf == IMU_LPF_EN)
    {
        ctrl4_c |= IMU_LPF1_SEL_G;              // enable lowpass filter
        ctrl6_c &= ~IMU_FTYPE_MASK;             // set bandwith to medium value
    }
    else
    {
        ctrl4_c &= ~IMU_LPF1_SEL_G;             // disable lowpass filter
    }

    if (imu->gyro.hpf == IMU_HPF_EN)
    {
        ctrl7_g |= IMU_HP_EN_G | imu->gyro.hp_cutoff;   // enable highpass filter and select cutoff
    }
    else
    {
        ctrl7_g &= ~IMU_HP_EN_G;                // disable highpass filter
    }

    imu_write_reg(imu, IMU_CTRL2_G, imu->gyro.odr | imu->gyro.fs);      // select output data rate and degrees per second
    imu_write_reg(imu, IMU_CTRL4_C, ctrl4_c | IMU_I2C_DISABLE);
    imu_write_reg(imu, IMU_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_CTRL7_G, ctrl7_g);
}

static void accel_init(IMUParams * imu)
{
    // configure accelerometer
    // odr should be configured as 2x gyro odr to match odr rates
    uint8_t ctrl6_c = imu_read_reg(imu, IMU_CTRL6_C);
    uint8_t ctrl8_xl = imu_read_reg(imu, IMU_CTRL8_XL);
    
    if (imu->xl.odr < IMU_ODR_416HZ)
    {
        ctrl6_c |= IMU_XL_HM_MODE;              // disable high performance mode
    }
    else
    {
        ctrl6_c &= ~IMU_XL_HM_MODE;             // enable high performance mode
    }
    
    if (imu->xl.filter_mode == IMU_LPF_EN)
    {
        ctrl8_xl &= ~IMU_HP_SLOPE_XL_EN;            // disable highpass filter
        ctrl8_xl |= IMU_LPF2_XL_EN;                 // enable lowpass filter
        ctrl8_xl &= ~(IMU_HPCF_XL1 | IMU_HPCF_XL0);     // ODR/50
    }
    else if (imu->xl.filter_mode == IMU_HPF_EN)
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

    imu_write_reg(imu, IMU_CTRL1_XL, imu->xl.odr | imu->xl.fs);         // select output data rate and degrees per second
    imu_write_reg(imu, IMU_CTRL6_C, ctrl6_c);
    imu_write_reg(imu, IMU_CTRL8_XL, ctrl8_xl);
    imu_write_reg(imu, IMU_CTRL9_XL, 0);
}
/***********************************************************************/


/************************* SPI Wrapper Functions ***********************/
static inline void imu_select(IMUParams * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_RESET);
}

static inline void imu_deselect(IMUParams * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_SET);
}

static void imu_write_reg(IMUParams * imu, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = {addr | IMU_WRITE, data};
    
    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, cmd, 2, HAL_TIMEOUT);
    imu_deselect(imu);
}

static uint8_t imu_read_reg(IMUParams * imu, uint8_t addr)
{
    uint8_t result;
    addr |= IMU_READ;
    
    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(imu->hspi, &result, 1, HAL_TIMEOUT);
    imu_deselect(imu);
    
    return result;
}

static void imu_read(IMUParams * imu, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    // read multiple byte burst from IMU
    // IF_INC must be enabled in CTRL3_C reg
    start_addr |= IMU_READ;

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, &start_addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(imu->hspi, data, len, HAL_TIMEOUT);
    imu_deselect(imu);
}

static uint8_t imu_status(IMUParams * imu)
{
    return imu_read_reg(imu, IMU_STATUS_REG);
}
/***********************************************************************/