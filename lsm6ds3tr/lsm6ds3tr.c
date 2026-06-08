#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "lsm6ds3tr.h"

/********************** SPI Wrapper Functions *****************/
static inline void imu_select(IMUParams * imu);
static inline void imu_deselect(IMUParams * imu);
static void imu_write_reg(IMUParams * imu, uint8_t addr, uint8_t data);
static uint8_t imu_read_reg(IMUParams * imu, uint8_t addr);
static uint8_t imu_status(IMUParams * imu);
/**************************************************************/

void imu_init(IMUParams * imu)
{
    imu_deselect(imu);

    while (imu_read_reg(imu, WHO_AM_I) != IMU_ID);  // ensure communication

    // configure FIFOs
    /*
    imu_write_reg(imu, FIFO_CTRL1, 10);             // 10 words max
    imu_write_reg(imu, FIFO_CTRL2, 0);              // disable pedometer
    imu_write_reg(imu, FIFO_CTRL3, 0b00001001);     // no decimation for gyroscope or accelerometer
    imu_write_reg(imu, FIFO_CTRL4, 0b10000000);     // enable FIFO threshold use
    imu_write_reg(imu, FIFO_CTRL5, 0b00001110);     // FIFO ODR is 12.5Hz and if FIFO is full, overwrite old data
    */
    imu_write_reg(imu, FIFO_CTRL5, 0b00000000);     // bypass mode --> FIFO disabled

    // INT1 pin
    imu_write_reg(imu, INT1_CTRL, 0b00000010);      // interrupt on gyro data ready
    // INT2 pin
    imu_write_reg(imu, INT2_CTRL, 0b00000001);      // interrupt on accel data ready

    // accelerometer configuration
    imu_write_reg(imu, CTRL1_XL, 0b01011100);       // 208 Hz data output, +- 8g

    // gyroscope configuration
    imu_write_reg(imu, CTRL2_G, 0b01011100);        // 208 Hz, 2000 dps

    // disable I2C (not strictly necessary)
    imu_write_reg(imu, CTRL4_C, 0b00000100);        // gyro sleep mode disable

    /************ Apply High pass filter on Gyroscope to negate drift ************/
    /************ Apply Low pass filter on accelerometer to eliminate noise */
}

void imu_read_accel(IMUParams * imu, int16_t * x, int16_t * y, int16_t * z)
{
    int8_t temp;

    *x = imu_read_reg(imu, OUTX_L_XL);
    temp = imu_read_reg(imu, OUTX_H_XL);
    *x |= temp << 8;

    *y = imu_read_reg(imu, OUTY_L_XL);
    temp = imu_read_reg(imu, OUTY_H_XL);
    *y |= temp << 8;

    *z = imu_read_reg(imu, OUTZ_L_XL);
    temp = imu_read_reg(imu, OUTZ_H_XL);
    *z |= temp << 8;
}

void imu_read_gyro(IMUParams * imu, int16_t * x, int16_t * y, int16_t * z)
{
    int8_t temp;

    *x = imu_read_reg(imu, OUTX_L_G);
    temp = imu_read_reg(imu, OUTX_H_G);
    *x |= temp << 8;

    *y = imu_read_reg(imu, OUTY_L_G);
    temp = imu_read_reg(imu, OUTY_H_G);
    *y |= temp << 8;

    *z = imu_read_reg(imu, OUTZ_L_G);
    temp = imu_read_reg(imu, OUTZ_H_G);
    *z |= temp << 8;
}

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
    uint8_t data[2] = {addr | IMU_WRITE, data};

    imu_select(imu);
    HAL_SPI_Transmit(imu->hspi, data, 2, HAL_TIMEOUT);
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

static uint8_t imu_status(IMUParams * imu)
{
    return imu_read_reg(imu, STATUS_REG);
}