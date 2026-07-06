#ifndef LSM6DS3TR_H_
#define LSM6DS3TR_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

#define IMU_READ                        0x80
#define IMU_WRITE                       0x00

/********************* Register Map **********************/
#define IMU_FUNC_CFG_ACCESS             0x01
#define IMU_SENSOR_SYNC_TIME_FRAME      0x04
#define IMU_SENSOR_SYNC_RES_RATIO       0x05
#define IMU_FIFO_CTRL1                  0x06
#define IMU_FIFO_CTRL2                  0x07
#define IMU_FIFO_CTRL3                  0x08
#define IMU_FIFO_CTRL4                  0x09
#define IMU_FIFO_CTRL5                  0x0A
#define IMU_DRDY_PULSE_CFG_G            0x0B
#define IMU_INT1_CTRL                   0x0D
#define IMU_INT2_CTRL                   0x0E
#define IMU_WHO_AM_I                    0x0F
#define IMU_CTRL1_XL                    0x10
#define IMU_CTRL2_G                     0x11
#define IMU_CTRL3_C                     0x12
#define IMU_CTRL4_C                     0x13
#define IMU_CTRL5_C                     0x14
#define IMU_CTRL6_C                     0x15
#define IMU_CTRL7_G                     0x16
#define IMU_CTRL8_XL                    0x17
#define IMU_CTRL9_XL                    0x18
#define IMU_CTRL10_C                    0x19
#define IMU_MASTER_CONFIG               0x1A
#define IMU_WAKE_UP_SRC                 0x1B
#define IMU_TAP_SRC                     0x1C
#define IMU_D6D_SRC                     0x1D
#define IMU_STATUS_REG                  0x1E
#define IMU_OUT_TEMP_L                  0x20
#define IMU_OUT_TEMP_H                  0x21
#define IMU_OUTX_L_G                    0x22
#define IMU_OUTX_H_G                    0x23
#define IMU_OUTY_L_G                    0x24
#define IMU_OUTY_H_G                    0x25
#define IMU_OUTZ_L_G                    0x26
#define IMU_OUTZ_H_G                    0x27
#define IMU_OUTX_L_XL                   0x28
#define IMU_OUTX_H_XL                   0x29
#define IMU_OUTY_L_XL                   0x2A
#define IMU_OUTY_H_XL                   0x2B
#define IMU_OUTZ_L_XL                   0x2C
#define IMU_OUTZ_H_XL                   0x2D
#define IMU_SENSORHUB1_REG              0x2E
#define IMU_SENSORHUB2_REG              0x2F
#define IMU_SENSORHUB3_REG              0x30
#define IMU_SENSORHUB4_REG              0x31
#define IMU_SENSORHUB5_REG              0x32
#define IMU_SENSORHUB6_REG              0x33
#define IMU_SENSORHUB7_REG              0x34
#define IMU_SENSORHUB8_REG              0x35
#define IMU_SENSORHUB9_REG              0x36
#define IMU_SENSORHUB10_REG             0x37
#define IMU_SENSORHUB11_REG             0x38
#define IMU_SENSORHUB12_REG             0x39
#define IMU_FIFO_STATUS1                0x3A
#define IMU_FIFO_STATUS2                0x3B
#define IMU_FIFO_STATUS3                0x3C
#define IMU_FIFO_STATUS4                0x3D
#define IMU_FIFO_DATA_OUT_L             0x3E
#define IMU_FIFO_DATA_OUT_H             0x3F
#define IMU_TIMESTAMP0_REG              0x40
#define IMU_TIMESTAMP1_REG              0x41
#define IMU_TIMESTAMP2_REG              0x42
#define IMU_STEP_TIMESTAMP_L            0x49
#define IMU_STEP_TIMESTAMP_H            0x4A
#define IMU_STEP_COUNTER_L              0x4B
#define IMU_STEP_COUNTER_H              0x4C
#define IMU_SENSORHUB13_REG             0x4D
#define IMU_SENSORHUB14_REG             0x4E
#define IMU_SENSORHUB15_REG             0x4F
#define IMU_SENSORHUB16_REG             0x50
#define IMU_SENSORHUB17_REG             0x51
#define IMU_SENSORHUB18_REG             0x52
#define IMU_FUNC_SRC1                   0x53
#define IMU_FUNC_SRC                    0x54
#define IMU_WRIST_TILT_IA               0x55
#define IMU_TAP_CFG                     0x58
#define IMU_TAP_THS_6D                  0x59
#define IMU_INT_DUR2                    0x5A
#define IMU_WAKE_UP_THS                 0x5B
#define IMU_WAKE_UP_DUR                 0x5C
#define IMU_FREE_FALL                   0x5D
#define IMU_MD1_CFG                     0x5E
#define IMU_MD2_CFG                     0x5F
#define IMU_MASTER_CMD_CODE             0x60
#define IMU_SENS_SYNC_SPI_ERROR_CODE    0x61
#define IMU_OUT_MAG_RAW_X_L             0x66
#define IMU_OUT_MAG_RAW_X_H             0x67
#define IMU_OUT_MAG_RAW_Y_L             0x68
#define IMU_OUT_MAG_RAW_Y_H             0x69
#define IMU_OUT_MAG_RAW_Z_L             0x6A
#define IMU_OUT_MAG_RAW_Z_H             0x6B
#define IMU_X_OFS_USR                   0x73
#define IMU_Y_OFS_USR                   0x74
#define IMU_Z_OFS_USR                   0x75
/*********************************************************/

/********************* Register Bitmaps ******************/
// CTRL1_XL
#define IMU_ODR_SHIFT                   4
#define IMU_ODR_XL_SHIFT                4
#define IMU_FS_XL_SHIFT                 2

// CTRL2_G
#define IMU_ODR_G_SHIFT                 4
#define IMU_FS_G_SHIFT                  2

// CTRL3_C
#define IMU_BOOT                        0x80
#define IMU_BDU                         0x40
#define IMU_H_LACTIVE                   0x20
#define IMU_PP_OD                       0x10
#define IMU_SIM                         0x08
#define IMU_IF_INC                      0x04
#define IMU_BLE                         0x02
#define IMU_SW_RESET                    0x01

// CTRL4_C
#define IMU_DEN_XL_EN                   0x80
#define IMU_SLEEP                       0x40
#define IMU_DEN_DRDY_INT1               0x20
#define IMU_INT2_ON_INT1                0x10
#define IMU_DRDY_MASK                   0x08
#define IMU_I2C_DISABLE                 0x04
#define IMU_LPF1_SEL_G                  0x02

// CTRL5_C
#define IMU_ROUNDING_SHIFT              5
#define IMU_DEN_LH                      0x10
#define IMU_ST_G_NORM                   0x00
#define IMU_ST_G_POS                    0x04
#define IMU_ST_G_NEG                    0x0C
#define IMU_ST_XL_NORM                  0x00
#define IMU_ST_XL_POS                   0x01
#define IMU_ST_XL_NEG                   0x02

// CTRL6_C
#define IMU_TRIG_EN                     0x80
#define IMU_LVL_EN                      0x40
#define IMU_LVL2_EN                     0x20
#define IMU_XL_HM_MODE                  0x10
#define IMU_USR_OFF_W                   0x08
#define IMU_FTYPE_MASK                  0x03                       

// CTRL7_G
#define IMU_G_HM_MODE                   0x80
#define IMU_HP_EN_G                     0x40
#define IMU_HP_G_16MILHZ                0x00
#define IMU_HP_G_65MILHZ                0x10
#define IMU_HP_G_260MILHZ               0x20
#define IMU_HP_G_1_04HZ                 0x30
#define IMU_HPM_G_MASK                  0x30
#define IMU_ROUNDING_STATUS             0x04

// CTRL8_XL
#define IMU_LPF2_XL_EN                  0x80
#define IMU_HPCF_XL1                    0x40
#define IMU_HPCF_XL0                    0x20
#define IMU_HP_REF_MODE                 0x10
#define IMU_INPUT_COMPOSITE             0x08
#define IMU_HP_SLOPE_XL_EN              0x04
#define IMU_LOW_PASS_ON_6D              0x01

// INT1_CTRL
#define IMU_INT1_STEP_DETECTOR          0x80
#define IMU_INT1_SIGN_MOT               0x40
#define IMU_INT1_FULL_FLAG              0x20
#define IMU_INT1_FIFO_OVR               0x10
#define IMU_INT1_FTH                    0x08
#define IMU_INT1_BOOT                   0x04
#define IMU_INT1_DRDY_G                 0x02
#define IMU_INT1_DRDY_XL                0x01

// INT2_CTRL
#define IMU_INT2_STEP_DELTA             0x80
#define IMU_INT2_STEP_COUNT_OV          0x40
#define IMU_INT2_FULL_FLAG              0x20
#define IMU_INT2_FIFO_OVR               0x10
#define IMU_INT2_FTH                    0x08
#define IMU_INT2_BOOT                   0x04
#define IMU_INT2_DRDY_G                 0x02
#define IMU_INT2_DRDY_XL                0x01
/*********************************************************/

/********************* Struct Defines ********************/
/***************************************************************************************** */
/*********************************Change these to all enums*********************** */
/***************************************************************************************** */
/***************************************************************************************** */
// Filter enables
#define IMU_LPF_EN                      4
#define IMU_LPF_DIS                     3
#define IMU_HPF_EN                      2
#define IMU_HPF_DIS                     1
#define IMU_NO_FILTER                   0


// ODR configurations
#define IMU_ODR_PWR_DOWN                0x00 << IMU_ODR_SHIFT
#define IMU_ODR_12_5HZ                  0x01 << IMU_ODR_SHIFT
#define IMU_ODR_26HZ                    0x02 << IMU_ODR_SHIFT
#define IMU_ODR_52HZ                    0x03 << IMU_ODR_SHIFT
#define IMU_ODR_104HZ                   0x04 << IMU_ODR_SHIFT
#define IMU_ODR_208HZ                   0x05 << IMU_ODR_SHIFT
#define IMU_ODR_416HZ                   0x06 << IMU_ODR_SHIFT
#define IMU_ODR_833HZ                   0x07 << IMU_ODR_SHIFT
#define IMU_ODR_1_66KHZ                 0x08 << IMU_ODR_SHIFT
#define IMU_ODR_3_33KHZ                 0x09 << IMU_ODR_SHIFT
#define IMU_ODR_6_66KHZ                 0x0A << IMU_ODR_SHIFT

// XL full-scale selection
#define IMU_FS_XL_2G                    0x00 << IMU_FS_XL_SHIFT
#define IMU_FS_XL_4G                    0x02 << IMU_FS_XL_SHIFT
#define IMU_FS_XL_8G                    0x03 << IMU_FS_XL_SHIFT
#define IMU_FS_XL_16G                   0x01 << IMU_FS_XL_SHIFT

// Gyro full-scale selection
#define IMU_FS_G_245DPS                 0x00 << IMU_FS_G_SHIFT
#define IMU_FS_G_500DPS                 0x01 << IMU_FS_G_SHIFT
#define IMU_FS_G_1000DPS                0x02 << IMU_FS_G_SHIFT
#define IMU_FS_G_2000DPS                0x03 << IMU_FS_G_SHIFT
/*********************************************************/

/******************** Read-Only Registers ****************/
// WHO_AM_I
#define IMU_ID                          0x6A
/*********************************************************/

/****************** Mathematical Constants ***************/
#define IMU_GRAVITY                     9.80665
#define IMU_DEG_TO_RAD                  0.01745329
/*********************************************************/

typedef struct {
    
    uint8_t odr;                            // output data rate
    uint8_t fs;                             // full-scale range
    uint8_t lpf;
    uint8_t hpf;
    uint8_t hp_cutoff;

} GyroParams;

typedef struct {

    uint8_t odr;                            // output data rate
    uint8_t fs;                             // full-scale range
    uint8_t filter_mode;                    // high/low pass filter

} AccelParams;

typedef struct {

    SPI_HandleTypeDef * hspi;               // SPI handle

    GPIO_TypeDef * cs_gpio;                 // SPI CS GPIO port
    uint16_t cs;                            // SPI CS pin

    void (*delay_ms)(uint32_t);             // pointer to delay function

    GyroParams gyro;                        // gyroscope parameters
    AccelParams xl;                         // accelerometer parameters

} IMUParams;

/* Initialize gyroscope + accelerometer, selecting filter types and output rates over SPI */
ErrorType imu_init(IMUParams * imu);
/* Read raw accelerometer data, then convert to m/s^2 */
ErrorType imu_read_accel_mps2(IMUParams * imu, float * a_x, float * a_y, float * a_z);
/* Read raw accelerometer data */
ErrorType imu_read_accel_raw(IMUParams * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb);
/* Read raw gyroscope data, then convert to rad/s */
ErrorType imu_read_gyro_radps(IMUParams * imu, float * w_x, float * w_y, float * w_z);
/* Read raw gyroscope data, then convert to deg/s */
ErrorType imu_read_gyro_degps(IMUParams * imu, float * w_x, float * w_y, float * w_z);
/* Read raw gyroscope data */
ErrorType imu_read_gyro_raw(IMUParams * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb);
/* Turn off gyroscope */
ErrorType gyro_powerdown(IMUParams * imu);
/* Turn off accelerometer */
ErrorType accel_powerdown(IMUParams * imu);
/* Turn on gyroscope */
ErrorType gyro_powerup(IMUParams * imu);
/* Turn on accelerometer */
ErrorType accel_powerup(IMUParams * imu);

#endif