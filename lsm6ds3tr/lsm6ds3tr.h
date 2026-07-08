#ifndef LSM6DS3TR_H_
#define LSM6DS3TR_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

#define IMU_READ                        0x80
#define IMU_WRITE                       0x00

/********************* Register Map **********************/
#define IMU_REG_FUNC_CFG_ACCESS             0x01
#define IMU_REG_SENSOR_SYNC_TIME_FRAME      0x04
#define IMU_REG_SENSOR_SYNC_RES_RATIO       0x05
#define IMU_REG_FIFO_CTRL1                  0x06
#define IMU_REG_FIFO_CTRL2                  0x07
#define IMU_REG_FIFO_CTRL3                  0x08
#define IMU_REG_FIFO_CTRL4                  0x09
#define IMU_REG_FIFO_CTRL5                  0x0A
#define IMU_REG_DRDY_PULSE_CFG_G            0x0B
#define IMU_REG_INT1_CTRL                   0x0D
#define IMU_REG_INT2_CTRL                   0x0E
#define IMU_REG_WHO_AM_I                    0x0F
#define IMU_REG_CTRL1_XL                    0x10
#define IMU_REG_CTRL2_G                     0x11
#define IMU_REG_CTRL3_C                     0x12
#define IMU_REG_CTRL4_C                     0x13
#define IMU_REG_CTRL5_C                     0x14
#define IMU_REG_CTRL6_C                     0x15
#define IMU_REG_CTRL7_G                     0x16
#define IMU_REG_CTRL8_XL                    0x17
#define IMU_REG_CTRL9_XL                    0x18
#define IMU_REG_CTRL10_C                    0x19
#define IMU_REG_MASTER_CONFIG               0x1A
#define IMU_REG_WAKE_UP_SRC                 0x1B
#define IMU_REG_TAP_SRC                     0x1C
#define IMU_REG_D6D_SRC                     0x1D
#define IMU_REG_STATUS                      0x1E
#define IMU_REG_OUT_TEMP_L                  0x20
#define IMU_REG_OUT_TEMP_H                  0x21
#define IMU_REG_OUTX_L_G                    0x22
#define IMU_REG_OUTX_H_G                    0x23
#define IMU_REG_OUTY_L_G                    0x24
#define IMU_REG_OUTY_H_G                    0x25
#define IMU_REG_OUTZ_L_G                    0x26
#define IMU_REG_OUTZ_H_G                    0x27
#define IMU_REG_OUTX_L_XL                   0x28
#define IMU_REG_OUTX_H_XL                   0x29
#define IMU_REG_OUTY_L_XL                   0x2A
#define IMU_REG_OUTY_H_XL                   0x2B
#define IMU_REG_OUTZ_L_XL                   0x2C
#define IMU_REG_OUTZ_H_XL                   0x2D
#define IMU_REG_SENSORHUB1_REG              0x2E
#define IMU_REG_SENSORHUB2_REG              0x2F
#define IMU_REG_SENSORHUB3_REG              0x30
#define IMU_REG_SENSORHUB4_REG              0x31
#define IMU_REG_SENSORHUB5_REG              0x32
#define IMU_REG_SENSORHUB6_REG              0x33
#define IMU_REG_SENSORHUB7_REG              0x34
#define IMU_REG_SENSORHUB8_REG              0x35
#define IMU_REG_SENSORHUB9_REG              0x36
#define IMU_REG_SENSORHUB10_REG             0x37
#define IMU_REG_SENSORHUB11_REG             0x38
#define IMU_REG_SENSORHUB12_REG             0x39
#define IMU_REG_FIFO_STATUS1                0x3A
#define IMU_REG_FIFO_STATUS2                0x3B
#define IMU_REG_FIFO_STATUS3                0x3C
#define IMU_REG_FIFO_STATUS4                0x3D
#define IMU_REG_FIFO_DATA_OUT_L             0x3E
#define IMU_REG_FIFO_DATA_OUT_H             0x3F
#define IMU_REG_TIMESTAMP0_REG              0x40
#define IMU_REG_TIMESTAMP1_REG              0x41
#define IMU_REG_TIMESTAMP2_REG              0x42
#define IMU_REG_STEP_TIMESTAMP_L            0x49
#define IMU_REG_STEP_TIMESTAMP_H            0x4A
#define IMU_REG_STEP_COUNTER_L              0x4B
#define IMU_REG_STEP_COUNTER_H              0x4C
#define IMU_REG_SENSORHUB13_REG             0x4D
#define IMU_REG_SENSORHUB14_REG             0x4E
#define IMU_REG_SENSORHUB15_REG             0x4F
#define IMU_REG_SENSORHUB16_REG             0x50
#define IMU_REG_SENSORHUB17_REG             0x51
#define IMU_REG_SENSORHUB18_REG             0x52
#define IMU_REG_FUNC_SRC1                   0x53
#define IMU_REG_FUNC_SRC                    0x54
#define IMU_REG_WRIST_TILT_IA               0x55
#define IMU_REG_TAP_CFG                     0x58
#define IMU_REG_TAP_THS_6D                  0x59
#define IMU_REG_INT_DUR2                    0x5A
#define IMU_REG_WAKE_UP_THS                 0x5B
#define IMU_REG_WAKE_UP_DUR                 0x5C
#define IMU_REG_FREE_FALL                   0x5D
#define IMU_REG_MD1_CFG                     0x5E
#define IMU_REG_MD2_CFG                     0x5F
#define IMU_REG_MASTER_CMD_CODE             0x60
#define IMU_REG_SENS_SYNC_SPI_ERROR_CODE    0x61
#define IMU_REG_OUT_MAG_RAW_X_L             0x66
#define IMU_REG_OUT_MAG_RAW_X_H             0x67
#define IMU_REG_OUT_MAG_RAW_Y_L             0x68
#define IMU_REG_OUT_MAG_RAW_Y_H             0x69
#define IMU_REG_OUT_MAG_RAW_Z_L             0x6A
#define IMU_REG_OUT_MAG_RAW_Z_H             0x6B
#define IMU_REG_X_OFS_USR                   0x73
#define IMU_REG_Y_OFS_USR                   0x74
#define IMU_REG_Z_OFS_USR                   0x75
/*********************************************************/

/********************* Register Bitmaps ******************/
// FIFO_CTRL5
#define IMU_BIT_FIFO_MODE_SHIFT             0
#define IMU_BIT_FIFO_ODR_SHIFT              3

// CTRL1_XL
#define IMU_BIT_ODR_XL_SHIFT                4
#define IMU_BIT_FS_XL_SHIFT                 2

// CTRL2_G
#define IMU_BIT_ODR_G_SHIFT                 4
#define IMU_BIT_FS_G_SHIFT                  2

// CTRL3_C
#define IMU_BIT_BOOT                        0x80
#define IMU_BIT_BDU                         0x40
#define IMU_BIT_H_LACTIVE                   0x20
#define IMU_BIT_PP_OD                       0x10
#define IMU_BIT_SIM                         0x08
#define IMU_BIT_IF_INC                      0x04
#define IMU_BIT_BLE                         0x02
#define IMU_BIT_SW_RESET                    0x01

// CTRL4_C
#define IMU_BIT_DEN_XL_EN                   0x80
#define IMU_BIT_SLEEP                       0x40
#define IMU_BIT_DEN_DRDY_INT1               0x20
#define IMU_BIT_INT2_ON_INT1                0x10
#define IMU_BIT_DRDY_MASK                   0x08
#define IMU_BIT_I2C_DISABLE                 0x04
#define IMU_BIT_LPF1_SEL_G                  0x02

// CTRL5_C
#define IMU_BIT_ROUNDING_SHIFT              5
#define IMU_BIT_DEN_LH                      0x10
#define IMU_BIT_ST_G_NORM                   0x00
#define IMU_BIT_ST_G_POS                    0x04
#define IMU_BIT_ST_G_NEG                    0x0C
#define IMU_BIT_ST_XL_NORM                  0x00
#define IMU_BIT_ST_XL_POS                   0x01
#define IMU_BIT_ST_XL_NEG                   0x02

// CTRL6_C
#define IMU_BIT_TRIG_EN                     0x80
#define IMU_BIT_LVL_EN                      0x40
#define IMU_BIT_LVL2_EN                     0x20
#define IMU_BIT_XL_HM_MODE                  0x10
#define IMU_BIT_USR_OFF_W                   0x08
#define IMU_BIT_FTYPE_MASK                  0x03                       

// CTRL7_G
#define IMU_BIT_G_HM_MODE                   0x80
#define IMU_BIT_HP_EN_G                     0x40
#define IMU_BIT_HPM_G_MASK                  0x30
#define IMU_BIT_HPM_G_SHIFT                 4
#define IMU_BIT_ROUNDING_STATUS             0x04

// CTRL8_XL
#define IMU_BIT_LPF2_XL_EN                  0x80
#define IMU_BIT_HPCF_XL1                    0x40
#define IMU_BIT_HPCF_XL0                    0x20
#define IMU_BIT_HP_REF_MODE                 0x10
#define IMU_BIT_INPUT_COMPOSITE             0x08
#define IMU_BIT_HP_SLOPE_XL_EN              0x04
#define IMU_BIT_LOW_PASS_ON_6D              0x01

// STATUS
#define IMU_BIT_STATUS_TDA                  0x04
#define IMU_BIT_STATUS_GDA                  0x02
#define IMU_BIT_STATUS_XLDA                 0x01
/*********************************************************/

/******************** Read-Only Registers ****************/
// WHO_AM_I
#define IMU_ID                          0x6A
/*********************************************************/

/****************** Mathematical Constants ***************/
#define IMU_GRAVITY                     9.80665
#define IMU_DEG_TO_RAD                  0.01745329
/*********************************************************/

// filter selections for acclerometer and gyroscope
typedef enum : uint8_t {
    
    IMU_NO_FILTER = 0,
    IMU_LPF_EN,
    IMU_HPF_EN,
    IMU_BPF_EN
    
} ImuFilterMode;

// gyroscope highpass filter cutoff selection
typedef enum : uint8_t {

    IMU_HP_G_16MILHZ = 0,
    IMU_HP_G_65MILHZ,
    IMU_HP_G_260MILHZ,
    IMU_HP_G_1_04HZ

} ImuGyroHighpassCutoff;

// gyroscope output data rates
typedef enum : uint8_t {

    IMU_ODR_G_PWR_DOWN = 0,
    IMU_ODR_G_12_5HZ,  
    IMU_ODR_G_26HZ,    
    IMU_ODR_G_52HZ,   
    IMU_ODR_G_104HZ,   
    IMU_ODR_G_208HZ,   
    IMU_ODR_G_416HZ,   
    IMU_ODR_G_833HZ,   
    IMU_ODR_G_1_66KHZ,
    IMU_ODR_G_3_33KHZ, 
    IMU_ODR_G_6_66KHZ

} ImuGyroOdr;

// accelerometer output data rates
typedef enum : uint8_t {

    IMU_ODR_XL_PWR_DOWN = 0,
    IMU_ODR_XL_12_5HZ,  
    IMU_ODR_XL_26HZ,    
    IMU_ODR_XL_52HZ,   
    IMU_ODR_XL_104HZ,   
    IMU_ODR_XL_208HZ,   
    IMU_ODR_XL_416HZ,   
    IMU_ODR_XL_833HZ,   
    IMU_ODR_XL_1_66KHZ,
    IMU_ODR_XL_3_33KHZ, 
    IMU_ODR_XL_6_66KHZ,
    IMU_ODR_XL_1_6HZ
    
} ImuAccelOdr;

// gyroscope full-scale selection
typedef enum : uint8_t {

    IMU_FS_G_245DPS = 0, 
    IMU_FS_G_500DPS,
    IMU_FS_G_1000DPS,
    IMU_FS_G_2000DPS

} ImuGyroFs;

// accelerometer full-scale selection
typedef enum : uint8_t {

    IMU_FS_XL_2G = 0, 
    IMU_FS_XL_16G,
    IMU_FS_XL_4G, 
    IMU_FS_XL_8G

} ImuAccelFs;

// FIFO modes
typedef enum : uint8_t {

    IMU_FIFO_BYPASS = 0,
    IMU_FIFO_FIFO,
    IMU_FIFO_CONT_TO_FIFO = 3,
    IMU_FIFO_BYPASS_TO_CONT,
    IMU_FIFO_CONT = 6,

} ImuFifoMode;

// FIFO output data rates
typedef enum : uint8_t {

    IMU_ODR_FIFO_DISABLE = 0,
    IMU_ODR_FIFO_12_5HZ,
    IMU_ODR_FIFO_26HZ,
    IMU_ODR_FIFO_52HZ,
    IMU_ODR_FIFO_104HZ,
    IMU_ODR_FIFO_208HZ,
    IMU_ODR_FIFO_416HZ,
    IMU_ODR_FIFO_833HZ,
    IMU_ODR_FIFO_1_66KHZ,
    IMU_ODR_FIFO_3_33KHZ,
    IMU_ODR_FIFO_6_66KHZ

} ImuFifoOdr;

// INT1 event selection
typedef enum : uint8_t {
    
    IMU_INT1_NONE = 0,
    IMU_INT1_DRDY_XL = 1,                       // accelerometer data ready
    IMU_INT1_DRDY_G = 2,                        // gyroscope data ready
    IMU_INT1_BOOT = 4,                          // boot status 
    IMU_INT1_FTH = 8,                           // FIFO threshold
    IMU_INT1_FIFO_OVR = 16,                     // FIFO overrun
    IMU_INT1_FULL_FLAG = 32,                    // FIFO full flag
    IMU_INT1_SIGN_MOT = 64,                     // significant motion
    IMU_INT1_STEP_DETECTOR = 128                // pedometer step recognition

} ImuInt1Sel;

// INT2 event selection
typedef enum : uint8_t {

    IMU_INT2_NONE = 0,
    IMU_INT2_DRDY_XL = 1,                       // accelerometer data ready
    IMU_INT2_DRDY_G = 2,                        // gyroscope data ready
    IMU_INT2_DRDY_TEMP = 4,                     // temperature data ready
    IMU_INT2_FTH = 8,                           // FIFO threshold
    IMU_INT2_FIFO_OVR = 16,                     // FIFO overrun
    IMU_INT2_FULL_FLAG = 32,                    // FIFO full flag
    IMU_INT2_STEP_COUNT_OV = 64,                // step counter overflow
    IMU_INT2_STEP_DELTA = 128                   // pedometer step recognition on delta time

} ImuInt2Sel;


typedef struct {

    SPI_HandleTypeDef * hspi;               // SPI handle

    GPIO_TypeDef * cs_gpio;                 // SPI CS GPIO port
    uint16_t cs;                            // SPI CS pin

    void (*delay_ms)(uint32_t);             // pointer to delay function

    struct {
        ImuGyroOdr odr;                        // gyro output data rate
        ImuGyroFs fs;                          // gyro full-scale range
        ImuFilterMode filt;                 // filter selection
        ImuGyroHighpassCutoff cutoff;          // high pass filter cutoff
    } GyroParams;

    struct {
        ImuAccelOdr odr;                       // accel output data rate
        ImuAccelFs fs;                         // accel full-scale range
        ImuFilterMode filt;                 // filter selection
    } AccelParams;

    struct {
        ImuFifoOdr odr;                        // FIFO odr must be < accel and gyro
        ImuFifoMode mode;                      // FIFO mode
    } FifoParams;

    struct {
        ImuInt1Sel int1;                       // INT1 event selection
        ImuInt2Sel int2;                       // INT2 event selection
    } IntParams;

} ImuParams;

/* Initialize gyroscope + accelerometer, selecting filter types and output rates over SPI */
ErrorType imu_init(ImuParams * imu);
/* Check if new accelerometer data is available (blocking) */
ErrorType imu_is_accel_data_ready(ImuParams * imu, uint32_t timeout_ms);
/* Check if new gyroscope data is available (blocking) */
ErrorType imu_is_gyro_data_ready(ImuParams * imu, uint32_t timeout_ms);
/* Read raw accelerometer data, then convert to m/s^2 */
ErrorType imu_read_accel_mps2(ImuParams * imu, float * a_x, float * a_y, float * a_z);
/* Read raw accelerometer data */
ErrorType imu_read_accel_raw(ImuParams * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb);
/* Read raw gyroscope data, then convert to rad/s */
ErrorType imu_read_gyro_radps(ImuParams * imu, float * w_x, float * w_y, float * w_z);
/* Read raw gyroscope data, then convert to deg/s */
ErrorType imu_read_gyro_degps(ImuParams * imu, float * w_x, float * w_y, float * w_z);
/* Read raw gyroscope data */
ErrorType imu_read_gyro_raw(ImuParams * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb);
/* Turn off gyroscope */
ErrorType gyro_powerdown(ImuParams * imu);
/* Turn off accelerometer */
ErrorType accel_powerdown(ImuParams * imu);
/* Turn on gyroscope */
ErrorType gyro_powerup(ImuParams * imu);
/* Turn on accelerometer */
ErrorType accel_powerup(ImuParams * imu);

#endif