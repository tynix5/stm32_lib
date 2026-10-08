#ifndef LSM6DS3TR__H_
#define LSM6DS3TR__H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

#define LSM6DS3TR_READ  0x80
#define LSM6DS3TR_WRITE 0x00

/********************* Register Map **********************/
#define LSM6DS3TR_REG_FUNC_CFG_ACCESS          0x01
#define LSM6DS3TR_REG_SENSOR_SYNC_TIME_FRAME   0x04
#define LSM6DS3TR_REG_SENSOR_SYNC_RES_RATIO    0x05
#define LSM6DS3TR_REG_FIFO_CTRL1               0x06
#define LSM6DS3TR_REG_FIFO_CTRL2               0x07
#define LSM6DS3TR_REG_FIFO_CTRL3               0x08
#define LSM6DS3TR_REG_FIFO_CTRL4               0x09
#define LSM6DS3TR_REG_FIFO_CTRL5               0x0A
#define LSM6DS3TR_REG_DRDY_PULSE_CFG_G         0x0B
#define LSM6DS3TR_REG_INT1_CTRL                0x0D
#define LSM6DS3TR_REG_INT2_CTRL                0x0E
#define LSM6DS3TR_REG_WHO_AM_I                 0x0F
#define LSM6DS3TR_REG_CTRL1_XL                 0x10
#define LSM6DS3TR_REG_CTRL2_G                  0x11
#define LSM6DS3TR_REG_CTRL3_C                  0x12
#define LSM6DS3TR_REG_CTRL4_C                  0x13
#define LSM6DS3TR_REG_CTRL5_C                  0x14
#define LSM6DS3TR_REG_CTRL6_C                  0x15
#define LSM6DS3TR_REG_CTRL7_G                  0x16
#define LSM6DS3TR_REG_CTRL8_XL                 0x17
#define LSM6DS3TR_REG_CTRL9_XL                 0x18
#define LSM6DS3TR_REG_CTRL10_C                 0x19
#define LSM6DS3TR_REG_MASTER_CONFIG            0x1A
#define LSM6DS3TR_REG_WAKE_UP_SRC              0x1B
#define LSM6DS3TR_REG_TAP_SRC                  0x1C
#define LSM6DS3TR_REG_D6D_SRC                  0x1D
#define LSM6DS3TR_REG_STATUS                   0x1E
#define LSM6DS3TR_REG_OUT_TEMP_L               0x20
#define LSM6DS3TR_REG_OUT_TEMP_H               0x21
#define LSM6DS3TR_REG_OUTX_L_G                 0x22
#define LSM6DS3TR_REG_OUTX_H_G                 0x23
#define LSM6DS3TR_REG_OUTY_L_G                 0x24
#define LSM6DS3TR_REG_OUTY_H_G                 0x25
#define LSM6DS3TR_REG_OUTZ_L_G                 0x26
#define LSM6DS3TR_REG_OUTZ_H_G                 0x27
#define LSM6DS3TR_REG_OUTX_L_XL                0x28
#define LSM6DS3TR_REG_OUTX_H_XL                0x29
#define LSM6DS3TR_REG_OUTY_L_XL                0x2A
#define LSM6DS3TR_REG_OUTY_H_XL                0x2B
#define LSM6DS3TR_REG_OUTZ_L_XL                0x2C
#define LSM6DS3TR_REG_OUTZ_H_XL                0x2D
#define LSM6DS3TR_REG_SENSORHUB1_REG           0x2E
#define LSM6DS3TR_REG_SENSORHUB2_REG           0x2F
#define LSM6DS3TR_REG_SENSORHUB3_REG           0x30
#define LSM6DS3TR_REG_SENSORHUB4_REG           0x31
#define LSM6DS3TR_REG_SENSORHUB5_REG           0x32
#define LSM6DS3TR_REG_SENSORHUB6_REG           0x33
#define LSM6DS3TR_REG_SENSORHUB7_REG           0x34
#define LSM6DS3TR_REG_SENSORHUB8_REG           0x35
#define LSM6DS3TR_REG_SENSORHUB9_REG           0x36
#define LSM6DS3TR_REG_SENSORHUB10_REG          0x37
#define LSM6DS3TR_REG_SENSORHUB11_REG          0x38
#define LSM6DS3TR_REG_SENSORHUB12_REG          0x39
#define LSM6DS3TR_REG_FIFO_STATUS1             0x3A
#define LSM6DS3TR_REG_FIFO_STATUS2             0x3B
#define LSM6DS3TR_REG_FIFO_STATUS3             0x3C
#define LSM6DS3TR_REG_FIFO_STATUS4             0x3D
#define LSM6DS3TR_REG_FIFO_DATA_OUT_L          0x3E
#define LSM6DS3TR_REG_FIFO_DATA_OUT_H          0x3F
#define LSM6DS3TR_REG_TIMESTAMP0_REG           0x40
#define LSM6DS3TR_REG_TIMESTAMP1_REG           0x41
#define LSM6DS3TR_REG_TIMESTAMP2_REG           0x42
#define LSM6DS3TR_REG_STEP_TIMESTAMP_L         0x49
#define LSM6DS3TR_REG_STEP_TIMESTAMP_H         0x4A
#define LSM6DS3TR_REG_STEP_COUNTER_L           0x4B
#define LSM6DS3TR_REG_STEP_COUNTER_H           0x4C
#define LSM6DS3TR_REG_SENSORHUB13_REG          0x4D
#define LSM6DS3TR_REG_SENSORHUB14_REG          0x4E
#define LSM6DS3TR_REG_SENSORHUB15_REG          0x4F
#define LSM6DS3TR_REG_SENSORHUB16_REG          0x50
#define LSM6DS3TR_REG_SENSORHUB17_REG          0x51
#define LSM6DS3TR_REG_SENSORHUB18_REG          0x52
#define LSM6DS3TR_REG_FUNC_SRC1                0x53
#define LSM6DS3TR_REG_FUNC_SRC                 0x54
#define LSM6DS3TR_REG_WRIST_TILT_IA            0x55
#define LSM6DS3TR_REG_TAP_CFG                  0x58
#define LSM6DS3TR_REG_TAP_THS_6D               0x59
#define LSM6DS3TR_REG_INT_DUR2                 0x5A
#define LSM6DS3TR_REG_WAKE_UP_THS              0x5B
#define LSM6DS3TR_REG_WAKE_UP_DUR              0x5C
#define LSM6DS3TR_REG_FREE_FALL                0x5D
#define LSM6DS3TR_REG_MD1_CFG                  0x5E
#define LSM6DS3TR_REG_MD2_CFG                  0x5F
#define LSM6DS3TR_REG_MASTER_CMD_CODE          0x60
#define LSM6DS3TR_REG_SENS_SYNC_SPI_ERROR_CODE 0x61
#define LSM6DS3TR_REG_OUT_MAG_RAW_X_L          0x66
#define LSM6DS3TR_REG_OUT_MAG_RAW_X_H          0x67
#define LSM6DS3TR_REG_OUT_MAG_RAW_Y_L          0x68
#define LSM6DS3TR_REG_OUT_MAG_RAW_Y_H          0x69
#define LSM6DS3TR_REG_OUT_MAG_RAW_Z_L          0x6A
#define LSM6DS3TR_REG_OUT_MAG_RAW_Z_H          0x6B
#define LSM6DS3TR_REG_X_OFS_USR                0x73
#define LSM6DS3TR_REG_Y_OFS_USR                0x74
#define LSM6DS3TR_REG_Z_OFS_USR                0x75
/*********************************************************/

/********************* Register Bitmaps ******************/
// FIFO_CTRL5
#define LSM6DS3TR_BIT_FIFO_MODE_SHIFT 0
#define LSM6DS3TR_BIT_FIFO_ODR_SHIFT  3

// CTRL1_XL
#define LSM6DS3TR_BIT_ODR_XL_SHIFT 4
#define LSM6DS3TR_BIT_FS_XL_SHIFT  2

// CTRL2_G
#define LSM6DS3TR_BIT_ODR_G_SHIFT 4
#define LSM6DS3TR_BIT_FS_G_SHIFT  2

// CTRL3_C
#define LSM6DS3TR_BIT_BOOT      0x80
#define LSM6DS3TR_BIT_BDU       0x40
#define LSM6DS3TR_BIT_H_LACTIVE 0x20
#define LSM6DS3TR_BIT_PP_OD     0x10
#define LSM6DS3TR_BIT_SIM       0x08
#define LSM6DS3TR_BIT_IF_INC    0x04
#define LSM6DS3TR_BIT_BLE       0x02
#define LSM6DS3TR_BIT_SW_RESET  0x01

// CTRL4_C
#define LSM6DS3TR_BIT_DEN_XL_EN     0x80
#define LSM6DS3TR_BIT_SLEEP         0x40
#define LSM6DS3TR_BIT_DEN_DRDY_INT1 0x20
#define LSM6DS3TR_BIT_INT2_ON_INT1  0x10
#define LSM6DS3TR_BIT_DRDY_MASK     0x08
#define LSM6DS3TR_BIT_I2C_DISABLE   0x04
#define LSM6DS3TR_BIT_LPF1_SEL_G    0x02

// CTRL5_C
#define LSM6DS3TR_BIT_ROUNDING_SHIFT 5
#define LSM6DS3TR_BIT_DEN_LH         0x10
#define LSM6DS3TR_BIT_ST_G_NORM      0x00
#define LSM6DS3TR_BIT_ST_G_POS       0x04
#define LSM6DS3TR_BIT_ST_G_NEG       0x0C
#define LSM6DS3TR_BIT_ST_XL_NORM     0x00
#define LSM6DS3TR_BIT_ST_XL_POS      0x01
#define LSM6DS3TR_BIT_ST_XL_NEG      0x02

// CTRL6_C
#define LSM6DS3TR_BIT_TRIG_EN    0x80
#define LSM6DS3TR_BIT_LVL_EN     0x40
#define LSM6DS3TR_BIT_LVL2_EN    0x20
#define LSM6DS3TR_BIT_XL_HM_MODE 0x10
#define LSM6DS3TR_BIT_USR_OFF_W  0x08
#define LSM6DS3TR_BIT_FTYPE_MASK 0x03

// CTRL7_G
#define LSM6DS3TR_BIT_G_HM_MODE       0x80
#define LSM6DS3TR_BIT_HP_EN_G         0x40
#define LSM6DS3TR_BIT_HPM_G_MASK      0x30
#define LSM6DS3TR_BIT_HPM_G_SHIFT     4
#define LSM6DS3TR_BIT_ROUNDING_STATUS 0x04

// CTRL8_XL
#define LSM6DS3TR_BIT_LPF2_XL_EN      0x80
#define LSM6DS3TR_BIT_HPCF_XL1        0x40
#define LSM6DS3TR_BIT_HPCF_XL0        0x20
#define LSM6DS3TR_BIT_HP_REF_MODE     0x10
#define LSM6DS3TR_BIT_INPUT_COMPOSITE 0x08
#define LSM6DS3TR_BIT_HP_SLOPE_XL_EN  0x04
#define LSM6DS3TR_BIT_LOW_PASS_ON_6D  0x01

// STATUS
#define LSM6DS3TR_BIT_STATUS_TDA  0x04
#define LSM6DS3TR_BIT_STATUS_GDA  0x02
#define LSM6DS3TR_BIT_STATUS_XLDA 0x01
/*********************************************************/

/******************** Read-Only Registers ****************/
// WHO_AM_I
#define LSM6DS3TR_ID 0x6A
/*********************************************************/

/****************** Mathematical Constants ***************/
#define LSM6DS3TR_CONST_G    9.80665
#define LSM6DS3TR_DEG_TO_RAD 0.01745329
/*********************************************************/

// return types
typedef enum : uint8_t
{
    LSM6DS3TR_OK = 0,
    LSM6DS3TR_INVALID_CFG,
    LSM6DS3TR_TIMEOUT,
    LSM6DS3TR_NULL_POINTER,
    LSM6DS3TR_NOT_INITIALIZED,
} lsm6ds3tr_status_t;

// filter selections for acclerometer and gyroscope
typedef enum : uint8_t
{
    LSM6DS3TR_NO_FILTER = 0,
    LSM6DS3TR_LPF_EN,
    LSM6DS3TR_HPF_EN,
    LSM6DS3TR_BPF_EN
} lsm6ds3tr_filter_mode_t;

// gyroscope highpass filter cutoff selection
typedef enum : uint8_t
{
    LSM6DS3TR_HP_G_16MILHZ = 0,
    LSM6DS3TR_HP_G_65MILHZ,
    LSM6DS3TR_HP_G_260MILHZ,
    LSM6DS3TR_HP_G_1_04HZ
} lsm6ds3tr_gyro_highpass_cutoff_t;

// gyroscope output data rates
typedef enum : uint8_t
{
    LSM6DS3TR_ODR_G_PWR_DOWN = 0,
    LSM6DS3TR_ODR_G_12_5HZ,
    LSM6DS3TR_ODR_G_26HZ,
    LSM6DS3TR_ODR_G_52HZ,
    LSM6DS3TR_ODR_G_104HZ,
    LSM6DS3TR_ODR_G_208HZ,
    LSM6DS3TR_ODR_G_416HZ,
    LSM6DS3TR_ODR_G_833HZ,
    LSM6DS3TR_ODR_G_1_66KHZ,
    LSM6DS3TR_ODR_G_3_33KHZ,
    LSM6DS3TR_ODR_G_6_66KHZ
} lsm6ds3tr_gyro_odr_t;

// accelerometer output data rates
typedef enum : uint8_t
{
    LSM6DS3TR_ODR_XL_PWR_DOWN = 0,
    LSM6DS3TR_ODR_XL_12_5HZ,
    LSM6DS3TR_ODR_XL_26HZ,
    LSM6DS3TR_ODR_XL_52HZ,
    LSM6DS3TR_ODR_XL_104HZ,
    LSM6DS3TR_ODR_XL_208HZ,
    LSM6DS3TR_ODR_XL_416HZ,
    LSM6DS3TR_ODR_XL_833HZ,
    LSM6DS3TR_ODR_XL_1_66KHZ,
    LSM6DS3TR_ODR_XL_3_33KHZ,
    LSM6DS3TR_ODR_XL_6_66KHZ,
    LSM6DS3TR_ODR_XL_1_6HZ
} lsm6ds3tr_accel_odr_t;

// gyroscope full-scale selection
typedef enum : uint8_t
{
    LSM6DS3TR_FS_G_245DPS = 0,
    LSM6DS3TR_FS_G_500DPS,
    LSM6DS3TR_FS_G_1000DPS,
    LSM6DS3TR_FS_G_2000DPS
} lsm6ds3tr_gyro_fullscale_t;

// accelerometer full-scale selection
typedef enum : uint8_t
{
    LSM6DS3TR_FS_XL_2G = 0,
    LSM6DS3TR_FS_XL_16G,
    LSM6DS3TR_FS_XL_4G,
    LSM6DS3TR_FS_XL_8G
} lsm6ds3tr_accel_fullscale_t;

// FIFO modes
typedef enum : uint8_t
{
    LSM6DS3TR_FIFO_BYPASS = 0,
    LSM6DS3TR_FIFO_FIFO,
    LSM6DS3TR_FIFO_CONT_TO_FIFO = 3,
    LSM6DS3TR_FIFO_BYPASS_TO_CONT,
    LSM6DS3TR_FIFO_CONT = 6,
} lsm6ds3tr_fifo_mode_t;

// FIFO output data rates
typedef enum : uint8_t
{
    LSM6DS3TR_ODR_FIFO_DISABLE = 0,
    LSM6DS3TR_ODR_FIFO_12_5HZ,
    LSM6DS3TR_ODR_FIFO_26HZ,
    LSM6DS3TR_ODR_FIFO_52HZ,
    LSM6DS3TR_ODR_FIFO_104HZ,
    LSM6DS3TR_ODR_FIFO_208HZ,
    LSM6DS3TR_ODR_FIFO_416HZ,
    LSM6DS3TR_ODR_FIFO_833HZ,
    LSM6DS3TR_ODR_FIFO_1_66KHZ,
    LSM6DS3TR_ODR_FIFO_3_33KHZ,
    LSM6DS3TR_ODR_FIFO_6_66KHZ
} lsm6ds3tr_fifo_odr_t;

// INT1 event selection
typedef enum : uint8_t
{
    LSM6DS3TR_INT1_NONE = 0,
    LSM6DS3TR_INT1_DRDY_XL = 1,        // accelerometer data ready
    LSM6DS3TR_INT1_DRDY_G = 2,         // gyroscope data ready
    LSM6DS3TR_INT1_BOOT = 4,           // boot status
    LSM6DS3TR_INT1_FTH = 8,            // FIFO threshold
    LSM6DS3TR_INT1_FIFO_OVR = 16,      // FIFO overrun
    LSM6DS3TR_INT1_FULL_FLAG = 32,     // FIFO full flag
    LSM6DS3TR_INT1_SIGN_MOT = 64,      // significant motion
    LSM6DS3TR_INT1_STEP_DETECTOR = 128 // pedometer step recognition
} lsm6ds3tr_int1_sel_t;

// INT2 event selection
typedef enum : uint8_t
{
    LSM6DS3TR_INT2_NONE = 0,
    LSM6DS3TR_INT2_DRDY_XL = 1,        // accelerometer data ready
    LSM6DS3TR_INT2_DRDY_G = 2,         // gyroscope data ready
    LSM6DS3TR_INT2_DRDY_TEMP = 4,      // temperature data ready
    LSM6DS3TR_INT2_FTH = 8,            // FIFO threshold
    LSM6DS3TR_INT2_FIFO_OVR = 16,      // FIFO overrun
    LSM6DS3TR_INT2_FULL_FLAG = 32,     // FIFO full flag
    LSM6DS3TR_INT2_STEP_COUNT_OV = 64, // step counter overflow
    LSM6DS3TR_INT2_STEP_DELTA = 128    // pedometer step recognition on delta time
} lsm6ds3tr_int2_sel_t;

typedef struct
{
    SPI_HandleTypeDef * hspi;   // SPI handle

    GPIO_TypeDef * cs_gpio;     // SPI CS GPIO port
    uint16_t cs;                // SPI CS pin

    void (*delay_ms)(uint32_t); // pointer to delay function

    struct
    {
        lsm6ds3tr_gyro_odr_t odr;                // gyro output data rate
        lsm6ds3tr_gyro_fullscale_t fs;           // gyro full-scale range
        lsm6ds3tr_filter_mode_t filt;            // filter selection
        lsm6ds3tr_gyro_highpass_cutoff_t cutoff; // high pass filter cutoff
    } gyro_handle_t;

    struct
    {
        lsm6ds3tr_accel_odr_t odr;      // accel output data rate
        lsm6ds3tr_accel_fullscale_t fs; // accel full-scale range
        lsm6ds3tr_filter_mode_t filt;   // filter selection
    } accel_handle_t;

    struct
    {
        lsm6ds3tr_fifo_odr_t odr;   // FIFO odr must be < accel and gyro
        lsm6ds3tr_fifo_mode_t mode; // FIFO mode
    } fifo_handle_t;

    struct
    {
        lsm6ds3tr_int1_sel_t int1; // INT1 event selection
        lsm6ds3tr_int2_sel_t int2; // INT2 event selection
    } int_handle_t;

    bool _initialized;

} lsm6ds3tr_handle_t;

/* Initialize gyroscope + accelerometer, selecting filter types and output rates over SPI */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if initialization successful */
lsm6ds3tr_status_t lsm6ds3tr_init(lsm6ds3tr_handle_t * imu);

/* Check if new accelerometer data is available */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] drdy - true if accelerometer data available */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_is_accel_data_ready(const lsm6ds3tr_handle_t * imu, bool * drdy);

/* Check if new gyroscope data is available */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] drdy - true if gyroscope data available */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_is_gyro_data_ready(const lsm6ds3tr_handle_t * imu, bool * drdy);

/* Read raw accelerometer data, then convert to m/s^2 */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] a_x - acceleleration in x axis */
/* [output] a_y - acceleleration in y axis */
/* [output] a_z - acceleleration in z axis */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_accel_mps2(const lsm6ds3tr_handle_t * imu, float * a_x, float * a_y, float * a_z);

/* Read raw accelerometer data */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] a_x - acceleleration in x axis */
/* [output] a_y - acceleleration in y axis */
/* [output] a_z - acceleleration in z axis */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_accel_raw(const lsm6ds3tr_handle_t * imu, int16_t * a_x_lsb, int16_t * a_y_lsb, int16_t * a_z_lsb);

/* Read raw gyroscope data, then convert to rad/s */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] w_x - angular rate in x axis */
/* [output] w_y - angular rate in y axis */
/* [output] w_z - angular rate in z axis */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_gyro_radps(const lsm6ds3tr_handle_t * imu, float * w_x, float * w_y, float * w_z);

/* Read raw gyroscope data, then convert to deg/s */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] w_x - angular rate in x axis */
/* [output] w_y - angular rate in y axis */
/* [output] w_z - angular rate in z axis */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_gyro_degps(const lsm6ds3tr_handle_t * imu, float * w_x, float * w_y, float * w_z);

/* Read raw gyroscope data */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] w_x - angular rate in x axis */
/* [output] w_y - angular rate in y axis */
/* [output] w_z - angular rate in z axis */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_gyro_raw(const lsm6ds3tr_handle_t * imu, int16_t * w_x_lsb, int16_t * w_y_lsb, int16_t * w_z_lsb);

/* Turn off gyroscope */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_gyro_powerdown(const lsm6ds3tr_handle_t * imu);

/* Turn off accelerometer */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_accel_powerdown(const lsm6ds3tr_handle_t * imu);

/* Turn on gyroscope */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_gyro_powerup(const lsm6ds3tr_handle_t * imu);

/* Turn on accelerometer */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_accel_powerup(const lsm6ds3tr_handle_t * imu);

/* Read status register */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [output] status - result from STATUS */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_read_status(const lsm6ds3tr_handle_t * imu, uint8_t * status);

#endif