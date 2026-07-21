#ifndef IIS2MDC_H_
#define IIS2MDC_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "error_types.h"

#define MAG_READ                            0x80
#define MAG_WRITE                           0x00

/************************ Registers ************************/
#define MAG_REG_OFFSET_X_REG_L              0x45
#define MAG_REG_OFFSET_X_REG_H              0x46
#define MAG_REG_OFFSET_Y_REG_L              0x47
#define MAG_REG_OFFSET_Y_REG_H              0x48
#define MAG_REG_OFFSET_Z_REG_L              0x49
#define MAG_REG_OFFSET_Z_REG_H              0x4A
#define MAG_REG_WHO_AM_I                    0x4F
#define MAG_REG_CFG_REG_A                   0x60
#define MAG_REG_CFG_REG_B                   0x61
#define MAG_REG_CFG_REG_C                   0x62
#define MAG_REG_INT_CTRL_REG                0x63
#define MAG_REG_INT_SOURCE_REG              0x64
#define MAG_REG_INT_THS_L_REG               0x65
#define MAG_REG_INT_THS_H_REG               0x66
#define MAG_REG_STATUS_REG                  0x67
#define MAG_REG_OUTX_L_REG                  0x68
#define MAG_REG_OUTX_H_REG                  0x69
#define MAG_REG_OUTY_L_REG                  0x6A
#define MAG_REG_OUTY_H_REG                  0x6B
#define MAG_REG_OUTZ_L_REG                  0x6C
#define MAG_REG_OUTZ_H_REG                  0x6D
#define MAG_REG_TEMP_OUT_L_REG              0x6E
#define MAG_REG_TEMP_OUT_H_REG              0x6F
/***********************************************************/

/********************** Register Bitmaps *******************/
// WHO_AM_I
#define MAG_ID                              0x40

// CFG_REG_A
#define MAG_BIT_COMP_TEMP_EN                0x80
#define MAG_BIT_REBOOT                      0x40
#define MAG_BIT_SOFT_RST                    0x20
#define MAG_BIT_LP                          0x10
#define MAG_BIT_ODR_1                       0x08
#define MAG_BIT_ODR_0                       0x04
#define MAG_BIT_MD_1                        0x02
#define MAG_BIT_MD_0                        0x01

#define MAG_BIT_ODR_SHIFT                   2

// CFG_REG_B
#define MAG_BIT_OFF_CANC_ONE_SHOT           0x10
#define MAG_BIT_INT_ON_DATA_OFF             0x08
#define MAG_BIT_SET_FREQ                    0x04
#define MAG_BIT_OFF_CANC                    0x02
#define MAG_BIT_LPF                         0x01

// CFG_REG_C
#define MAG_BIT_INT_ON_PIN                  0x40
#define MAG_BIT_I2C_DIS                     0x20
#define MAG_BIT_BDU                         0x10
#define MAG_BIT_BLE                         0x08
#define MAG_BIT_SELF_TEST                   0x02
#define MAG_BIT_DRDY_ON_PIN                 0x01

// INT_CTRL_REG
#define MAG_BIT_XIEN                        0x80
#define MAG_BIT_YIEN                        0x40
#define MAG_BIT_ZIEN                        0x20
#define MAG_BIT_IEA                         0x04
#define MAG_BIT_IEL                         0x02
#define MAG_BIT_IEN                         0x01

// INT_SOURCE_REG
#define MAG_BIT_P_TH_S_X                    0x80
#define MAG_BIT_P_TH_S_Y                    0x40
#define MAG_BIT_P_TH_S_Z                    0x20
#define MAG_BIT_N_TH_S_X                    0x10
#define MAG_BIT_N_TH_S_Y                    0x08
#define MAG_BIT_N_TH_S_Z                    0x04
#define MAG_BIT_MROI                        0x02
#define MAG_BIT_INT                         0x01

// STATUS_REG
#define MAG_BIT_ZYXOR                       0x80
#define MAG_BIT_ZOR                         0x40
#define MAG_BIT_YOR                         0x20
#define MAG_BIT_XOR                         0x10
#define MAG_BIT_ZYXDA                       0x08
#define MAG_BIT_ZDA                         0x04
#define MAG_BIT_YDA                         0x02
#define MAG_BIT_XDA                         0x01
/***********************************************************/

/******************** Mathematical Constants ***************/
#define MAG_MILLIGAUSS_SCALING_FACTOR       1.0 / 1.5
#define MAG_TEMP_OFFSET                     25.0
#define MAG_TEMP_SCALING_FACTOR             1.0 / 8.0
/***********************************************************/

// output data rates
typedef enum : uint8_t {

    MAG_ODR_10HZ = 0,
    MAG_ODR_20HZ,
    MAG_ODR_50HZ,
    MAG_ODR_100HZ,

} MagOdr;

// sampling modes
typedef enum : uint8_t {

    MAG_MODE_CONT = 0,
    MAG_MODE_SINGLE,
    MAG_MODE_IDLE

} MagMode;

// select whether INT or DRDY events appear on INT/DRDY pin
typedef enum : uint8_t {

    MAG_INT_DRDY_NONE,
    MAG_INT_ON_PIN,
    MAG_DRDY_ON_PIN

} MagIntDrdy;

// enabled interrupts for INT events
typedef enum : uint8_t {

    MAG_INT_IEN = 1,                        // enable interrupt
    MAG_INT_IEL = 2,                        // latched(1)/pulsed(0) interrupts
    MAG_INT_IEA = 4,                        // polarity of INT
    MAG_INT_ZIEN = 32,                      // int detection Z-axis
    MAG_INT_YIEN = 64,                      // int detection Y-axis
    MAG_INT_XIEN = 128                      // int detection X-axis

} MagIntSel;    

typedef struct {

    SPI_HandleTypeDef * hspi;               // SPI handle
    
    GPIO_TypeDef * cs_gpio;                 // SPI CS GPIO port
    uint16_t cs;                            // SPI CS pin

    void (*delay_ms)(uint32_t);             // pointer to delay function
    
    MagOdr odr;                             // output data rate
    MagMode mode;                           // continuous, single, or idle sampling
    EnableType lpf;                         // lowpass filter enable/disable
    EnableType low_pwr;                     // low-power mode

    struct {
        MagIntDrdy int_drdy;                // select INT_ON_PIN or DRDY_ON_PIN
        MagIntSel it;                       // select interrupts which appear on INT_ON_PIN (only applicable for MAG_INT_ON_PIN)
    } IntParams;
    
} MagParams;

/* Initiate magnetometer over SPI, choosing filter, output data rate, and modes */
ErrorType mag_init(MagParams * mag);
/* Poll magnetometer until new data is available (blocking) */
ErrorType mag_is_data_ready(MagParams * mag, uint32_t timeout_ms);
/* Read magnetometer X, Y, and Z registers */
ErrorType mag_read_raw(MagParams * mag, int16_t * magx_lsb, int16_t * magy_lsb, int16_t * magz_lsb);
/* Read magnetometer X, Y, and Z registers, then convert to gauss */
ErrorType mag_read_mgauss(MagParams * mag, float * magx, float * magy, float * magz);
/* Read magnetometer temp register and convert to degrees C */
ErrorType mag_read_temp(MagParams * mag, float * temp_c);

#endif