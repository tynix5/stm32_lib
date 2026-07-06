#ifndef IIS2MDC_H_
#define IIS2MDC_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "error_types.h"

#define MAG_READ                        0x80
#define MAG_WRITE                       0x00

/************************ Registers ************************/
#define MAG_OFFSET_X_REG_L              0x45
#define MAG_OFFSET_X_REG_H              0x46
#define MAG_OFFSET_Y_REG_L              0x47
#define MAG_OFFSET_Y_REG_H              0x48
#define MAG_OFFSET_Z_REG_L              0x49
#define MAG_OFFSET_Z_REG_H              0x4A
#define MAG_WHO_AM_I                    0x4F
#define MAG_CFG_REG_A                   0x60
#define MAG_CFG_REG_B                   0x61
#define MAG_CFG_REG_C                   0x62
#define MAG_INT_CTRL_REG                0x63
#define MAG_INT_SOURCE_REG              0x64
#define MAG_INT_THS_L_REG               0x65
#define MAG_INT_THS_H_REG               0x66
#define MAG_STATUS_REG                  0x67
#define MAG_OUTX_L_REG                  0x68
#define MAG_OUTX_H_REG                  0x69
#define MAG_OUTY_L_REG                  0x6A
#define MAG_OUTY_H_REG                  0x6B
#define MAG_OUTZ_L_REG                  0x6C
#define MAG_OUTZ_H_REG                  0x6D
#define MAG_TEMP_OUT_L_REG              0x6E
#define MAG_TEMP_OUT_H_REG              0x6F
/***********************************************************/

/********************** Register Bitmaps *******************/
// WHO_AM_I
#define MAG_ID                          0x40

// CFG_REG_A
#define MAG_COMP_TEMP_EN                (1 << 7)
#define MAG_REBOOT                      (1 << 6)
#define MAG_SOFT_RST                    (1 << 5)
#define MAG_LP                          (1 << 4)
#define MAG_ODR_1                       (1 << 3)
#define MAG_ODR_0                       (1 << 2)
#define MAG_MD_1                        (1 << 1)
#define MAG_MD_0                        (1 << 0)

#define MAG_ODR_10HZ                    0
#define MAG_ODR_20HZ                    MAG_ODR_0
#define MAG_ODR_50HZ                    MAG_ODR_1
#define MAG_ODR_100HZ                   MAG_ODR_0 | MAG_ODR_1

#define MAG_MODE_CONT                   0
#define MAG_MODE_SINGLE                 MAG_MD_0
#define MAG_MODE_IDLE                   MAG_MD_1

// CFG_REG_B
#define MAG_OFF_CANC_ONE_SHOT           (1 << 4)
#define MAG_INT_ON_DATA_OFF             (1 << 3)
#define MAG_SET_FREQ                    (1 << 2)
#define MAG_OFF_CANC                    (1 << 1)
#define MAG_LPF                         (1 << 0)

// CFG_REG_C
#define MAG_INT_ON_PIN                  (1 << 6)
#define MAG_I2C_DIS                     (1 << 5)
#define MAG_BDU                         (1 << 4)
#define MAG_BLE                         (1 << 3)
#define MAG_SELF_TEST                   (1 << 1)
#define MAG_DRDY_ON_PIN                 (1 << 0)

// INT_CTRL_REG
#define MAG_XIEN                        (1 << 7)
#define MAG_YIEN                        (1 << 6)
#define MAG_ZIEN                        (1 << 5)
#define MAG_IEA                         (1 << 2)
#define MAG_IEL                         (1 << 1)
#define MAG_IEN                         (1 << 0)

// INT_SOURCE_REG
#define MAG_P_TH_S_X                    (1 << 7)
#define MAG_P_TH_S_Y                    (1 << 6)
#define MAG_P_TH_S_Z                    (1 << 5)
#define MAG_N_TH_S_X                    (1 << 4)
#define MAG_N_TH_S_Y                    (1 << 3)
#define MAG_N_TH_S_Z                    (1 << 2)
#define MAG_MROI                        (1 << 1)
#define MAG_INT                         (1 << 0)

// STATUS_REG
#define MAG_ZYXOR                       (1 << 7)
#define MAG_ZOR                         (1 << 6)
#define MAG_YOR                         (1 << 5)
#define MAG_XOR                         (1 << 4)
#define MAG_ZYXDA                       (1 << 3)
#define MAG_ZDA                         (1 << 2)
#define MAG_YDA                         (1 << 1)
#define MAG_XDA                         (1 << 0)
/***********************************************************/

typedef struct {

    SPI_HandleTypeDef * hspi;           // SPI handle
    
    GPIO_TypeDef * cs_gpio;             // SPI CS GPIO port
    uint16_t cs;                        // SPI CS pin

    void (*delay_ms)(uint32_t);         // pointer to delay function
    
    uint8_t lpf;                        // lowpass filter enable/disable
    uint8_t odr;                        // output data rate
    uint8_t mode;                       // sampling mode

    uint8_t int_drdy;                   // interrupt on DRDY pin enable/disable
    
} MagParams;

/* Initiate magnetometer over SPI, choosing filter, output data rate, and modes */
ErrorType mag_init(MagParams * mag);
/* Poll magnetometer until new data is available (blocking) */
ErrorType mag_is_data_ready(MagParams * mag, uint32_t timeout_ms);
/* Read magnetometer X, Y, and Z registers */
ErrorType mag_read_raw(MagParams * mag, int16_t * magx_lsb, int16_t * magy_lsb, int16_t * magz_lsb);
/* Read magnetometer X, Y, and Z registers, then convert to gauss */
ErrorType mag_read_gauss(MagParams * mag, float * magx, float * magy, float * magz);

#endif