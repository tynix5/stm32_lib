#ifndef LPS25HB_H_
#define LPS25HB_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

#define BAR_READ                    0x80
#define BAR_WRITE                   0x00

/********************* Register Map **********************/
#define REF_P_XL                    0x08
#define REF_P_L                     0x09
#define REF_P_H                     0x0A
#define WHO_AM_I                    0x0F
#define RES_CONF                    0x10
#define CTRL_REG1                   0x20
#define CTRL_REG2                   0x21
#define CTRL_REG3                   0x22
#define CTRL_REG4                   0x23
#define INTERRUPT_CFG               0x24
#define INT_SOURCE                  0x25
#define STATUS_REG                  0x27
#define PRESS_OUT_XL                0x28
#define PRESS_OUT_L                 0x29
#define PRESS_OUT_H                 0x2A
#define TEMP_OUT_L                  0x2B
#define TEMP_OUT_H                  0x2C
#define FIFO_CTRL                   0x2E
#define FIFO_STATUS                 0x2F
#define THS_P_L                     0x30
#define THS_P_H                     0x31
#define RPDS_L                      0x39
#define RPDS_H                      0x3A
/*********************************************************/

/********************* Register Bitmaps ******************/
// WHO_AM_I
#define BAR_ID                      0xBD

// RES_CONF
#define AVGT1                       (1 << 3)
#define AVGT0                       (1 << 2)
#define AVGP1                       (1 << 1)
#define AVGP0                       (1 << 0)

// CTRL_REG1
#define PD                          (1 << 7)
#define ODR2                        (1 << 6)
#define ODR1                        (1 << 5)
#define ODR0                        (1 << 4)
#define DIFF_EN                     (1 << 3)
#define BDU                         (1 << 2)
#define RESET_AZ                    (1 << 1)
#define SIM                         (1 << 0)

#define ODR_ONE_SHOT                0
#define ODR_1HZ                     ODR0
#define ODR_7HZ                     ODR1
#define ODR_12_5HZ                  ODR1 | ODR0
#define ODR_25HZ                    ODR2

// CTRL_REG2
#define BOOT                        (1 << 7)
#define FIFO_EN                     (1 << 6)
#define STOP_ON_FTH                 (1 << 5)
#define FIFO_MEAN_DEC               (1 << 4)
#define I2C_DIS                     (1 << 3)
#define SWRESET                     (1 << 2)
#define AUTOZERO                    (1 << 1)
#define ONE_SHOT                    (1 << 0)

#define DECIMATION_ON               FIFO_MEAN_DEC
#define DECIMATION_OFF              0

// CTRL_REG3
#define INT_H_L                     (1 << 7)
#define PP_OD                       (1 << 6)
#define INT_S2                      (1 << 1)
#define INT_S1                      (1 << 0)

#define INT_D_SIG                   0
#define INT_P_HIGH                  INT_S1
#define INT_P_LOW                   INT_S2
#define INT_P_LOW_HIGH              INT_S2 | INT_S1

// CTRL_REG4
#define F_EMPTY                     (1 << 3)
#define F_FTH                       (1 << 2)
#define F_OVR                       (1 << 1)
#define DRDY                        (1 << 0)

// INTERRUPT_CFG
#define LIR                         (1 << 2)
#define PL_E                        (1 << 1)
#define PH_E                        (1 << 0)

// INT_SOURCE
#define IA                          (1 << 2)
#define PL                          (1 << 1)
#define PH                          (1 << 0)

// STATUS_REG
#define P_OR                        (1 << 5)
#define T_OR                        (1 << 4)
#define P_DA                        (1 << 1)
#define T_DA                        (1 << 0)

// FIFO_CTRL
#define F_MODE2                     (1 << 7)
#define F_MODE1                     (1 << 6)
#define F_MODE0                     (1 << 5)
#define WTM_POINT4                  (1 << 4)
#define WTM_POINT3                  (1 << 3)
#define WTM_POINT2                  (1 << 2)
#define WTM_POINT1                  (1 << 1)
#define WTM_POINT0                  (1 << 0)

#define BYPASS_MODE                 0
#define FIFO_MODE                   F_MODE0
#define STREAM_MODE                 F_MODE1
#define STREAM_TO_FIFO_MODE         F_MODE1 | F_MODE0
#define BYPASS_TO_STREAM_MODE       F_MODE2
#define FIFO_MEAN_MODE              F_MODE2 | F_MODE1
#define BYPASS_TO_FIFO_MODE         F_MODE2 | F_MODE1 | F_MODE0

#define MOV_AVG_SAMP_2              WTM_POINT0
#define MOV_AVG_SAMP_4              WTM_POINT1 | WTM_POINT0
#define MOV_AVG_SAMP_8              WTM_POINT2 | WTM_POINT1 | WTM_POINT0
#define MOV_AVG_SAMP_16             WTM_POINT3 | WTM_POINT2 | WTM_POINT1 | WTM_POINT0
#define MOV_AVG_SAMP_32             WTM_POINT4 | WTM_POINT3 | WTM_POINT2 | WTM_POINT1 | WTM_POINT0

// FIFO_STATUS
#define FTH_FIFO                    (1 << 7)
#define OVR                         (1 << 6)
#define EMPTY_FIFO                  (1 << 5)
#define FSS4                        (1 << 4)
#define FSS3                        (1 << 3)
#define FSS2                        (1 << 2)
#define FSS1                        (1 << 1)
#define FSS0                        (1 << 0)
/*********************************************************/

/****************** Mathematical Constants ***************/
#define SCALING_FACTOR              1.0 / 4069.0
/*********************************************************/

typedef struct {

    SPI_HandleTypeDef * hspi;

    GPIO_TypeDef * cs_gpio;
    uint16_t cs;

    uint8_t odr;                    // output data rate

    uint8_t fifo_mode;
    uint8_t fifo_thresh;            // only applicable for FIFO modes
    uint8_t fifo_moving_avg_size;         // only applicable for FIFO_MEAN_MODE
    uint8_t decimate;

} BarParams;

void bar_init(BarParams * bar);
void bar_is_data_ready(BarParams * bar);            // returns when new data is available
int32_t bar_read_press_lsb(BarParams * bar);        // retrieves raw data from registers
float bar_read_press_hpa(BarParams * bar);          // reads and converts pressure data to hPa
void bar_powerdown(BarParams * bar);                // power-down sensor
void bar_powerup(BarParams * bar);                  // turn sensor on

#endif