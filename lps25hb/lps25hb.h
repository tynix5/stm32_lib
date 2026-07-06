#ifndef LPS25HB_H_
#define LPS25HB_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "error_types.h"

#define BAR_READ                        0x80
#define BAR_WRITE                       0x00
#define BAR_ADDR_INC                    0x40

/********************* Register Map **********************/
#define BAR_REF_P_XL                    0x08
#define BAR_REF_P_L                     0x09
#define BAR_REF_P_H                     0x0A
#define BAR_WHO_AM_I                    0x0F
#define BAR_RES_CONF                    0x10
#define BAR_CTRL_REG1                   0x20
#define BAR_CTRL_REG2                   0x21
#define BAR_CTRL_REG3                   0x22
#define BAR_CTRL_REG4                   0x23
#define BAR_INTERRUPT_CFG               0x24
#define BAR_INT_SOURCE                  0x25
#define BAR_STATUS_REG                  0x27
#define BAR_PRESS_OUT_XL                0x28
#define BAR_PRESS_OUT_L                 0x29
#define BAR_PRESS_OUT_H                 0x2A
#define BAR_TEMP_OUT_L                  0x2B
#define BAR_TEMP_OUT_H                  0x2C
#define BAR_FIFO_CTRL                   0x2E
#define BAR_FIFO_STATUS                 0x2F
#define BAR_THS_P_L                     0x30
#define BAR_THS_P_H                     0x31
#define BAR_RPDS_L                      0x39
#define BAR_RPDS_H                      0x3A
/*********************************************************/

/********************* Register Bitmaps ******************/
// WHO_AM_I
#define BAR_ID                          0xBD

// RES_CONF
#define BAR_AVGT1                       (1 << 3)
#define BAR_AVGT0                       (1 << 2)
#define BAR_AVGP1                       (1 << 1)
#define BAR_AVGP0                       (1 << 0)

// CTRL_REG1
#define BAR_PD                          (1 << 7)
#define BAR_ODR2                        (1 << 6)
#define BAR_ODR1                        (1 << 5)
#define BAR_ODR0                        (1 << 4)
#define BAR_DIFF_EN                     (1 << 3)
#define BAR_BDU                         (1 << 2)
#define BAR_RESET_AZ                    (1 << 1)
#define BAR_SIM                         (1 << 0)

#define BAR_ODR_ONE_SHOT                0
#define BAR_ODR_1HZ                     (BAR_ODR0)
#define BAR_ODR_7HZ                     (BAR_ODR1)
#define BAR_ODR_12_5HZ                  (BAR_ODR1 | BAR_ODR0)
#define BAR_ODR_25HZ                    (BAR_ODR2)

// CTRL_REG2
#define BAR_BOOT                        (1 << 7)
#define BAR_FIFO_EN                     (1 << 6)
#define BAR_STOP_ON_FTH                 (1 << 5)
#define BAR_FIFO_MEAN_DEC               (1 << 4)
#define BAR_I2C_DIS                     (1 << 3)
#define BAR_SWRESET                     (1 << 2)
#define BAR_AUTOZERO                    (1 << 1)
#define BAR_ONE_SHOT                    (1 << 0)

#define BAR_DECIMATION_ON               BAR_FIFO_MEAN_DEC
#define BAR_DECIMATION_OFF              0

// CTRL_REG3
#define BAR_INT_H_L                     (1 << 7)
#define BAR_PP_OD                       (1 << 6)
#define BAR_INT_S2                      (1 << 1)
#define BAR_INT_S1                      (1 << 0)

#define BAR_INT_D_SIG                   0
#define BAR_INT_P_HIGH                  (BAR_INT_S1)
#define BAR_INT_P_LOW                   (BAR_INT_S2)
#define BAR_INT_P_LOW_HIGH              (BAR_INT_S2 | BAR_INT_S1)

// CTRL_REG4
#define BAR_F_EMPTY                     (1 << 3)
#define BAR_F_FTH                       (1 << 2)
#define BAR_F_OVR                       (1 << 1)
#define BAR_DRDY                        (1 << 0)

// INTERRUPT_CFG
#define BAR_LIR                         (1 << 2)
#define BAR_PL_E                        (1 << 1)
#define BAR_PH_E                        (1 << 0)

// INT_SOURCE
#define BAR_IA                          (1 << 2)
#define BAR_PL                          (1 << 1)
#define BAR_PH                          (1 << 0)

// STATUS_REG
#define BAR_P_OR                        (1 << 5)
#define BAR_T_OR                        (1 << 4)
#define BAR_P_DA                        (1 << 1)
#define BAR_T_DA                        (1 << 0)

// FIFO_CTRL
#define BAR_F_MODE2                     (1 << 7)
#define BAR_F_MODE1                     (1 << 6)
#define BAR_F_MODE0                     (1 << 5)
#define BAR_WTM_POINT4                  (1 << 4)
#define BAR_WTM_POINT3                  (1 << 3)
#define BAR_WTM_POINT2                  (1 << 2)
#define BAR_WTM_POINT1                  (1 << 1)
#define BAR_WTM_POINT0                  (1 << 0)

#define BAR_BYPASS_MODE                 0
#define BAR_FIFO_MODE                   (BAR_F_MODE0)
#define BAR_STREAM_MODE                 (BAR_F_MODE1)
#define BAR_STREAM_TO_FIFO_MODE         (BAR_F_MODE1 | BAR_F_MODE0)
#define BAR_BYPASS_TO_STREAM_MODE       (BAR_F_MODE2)
#define BAR_FIFO_MEAN_MODE              (BAR_F_MODE2 | BAR_F_MODE1)
#define BAR_BYPASS_TO_FIFO_MODE         (BAR_F_MODE2 | BAR_F_MODE1 | BAR_F_MODE0)

#define BAR_MOV_AVG_SAMP_2              (BAR_WTM_POINT0)
#define BAR_MOV_AVG_SAMP_4              (BAR_WTM_POINT1 | BAR_WTM_POINT0)
#define BAR_MOV_AVG_SAMP_8              (BAR_WTM_POINT2 | BAR_WTM_POINT1 | BAR_WTM_POINT0)
#define BAR_MOV_AVG_SAMP_16             (BAR_WTM_POINT3 | BAR_WTM_POINT2 | BAR_WTM_POINT1 | BAR_WTM_POINT0)
#define BAR_MOV_AVG_SAMP_32             (BAR_WTM_POINT4 | BAR_WTM_POINT3 | BAR_WTM_POINT2 | BAR_WTM_POINT1 | BAR_WTM_POINT0)

// FIFO_STATUS
#define BAR_FTH_FIFO                    (1 << 7)
#define BAR_OVR                         (1 << 6)
#define BAR_EMPTY_FIFO                  (1 << 5)
#define BAR_FSS4                        (1 << 4)
#define BAR_FSS3                        (1 << 3)
#define BAR_FSS2                        (1 << 2)
#define BAR_FSS1                        (1 << 1)
#define BAR_FSS0                        (1 << 0)
/*********************************************************/

/****************** Mathematical Constants ***************/
#define BAR_SCALING_FACTOR              1.0 / 4069.0
/*********************************************************/

#define SPI_TIMEOUT_MS                  5
#define SPI_RETRIES                     5

typedef struct {

    SPI_HandleTypeDef * hspi;               // SPI handle

    GPIO_TypeDef * cs_gpio;                 // SPI CS GPIO port
    uint16_t cs;                            // SPI CS pin

    void (*delay_ms)(uint32_t);             // pointer to delay function

    uint8_t odr;                            // output data rate

    uint8_t fifo_mode;
    uint8_t fifo_thresh;                    // only applicable for FIFO modes
    uint8_t fifo_moving_avg_size;           // only applicable for FIFO_MEAN_MODE
    uint8_t decimate;

} BarParams;

/* Initialize LPS25HB barometer over SPI, choosing FIFO/output modes and decimation */
ErrorType bar_init(BarParams * bar);
/* Wait until new data is available by polling status register (blocking) */
ErrorType bar_is_data_ready(BarParams * bar, uint32_t timeout_ms);
/* Read raw pressure output registers */
int32_t bar_read_press_lsb(BarParams * bar);
/* Read raw pressure output registers, then convert to hPa */
float bar_read_press_hpa(BarParams * bar);
/* Power down barometer */
ErrorType bar_powerdown(BarParams * bar);
/* Power up barometer */
ErrorType bar_powerup(BarParams * bar);

#endif