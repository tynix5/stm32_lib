#ifndef LPS25HB_H_
#define LPS25HB_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

#define LPS25HB_READ     0x80
#define LPS25HB_WRITE    0x00
#define LPS25HB_ADDR_INC 0x40

/********************* Register Map **********************/
#define LPS25HB_REG_REF_P_XL      0x08
#define LPS25HB_REG_REF_P_L       0x09
#define LPS25HB_REG_REF_P_H       0x0A
#define LPS25HB_REG_WHO_AM_I      0x0F
#define LPS25HB_REG_RES_CONF      0x10
#define LPS25HB_REG_CTRL_REG1     0x20
#define LPS25HB_REG_CTRL_REG2     0x21
#define LPS25HB_REG_CTRL_REG3     0x22
#define LPS25HB_REG_CTRL_REG4     0x23
#define LPS25HB_REG_INTERRUPT_CFG 0x24
#define LPS25HB_REG_INT_SOURCE    0x25
#define LPS25HB_REG_STATUS        0x27
#define LPS25HB_REG_PRESS_OUT_XL  0x28
#define LPS25HB_REG_PRESS_OUT_L   0x29
#define LPS25HB_REG_PRESS_OUT_H   0x2A
#define LPS25HB_REG_TEMP_OUT_L    0x2B
#define LPS25HB_REG_TEMP_OUT_H    0x2C
#define LPS25HB_REG_FIFO_CTRL     0x2E
#define LPS25HB_REG_FIFO_STATUS   0x2F
#define LPS25HB_REG_THS_P_L       0x30
#define LPS25HB_REG_THS_P_H       0x31
#define LPS25HB_REG_RPDS_L        0x39
#define LPS25HB_REG_RPDS_H        0x3A
/*********************************************************/

/********************* Register Bitmaps ******************/
// WHO_AM_I
#define LPS25HB_ID 0xBD

// RES_CONF
#define LPS25HB_AVGT1 0x08
#define LPS25HB_AVGT0 0x04
#define LPS25HB_AVGP1 0x02
#define LPS25HB_AVGP0 0x01

// CTRL_REG1
#define LPS25HB_BIT_PD       0x80
#define LPS25HB_BIT_ODR2     0x40
#define LPS25HB_BIT_ODR1     0x20
#define LPS25HB_BIT_ODR0     0x10
#define LPS25HB_BIT_DIFF_EN  0x08
#define LPS25HB_BIT_BDU      0x04
#define LPS25HB_BIT_RESET_AZ 0x02
#define LPS25HB_BIT_SIM      0x01

#define LPS25HB_BIT_ODR_SHIFT 4

// CTRL_REG2
#define LPS25HB_BIT_BOOT          0x80
#define LPS25HB_BIT_FIFO_EN       0x40
#define LPS25HB_BIT_STOP_ON_FTH   0x20
#define LPS25HB_BIT_FIFO_MEAN_DEC 0x10
#define LPS25HB_BIT_I2C_DIS       0x08
#define LPS25HB_BIT_SWRESET       0x04
#define LPS25HB_BIT_AUTOZERO      0x02
#define LPS25HB_BIT_ONE_SHOT      0x01

// CTRL_REG3
#define LPS25HB_BIT_INT_H_L 0x80
#define LPS25HB_BIT_PP_OD   0x40
#define LPS25HB_BIT_INT_S2  0x02
#define LPS25HB_BIT_INT_S1  0x01

#define LPS25HB_BIT_INT_D_SIG      0
#define LPS25HB_BIT_INT_P_HIGH     (LPS25HB_INT_S1)
#define LPS25HB_BIT_INT_P_LOW      (LPS25HB_INT_S2)
#define LPS25HB_BIT_INT_P_LOW_HIGH (LPS25HB_INT_S2 | LPS25HB_INT_S1)

// CTRL_REG4
#define LPS25HB_BIT_F_EMPTY 0x08
#define LPS25HB_BIT_F_FTH   0x04
#define LPS25HB_BIT_F_OVR   0x02
#define LPS25HB_BIT_DRDY    0x01

// INTERRUPT_CFG
#define LPS25HB_BIT_LIR  0x04
#define LPS25HB_BIT_PL_E 0x02
#define LPS25HB_BIT_PH_E 0x01

// INT_SOURCE
#define LPS25HB_BIT_IA 0x04
#define LPS25HB_BIT_PL 0x02
#define LPS25HB_BIT_PH 0x01

// STATUS_REG
#define LPS25HB_BIT_P_OR 0x20
#define LPS25HB_BIT_T_OR 0x10
#define LPS25HB_BIT_P_DA 0x02
#define LPS25HB_BIT_T_DA 0x01

// FIFO_CTRL
#define LPS25HB_BIT_F_MODE2    0x80
#define LPS25HB_BIT_F_MODE1    0x40
#define LPS25HB_BIT_F_MODE0    0x20
#define LPS25HB_BIT_WTM_POINT4 0x10
#define LPS25HB_BIT_WTM_POINT3 0x08
#define LPS25HB_BIT_WTM_POINT2 0x04
#define LPS25HB_BIT_WTM_POINT1 0x02
#define LPS25HB_BIT_WTM_POINT0 0x01

#define LPS25HB_BIT_FIFO_MODE_SHIFT 6

// FIFO_STATUS
#define LPS25HB_BIT_FTH_FIFO   0x80
#define LPS25HB_BIT_OVR        0x40
#define LPS25HB_BIT_EMPTY_FIFO 0x20
#define LPS25HB_BIT_FSS4       0x10
#define LPS25HB_BIT_FSS3       0x08
#define LPS25HB_BIT_FSS2       0x04
#define LPS25HB_BIT_FSS1       0x02
#define LPS25HB_BIT_FSS0       0x01

#define LPS25HB_FIFO_DEPTH 32
/*********************************************************/

/****************** Mathematical Constants ***************/
#define LPS25HB_PRESS_SCALING_FACTOR 1.0 / 4069.0
#define LPS25HB_TEMP_OFFSET          42.5
#define LPS25HB_TEMP_SCALING_FACTOR  1.0 / 480.0
/*********************************************************/

typedef enum : uint8_t
{
    LPS25HB_OK = 0,
    LPS25HB_INVALID_CFG,
    LPS25HB_TIMEOUT,
    LPS25HB_NULL_POINTER,
    LPS25HB_NOT_INITIALIZED,
} lps25hb_status_t;
// FIFO mode selection
typedef enum : uint8_t
{
    LPS25HB_FIFO_BYPASS = 0,
    LPS25HB_FIFO_FIFO,
    LPS25HB_FIFO_STREAM,
    LPS25HB_FIFO_STREAM_TO_FIFO,
    LPS25HB_FIFO_BYPASS_TO_STREAM,
    LPS25HB_FIFO_FIFO_MEAN = 6,
    LPS25HB_FIFO_BYPASS_TO_FIFO
} lps25hb_fifo_mode_t;

// output data rate selection
typedef enum : uint8_t
{
    LPS25HB_ODR_ONE_SHOT = 0,
    LPS25HB_ODR_1HZ,
    LPS25HB_ODR_7HZ,
    LPS25HB_ODR_12_5HZ,
    LPS25HB_ODR_25HZ
} lps25hb_odr_t;

// Running average sample size (FIFO Mean mode ONLY)
typedef enum : uint8_t
{
    LPS25HB_MOV_AVG_SMP_NONE = 0,
    LPS25HB_MOV_AGV_SMP_2 = 1,
    LPS25HB_MOV_AVG_SMP_4 = 3,
    LPS25HB_MOV_AVG_SMP_8 = 7,
    LPS25HB_MOV_AVG_SMP_16 = 15,
    LPS25HB_MOV_AVG_SMP_32 = 31
} lps25hb_fifo_avg_samp_t;

// interrupt selections
typedef enum : uint8_t
{
    LPS25HB_INT_DRDY = 1,   // data ready
    LPS25HB_INT_F_OVR = 2,  // FIFO overrun
    LPS25HB_INT_F_FTH = 4,  // FIFO threshold
    LPS25HB_INT_F_EMPTY = 8 // FIFO empty
} lps25hb_irq_t;

typedef struct
{
    SPI_HandleTypeDef * hspi;   // SPI handle

    GPIO_TypeDef * cs_gpio;     // SPI CS GPIO port
    uint16_t cs;                // SPI CS pin

    void (*delay_ms)(uint32_t); // pointer to delay function

    lps25hb_odr_t odr;          // output data rate
    lps25hb_irq_t it;           // interrupt selection

    struct
    {
        lps25hb_fifo_mode_t mode;
        uint8_t thresh;                  // only applicable for FIFO modes
        lps25hb_fifo_avg_samp_t mov_smp; // only applicable for FIFO_MEAN_MODE
    } fifo_handle_t;

    bool _initialized;

} lps25hb_handle_t;

/* Initialize LPS25HB barometer over SPI, choosing FIFO/output modes and decimation */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_init(lps25hb_handle_t * bar);

/* Trigger temperature and pressure sample in ONE_SHOT mode */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if sample trigger is successful, INVALID_CFG if not ONE_SHOT mode */
lps25hb_status_t lps25hb_trigger_sample(const lps25hb_handle_t * bar);

/* Check if new pressure data is available by polling status register */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] drdy - true if pressure data available, else false */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_is_press_data_ready(const lps25hb_handle_t * bar, bool * drdy);

/* Check if new temperature data is available by polling status register */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] drdy - true if temperature data available, else false */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_is_temp_data_ready(const lps25hb_handle_t * bar, bool * drdy);

/* Read temperature in degrees C */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] temp_c - temperature in degrees Celsius */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_read_temp(const lps25hb_handle_t * bar, float * temp_c);

/* Read raw pressure output registers */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] press_lsb - raw pressure value */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_read_press_lsb(const lps25hb_handle_t * bar, int32_t * press_lsb);

/* Read raw pressure output registers, then convert to hPa */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] hpa - pressure in hPa */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_read_press_hpa(const lps25hb_handle_t * bar, float * hpa);

/* Power down barometer */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_powerdown(const lps25hb_handle_t * bar);

/* Power up barometer */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_powerup(const lps25hb_handle_t * bar);

/* Read from status register */
/* [input] bar - pointer to LPS25HB structure handle */
/* [output] status - contents of STATUS reg */
/* [return] LPS25HB_OK if communication successful */
lps25hb_status_t lps25hb_read_status(const lps25hb_handle_t * bar, uint8_t * status);

#endif