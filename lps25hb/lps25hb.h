#ifndef LPS25HB_H_
#define LPS25HB_H_

#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "generic_types.h"

#define BAR_READ                 0x80
#define BAR_WRITE                0x00
#define BAR_ADDR_INC             0x40

/********************* Register Map **********************/
#define BAR_REG_REF_P_XL         0x08
#define BAR_REG_REF_P_L          0x09
#define BAR_REG_REF_P_H          0x0A
#define BAR_REG_WHO_AM_I         0x0F
#define BAR_REG_RES_CONF         0x10
#define BAR_REG_CTRL_REG1        0x20
#define BAR_REG_CTRL_REG2        0x21
#define BAR_REG_CTRL_REG3        0x22
#define BAR_REG_CTRL_REG4        0x23
#define BAR_REG_INTERRUPT_CFG    0x24
#define BAR_REG_INT_SOURCE       0x25
#define BAR_REG_STATUS           0x27
#define BAR_REG_PRESS_OUT_XL     0x28
#define BAR_REG_PRESS_OUT_L      0x29
#define BAR_REG_PRESS_OUT_H      0x2A
#define BAR_REG_TEMP_OUT_L       0x2B
#define BAR_REG_TEMP_OUT_H       0x2C
#define BAR_REG_FIFO_CTRL        0x2E
#define BAR_REG_FIFO_STATUS      0x2F
#define BAR_REG_THS_P_L          0x30
#define BAR_REG_THS_P_H          0x31
#define BAR_REG_RPDS_L           0x39
#define BAR_REG_RPDS_H           0x3A
/*********************************************************/

/********************* Register Bitmaps ******************/
// WHO_AM_I
#define BAR_ID                   0xBD

// RES_CONF
#define BAR_AVGT1                0x08
#define BAR_AVGT0                0x04
#define BAR_AVGP1                0x02
#define BAR_AVGP0                0x01

// CTRL_REG1
#define BAR_BIT_PD               0x80
#define BAR_BIT_ODR2             0x40
#define BAR_BIT_ODR1             0x20
#define BAR_BIT_ODR0             0x10
#define BAR_BIT_DIFF_EN          0x08
#define BAR_BIT_BDU              0x04
#define BAR_BIT_RESET_AZ         0x02
#define BAR_BIT_SIM              0x01

#define BAR_BIT_ODR_SHIFT        4

// CTRL_REG2
#define BAR_BIT_BOOT             0x80
#define BAR_BIT_FIFO_EN          0x40
#define BAR_BIT_STOP_ON_FTH      0x20
#define BAR_BIT_FIFO_MEAN_DEC    0x10
#define BAR_BIT_I2C_DIS          0x08
#define BAR_BIT_SWRESET          0x04
#define BAR_BIT_AUTOZERO         0x02
#define BAR_BIT_ONE_SHOT         0x01

// CTRL_REG3
#define BAR_BIT_INT_H_L          0x80
#define BAR_BIT_PP_OD            0x40
#define BAR_BIT_INT_S2           0x02
#define BAR_BIT_INT_S1           0x01

#define BAR_BIT_INT_D_SIG        0
#define BAR_BIT_INT_P_HIGH       (BAR_INT_S1)
#define BAR_BIT_INT_P_LOW        (BAR_INT_S2)
#define BAR_BIT_INT_P_LOW_HIGH   (BAR_INT_S2 | BAR_INT_S1)

// CTRL_REG4
#define BAR_BIT_F_EMPTY          0x08
#define BAR_BIT_F_FTH            0x04
#define BAR_BIT_F_OVR            0x02
#define BAR_BIT_DRDY             0x01

// INTERRUPT_CFG
#define BAR_BIT_LIR              0x04
#define BAR_BIT_PL_E             0x02
#define BAR_BIT_PH_E             0x01

// INT_SOURCE
#define BAR_BIT_IA               0x04
#define BAR_BIT_PL               0x02
#define BAR_BIT_PH               0x01

// STATUS_REG
#define BAR_BIT_P_OR             0x20
#define BAR_BIT_T_OR             0x10
#define BAR_BIT_P_DA             0x02
#define BAR_BIT_T_DA             0x01

// FIFO_CTRL
#define BAR_BIT_F_MODE2          0x80
#define BAR_BIT_F_MODE1          0x40
#define BAR_BIT_F_MODE0          0x20
#define BAR_BIT_WTM_POINT4       0x10
#define BAR_BIT_WTM_POINT3       0x08
#define BAR_BIT_WTM_POINT2       0x04
#define BAR_BIT_WTM_POINT1       0x02
#define BAR_BIT_WTM_POINT0       0x01

#define BAR_BIT_FIFO_MODE_SHIFT  6

// FIFO_STATUS
#define BAR_BIT_FTH_FIFO         0x80
#define BAR_BIT_OVR              0x40
#define BAR_BIT_EMPTY_FIFO       0x20
#define BAR_BIT_FSS4             0x10
#define BAR_BIT_FSS3             0x08
#define BAR_BIT_FSS2             0x04
#define BAR_BIT_FSS1             0x02
#define BAR_BIT_FSS0             0x01

#define BAR_FIFO_DEPTH           32
/*********************************************************/

/****************** Mathematical Constants ***************/
#define BAR_PRESS_SCALING_FACTOR 1.0 / 4069.0
#define BAR_TEMP_OFFSET          42.5
#define BAR_TEMP_SCALING_FACTOR  1.0 / 480.0
/*********************************************************/

// FIFO mode selection
typedef enum : uint8_t
{

    BAR_FIFO_BYPASS = 0,
    BAR_FIFO_FIFO,
    BAR_FIFO_STREAM,
    BAR_FIFO_STREAM_TO_FIFO,
    BAR_FIFO_BYPASS_TO_STREAM,
    BAR_FIFO_FIFO_MEAN = 6,
    BAR_FIFO_BYPASS_TO_FIFO

} lps25_fifo_mode_t;

// output data rate selection
typedef enum : uint8_t
{

    BAR_ODR_ONE_SHOT = 0,
    BAR_ODR_1HZ,
    BAR_ODR_7HZ,
    BAR_ODR_12_5HZ,
    BAR_ODR_25HZ

} lps25_odr_t;

// Running average sample size (FIFO Mean mode ONLY)
typedef enum : uint8_t
{

    BAR_MOV_AVG_SMP_NONE = 0,
    BAR_MOV_AGV_SMP_2 = 1,
    BAR_MOV_AVG_SMP_4 = 3,
    BAR_MOV_AVG_SMP_8 = 7,
    BAR_MOV_AVG_SMP_16 = 15,
    BAR_MOV_AVG_SMP_32 = 31

} lps25_fifo_avg_samp_t;

// interrupt selections
typedef enum : uint8_t
{

    BAR_INT_DRDY = 1,   // data ready
    BAR_INT_F_OVR = 2,  // FIFO overrun
    BAR_INT_F_FTH = 4,  // FIFO threshold
    BAR_INT_F_EMPTY = 8 // FIFO empty

} lps25_irq_t;

typedef struct
{

    SPI_HandleTypeDef * hspi;   // SPI handle

    GPIO_TypeDef * cs_gpio;     // SPI CS GPIO port
    uint16_t cs;                // SPI CS pin

    void (*delay_ms)(uint32_t); // pointer to delay function

    lps25_odr_t odr;            // output data rate
    lps25_irq_t it;             // interrupt selection

    struct
    {
        lps25_fifo_mode_t mode;
        uint8_t thresh;                // only applicable for FIFO modes
        lps25_fifo_avg_samp_t mov_smp; // only applicable for FIFO_MEAN_MODE
    } fifo_handle_t;

} lps25_handle_t;

/* Initialize LPS25HB barometer over SPI, choosing FIFO/output modes and decimation */
status_t bar_init(lps25_handle_t * bar);
/* Wait until new data is available by polling status register (blocking) */
status_t bar_is_press_data_ready(lps25_handle_t * bar, uint32_t timeout_ms);
/* Wait until new data is available by polling status register (blocking) */
status_t bar_is_temp_data_ready(lps25_handle_t * bar, uint32_t timeout_ms);
/* Read temperature in degrees C */
status_t bar_read_temp(lps25_handle_t * bar, float * temp_c);
/* Read raw pressure output registers */
status_t bar_read_press_lsb(lps25_handle_t * bar, int32_t * press_lsb);
/* Read raw pressure output registers, then convert to hPa */
status_t bar_read_press_hpa(lps25_handle_t * bar, float * hpa);
/* Power down barometer */
status_t bar_powerdown(lps25_handle_t * bar);
/* Power up barometer */
status_t bar_powerup(lps25_handle_t * bar);

#endif