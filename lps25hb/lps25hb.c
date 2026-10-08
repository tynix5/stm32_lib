#include <stdint.h>
#include "lps25hb.h"
#include "lps25hb_hal.h"

/********************************************************* Helper Functions ************************************************************/
/* Software reset to power-on configuration */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if process is successful */
static lps25hb_status_t lps25hb_sw_reset(const lps25hb_handle_t * bar);

/* Reboot memory content */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if process is successful */
static lps25hb_status_t lps25hb_reboot(const lps25hb_handle_t * bar);

/* Configure output data rate for barometer */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if process is successful */
static lps25hb_status_t lps25hb_set_odr(const lps25hb_handle_t * bar);

/* Configure FIFO modes */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if process is successful */
static lps25hb_status_t lps25hb_set_fifo_mode(const lps25hb_handle_t * bar);

/* Configure interrupt events */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if process is successful */
static lps25hb_status_t lps25hb_set_it(const lps25hb_handle_t * bar);
/***************************************************************************************************************************************/

lps25hb_status_t lps25hb_init(lps25hb_handle_t * bar)
{
    if (bar->hspi == NULL || bar->delay_ms == NULL)
        return LPS25HB_NULL_POINTER;

    lps25hb_hal_deselect(bar);

    lps25hb_sw_reset(bar);
    lps25hb_reboot(bar);
    lps25hb_powerup(bar);

    lps25hb_status_t verify_status = lps25hb_hal_verify_id(bar);
    if (verify_status != LPS25HB_OK)
        return verify_status;

    lps25hb_status_t odr_status = lps25hb_set_odr(bar);
    if (odr_status != LPS25HB_OK)
        return odr_status;

    lps25hb_status_t fifo_status = lps25hb_set_fifo_mode(bar);
    if (fifo_status != LPS25HB_OK)
        return fifo_status;

    lps25hb_status_t it_status = lps25hb_set_it(bar);
    if (it_status != LPS25HB_OK)
        return it_status;

    // enable block update
    uint8_t ctrl_reg1;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG1, &ctrl_reg1);
    ctrl_reg1 |= LPS25HB_BIT_BDU;
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG1, ctrl_reg1);

    bar->_initialized = true;

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_trigger_sample(const lps25hb_handle_t * bar)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == LPS25HB_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2;
        lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG2, &ctrl_reg2);
        ctrl_reg2 |= LPS25HB_BIT_ONE_SHOT; // enable one shot --> begins transaction
        lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG2, ctrl_reg2);

        return LPS25HB_OK;
    }

    return LPS25HB_INVALID_CFG;
}

lps25hb_status_t lps25hb_is_press_data_ready(const lps25hb_handle_t * bar, bool * drdy)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    uint8_t status_reg;

    lps25hb_status_t status = lps25hb_read_status(bar, &status_reg);
    if (status != LPS25HB_OK)
        return status;

    if (status_reg & LPS25HB_BIT_P_DA)
        *drdy = true;
    else
        *drdy = false;

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_is_temp_data_ready(const lps25hb_handle_t * bar, bool * drdy)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    uint8_t status_reg;

    lps25hb_status_t status = lps25hb_read_status(bar, &status_reg);
    if (status != LPS25HB_OK)
        return status;

    if (status_reg & LPS25HB_BIT_T_DA)
        *drdy = true;
    else
        *drdy = false;

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_read_temp(const lps25hb_handle_t * bar, float * temp_c)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    uint8_t t[2];

    lps25hb_status_t status = lps25hb_hal_read(bar, LPS25HB_REG_TEMP_OUT_L, 2, t); // read XL, L, and H registers
    if (status != LPS25HB_OK)
        return status;

    int16_t temp_lsb = (t[1] << 8) | t[0]; // build 16-bit temp value

    // TN1228 application note
    *temp_c = LPS25HB_TEMP_OFFSET + (float)temp_lsb * LPS25HB_TEMP_SCALING_FACTOR;

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_read_press_lsb(const lps25hb_handle_t * bar, int32_t * press_lsb)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    uint8_t p[3];

    lps25hb_status_t status = lps25hb_hal_read(bar, LPS25HB_REG_PRESS_OUT_XL, 3, p); // read XL, L, and H registers
    if (status != LPS25HB_OK)
        return status;

    *press_lsb = (p[2] << 16) | (p[1] << 8) | p[0]; // create 24-bit pressure value

    // if PRESS_OUT_H is negative, make sure press_lsb is negative
    // check this
    if (p[2] & 0x80)
    {
        *press_lsb |= 0xff000000; // convert to negative
    }

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_read_press_hpa(const lps25hb_handle_t * bar, float * hpa)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    if (!bar->_initialized)
        return LPS25HB_NOT_INITIALIZED;

    int32_t press_lsb;

    lps25hb_status_t status = lps25hb_read_press_lsb(bar, &press_lsb);
    if (status != LPS25HB_OK)
        return status;

    // Figure 5 datasheet
    *hpa = (float)press_lsb * LPS25HB_PRESS_SCALING_FACTOR;

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_powerdown(const lps25hb_handle_t * bar)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    uint8_t ctrl_reg1;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG1, &ctrl_reg1);
    ctrl_reg1 &= ~LPS25HB_BIT_PD; // power-down mode
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG1, ctrl_reg1);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_powerup(const lps25hb_handle_t * bar)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;

    uint8_t ctrl_reg1;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG1, &ctrl_reg1);
    ctrl_reg1 |= LPS25HB_BIT_PD; // active mode
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG1, ctrl_reg1);

    return LPS25HB_OK;
}

lps25hb_status_t lps25hb_read_status(const lps25hb_handle_t * bar, uint8_t * status)
{
    if (bar->hspi == NULL)
        return LPS25HB_NULL_POINTER;
    return lps25hb_hal_read_reg(bar, LPS25HB_REG_STATUS, status);
}

/**************************************** Helper Functions *******************************************/
static lps25hb_status_t lps25hb_sw_reset(const lps25hb_handle_t * bar)
{
    uint8_t ctrl_reg2;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG2, &ctrl_reg2);
    ctrl_reg2 |= LPS25HB_BIT_SWRESET;
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG2, ctrl_reg2);

    // wait for software reset bit to clear
    uint8_t sw_rst;
    do
    {
        lps25hb_status_t status = lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG2, &sw_rst);
        if (status != LPS25HB_OK)
            return status;

    } while (sw_rst & LPS25HB_BIT_SWRESET);

    return LPS25HB_OK;
}

static lps25hb_status_t lps25hb_reboot(const lps25hb_handle_t * bar)
{

    uint8_t ctrl_reg2;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG2, &ctrl_reg2);
    ctrl_reg2 |= LPS25HB_BIT_BOOT;
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG2, ctrl_reg2);

    // boot process takes 2.2ms
    bar->delay_ms(5);

    return LPS25HB_OK;
}

static lps25hb_status_t lps25hb_set_odr(const lps25hb_handle_t * bar)
{
    lps25hb_odr_t odr = bar->odr;

    // check for invalid configurations
    if (odr > LPS25HB_ODR_25HZ)
        return LPS25HB_INVALID_CFG;

    // only modify ODR bits
    uint8_t ctrl_reg1;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG1, &ctrl_reg1);
    ctrl_reg1 = (ctrl_reg1 & 0x8f) | (odr << LPS25HB_BIT_ODR_SHIFT);
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG1, ctrl_reg1);

    return LPS25HB_OK;
}

static lps25hb_status_t lps25hb_set_fifo_mode(const lps25hb_handle_t * bar)
{
    lps25hb_fifo_mode_t fifo_mode = bar->fifo_handle_t.mode;
    lps25hb_fifo_avg_samp_t mov_smp = bar->fifo_handle_t.mov_smp;
    uint8_t waterline = bar->fifo_handle_t.thresh;

    // check for invalid configurations
    if (fifo_mode > LPS25HB_FIFO_BYPASS_TO_FIFO || mov_smp > LPS25HB_MOV_AVG_SMP_32 || waterline > LPS25HB_FIFO_DEPTH - 1)
        return LPS25HB_INVALID_CFG;

    uint8_t fifo_ctrl = fifo_mode << LPS25HB_BIT_FIFO_MODE_SHIFT;
    // select FIFO modes and moving sample size (if applicable)
    if (fifo_mode == LPS25HB_FIFO_FIFO_MEAN)
        fifo_ctrl |= mov_smp;
    else
        fifo_ctrl |= waterline;

    uint8_t ctrl_reg2;
    lps25hb_hal_read_reg(bar, LPS25HB_REG_CTRL_REG2, &ctrl_reg2);

    if (fifo_mode != LPS25HB_FIFO_BYPASS)
        ctrl_reg2 |= LPS25HB_BIT_FIFO_EN;     // enable FIFO
    else
        ctrl_reg2 &= ~LPS25HB_BIT_FIFO_EN;    // disable FIFO

    if (fifo_mode == LPS25HB_FIFO_FIFO || fifo_mode == LPS25HB_FIFO_STREAM_TO_FIFO)
        ctrl_reg2 |= LPS25HB_BIT_STOP_ON_FTH; // enable FIFO waterline
    else
        ctrl_reg2 &= ~LPS25HB_BIT_STOP_ON_FTH;

    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG2, ctrl_reg2);
    lps25hb_hal_write_reg(bar, LPS25HB_REG_FIFO_CTRL, fifo_ctrl);

    return LPS25HB_OK;
}

static lps25hb_status_t lps25hb_set_it(const lps25hb_handle_t * bar)
{
    lps25hb_irq_t it = bar->it;

    if (it > (LPS25HB_INT_DRDY | LPS25HB_INT_F_OVR | LPS25HB_INT_F_FTH | LPS25HB_INT_F_EMPTY))
        return LPS25HB_INVALID_CFG;

    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG4, it); // select interrupt type
    lps25hb_hal_write_reg(bar, LPS25HB_REG_CTRL_REG3, 0);  // data signal to INT_DRDY pin, active high interrupt, push/pull output

    return LPS25HB_OK;
}
/*****************************************************************************************************/
