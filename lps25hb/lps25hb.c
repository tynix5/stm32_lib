#include <stdint.h>
#include "lps25hb.h"

/**************************************** Helper Functions *******************************************/
/* Software reset to power-on configuration */
static status_t bar_sw_reset(lps25_handle_t * bar);
/* Reboot memory content */
static status_t bar_reboot(lps25_handle_t * bar);
/* Configure output data rate for barometer */
static status_t bar_set_odr(lps25_handle_t * bar);
/* Configure FIFO modes */
static status_t bar_set_fifo_mode(lps25_handle_t * bar);
/* Configure interrupt events */
static status_t bar_set_it(lps25_handle_t * bar);
/*****************************************************************************************************/

/************************************* SPI Wrapper Functions *****************************************/
/* CS low */
static inline void bar_select(lps25_handle_t * bar);
/* CS high */
static inline void bar_deselect(lps25_handle_t * bar);
/* Write data to specific address using SPI */
static void bar_write_reg(lps25_handle_t * bar, uint8_t addr, uint8_t data);
/* Read byte from selected address using SPI */
static uint8_t bar_read_reg(lps25_handle_t * bar, uint8_t addr);
/* Multi-byte read over SPI */
static void bar_read(lps25_handle_t * bar, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Read status register */
static uint8_t bar_status(lps25_handle_t * bar);
/*****************************************************************************************************/

status_t bar_init(lps25_handle_t * bar)
{
    bar_deselect(bar);

    bar_sw_reset(bar);
    bar_reboot(bar);
    bar_powerup(bar);

    uint8_t retries = 0;
    // wait until communication is successful
    while (bar_read_reg(bar, BAR_REG_WHO_AM_I) != BAR_ID)
    {
        if (retries == SPI_MAX_RETRIES)
            return STATUS_TIMEOUT;

        retries++;
        bar->delay_ms(10);
    }

    if (bar_set_odr(bar) != STATUS_OK)
        return STATUS_INVALID;

    if (bar_set_fifo_mode(bar) != STATUS_OK)
        return STATUS_INVALID;

    if (bar_set_it(bar) != STATUS_OK)
        return STATUS_INVALID;

    // enable block update
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);
    ctrl_reg1 |= BAR_BIT_BDU;
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return STATUS_OK;
}

status_t bar_is_press_data_ready(lps25_handle_t * bar, uint32_t timeout_ms)
{
    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == BAR_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
        ctrl_reg2 |= BAR_BIT_ONE_SHOT; // enable one shot --> begins transaction
    }

    uint32_t start_tick = HAL_GetTick();

    // wait until pressure data becomes available, or timeout
    while (!(bar_status(bar) & BAR_BIT_P_DA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}

status_t bar_is_temp_data_ready(lps25_handle_t * bar, uint32_t timeout_ms)
{
    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == BAR_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
        ctrl_reg2 |= BAR_BIT_ONE_SHOT; // enable one shot --> begins transaction
    }

    uint32_t start_tick = HAL_GetTick();

    // wait until pressure data becomes available, or timeout
    while (!(bar_status(bar) & BAR_BIT_T_DA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}

status_t bar_read_temp(lps25_handle_t * bar, float * temp_c)
{
    uint8_t t[2];
    bar_read(bar, BAR_REG_TEMP_OUT_L, 2, t); // read L and H registers

    int16_t temp_lsb = (t[1] << 8) | t[0];   // build 16-bit temp value

    // TN1228 application note
    *temp_c = BAR_TEMP_OFFSET + (float)temp_lsb * BAR_TEMP_SCALING_FACTOR;

    return STATUS_OK;
}

status_t bar_read_press_lsb(lps25_handle_t * bar, int32_t * press_lsb)
{
    uint8_t p[3];
    bar_read(bar, BAR_REG_PRESS_OUT_XL, 3, p);      // read XL, L, and H registers

    *press_lsb = (p[2] << 16) | (p[1] << 8) | p[0]; // create 24-bit pressure value

    // if PRESS_OUT_H is negative, make sure press_lsb is negative
    // check this
    if (p[2] & 0x80)
    {
        *press_lsb |= 0xff000000; // convert to negative
    }

    return STATUS_OK;
}

status_t bar_read_press_hpa(lps25_handle_t * bar, float * hpa)
{
    int32_t press_lsb;

    if (bar_read_press_lsb(bar, &press_lsb) != STATUS_OK) // always returns STATUS_OK (change?)
        return STATUS_TIMEOUT;

    // Figure 5 datasheet
    *hpa = (float)press_lsb * BAR_PRESS_SCALING_FACTOR;

    return STATUS_OK;
}

status_t bar_powerdown(lps25_handle_t * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);

    ctrl_reg1 &= ~BAR_BIT_PD; // power-down mode
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return STATUS_OK;
}

status_t bar_powerup(lps25_handle_t * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);

    ctrl_reg1 |= BAR_BIT_PD; // active mode
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return STATUS_OK;
}

/**************************************** Helper Functions *******************************************/
static status_t bar_sw_reset(lps25_handle_t * bar)
{
    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
    ctrl_reg2 |= BAR_BIT_SWRESET;
    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);

    // wait for software reset bit to clear
    while (bar_read_reg(bar, BAR_REG_CTRL_REG2) & BAR_BIT_SWRESET)
        ;

    return STATUS_OK;
}

static status_t bar_reboot(lps25_handle_t * bar)
{
    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
    ctrl_reg2 |= BAR_BIT_BOOT;
    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);

    // boot process takes 2.2ms
    bar->delay_ms(5);

    return STATUS_OK;
}

static status_t bar_set_odr(lps25_handle_t * bar)
{
    lps25_odr_t odr = bar->odr;

    // check for invalid configurations
    if (odr > BAR_ODR_25HZ)
        return STATUS_INVALID;

    // only modify ODR bits
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);
    ctrl_reg1 = (ctrl_reg1 & 0x8f) | (odr << BAR_BIT_ODR_SHIFT);
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return STATUS_OK;
}

static status_t bar_set_fifo_mode(lps25_handle_t * bar)
{
    lps25_fifo_mode_t fifo_mode = bar->fifo_handle_t.mode;
    lps25_fifo_avg_samp_t mov_smp = bar->fifo_handle_t.mov_smp;
    uint8_t waterline = bar->fifo_handle_t.thresh;

    // check for invalid configurations
    if (fifo_mode > BAR_FIFO_BYPASS_TO_FIFO || mov_smp > BAR_MOV_AVG_SMP_32 || waterline > BAR_FIFO_DEPTH - 1)
        return STATUS_INVALID;

    uint8_t fifo_ctrl = fifo_mode << BAR_BIT_FIFO_MODE_SHIFT;
    // select FIFO modes and moving sample size (if applicable)
    if (fifo_mode == BAR_FIFO_FIFO_MEAN)
        fifo_ctrl |= mov_smp;
    else
        fifo_ctrl |= waterline;

    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);

    if (fifo_mode != BAR_FIFO_BYPASS)
        ctrl_reg2 |= BAR_BIT_FIFO_EN;     // enable FIFO
    else
        ctrl_reg2 &= ~BAR_BIT_FIFO_EN;    // disable FIFO

    if (fifo_mode == BAR_FIFO_FIFO || fifo_mode == BAR_FIFO_STREAM_TO_FIFO)
        ctrl_reg2 |= BAR_BIT_STOP_ON_FTH; // enable FIFO waterline
    else
        ctrl_reg2 &= ~BAR_BIT_STOP_ON_FTH;

    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);
    bar_write_reg(bar, BAR_REG_FIFO_CTRL, fifo_ctrl);

    return STATUS_OK;
}

static status_t bar_set_it(lps25_handle_t * bar)
{
    lps25_irq_t it = bar->it;

    if (it > (BAR_INT_DRDY | BAR_INT_F_OVR | BAR_INT_F_FTH | BAR_INT_F_EMPTY))
        return STATUS_INVALID;

    bar_write_reg(bar, BAR_REG_CTRL_REG4, it); // select interrupt type
    bar_write_reg(bar, BAR_REG_CTRL_REG3, 0);  // data signal to INT_DRDY pin, active high interrupt, push/pull output

    return STATUS_OK;
}
/*****************************************************************************************************/

/************************************* SPI Wrapper Functions *****************************************/
static inline void bar_select(lps25_handle_t * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_RESET);
}

static inline void bar_deselect(lps25_handle_t * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_SET);
}

static void bar_write_reg(lps25_handle_t * bar, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = { addr | BAR_WRITE, data };

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, cmd, 2, SPI_TIMEOUT_MS);
    bar_deselect(bar);
}

static uint8_t bar_read_reg(lps25_handle_t * bar, uint8_t addr)
{
    addr |= BAR_READ;

    uint8_t data;

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, &data, 1, SPI_TIMEOUT_MS);
    bar_deselect(bar);

    return data;
}

static void bar_read(lps25_handle_t * bar, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    start_addr |= BAR_READ;
    start_addr |= BAR_ADDR_INC;

    // page 28 of datasheet, bit 6 (MS) must be 1 to increment address

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &start_addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, data, len, SPI_TIMEOUT_MS);
    bar_deselect(bar);
}

static uint8_t bar_status(lps25_handle_t * bar)
{
    return bar_read_reg(bar, BAR_REG_STATUS);
}
/*****************************************************************************************************/
