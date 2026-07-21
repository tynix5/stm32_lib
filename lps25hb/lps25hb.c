#include <stdint.h>
#include "lps25hb.h"

/**************************************** Helper Functions *******************************************/
/* Software reset to power-on configuration */
static ErrorType bar_sw_reset(BarParams * bar);
/* Reboot memory content */
static ErrorType bar_reboot(BarParams * bar);
/* Configure output data rate for barometer */
static ErrorType bar_set_odr(BarParams * bar);
/* Configure FIFO modes */
static ErrorType bar_set_fifo_mode(BarParams * bar);
/* Configure interrupt events */
static ErrorType bar_set_it(BarParams * bar);
/*****************************************************************************************************/


/************************************* SPI Wrapper Functions *****************************************/
/* CS low */
static inline void bar_select(BarParams * bar);
/* CS high */
static inline void bar_deselect(BarParams * bar);
/* Write data to specific address using SPI */
static void bar_write_reg(BarParams * bar, uint8_t addr, uint8_t data);
/* Read byte from selected address using SPI */
static uint8_t bar_read_reg(BarParams * bar, uint8_t addr);
/* Multi-byte read over SPI */
static void bar_read(BarParams * bar, uint8_t start_addr, uint8_t len, uint8_t * data);
/* Read status register */
static uint8_t bar_status(BarParams * bar);
/*****************************************************************************************************/


ErrorType bar_init(BarParams * bar)
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
            return RET_TIMEOUT;
        
        retries++;
        bar->delay_ms(10);
    }

    if (bar_set_odr(bar) != RET_OK)
        return RET_INVALID_CFG;

    if (bar_set_fifo_mode(bar) != RET_OK)
        return RET_INVALID_CFG;

    if (bar_set_it(bar) != RET_OK)
        return RET_INVALID_CFG;

    // enable block update
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);
    ctrl_reg1 |= BAR_BIT_BDU;
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return RET_OK;
}

ErrorType bar_is_press_data_ready(BarParams * bar, uint32_t timeout_ms)
{
    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == BAR_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
        ctrl_reg2 |= BAR_BIT_ONE_SHOT;              // enable one shot --> begins transaction
    }

    uint32_t start_tick = HAL_GetTick();

    // wait until pressure data becomes available, or timeout
    while (!(bar_status(bar) & BAR_BIT_P_DA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return RET_TIMEOUT;
    }

    return RET_OK;
}

ErrorType bar_is_temp_data_ready(BarParams * bar, uint32_t timeout_ms)
{
    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == BAR_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
        ctrl_reg2 |= BAR_BIT_ONE_SHOT;              // enable one shot --> begins transaction
    }

    uint32_t start_tick = HAL_GetTick();

    // wait until pressure data becomes available, or timeout
    while (!(bar_status(bar) & BAR_BIT_T_DA))
    {
        if (HAL_GetTick() - start_tick > timeout_ms)
            return RET_TIMEOUT;
    }

    return RET_OK;
}

ErrorType bar_read_temp(BarParams * bar, float * temp_c)
{
    uint8_t t[2];
    bar_read(bar, BAR_REG_TEMP_OUT_L, 2, t);        // read L and H registers

    int16_t temp_lsb = (t[1] << 8) | t[0];         // build 16-bit temp value

    // TN1228 application note
    *temp_c = BAR_TEMP_OFFSET + (float) temp_lsb * BAR_TEMP_SCALING_FACTOR;

    return RET_OK;
}

ErrorType bar_read_press_lsb(BarParams * bar, int32_t * press_lsb)
{
    uint8_t p[3];
    bar_read(bar, BAR_REG_PRESS_OUT_XL, 3, p);      // read XL, L, and H registers

    *press_lsb = (p[2] << 16) | (p[1] << 8) | p[0];      // create 24-bit pressure value

    // if PRESS_OUT_H is negative, make sure press_lsb is negative
    // check this 
    if (p[2] & 0x80)
    {
        *press_lsb |= 0xff000000;        // convert to negative
    }

    return RET_OK;
}

ErrorType bar_read_press_hpa(BarParams * bar, float * hpa)
{
    int32_t press_lsb;

    if (bar_read_press_lsb(bar, &press_lsb) != RET_OK)          // always returns RET_OK (change?)
        return RET_TIMEOUT;
    
    // Figure 5 datasheet
    *hpa = (float) press_lsb * BAR_PRESS_SCALING_FACTOR;

    return RET_OK;
}

ErrorType bar_powerdown(BarParams * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);

    ctrl_reg1 &= ~BAR_BIT_PD;           // power-down mode
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return RET_OK;
}

ErrorType bar_powerup(BarParams * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);

    ctrl_reg1 |= BAR_BIT_PD;           // active mode
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return RET_OK;
}

/**************************************** Helper Functions *******************************************/
static ErrorType bar_sw_reset(BarParams * bar)
{
    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
    ctrl_reg2 |= BAR_BIT_SWRESET;
    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);

    // wait for software reset bit to clear
    while (bar_read_reg(bar, BAR_REG_CTRL_REG2) & BAR_BIT_SWRESET);

    return RET_OK;
}

static ErrorType bar_reboot(BarParams * bar)
{
    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);
    ctrl_reg2 |= BAR_BIT_BOOT;
    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);

    // boot process takes 2.2ms
    bar->delay_ms(5);

    return RET_OK;
}

static ErrorType bar_set_odr(BarParams * bar)
{
    BarOdr odr = bar->odr;

    // check for invalid configurations
    if (odr > BAR_ODR_25HZ)
        return RET_INVALID_CFG;
    
    // only modify ODR bits
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_REG_CTRL_REG1);
    ctrl_reg1 = (ctrl_reg1 & 0x8f) | (odr << BAR_BIT_ODR_SHIFT);
    bar_write_reg(bar, BAR_REG_CTRL_REG1, ctrl_reg1);

    return RET_OK;
}

static ErrorType bar_set_fifo_mode(BarParams * bar)
{
    BarFifoMode fifo_mode = bar->FifoParams.mode;
    BarFifoAvgSmpSize mov_smp = bar->FifoParams.mov_smp;
    uint8_t waterline = bar->FifoParams.thresh;

    // check for invalid configurations
    if (fifo_mode > BAR_FIFO_BYPASS_TO_FIFO || mov_smp > BAR_MOV_AVG_SMP_32 || waterline > BAR_FIFO_DEPTH - 1)
        return RET_INVALID_CFG;

    uint8_t fifo_ctrl = fifo_mode << BAR_BIT_FIFO_MODE_SHIFT;
    // select FIFO modes and moving sample size (if applicable)
    if (fifo_mode == BAR_FIFO_FIFO_MEAN)
        fifo_ctrl |= mov_smp;
    else
        fifo_ctrl |= waterline;


    uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_REG_CTRL_REG2);

    if (fifo_mode != BAR_FIFO_BYPASS)
        ctrl_reg2 |= BAR_BIT_FIFO_EN;               // enable FIFO
    else
        ctrl_reg2 &= ~BAR_BIT_FIFO_EN;              // disable FIFO

    if (fifo_mode == BAR_FIFO_FIFO || fifo_mode == BAR_FIFO_STREAM_TO_FIFO)
        ctrl_reg2 |= BAR_BIT_STOP_ON_FTH;           // enable FIFO waterline
    else
        ctrl_reg2 &= ~BAR_BIT_STOP_ON_FTH;

    bar_write_reg(bar, BAR_REG_CTRL_REG2, ctrl_reg2);
    bar_write_reg(bar, BAR_REG_FIFO_CTRL, fifo_ctrl);

    return RET_OK;
}

static ErrorType bar_set_it(BarParams * bar)
{
    BarIntSel it = bar->it;

    if (it > (BAR_INT_DRDY | BAR_INT_F_OVR | BAR_INT_F_FTH | BAR_INT_F_EMPTY))
        return RET_INVALID_CFG;
    
    bar_write_reg(bar, BAR_REG_CTRL_REG4, it);      // select interrupt type
    bar_write_reg(bar, BAR_REG_CTRL_REG3, 0);       // data signal to INT_DRDY pin, active high interrupt, push/pull output

    return RET_OK;
}
/*****************************************************************************************************/


/************************************* SPI Wrapper Functions *****************************************/
static inline void bar_select(BarParams * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_RESET);
}

static inline void bar_deselect(BarParams * bar)
{
    HAL_GPIO_WritePin(bar->cs_gpio, bar->cs, GPIO_PIN_SET);
}

static void bar_write_reg(BarParams * bar, uint8_t addr, uint8_t data)
{
    uint8_t cmd[2] = {addr | BAR_WRITE, data};

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, cmd, 2, SPI_TIMEOUT_MS);
    bar_deselect(bar);
}

static uint8_t bar_read_reg(BarParams * bar, uint8_t addr)
{
    addr |= BAR_READ;

    uint8_t data;

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, &data, 1, SPI_TIMEOUT_MS);
    bar_deselect(bar);

    return data;
}

static void bar_read(BarParams * bar, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    start_addr |= BAR_READ;
    start_addr |= BAR_ADDR_INC;

    // page 28 of datasheet, bit 6 (MS) must be 1 to increment address

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &start_addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(bar->hspi, data, len, SPI_TIMEOUT_MS);
    bar_deselect(bar);
}

static uint8_t bar_status(BarParams * bar)
{
    return bar_read_reg(bar, BAR_REG_STATUS);
}
/*****************************************************************************************************/

