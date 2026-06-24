#include <stdint.h>
#include "lps25hb.h"

/********************** SPI Wrapper Functions *****************/
static inline void bar_select(BarParams * bar);
static inline void bar_deselect(BarParams * bar);
static void bar_write_reg(BarParams * bar, uint8_t addr, uint8_t data);
static uint8_t bar_read_reg(BarParams * bar, uint8_t addr);
static void bar_read(BarParams * bar, uint8_t start_addr, uint8_t len, uint8_t * data);
static uint8_t bar_status(BarParams * bar);
/**************************************************************/

void bar_init(BarParams * bar)
{
    bar_deselect(bar);

    while (bar_read_reg(bar, BAR_WHO_AM_I) != BAR_ID);          // ensure communication is successful

    uint8_t ctrl_reg2 = 0, fifo_ctrl = 0;

    if (bar->fifo_mode == BAR_BYPASS_MODE)
    {
        ctrl_reg2 &= ~BAR_FIFO_EN;              // disable FIFO
        ctrl_reg2 &= ~BAR_STOP_ON_FTH;          // disable waterline level
    }
    else
    {
        ctrl_reg2 |= BAR_FIFO_EN;               // enable FIFO
    }

    if (bar->fifo_mode == BAR_FIFO_MODE)
    {
        ctrl_reg2 |= BAR_STOP_ON_FTH;           // enable FIFO waterline
        fifo_ctrl |= bar->fifo_thresh & 0x1f;       // set waterline level
    }
    else if (bar->fifo_mode == BAR_FIFO_MEAN_MODE)
    {
        if (bar->decimate == BAR_DECIMATION_ON)
        {
            ctrl_reg2 |= BAR_FIFO_MEAN_DEC;         // enable 1Hz output pressure with FIFO_MEAN_MODE
        }

        fifo_ctrl |= bar->fifo_moving_avg_size & 0x1f;       // set FIFO mean mode sample size
    }

    fifo_ctrl |= bar->fifo_mode & 0xe0;         // set FIFO mode

    bar_write_reg(bar, BAR_CTRL_REG1, BAR_PD | bar->odr);     // power-on and select output data rate selection
    bar_write_reg(bar, BAR_CTRL_REG2, ctrl_reg2);
    bar_write_reg(bar, BAR_CTRL_REG3, BAR_INT_D_SIG);
    bar_write_reg(bar, BAR_CTRL_REG4, BAR_DRDY);                // interrupt on data ready signal
    bar_write_reg(bar, BAR_FIFO_CTRL, fifo_ctrl);
}

void bar_is_data_ready(BarParams * bar)
{
    // for ONE_SHOT mode, trigger sample and wait for STATUS_REG
    // if not in ONE_SHOT mode, wait until STATUS_REG is updated
    if (bar->odr == BAR_ODR_ONE_SHOT)
    {
        uint8_t ctrl_reg2 = bar_read_reg(bar, BAR_CTRL_REG2);
        ctrl_reg2 |= BAR_ONE_SHOT;              // enable one shot --> begins transaction
    }

    while (!(bar_status(bar) & BAR_P_DA));      // wait for pressure data to become available, then return
}

int32_t bar_read_press_lsb(BarParams * bar)
{
    uint8_t p[3];
    bar_read(bar, BAR_PRESS_OUT_XL, 3, p);      // read XL, L, and H registers

    int32_t press_lsb = (p[2] << 16) | (p[1] << 8) | p[0];      // create 24-bit pressure value

    // if PRESS_OUT_H is negative, make sure press_lsb is negative
    if (p[2] & 0x80)
    {
        press_lsb |= 0xff000000;        // convert to negative
    }

    return press_lsb;
}

float bar_read_press_hpa(BarParams * bar)
{
    // Figure 5 datasheet
    int32_t press_lsb = bar_read_press_lsb(bar);

    return (float) press_lsb * BAR_SCALING_FACTOR;
}

void bar_powerdown(BarParams * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_CTRL_REG1);

    ctrl_reg1 &= ~BAR_PD;           // power-down mode
    bar_write_reg(bar, BAR_CTRL_REG1, ctrl_reg1);
}

void bar_powerup(BarParams * bar)
{
    uint8_t ctrl_reg1 = bar_read_reg(bar, BAR_CTRL_REG1);

    ctrl_reg1 |= BAR_PD;           // active mode
    bar_write_reg(bar, BAR_CTRL_REG1, ctrl_reg1);
}

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
    HAL_SPI_Transmit(bar->hspi, cmd, 2, HAL_TIMEOUT);
    bar_deselect(bar);
}

static uint8_t bar_read_reg(BarParams * bar, uint8_t addr)
{
    addr |= BAR_READ;

    uint8_t data;

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(bar->hspi, &data, 1, HAL_TIMEOUT);
    bar_deselect(bar);

    return data;
}

static void bar_read(BarParams * bar, uint8_t start_addr, uint8_t len, uint8_t * data)
{
    start_addr |= BAR_READ;

    bar_select(bar);
    HAL_SPI_Transmit(bar->hspi, &start_addr, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(bar->hspi, data, len, HAL_TIMEOUT);
    bar_deselect(bar);
}

static uint8_t bar_status(BarParams * bar)
{
    return bar_read_reg(bar, BAR_STATUS_REG);
}