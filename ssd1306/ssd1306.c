#include "ssd1306.h"
#include "ssd1306_hal.h"
#include "font8x8_basic.h"
#include <string.h> // memcpy
#include <math.h>

/********************************************** OLED Helper Functions *******************************************/
static ssd1306_status_t ssd1306_write_buff_bit(ssd1306_handle_t * holed, uint8_t x_new, uint8_t y_new, uint8_t offset, ssd1306_pix_color_t color);
static ssd1306_status_t ssd1306_write_buff_col(ssd1306_handle_t * holed, uint8_t x_new, uint8_t page, ssd1306_pix_color_t color);
static ssd1306_status_t ssd1306_transform_xy(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t * x_new, uint8_t * y_new, uint8_t * offset);
/****************************************************************************************************************/

ssd1306_handle_t * ssd1306_devices[SSD1306_MAX_DEVICES];
uint8_t ssd1306_device_cnt = 0;

ssd1306_status_t ssd1306_init(ssd1306_handle_t * holed)
{
    if (holed->hi2c == NULL)
        return SSD1306_NULL_POINTER;

    // register device
    ssd1306_devices[ssd1306_device_cnt++] = holed;

    holed->_dma_busy = SSD1306_DMA_OK;

    // wait for display to power up
    HAL_Delay(100);

    uint8_t init[] = { SSD1306_I2C_COMMAND,   SSD1306_SET_START_LINE,  SSD1306_DISPLAY_OFF,     SSD1306_RESUME_RAM,     SSD1306_NORMAL_DISP,
                       SSD1306_MEM_ADDR_MODE, SSD1306_HORIZONTAL_ADDR, SSD1306_CHARGE_PUMP_REG, SSD1306_EN_CHARGE_PUMP, SSD1306_DISPLAY_ON };

    memcpy(holed->tx_buff, init, sizeof(init));
    ssd1306_status_t status = ssd1306_hal_write(holed, holed->tx_buff, sizeof(init));

    if (status == SSD1306_OK)
    {
        holed->_initalized = true;
        return SSD1306_OK;
    }

    holed->_initalized = false;
    return status;
}

ssd1306_status_t ssd1306_draw_pixel(ssd1306_handle_t * holed, uint8_t x, uint8_t y)
{
    uint8_t x_new, y_new, offset;
    ssd1306_transform_xy(holed, x, y, &x_new, &y_new, &offset);
    return ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
}

ssd1306_status_t ssd1306_clear_pixel(ssd1306_handle_t * holed, uint8_t x, uint8_t y)
{
    uint8_t x_new, y_new, offset;
    ssd1306_transform_xy(holed, x, y, &x_new, &y_new, &offset);
    return ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_BLACK);
}

ssd1306_status_t ssd1306_draw_line(ssd1306_handle_t * holed, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    if (x0 > holed->res_x - 1 || x1 > holed->res_x - 1 || y0 > holed->res_y - 1 || y1 > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    uint8_t x_start = x0, y_start = y0;
    uint8_t x_end = x1, y_end = y1;

    if (x0 > x1)
    {
        x_start = x1;
        x_end = x0;
    }
    if (y0 > y1)
    {
        y_start = y1;
        y_end = y0;
    }

    uint8_t x_xform, y_xform, offset;

    // if slope is infinite (vertical line)
    if (x0 == x1)
    {
        for (uint8_t i = y_start; i < y_end; i++)
        {
            ssd1306_transform_xy(holed, x0, i, &x_xform, &y_xform, &offset);
            ssd1306_write_buff_bit(holed, x_xform, y_xform, offset, SSD1306_PIX_WHITE);
        }
        return SSD1306_OK;
    }

    double slope = (double)(y_end - y_start) / (x_end - x_start);

    double y_curr = (double)y_start;

    for (uint8_t i = x_start + 1; i <= x_end; i++)
    {
        y_curr += slope;
        ssd1306_transform_xy(holed, i, (uint8_t)round(y_curr), &x_xform, &y_xform, &offset);
        ssd1306_write_buff_bit(holed, x_xform, y_xform, offset, SSD1306_PIX_WHITE);
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_draw_rect(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    // (x, y) is top left corner of rectangle
    if (x > holed->res_x - 1 || y > holed->res_y - 1 || x + w > holed->res_x - 1 || y + h > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    uint8_t x_new, y_new, offset;

    // top + bottom
    for (uint8_t i = 0; i < w; i++)
    {
        ssd1306_transform_xy(holed, x + i, y, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
        ssd1306_transform_xy(holed, x + i, y + h, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
    }
    // left + right
    for (uint8_t i = 0; i < h; i++)
    {
        ssd1306_transform_xy(holed, x, y + i, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
        ssd1306_transform_xy(holed, x + w, y + i, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_draw_filled_rect(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    if (x > holed->res_x - 1 || y > holed->res_y - 1 || x + w > holed->res_x - 1 || y + h > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    for (uint8_t i = 0; i < w; i++)
    {
        for (uint8_t j = 0; j < h; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(holed, x + i, y + j, &x_new, &y_new, &offset);
            ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
        }
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_draw_circle(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t r)
{
    // (x, y) is center of circle
    if (x > holed->res_x - 1 || x + r > holed->res_x - 1 || y > holed->res_y - 1 || y + r > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    for (uint8_t curr_x = x - r; curr_x <= x + r; curr_x++)
    {
        // x^2 + y^2 = r^2
        // y^2 = r^2 - x^2
        uint8_t y_sol = (uint8_t)round(sqrt((double)r * r - (double)(curr_x - x) * (curr_x - x)));

        uint8_t x_new, y_new, offset;
        ssd1306_transform_xy(holed, curr_x, y + y_sol, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
        ssd1306_transform_xy(holed, curr_x, y - y_sol, &x_new, &y_new, &offset);
        ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_draw_filled_circle(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t r)
{
    // (x, y) is center of circle
    if (x > holed->res_x - 1 || x + r > holed->res_x - 1 || y > holed->res_y - 1 || y + r > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    for (uint8_t curr_x = x - r; curr_x <= x + r; curr_x++)
    {
        // x^2 + y^2 = r^2
        // y^2 = r^2 - x^2
        uint8_t y_sol = (uint8_t)round(sqrt((double)r * r - (double)(curr_x - x) * (curr_x - x)));

        for (uint8_t curr_y = y - y_sol; curr_y <= y + y_sol; curr_y++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(holed, curr_x, curr_y, &x_new, &y_new, &offset);
            ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
        }
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_write_ch(ssd1306_handle_t * holed, uint8_t x, uint8_t y, char ch)
{
    if (x + FONT_WIDTH > holed->res_x - 1 || y + FONT_HEIGHT > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    const uint8_t * bitmap = (uint8_t *)font8x8_basic[(uint8_t)ch];

    for (uint8_t i = 0; i < FONT_HEIGHT; i++)
    {
        for (uint8_t j = 0; j < FONT_WIDTH; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(holed, x + j, y + i, &x_new, &y_new, &offset);

            if (bitmap[i] & (1 << j))
                ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
            else
                ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_BLACK);
        }
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_write_str(ssd1306_handle_t * holed, uint8_t x, uint8_t y, char * str)
{
    for (uint8_t i = 0; *str; i++)
    {
        ssd1306_write_ch(holed, x + i * FONT_WIDTH, y, *str++);
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_draw_bitmap(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t bitmap[], uint8_t size_x, uint8_t size_y)
{
    if (x + size_x > holed->res_x - 1 || y + size_y > holed->res_y - 1)
        return SSD1306_INVALID_CFG;

    for (uint8_t i = 0; i < size_x; i++)
    {
        for (uint8_t j = 0; j < size_y; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(holed, x + i, y + j, &x_new, &y_new, &offset);

            if (bitmap[j * size_x + i])
                ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_WHITE);
            else
                ssd1306_write_buff_bit(holed, x_new, y_new, offset, SSD1306_PIX_BLACK);
        }
    }

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_clear(ssd1306_handle_t * holed)
{
    for (uint8_t i = 0; i < holed->res_y / 8; i++)
        for (uint8_t j = 0; j < holed->res_x; j++)
            ssd1306_write_buff_col(holed, j, i, SSD1306_PIX_BLACK);

    return SSD1306_OK;
}

ssd1306_status_t ssd1306_update(ssd1306_handle_t * holed)
{
    if (!holed->_initalized)
        return SSD1306_NOT_INIT;
        
    // do not overwrite TX buffer if in use
    if (holed->_dma_busy == SSD1306_DMA_BUSY)
        return SSD1306_BUSY;

    // set cursors to (0, 0)
    // combines command bytes and data bytes into single transaction for DMA purposes
    uint8_t set_cursor[] = { SSD1306_I2C_COMMAND_CO, SSD1306_SET_COL_ADDR,  SSD1306_I2C_COMMAND_CO, 0, SSD1306_I2C_COMMAND_CO, holed->res_x - 1,
                             SSD1306_I2C_COMMAND_CO, SSD1306_SET_PAGE_ADDR, SSD1306_I2C_COMMAND_CO, 0, SSD1306_I2C_COMMAND_CO, (holed->res_y / 8) - 1,
                             SSD1306_I2C_DATA };

    uint32_t frame_size = holed->res_x * holed->res_y / 8;
    uint32_t cursor_cmd_size = sizeof(set_cursor);

    memcpy(holed->tx_buff, set_cursor, cursor_cmd_size);
    memcpy(holed->tx_buff + cursor_cmd_size, holed->frame_buff, frame_size);

    return ssd1306_hal_write(holed, holed->tx_buff, frame_size + cursor_cmd_size);
}

ssd1306_status_t ssd1306_callback(I2C_HandleTypeDef * hi2c)
{
    if (hi2c == NULL)
        return SSD1306_NULL_POINTER;
    
    ssd1306_hal_i2c_dma_callback(hi2c);

    return SSD1306_OK;
}

/********************************************** OLED Helper Functions *******************************************/
static ssd1306_status_t ssd1306_write_buff_bit(ssd1306_handle_t * holed, uint8_t x_new, uint8_t y_new, uint8_t offset, ssd1306_pix_color_t color)
{
    uint32_t idx = y_new * holed->res_x + x_new;
    uint8_t byte = holed->frame_buff[idx];

    if (color == SSD1306_PIX_WHITE)
        byte |= (1 << offset);
    else
        byte &= ~(1 << offset);

    holed->frame_buff[idx] = byte;

    return SSD1306_OK;
}

static ssd1306_status_t ssd1306_write_buff_col(ssd1306_handle_t * holed, uint8_t x_new, uint8_t page, ssd1306_pix_color_t color)
{
    uint32_t idx = page * holed->res_x + x_new;
    uint8_t byte;

    if (color == SSD1306_PIX_WHITE)
        byte = 0xff;
    else
        byte = 0;

    holed->frame_buff[idx] = byte;

    return SSD1306_OK;
}

static ssd1306_status_t ssd1306_transform_xy(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t * x_new, uint8_t * y_new, uint8_t * offset)
{
    if (x > holed->res_x - 1)
        x = holed->res_x - 1;
    if (y > holed->res_y - 1)
        y = holed->res_y - 1;

    *x_new = x;
    *y_new = y / 8;
    *offset = y % 8;

    return SSD1306_OK;
}
/****************************************************************************************************************/
