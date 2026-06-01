#include "ssd1306.h"
#include "stm32l4xx_hal.h"
#include "font8x8_basic.h"
#include <string.h>             // memcpy
#include <math.h>

I2C_HandleTypeDef * hi2c;

uint8_t buff[RAM_PAGES][RES_X] = {0};

/******************* I2C Wrapper Functions ****************/
static void ssd1306_write_cmd(uint8_t cmd);
static void ssd1306_write_ram(uint8_t byte);
static void ssd1306_transform_xy(uint8_t x, uint8_t y, uint8_t * x_new, uint8_t * y_new, uint8_t * offset);
static void ssd1306_set_column(uint8_t col);
static void ssd1306_set_page(uint8_t page);
static void i2c_write_byte(uint8_t addr, uint8_t byte);
/**********************************************************/

void ssd1306_init(I2C_HandleTypeDef * hi2c_)
{
    hi2c = hi2c_;

    // wait for display to power up
    HAL_Delay(100);

    uint8_t init[] = {
        COMMAND,
        DISPLAY_OFF,
        RESUME_RAM,
        NORMAL_DISP,
        MEM_ADDR_MODE,
        HORIZONTAL_ADDR,
        CHARGE_PUMP_REG,
        EN_CHARGE_PUMP,
        DISPLAY_ON
    };

    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, init, sizeof(init), HAL_TIMEOUT);
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y)
{
    uint8_t x_new, y_new, offset;
    ssd1306_transform_xy(x, y, &x_new, &y_new, &offset);
    buff[y_new][x_new] |= (1 << offset);
}

void ssd1306_clear_pixel(uint8_t x, uint8_t y)
{
    uint8_t x_new, y_new, offset;
    ssd1306_transform_xy(x, y, &x_new, &y_new, &offset);
    buff[y_new][x_new] &= ~(1 << offset);
}

void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    if (x0 > RES_X - 1 || x1 > RES_X - 1 || y0 > RES_Y - 1 || y1 > RES_Y - 1)
        return;

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
            ssd1306_transform_xy(x0, i, &x_xform, &y_xform, &offset);
            buff[y_xform][x_xform] |= (1 << offset);
        }
        return;
    }

    double slope = (double) (y_end - y_start) / (x_end - x_start);

    double y_curr = (double) y_start;

    for (uint8_t i = x_start + 1; i <= x_end; i++)
    {
        y_curr += slope;
        ssd1306_transform_xy(i, (uint8_t) round(y_curr), &x_xform, &y_xform, &offset);
        buff[y_xform][x_xform] |= (1 << offset);
    }
}

void ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    // (x, y) is top left corner of rectangle
    if (x > RES_X - 1 || y > RES_Y - 1 || x + w > RES_X - 1 || y + h > RES_Y - 1)
        return;

    uint8_t x_new, y_new, offset;

    // top
    for (uint8_t i = 0; i < w; i++)
    {
        ssd1306_transform_xy(x + i, y, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
    }

    // bottom
    for (uint8_t i = 0; i < w; i++)
    {
        ssd1306_transform_xy(x + i, y + h, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
    }

    // left
    for (uint8_t i = 0; i < h; i++)
    {
        ssd1306_transform_xy(x, y + i, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
    }

    // right
    for (uint8_t i = 0; i < h; i++)
    {
        ssd1306_transform_xy(x + w, y + i, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
    }    
}

void ssd1306_draw_filled_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    if (x > RES_X - 1 || y > RES_Y - 1 || x + w > RES_X - 1 || y + h > RES_Y - 1)
        return;

    for (uint8_t i = 0; i < w; i++)
    {
        for (uint8_t j = 0; j < h; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(x + i, y + j, &x_new, &y_new, &offset);
            buff[y_new][x_new] |= (1 << offset);
        }
    }
}

void ssd1306_draw_circle(uint8_t x, uint8_t y, uint8_t r)
{
    // (x, y) is center of circle
    if (x > RES_X - 1 || x + r > RES_X - 1 || y > RES_Y - 1 || y + r > RES_Y - 1)
        return;

    for (uint8_t curr_x = x - r; curr_x <= x + r; curr_x++)
    {
        // x^2 + y^2 = r^2
        // y^2 = r^2 - x^2
        uint8_t y_sol = (uint8_t) round(sqrt((double) r * r - (double) (curr_x - x) * (curr_x - x)));

        uint8_t x_new, y_new, offset;
        ssd1306_transform_xy(curr_x, y + y_sol, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
        ssd1306_transform_xy(curr_x, y - y_sol, &x_new, &y_new, &offset);
        buff[y_new][x_new] |= (1 << offset);
    }
}

void ssd1306_draw_filled_circle(uint8_t x, uint8_t y, uint8_t r)
{
    // (x, y) is center of circle
    if (x > RES_X - 1 || x + r > RES_X - 1 || y > RES_Y - 1 || y + r > RES_Y - 1)
        return;

    for (uint8_t curr_x = x - r; curr_x <= x + r; curr_x++)
    {
        // x^2 + y^2 = r^2
        // y^2 = r^2 - x^2
        uint8_t y_sol = (uint8_t) round(sqrt((double) r * r - (double) (curr_x - x) * (curr_x - x)));

        for (uint8_t curr_y = y - y_sol; curr_y <= y + y_sol; curr_y++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(curr_x, curr_y, &x_new, &y_new, &offset);
            buff[y_new][x_new] |= (1 << offset);
        }
    }
}


void ssd1306_write_ch(uint8_t x, uint8_t y, char ch)
{
    if (x + FONT_WIDTH > RES_X - 1 || y + FONT_HEIGHT > RES_Y - 1)
        return;
    
    const uint8_t * bitmap = (uint8_t *) font8x8_basic[(uint8_t) ch];
    
    for (uint8_t i = 0; i < FONT_HEIGHT; i++)
    {
        for (uint8_t j = 0; j < FONT_WIDTH; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(x + j, y + i, &x_new, &y_new, &offset);
            
            if (bitmap[i] & (1 << j))
                buff[y_new][x_new] |= (1 << offset);
            else
                buff[y_new][x_new] &= ~(1 << offset);
        } 
    }
}

void ssd1306_write_str(uint8_t x, uint8_t y, char * str)
{
    for (uint8_t i = 0; *str; i++)
    {
        ssd1306_write_ch(x + i * FONT_WIDTH, y, *str++);
    }
}

void ssd1306_draw_bitmap(uint8_t x, uint8_t y, uint8_t  bitmap[], uint8_t size_x, uint8_t size_y)
{
    if (x + size_x > RES_X - 1 || y + size_y > RES_Y - 1)
        return;

    for (uint8_t i = 0; i < size_x; i++)
    {
        for (uint8_t j = 0; j < size_y; j++)
        {
            uint8_t x_new, y_new, offset;
            ssd1306_transform_xy(x + i, y + j, &x_new, &y_new, &offset);

            if (bitmap[j * size_x + i])
                buff[y_new][x_new] |= (1 << offset);
            else
                buff[y_new][x_new] &= ~(1 << offset);
        }
    }
}

void ssd1306_set_cursor(uint8_t x, uint8_t y)
{
    if (x > RES_X - 1)
        x = RES_X - 1;
    if (y > RES_Y - 1)
        y = RES_Y - 1;

    ssd1306_set_column(x);
    ssd1306_set_page(y / 8);
}

void ssd1306_clear()
{
    for (uint8_t i = 0; i < RAM_PAGES; i++)
        for (uint8_t j = 0; j < RES_X; j++)
            buff[i][j] = 0;
}

void ssd1306_update()
{
    ssd1306_set_cursor(0, 0);

    // display write command must start with DATA control byte
    uint8_t ram[RES_X * RAM_PAGES + 1] = {DATA};

    memcpy(ram + 1, buff, sizeof(buff));
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, ram, sizeof(ram), 250);
}

static void ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {COMMAND, cmd};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, data, sizeof(data), HAL_TIMEOUT);
}

static void ssd1306_write_ram(uint8_t byte)
{
    uint8_t data[2] = {DATA, byte};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, data, sizeof(data), HAL_TIMEOUT);
}

static void ssd1306_transform_xy(uint8_t x, uint8_t y, uint8_t * x_new, uint8_t * y_new, uint8_t * offset)
{
    if (x > RES_X - 1)
        x = RES_X - 1;
    if (y > RES_Y - 1)
        y = RES_Y - 1;

    *x_new = x;
    *y_new = y / 8;
    *offset = y % 8;
}

static void ssd1306_set_column(uint8_t col)
{
    if (col > RES_X - 1) 
        col = RES_X - 1;
    uint8_t data[4] = {COMMAND, SET_COL_ADDR, col, RES_X - 1};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, data, sizeof(data), HAL_TIMEOUT);
}

static void ssd1306_set_page(uint8_t page)
{
    if (page > RAM_PAGES - 1) 
        page = RAM_PAGES - 1;
    uint8_t data[4] = {COMMAND, SET_PAGE_ADDR, page, RAM_PAGES - 1};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, data, sizeof(data), HAL_TIMEOUT);
}

static void i2c_write_byte(uint8_t addr, uint8_t byte)
{
    uint8_t data[2] = {addr, byte};
    HAL_I2C_Master_Transmit(hi2c, SSD1306_WRITE, data, sizeof(data), HAL_TIMEOUT);
}

