#ifndef SSD1306_H_
#define SSD1306_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/*************************** Registers + Commands ********************************/
// Fundamental Commands
#define CONTRAST_REG            0x81
#define RESUME_RAM              0xA4
#define ENTIRE_ON               0xA5
#define NORMAL_DISP             0xA6
#define INVERSE_DISP            0xA7
#define DISPLAY_OFF             0xAE
#define DISPLAY_ON              0xAF

// Scrolling Commands
#define R_H_SCROLL_REG          0x26
#define L_H_SCROLL_REG          0x27
#define V_R_H_SCROLL_REG        0x29
#define V_L_H_SCROLL_REG        0x2A
#define DEACTIVATE_SCROLL       0x2E
#define ACTIVATE_SCROLL         0x2F
#define SET_V_SCROLL_AREA       0xA3

// Addressing Commands
#define LOW_COL_START           0x00
#define HIGH_COL_START          0x10
#define MEM_ADDR_MODE           0x20
#define HORIZONTAL_ADDR         0x00 
#define VERTICAL_ADDR           0x01
#define PAGE_ADDR               0x10
#define SET_COL_ADDR            0x21
#define SET_PAGE_ADDR           0x22
#define SET_PAGE_START          0xB0

// Hardware Config
#define SET_START_LINE          0x40
#define SET_SEG_REMAP           0xA0
#define SET_SEG_REMAP_INV       0xA1
#define SET_MULTIPLEX_RATIO     0xA8
#define COM_SCAN_DIR            0xC0
#define COM_SCAN_DIR_INV        0xC8
#define SET_VERT_OFFSET         0xD3
#define SET_COM_CONFIG          0xDA

// Timing and Driving Config
#define SET_CLK_DIV             0xD5
#define SET_PRECHARGE           0xD9
#define SET_VCOMH               0xDB
#define NOP_REG                 0xE3

// Charge Pump Config
#define CHARGE_PUMP_REG         0x8D
#define EN_CHARGE_PUMP          0x14
#define DIS_CHARGE_PUMP         0x10
/*********************************************************************************/

/**************************** Transaction Type ***********************************/
#define DATA                    0x40            // D/C# bit active
#define COMMAND                 0x00
/*********************************************************************************/

/******************************** Address Type ***********************************/
#define SSD1306_WRITE           0b01111000
#define SSD1306_READ            0b01111001  
/*********************************************************************************/

/****************************** Display Parameters *******************************/
#define RES_X                   128             // pixels
#define RES_Y                   64              // pixels
#define RAM_PAGES               8               // RES_Y / 8 bits per byte
/*********************************************************************************/


void ssd1306_init(I2C_HandleTypeDef * hi2c_);
void ssd1306_draw_pixel(uint8_t x, uint8_t y);
void ssd1306_clear_pixel(uint8_t x, uint8_t y);
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void ssd1306_draw_filled_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void ssd1306_draw_circle(uint8_t x, uint8_t y, uint8_t r);
void ssd1306_draw_filled_circle(uint8_t x, uint8_t y, uint8_t r);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
void ssd1306_write_ch(uint8_t x, uint8_t y, char ch);
void ssd1306_write_str(uint8_t x, uint8_t y, char * str);
void ssd1306_clear();
void ssd1306_update();

#endif