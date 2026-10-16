#ifndef SSD1306_H_
#define SSD1306_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*************************** Registers + Commands ********************************/
// Fundamental Commands
#define SSD1306_CONTRAST_REG        0x81
#define SSD1306_RESUME_RAM          0xA4
#define SSD1306_ENTIRE_ON           0xA5
#define SSD1306_NORMAL_DISP         0xA6
#define SSD1306_INVERSE_DISP        0xA7
#define SSD1306_DISPLAY_OFF         0xAE
#define SSD1306_DISPLAY_ON          0xAF

// Scrolling Commands
#define SSD1306_R_H_SCROLL_REG      0x26
#define SSD1306_L_H_SCROLL_REG      0x27
#define SSD1306_V_R_H_SCROLL_REG    0x29
#define SSD1306_V_L_H_SCROLL_REG    0x2A
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_ACTIVATE_SCROLL     0x2F
#define SSD1306_SET_V_SCROLL_AREA   0xA3

// Addressing Commands
#define SSD1306_LOW_COL_START       0x00
#define SSD1306_HIGH_COL_START      0x10
#define SSD1306_MEM_ADDR_MODE       0x20
#define SSD1306_HORIZONTAL_ADDR     0x00
#define SSD1306_VERTICAL_ADDR       0x01
#define SSD1306_PAGE_ADDR           0x10
#define SSD1306_SET_COL_ADDR        0x21
#define SSD1306_SET_PAGE_ADDR       0x22
#define SSD1306_SET_PAGE_START      0xB0

// Hardware Config
#define SSD1306_SET_START_LINE      0x40
#define SSD1306_SET_SEG_REMAP       0xA0
#define SSD1306_SET_SEG_REMAP_INV   0xA1
#define SSD1306_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_COM_SCAN_DIR        0xC0
#define SSD1306_COM_SCAN_DIR_INV    0xC8
#define SSD1306_SET_VERT_OFFSET     0xD3
#define SSD1306_SET_COM_CONFIG      0xDA

// Timing and Driving Config
#define SSD1306_SET_CLK_DIV         0xD5
#define SSD1306_SET_PRECHARGE       0xD9
#define SSD1306_SET_VCOMH           0xDB
#define SSD1306_NOP_REG             0xE3

// Charge Pump Config
#define SSD1306_CHARGE_PUMP_REG     0x8D
#define SSD1306_EN_CHARGE_PUMP      0x14
#define SSD1306_DIS_CHARGE_PUMP     0x10
/*********************************************************************************/

/**************************** Transaction Type ***********************************/
#define SSD1306_I2C_DATA            0x40 // D/C# bit active
#define SSD1306_I2C_COMMAND         0x00
#define SSD1306_I2C_COMMAND_CO      0x80 // Co bit active

// Co = 0, D/C = 0 - next bytes are commands, after which stop is expected
// Co = 1, D/C = 0 - next bytes are commands, after which another control byte is expected
// Co = 0, D/C = 1 - next bytes are graphical data, after which I2C stop is expected
// Co = 1, D/C = 1 - illegal
/*********************************************************************************/

/******************************** Address Type ***********************************/
#define SSD1306_I2C_WRITE           0b01111000
#define SSD1306_I2C_READ            0b01111001
/*********************************************************************************/

/****************************** Display Parameters *******************************/
#define SSD1306_MAX_DEVICES         2
/*********************************************************************************/

typedef enum
{
    SSD1306_PIX_BLACK = 0,
    SSD1306_PIX_WHITE,

} ssd1306_pix_color_t;

typedef enum
{
    SSD1306_PHY_MODE_BLOCKING = 0,
    SSD1306_PHY_MODE_DMA

} ssd1306_phy_mode_t;

typedef enum
{
    SSD1306_DMA_OK = 0,
    SSD1306_DMA_BUSY

} ssd1306_dma_status_t;

typedef enum
{
    SSD1306_OK = 0,
    SSD1306_INVALID_CFG,
    SSD1306_BUSY,
    SSD1306_NULL_POINTER,
    SSD1306_NOT_INIT,
    SSD1306_PHY_ERR,
} ssd1306_status_t;
typedef struct
{
    I2C_HandleTypeDef * hi2c;      // I2C handle

    uint8_t * frame_buff;          // pointer to display buffer (res_x * res_y / 8)
    uint8_t * tx_buff;             // pointer to tx buffer --> should never be modified by user (res_x * res_y / 8 + 13)
    ssd1306_phy_mode_t mode;       // DMA mode or blocking mode
    
    uint32_t res_x;                // horizontal resolution (pixels)
    uint32_t res_y;                // vertical resolution (pixels)
    
    // private variables
    ssd1306_dma_status_t _dma_busy; // is DMA in use
    bool _initialized;               // has SSD1306 been initialized?

} ssd1306_handle_t;

extern ssd1306_handle_t * ssd1306_devices[SSD1306_MAX_DEVICES]; // tracks all OLEDs initialized
extern uint8_t ssd1306_device_cnt;

/* Turn on and initialize SSD1306 OLED */
ssd1306_status_t ssd1306_init(ssd1306_handle_t * holed);
/* Set pixel at (x, y) in buffer */
ssd1306_status_t ssd1306_draw_pixel(ssd1306_handle_t * holed, uint8_t x, uint8_t y);
/* Clear pixel at (x, y) in buffer */
ssd1306_status_t ssd1306_clear_pixel(ssd1306_handle_t * holed, uint8_t x, uint8_t y);
/* Draw line in buffer */
ssd1306_status_t ssd1306_draw_line(ssd1306_handle_t * holed, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
/* Draw unfilled rectangle in buffer */
ssd1306_status_t ssd1306_draw_rect(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
/* Draw filled rectangle in buffer */
ssd1306_status_t ssd1306_draw_filled_rect(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
/* Draw unfilled circle in buffer */
ssd1306_status_t ssd1306_draw_circle(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t r);
/* Draw filled circle in buffer */
ssd1306_status_t ssd1306_draw_filled_circle(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t r);
/* Write character into buffer (from font8x8_basic)*/
ssd1306_status_t ssd1306_write_ch(ssd1306_handle_t * holed, uint8_t x, uint8_t y, char ch);
/* Write string of characters into buffer */
ssd1306_status_t ssd1306_write_str(ssd1306_handle_t * holed, uint8_t x, uint8_t y, char * str);
/* Draw bitmap of size x * y in buffer */
ssd1306_status_t ssd1306_draw_bitmap(ssd1306_handle_t * holed, uint8_t x, uint8_t y, uint8_t bitmap[], uint8_t size_x, uint8_t size_y);
/* Clear buffer */
ssd1306_status_t ssd1306_clear(ssd1306_handle_t * holed);
/* Send buffer contents to SSD1306 */
ssd1306_status_t ssd1306_update(ssd1306_handle_t * holed);
/* DMA callback */
ssd1306_status_t ssd1306_callback(I2C_HandleTypeDef * hi2c); // place into HAL_I2C_MasterTxCpltCallback()

#endif