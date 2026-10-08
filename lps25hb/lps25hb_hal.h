#ifndef LPS25HB_HAL_H
#define LPS25HB_HAL_H

#include "lps25hb.h"

#define LPS25HB_SPI_TIMEOUT_MS  5
#define LPS25HB_SPI_MAX_RETRIES 5

/* Pull CS low */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_select(const lps25hb_handle_t * bar);

/* Pull CS high */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_deselect(const lps25hb_handle_t * bar);

/* Write byte of data to register over SPI */
/* [input] bar - pointer to LPS25HB structure handle */
/* [input] reg - LPS25HB register being written to */
/* [input] data - data written to reg */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_write_reg(const lps25hb_handle_t * bar, uint8_t reg, const uint8_t data);

/* Read one byte of data from register over SPI */
/* [input] bar - pointer to LPS25HB structure handle */
/* [input] reg - LPS25HB register being read from */
/* [output] data - data read from reg */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_read_reg(const lps25hb_handle_t * bar, uint8_t reg, uint8_t * data);

/* Read byte(s) of data over SPI */
/* [input] bar - pointer to LPS25HB structure handle */
/* [input] start_reg - starting LPS25HB register being read from */
/* [input] dlen - number of bytes to read */
/* [input] data - dlen number of byte(s) read from start_reg on */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_read(const lps25hb_handle_t * bar, uint8_t start_reg, const uint8_t dlen, uint8_t * data);

/* Check WHO_AM_I reg matches expected value */
/* [input] bar - pointer to LPS25HB structure handle */
/* [return] LPS25HB_OK if initialization is successful */
lps25hb_status_t lps25hb_hal_verify_id(const lps25hb_handle_t * bar);

#endif