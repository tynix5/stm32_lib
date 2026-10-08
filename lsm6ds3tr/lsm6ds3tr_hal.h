#ifndef LSM6DS3TR_HAL_H
#define LSM6DS3TR_HAL_H

#include "lsm6ds3tr.h"

#define LSM6DS3TR_SPI_TIMEOUT_MS  5
#define LSM6DS3TR_SPI_MAX_RETRIES 5

/* Pull CS low */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_select(const lsm6ds3tr_handle_t * imu);

/* Pull CS high */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_deselect(const lsm6ds3tr_handle_t * imu);

/* Write one byte to IMU register over SPI */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [input] reg - LSM6DS3TR register being written to */
/* [input] data - data written to register */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_hal_write_reg(const lsm6ds3tr_handle_t * imu, uint8_t reg, const uint8_t data);

/* Read one byte from IMU register over SPI */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [input] reg - LSM6DS3TR register being read */
/* [output] data - data read from register */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_hal_read_reg(const lsm6ds3tr_handle_t * imu, uint8_t reg, uint8_t * data);

/* Multi-byte read from IMU registers over SPI */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [input] start_reg - starting LSM6DS3TR register being read */
/* [input] dlen - number of bytes to read starting from start_reg */
/* [output] data - data read from register(s) */
/* [return] LSM6DS3TR_OK if successful */
lsm6ds3tr_status_t lsm6ds3tr_hal_read(const lsm6ds3tr_handle_t * imu, const uint8_t start_reg, const uint8_t dlen, uint8_t * data);

/* Read and verify WHO_AM_I register matches expected value (test for successful communication) */
/* [input] imu - pointer to LSM6DS3TR structure handle */
/* [return] LSM6DS3TR_OK if successful, LSM6DS3TR_TIMEOUT otherwise */
lsm6ds3tr_status_t lsm6ds3tr_hal_verify_id(const lsm6ds3tr_handle_t * imu);

#endif