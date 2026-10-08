#include "lsm6ds3tr_hal.h"
#include "lsm6ds3tr.h"

lsm6ds3tr_status_t lsm6ds3tr_select(const lsm6ds3tr_handle_t * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_RESET);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_deselect(const lsm6ds3tr_handle_t * imu)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs, GPIO_PIN_SET);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_hal_write_reg(const lsm6ds3tr_handle_t * imu, uint8_t reg, const uint8_t data)
{
    reg |= LSM6DS3TR_WRITE;

    lsm6ds3tr_select(imu);
    HAL_SPI_Transmit(imu->hspi, &reg, 1, LSM6DS3TR_SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(imu->hspi, &data, 1, LSM6DS3TR_SPI_TIMEOUT_MS);
    lsm6ds3tr_deselect(imu);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_hal_read_reg(const lsm6ds3tr_handle_t * imu, uint8_t reg, uint8_t * data)
{
    // READ command involves ORing with 0x80
    reg |= LSM6DS3TR_READ;
    
    lsm6ds3tr_select(imu);
    HAL_SPI_Transmit(imu->hspi, &reg, 1, LSM6DS3TR_SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, data, 1, LSM6DS3TR_SPI_TIMEOUT_MS);
    lsm6ds3tr_deselect(imu);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_hal_read(const lsm6ds3tr_handle_t * imu, uint8_t start_reg, const uint8_t dlen, uint8_t * data)
{
    // READ command involves ORing with 0x80
    start_reg |= LSM6DS3TR_READ;
    
    // read multiple byte burst from IMU
    // IF_INC must be enabled in CTRL3_C reg
    lsm6ds3tr_select(imu);
    HAL_SPI_Transmit(imu->hspi, &start_reg, 1, LSM6DS3TR_SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, data, dlen, LSM6DS3TR_SPI_TIMEOUT_MS);
    lsm6ds3tr_deselect(imu);

    return LSM6DS3TR_OK;
}

lsm6ds3tr_status_t lsm6ds3tr_hal_verify_id(const lsm6ds3tr_handle_t * imu)
{
    uint8_t retries = 0;
    uint8_t who_am_i;

    do
    {
        if (retries == LSM6DS3TR_SPI_MAX_RETRIES) // timeout after too many attempts
            return LSM6DS3TR_TIMEOUT;
        else if (retries)                         // delay after failed attempt
            imu->delay_ms(10);

        lsm6ds3tr_status_t status = lsm6ds3tr_hal_read_reg(imu, LSM6DS3TR_REG_WHO_AM_I, &who_am_i);

        if (status != LSM6DS3TR_OK)
            return status;

        retries++;

    } while (who_am_i != LSM6DS3TR_ID);

    return LSM6DS3TR_OK;
}
