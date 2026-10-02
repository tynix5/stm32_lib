#include "nrf24_hal.h"
#include "nrf24.h"

void nrf24_hal_select(const nrf24_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
}

void nrf24_hal_deselect(const nrf24_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}

void nrf24_hal_enable(const nrf24_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_SET);
}

void nrf24_hal_disable(const nrf24_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_RESET);
}

nrf24_status_t nrf24_hal_write_reg(const nrf24_handle_t * radio, uint8_t reg, const uint8_t * data, const uint8_t dlen)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (dlen < 1)
        return NRF24_INVALID_CFG;

    reg |= NRF24_W_REGISTER;

    nrf24_hal_select(radio);
    HAL_SPI_Transmit(radio->hspi, &reg, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(radio->hspi, data, dlen, SPI_TIMEOUT_MS);
    nrf24_hal_deselect(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_hal_read_reg(const nrf24_handle_t * radio, const uint8_t reg, uint8_t * data)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    nrf24_hal_select(radio);
    HAL_SPI_Transmit(radio->hspi, &reg, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(radio->hspi, data, 1, SPI_TIMEOUT_MS);
    nrf24_hal_deselect(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_hal_write_cmd(const nrf24_handle_t * radio, const uint8_t reg, uint8_t * response)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    nrf24_hal_select(radio);
    HAL_SPI_TransmitReceive(radio->hspi, &reg, response, 1, SPI_TIMEOUT_MS);
    nrf24_hal_deselect(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_hal_write_cmd_data(const nrf24_handle_t * radio, const uint8_t reg, const uint8_t * data, uint8_t dlen)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    nrf24_hal_select(radio);
    HAL_SPI_Transmit(radio->hspi, &reg, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(radio->hspi, data, dlen, SPI_TIMEOUT_MS);
    nrf24_hal_deselect(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_hal_write_tx_ack(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen)
{
    return nrf24_hal_write_cmd_data(radio, NRF24_W_TX_PAYLOAD, pkt, plen);
}

nrf24_status_t nrf24_hal_write_tx_nack(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen)
{
    return nrf24_hal_write_cmd_data(radio, NRF24_W_TX_PAYLOAD_NOACK, pkt, plen);
}

nrf24_status_t nrf24_hal_read_payload(const nrf24_handle_t * radio, const uint8_t plen, uint8_t * pkt)
{
    uint8_t cmd = NRF24_R_RX_PAYLOAD;

    nrf24_hal_select(radio);
    HAL_SPI_Transmit(radio->hspi, &cmd, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(radio->hspi, pkt, plen, SPI_TIMEOUT_MS);
    nrf24_hal_deselect(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_hal_write_ack(const nrf24_handle_t * radio, const uint8_t * ack, const uint8_t alen)
{
    return nrf24_hal_write_cmd_data(radio, NRF24_W_ACK_PAYLOAD, ack, alen);
}