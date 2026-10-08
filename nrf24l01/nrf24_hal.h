#ifndef NRF24_HAL_H
#define NRF24_HAL_H

#include "nrf24.h"

#define NRF24_SPI_TIMEOUT_MS 5

/* Pull CS pin low */
/* [input] radio - nRF24L01 structure handle */
void nrf24_hal_select(const nrf24_handle_t * radio);

/* Pull CS pin high */
/* [input] radio - nRF24L01 structure handle */
void nrf24_hal_deselect(const nrf24_handle_t * radio);

/* Pull CE pin high */
/* [input] radio - nRF24L01 structure handle */
void nrf24_hal_enable(const nrf24_handle_t * radio);

/* Pull CE pin low */
/* [input] radio - nRF24L01 structure handle */
void nrf24_hal_disable(const nrf24_handle_t * radio);

/* Write bytes to a register over SPI */
/* [input] radio - nRF24L01 structure handle */
/* [input] reg - nRF24L01 register */
/* [input] data - register data */
/* [input] dlen - size of data in bytes */
/* [return] NRF24_OK if SPI write successful */
nrf24_status_t nrf24_hal_write_reg(const nrf24_handle_t * radio, uint8_t reg, const uint8_t * data, const uint8_t dlen);

/* Read byte from a register */
/* [input] radio - nRF24L01 structure handle */
/* [input] reg - nRF24L01 register */
/* [output] data - register data */
/* [return] NRF24_OK if SPI read successful */
nrf24_status_t nrf24_hal_read_reg(const nrf24_handle_t * radio, const uint8_t reg, uint8_t * data);

/* Write a command and return response */
/* [input] radio - nRF24L01 structure handle */
/* [input] reg - nRF24L01 command */
/* [output] response - command response */
/* [return] NRF24_OK if SPI read successful */
nrf24_status_t nrf24_hal_write_cmd(const nrf24_handle_t * radio, const uint8_t reg, uint8_t * response);

/* Write to command register with data bytes */
/* [input] radio - nRF24L01 structure handle */
/* [input] reg - nRF24L01 register */
/* [input] data - register data */
/* [input] dlen - size of data in bytes */
/* [return] NRF24_OK if SPI write successful */
nrf24_status_t nrf24_hal_write_cmd_data(const nrf24_handle_t * radio, const uint8_t reg, const uint8_t * data, uint8_t dlen);

/* Write to TX buffer with ACKs enabled */
/* [input] radio - nRF24L01 structure handle */
/* [input] pkt - data sent to TX buffer */
/* [input] plen - size of pkt in bytes */
/* [return] NRF24_OK if SPI write successful */
nrf24_status_t nrf24_hal_write_tx_ack(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen);

/* Write to TX buffer with ACKs disabled */
/* [input] radio - nRF24L01 structure handle */
/* [input] pkt - data sent to TX buffer */
/* [input] plen - size of pkt in bytes */
/* [return] NRF24_OK if SPI write successful */
nrf24_status_t nrf24_hal_write_tx_nack(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen);

/* Read from RX buffer */
/* [input] radio - nRF24L01 structure handle */
/* [input] plen - size of pkt in bytes */
/* [output] pkt - data read from RX buffer */
/* [return] NRF24_OK if SPI read successful */
nrf24_status_t nrf24_hal_read_payload(const nrf24_handle_t * radio, const uint8_t plen, uint8_t * pkt);

/* Write ACK response to TX buffer */
/* [input] radio - nRF24L01 structure handle */
/* [input] ack - data sent to TX buffer */
/* [input] alen - size of ack in bytes */
/* [return] NRF24_OK if SPI write successful */
nrf24_status_t nrf24_hal_write_ack(const nrf24_handle_t * radio, const uint8_t * ack, const uint8_t alen);

#endif