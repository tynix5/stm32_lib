#ifndef NRF24_H_
#define NRF24_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

#define RADIO_MODE_TX 1
#define RADIO_MODE_RX 0

#define NRF24_MAX_PIPES          6
#define NRF24_MAX_PKT_SIZE_BYTES 32

/************************* Commands *************************/
#define NRF24_R_REGISTER         0
#define NRF24_W_REGISTER         0b00100000
#define NRF24_R_RX_PAYLOAD       0b01100001
#define NRF24_W_TX_PAYLOAD       0b10100000
#define NRF24_FLUSH_TX           0b11100001
#define NRF24_FLUSH_RX           0b11100010
#define NRF24_REUSE_TX_PL        0b11100011
#define NRF24_ACTIVATE           0b01010000
#define NRF24_R_RX_PL_WID        0b01100000
#define NRF24_W_ACK_PAYLOAD      0b10101000
#define NRF24_W_TX_PAYLOAD_NOACK 0b10110000
#define NRF24_NOP                0b11111111
/************************************************************/

/********************* Register Address *********************/
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_CD          0x09
#define NRF24_REG_RX_ADDR_P0  0x0a
#define NRF24_REG_RX_ADDR_P1  0x0b
#define NRF24_REG_RX_ADDR_P2  0x0c
#define NRF24_REG_RX_ADDR_P3  0x0d
#define NRF24_REG_RX_ADDR_P4  0x0e
#define NRF24_REG_RX_ADDR_P5  0x0f
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_RX_PW_P1    0x12
#define NRF24_REG_RX_PW_P2    0x13
#define NRF24_REG_RX_PW_P3    0x14
#define NRF24_REG_RX_PW_P4    0x15
#define NRF24_REG_RX_PW_P5    0x16
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD       0x1c
#define NRF24_REG_FEATURE     0x1d
/************************************************************/

/*********************** Bit Mapping ************************/
// CONFIG
#define NRF24_BIT_MASK_RX_DR  0x40
#define NRF24_BIT_MASK_TX_DS  0x20
#define NRF24_BIT_MASK_MAX_RT 0x10
#define NRF24_BIT_EN_CRC      0x08
#define NRF24_BIT_CRCO        0x04
#define NRF24_BIT_PWR_UP      0x02
#define NRF24_BIT_PRIM_RX     0x01

// EN_AA
#define NRF24_BIT_ENAA_P5 0x20
#define NRF24_BIT_ENAA_P4 0x10
#define NRF24_BIT_ENAA_P3 0x08
#define NRF24_BIT_ENAA_P2 0x04
#define NRF24_BIT_ENAA_P1 0x02
#define NRF24_BIT_ENAA_P0 0x01

// EN_RXADDR
#define NRF24_BIT_ERX_P5 0x20
#define NRF24_BIT_ERX_P4 0x10
#define NRF24_BIT_ERX_P3 0x08
#define NRF24_BIT_ERX_P2 0x04
#define NRF24_BIT_ERX_P1 0x02
#define NRF24_BIT_ERX_P0 0x01

// SETUP_AW
#define NRF24_BIT_AW_3 0x01
#define NRF24_BIT_AW_4 0x02
#define NRF24_BIT_AW_5 0x01 | 0x02

// RF_SETUP
#define NRF24_BIT_PLL_LOCK  0x10
#define NRF24_BIT_RF_DR     0x08
#define NRF24_BIT_PWR_1     0x04
#define NRF24_BIT_PWR_0     0x02
#define NRF24_BIT_LNA_HCURR 0x01

#define NRF24_BIT_RF_DR_SHIFT  3
#define NRF24_BIT_RF_PWR_SHIFT 1

// STATUS
#define NRF24_BIT_RX_DR   0x40
#define NRF24_BIT_TX_DS   0x20
#define NRF24_BIT_MAX_RT  0x10
#define NRF24_BIT_RX_P_NO 0x02 | 0x04 | 0x08
#define NRF24_BIT_TX_FULL 0x01

// DYNPD
#define NRF24_BIT_DPL_P5 0x20
#define NRF24_BIT_DPL_P4 0x10
#define NRF24_BIT_DPL_P3 0x08
#define NRF24_BIT_DPL_P2 0x04
#define NRF24_BIT_DPL_P1 0x02
#define NRF24_BIT_DPL_P0 0x01

// FEATURE
#define NRF24_BIT_EN_DPL     0x04
#define NRF24_BIT_EN_ACK_PAY 0x02
#define NRF24_BIT_EN_DYN_ACK 0x01
/************************************************************/

// nRF24L01 return types
typedef enum
{
    NRF24_OK = 0,
    NRF24_INVALID_CFG,
    NRF24_INVALID_PKT,
    NRF24_MAX_RETRIES,
    NRF24_TIMEOUT,
    NRF24_NULL_POINTER,
    NRF24_NOT_INIT,
    NRF24_TOO_MANY_PIPES,
} nrf24_status_t;

// RF air data rates
typedef enum : uint8_t
{
    NRF24_DATARATE_1MBPS = 0,
    NRF24_DATARATE_2MBPS
} nrf24_datarate_t;

// RF output power in TX mode
typedef enum : uint8_t
{
    NRF24_TX_PWR_M18DBM = 0,
    NRF24_TX_PWR_M12DBM,
    NRF24_TX_PWR_M6DBM,
    NRF24_TX_PWR_0DBM
} nrf24_power_t;

typedef enum : uint8_t
{
    NRF24_RETRY_CNT_DISABLE = 0,
    NRF24_RETRY_CNT_1,
    NRF24_RETRY_CNT_2,
    NRF24_RETRY_CNT_3,
    NRF24_RETRY_CNT_4,
    NRF24_RETRY_CNT_5,
    NRF24_RETRY_CNT_6,
    NRF24_RETRY_CNT_7,
    NRF24_RETRY_CNT_8,
    NRF24_RETRY_CNT_9,
    NRF24_RETRY_CNT_10,
    NRF24_RETRY_CNT_11,
    NRF24_RETRY_CNT_12,
    NRF24_RETRY_CNT_13,
    NRF24_RETRY_CNT_14,
    NRF24_RETRY_CNT_15
} nrf24_retry_cnt_t;

// retry delay between failed sends
typedef enum : uint8_t
{
    NRF24_RETRY_DELAY_250 = 0,
    NRF24_RETRY_DELAY_500,
    NRF24_RETRY_DELAY_750,
    NRF24_RETRY_DELAY_1000,
    NRF24_RETRY_DELAY_1250,
    NRF24_RETRY_DELAY_1500,
    NRF24_RETRY_DELAY_1750,
    NRF24_RETRY_DELAY_2000,
    NRF24_RETRY_DELAY_2250,
    NRF24_RETRY_DELAY_2500,
    NRF24_RETRY_DELAY_2750,
    NRF24_RETRY_DELAY_3000,
    NRF24_RETRY_DELAY_3250,
    NRF24_RETRY_DELAY_3500,
    NRF24_RETRY_DELAY_3750,
    NRF24_RETRY_DELAY_4000
} nrf24_retry_delay_t;

// static or dynamic payload lengths
typedef enum : uint8_t
{
    NRF24_PAYLOAD_STATIC = 0,
    NRF24_PAYLOAD_DYNAMIC,
} nrf24_payload_type_t;

// auto-acknowledgement behavior selection
typedef enum : uint8_t
{
    NRF24_AUTO_ACK_DISABLE = 0,
    NRF24_AUTO_ACK_ENABLE,
    NRF24_AUTO_ACK_PAYLOAD, // auto-ack + optional payload
} nrf24_ack_t;

// address of reading pipes
typedef enum : uint8_t
{
    NRF24_PIPE0 = 0,
    NRF24_PIPE1,
    NRF24_PIPE2,
    NRF24_PIPE3,
    NRF24_PIPE4,
    NRF24_PIPE5,
    NRF24_PIPE_EMPTY = 7,
} nrf24_pipe_num_t;

// address length (bytes) selection
typedef enum : uint8_t
{
    NRF24_ADDR_W_3 = 1,
    NRF24_ADDR_W_4,
    NRF24_ADDR_W_5,
} nrf24_addr_width_t;

typedef struct
{

    uint64_t address;                  // address of RX pipe (5 bytes)

    nrf24_payload_type_t payload_type; // dynamic or static payloads
    nrf24_ack_t ack;                   // enable/disable acks, optional acks with payload

    uint8_t payload_length;            // only applies to NRF24_PAYLOAD_STATIC

    nrf24_pipe_num_t num;              // pipe number

} nrf24_pipe_t;

typedef struct
{
    SPI_HandleTypeDef * hspi;   // SPI handle

    void (*delay_us)(uint32_t); // pointer to delay function
    void (*delay_ms)(uint32_t); // pointer to delay function

    GPIO_TypeDef * ce_gpio;     // chip enable GPIO
    GPIO_TypeDef * cs_gpio;     // chip select GPIO
    uint16_t ce_pin;            // chip enable pin
    uint16_t cs_pin;            // chip select pin

    uint8_t freq_ch;            // frequency channel
    nrf24_datarate_t data_rate; // data rate
    nrf24_power_t power_level;  // transmitting power level
    struct
    {
        bool rx_dr_en;          // RX data received
        bool tx_ds_en;          // TX data sent
        bool max_rt_en;         // Max retries
    } nrf24_irqs_t;

    nrf24_pipe_t _pipes[6];     // stored RX pipe information
    bool _initialized;          // has nRF24L01 been initalized?

} nrf24_handle_t;

/* Power on, clear previous states, set retransmit, power level, and enable interrupts */
/* [in/out] radio - nRF24L01 structure handle */
nrf24_status_t nrf24_init(nrf24_handle_t * radio);

/* Enable a reading pipe on 1 of the 6 available pipes */
/* [in/out] radio - nRF24L01 structure handle */
/* [input] pipe - RX data pipe structure */
nrf24_status_t nrf24_open_rx_pipe(const nrf24_handle_t * radio, const nrf24_pipe_t * pipe);

/* Set the TX pipe address */
/* [input] radio - nRF24L01 structure handle */
/* [input] address - TX pipe address */
nrf24_status_t nrf24_open_tx_pipe(const nrf24_handle_t * radio, const uint64_t address);

/* Send a packet and if expecting an ack payload, return one */
/* [input] radio - nRF24L01 structure handle */
/* [input] pkt - serialized data being sent */
/* [input] plen - pkt size in bytes */
/* [input] expect_ack - true if expecting an ack with a payload, false otherwise */
/* [output] ack - received ack payload, not populated if none received */
/* [output] alen -  size of received ack payload in bytes */
nrf24_status_t nrf24_send(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen, const bool expect_ack, uint8_t * ack, uint8_t * alen);

/* Listen for a timed period, waiting for a packet to arrive */
/* [input] radio - nRF24L01 structure handle */
/* [input] timeout_ms - period before exiting function if packet not received */
/* [return] NRF24_OK if packet received, TIMEOUT otherwise */
nrf24_status_t nrf24_listen(const nrf24_handle_t * radio, const uint32_t timeout_ms);

/* Enter listening mode and exit, waiting for interrupt */
/* [input] radio - nRF24L01 structure handle */
/* [return] NRF24_OK if initialized and SPI handle valid */
nrf24_status_t nrf24_listen_it(const nrf24_handle_t * radio);

/* After packet reception, dump contents and optionally send ack packet back */
/* [input] radio - nRF24L01 structure handle */
/* [input] ack - serialized ack packet sent after packet received */
/* [input] alen - size of ack in bytes */
/* [output] pkt - received packet */
/* [output] plen - size of pkt in bytes */
/* [output] pipe_num - address of RX pipe data received on */
/* [return] NRF24_OK if reception successful, INVALID_PKT if garbage */
nrf24_status_t nrf24_receive(const nrf24_handle_t * radio, const uint8_t * ack, const uint8_t alen, uint8_t * pkt, uint8_t * plen, nrf24_pipe_num_t * pipe_num);

/* Set TX retransmit delay and count */
/* [input] radio - nRF24L01 structure handle */
/* [input] delay - delay between successive packet send attempts */
/* [input] count - number of packet attempts */
/* [return] NRF24_OK if successful SPI write, INVALID_CFG if arguments out of range */
nrf24_status_t nrf24_set_retransmit(const nrf24_handle_t * radio, const nrf24_retry_delay_t delay, const nrf24_retry_cnt_t count);

/* Set frequency channel (must be matched between TX and RX) */
/* [in/out] radio - nRF24L01 structure handle */
/* [input] ch - frequency channel [0, 127] */
/* [return] NRF24_OK if successful SPI write, INVALID_CFG if arguments out of range */
nrf24_status_t nrf24_set_channel(nrf24_handle_t * radio, const uint8_t ch);

/* Set data rate (must be matched between TX and RX) */
/* [in/out] radio - nRF24L01 structure handle */
/* [input] rate - RF air data rate */
/* [return] NRF24_OK if successful SPI write, INVALID_CFG if arguments out of range */
nrf24_status_t nrf24_set_data_rate(nrf24_handle_t * radio, const nrf24_datarate_t rate);

/* Set TX power level */
/* [in/out] radio - nRF24L01 structure handle */
/* [input] power - power output level in dBm */
/* [return] NRF24_OK if successful SPI write, INVALID_CFG if arguments out of range */
nrf24_status_t nrf24_set_power_level(nrf24_handle_t * radio, const nrf24_power_t power);

/* Clear RX FIFO */
/* [input] radio - nRF24L01 structure handle */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_flush_rx(const nrf24_handle_t * radio);

/* Clear TX FIFO */
/* [input] radio - nRF24L01 structure handle */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_flush_tx(const nrf24_handle_t * radio);

/* Read from status register */
/* [input] radio - nRF24L01 structure handle */
/* [output] status - output of status register */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_read_status(const nrf24_handle_t * radio, uint8_t * status);

/* Clear all status register bits */
/* [input] radio - nRF24L01 structure handle */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_clear_status(const nrf24_handle_t * radio);

/* Read from FIFO status register */
/* [input] radio - nRF24L01 structure handle */
/* [output] status - output of FIFO status register */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_read_fifo_status(const nrf24_handle_t * radio, uint8_t * status);

/* Carrier detect */
/* [input] radio - nRF24L01 structure handle */
/* [output] carrier - boolean value if carrier has been detected */
/* [return] NRF24_OK if successful SPI write */
nrf24_status_t nrf24_carrier_detect(const nrf24_handle_t * radio, uint8_t * carrier);

#endif