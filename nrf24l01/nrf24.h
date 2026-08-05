
#ifndef RF_H_
#define RF_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "error_types.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

#define RADIO_MODE_TX           1
#define RADIO_MODE_RX           0

#define MAX_RF_CONN             5

#define MAX_PKT_SIZE            32

/************************* Commands *************************/
#define NRF_R_REGISTER              0
#define NRF_W_REGISTER              0b00100000
#define NRF_R_RX_PAYLOAD            0b01100001
#define NRF_W_TX_PAYLOAD            0b10100000
#define NRF_FLUSH_TX                0b11100001
#define NRF_FLUSH_RX                0b11100010
#define NRF_REUSE_TX_PL             0b11100011
#define NRF_ACTIVATE                0b01010000
#define NRF_R_RX_PL_WID             0b01100000
#define NRF_W_ACK_PAYLOAD           0b10101000
#define NRF_W_TX_PAYLOAD_NOACK      0b10110000
#define NRF_NOP                     0b11111111
/************************************************************/

/********************* Register Address *********************/
#define NRF_REG_CONFIG                  0x00
#define NRF_REG_EN_AA                   0x01
#define NRF_REG_EN_RXADDR               0x02
#define NRF_REG_SETUP_AW                0x03
#define NRF_REG_SETUP_RETR              0x04
#define NRF_REG_RF_CH                   0x05
#define NRF_REG_RF_SETUP                0x06
#define NRF_REG_STATUS                  0x07
#define NRF_REG_OBSERVE_TX              0x08
#define NRF_REG_CD                      0x09
#define NRF_REG_RX_ADDR_P0              0x0a
#define NRF_REG_RX_ADDR_P1              0x0b
#define NRF_REG_RX_ADDR_P2              0x0c
#define NRF_REG_RX_ADDR_P3              0x0d
#define NRF_REG_RX_ADDR_P4              0x0e
#define NRF_REG_RX_ADDR_P5              0x0f
#define NRF_REG_TX_ADDR                 0x10
#define NRF_REG_RX_PW_P0                0x11
#define NRF_REG_RX_PW_P1                0x12
#define NRF_REG_RX_PW_P2                0x13
#define NRF_REG_RX_PW_P3                0x14
#define NRF_REG_RX_PW_P4                0x15
#define NRF_REG_RX_PW_P5                0x16
#define NRF_REG_FIFO_STATUS             0x17
#define NRF_REG_DYNPD                   0x1c
#define NRF_REG_FEATURE                 0x1d
/************************************************************/

/*********************** Bit Mapping ************************/
// CONFIG
#define NRF_BIT_MASK_RX_DR              0x40
#define NRF_BIT_MASK_TX_DS              0x20
#define NRF_BIT_MASK_MAX_RT             0x10
#define NRF_BIT_EN_CRC                  0x08
#define NRF_BIT_CRCO                    0x04
#define NRF_BIT_PWR_UP                  0x02
#define NRF_BIT_PRIM_RX                 0x01

// EN_AA
#define NRF_BIT_ENAA_P5                 0x20
#define NRF_BIT_ENAA_P4                 0x10
#define NRF_BIT_ENAA_P3                 0x08
#define NRF_BIT_ENAA_P2                 0x04
#define NRF_BIT_ENAA_P1                 0x02
#define NRF_BIT_ENAA_P0                 0x01

// EN_RXADDR
#define NRF_BIT_ERX_P5                  0x20
#define NRF_BIT_ERX_P4                  0x10
#define NRF_BIT_ERX_P3                  0x08
#define NRF_BIT_ERX_P2                  0x04
#define NRF_BIT_ERX_P1                  0x02
#define NRF_BIT_ERX_P0                  0x01

// SETUP_AW
#define NRF_BIT_AW_3                    0x01
#define NRF_BIT_AW_4                    0x02
#define NRF_BIT_AW_5                    0x01 | 0x02

// RF_SETUP
#define NRF_BIT_PLL_LOCK                0x10
#define NRF_BIT_RF_DR                   0x08
#define NRF_BIT_PWR_1                   0x04
#define NRF_BIT_PWR_0                   0x02
#define NRF_BIT_LNA_HCURR               0x01

#define NRF_BIT_RF_DR_SHIFT             3
#define NRF_BIT_RF_PWR_SHIFT            1

// STATUS
#define NRF_BIT_RX_DR                   0x40
#define NRF_BIT_TX_DS                   0x20
#define NRF_BIT_MAX_RT                  0x10
#define NRF_BIT_RX_P_NO                 0x02 | 0x04 | 0x08
#define NRF_BIT_TX_FULL                 0x01

// DYNPD
#define NRF_BIT_DPL_P5                  0x20
#define NRF_BIT_DPL_P4                  0x10
#define NRF_BIT_DPL_P3                  0x08
#define NRF_BIT_DPL_P2                  0x04
#define NRF_BIT_DPL_P1                  0x02
#define NRF_BIT_DPL_P0                  0x01

// FEATURE
#define NRF_BIT_EN_DPL                  0x04
#define NRF_BIT_EN_ACK_PAY              0x02
#define NRF_BIT_EN_DYN_ACK              0x01
/************************************************************/

// return types for RF situations
typedef enum {

    RF_SUCCESS = 0,
    RF_INVALID_CONFIG,
    RF_INVALID_PACKET,
    RF_MAX_RETRIES,
    RF_TIMEOUT

} ErrorRF;

// RF payload types
typedef enum : uint8_t {

    NRF_PAYLOAD_DYNAMIC = 0,
    NRF_PAYLOAD_STATIC

} NrfPayloadType;

// RF air data rates
typedef enum : uint8_t {

    NRF_DATARATE_1MBPS = 0,
    NRF_DATARATE_2MBPS

} NrfDataRate;

// RF output power in TX mode
typedef enum : uint8_t {

    NRF_TX_PWR_M18DBM = 0,
    NRF_TX_PWR_M12DBM,
    NRF_TX_PWR_M6DBM,
    NRF_TX_PWR_0DBM

} NrfPower;

typedef enum : uint8_t {

    NRF_RETRY_CNT_DISABLE = 0,
    NRF_RETRY_CNT_1,
    NRF_RETRY_CNT_2,
    NRF_RETRY_CNT_3,
    NRF_RETRY_CNT_4,
    NRF_RETRY_CNT_5,
    NRF_RETRY_CNT_6,
    NRF_RETRY_CNT_7,
    NRF_RETRY_CNT_8,
    NRF_RETRY_CNT_9,
    NRF_RETRY_CNT_10,
    NRF_RETRY_CNT_11,
    NRF_RETRY_CNT_12,
    NRF_RETRY_CNT_13,
    NRF_RETRY_CNT_14,
    NRF_RETRY_CNT_15

} NrfRetryCnt;

// retry delay between failed sends
typedef enum : uint8_t {

    NRF_RETRY_DELAY_250 = 0,
    NRF_RETRY_DELAY_500,         
    NRF_RETRY_DELAY_750,         
    NRF_RETRY_DELAY_1000,        
    NRF_RETRY_DELAY_1250,        
    NRF_RETRY_DELAY_1500,        
    NRF_RETRY_DELAY_1750,        
    NRF_RETRY_DELAY_2000,        
    NRF_RETRY_DELAY_2250,        
    NRF_RETRY_DELAY_2500,        
    NRF_RETRY_DELAY_2750,        
    NRF_RETRY_DELAY_3000,        
    NRF_RETRY_DELAY_3250,        
    NRF_RETRY_DELAY_3500,        
    NRF_RETRY_DELAY_3750,        
    NRF_RETRY_DELAY_4000

} NrfRetryDelay;

typedef struct {

    SPI_HandleTypeDef * hspi;           // SPI handle

    void (*delay_us)(uint32_t);         // pointer to delay function
    void (*delay_ms)(uint32_t);         // pointer to delay function

    GPIO_TypeDef * ce_gpio;             // chip enable GPIO
    GPIO_TypeDef * cs_gpio;             // chip select GPIO
    uint16_t ce_pin;                    // chip enable pin
    uint16_t cs_pin;                    // chip select pin

    NrfPayloadType payload_type;        // dynamic/static payloads
    EnableType ack;                     // enable/disable acks

    uint8_t payload_length;             // only applies to STATIC_PAYLOAD
    
    uint8_t freq_ch;                    // frequency channel
    NrfDataRate data_rate;              // data rate
    NrfPower power_level;               // transmitting power level

    uint64_t this_addr;                 // address of this node
    uint64_t node_addr;                 // address of receiving node

    struct {
        EnableType rx_dr;               // RX data received
        EnableType tx_ds;               // TX data sent
        EnableType max_rt;              // Max retries
    } RadioIrqs;

} RadioParams;

ErrorRF rf_init(RadioParams * radio);
ErrorRF rf_send(RadioParams * radio, uint8_t * packet, uint8_t plen, uint8_t * ack, uint8_t * alen);
ErrorRF rf_listen(RadioParams * radio, uint32_t timeout);
ErrorRF rf_listen_it(RadioParams * radio);
ErrorRF rf_receive(RadioParams * radio, uint8_t * packet, uint8_t * plen, uint8_t * ack, uint8_t alen);
void rf_set_retransmit(RadioParams * radio, NrfRetryDelay delay, NrfRetryCnt count);
void rf_set_channel(RadioParams * radio, uint8_t ch);
void rf_set_data_rate(RadioParams * radio, NrfDataRate rate);
void rf_set_power_level(RadioParams * radio, NrfPower power);
void rf_flush_rx(RadioParams * radio);
void rf_flush_tx(RadioParams * radio);
uint8_t rf_read_status(RadioParams * radio);
void rf_clear_status(RadioParams * radio);
uint8_t rf_carrier_detect(RadioParams * radio);

#endif