
#ifndef RF_H_
#define RF_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

#define RADIO_MODE_TX           1
#define RADIO_MODE_RX           0

#define HAL_TIMEOUT             1000

#define MAX_RF_CONN             5

#define MAX_PKT_SIZE            32

#define USE_DYNAMIC_PAYLOAD     1
#define STATIC_PAYLOAD          0
#define USE_ACKS                1
#define NO_ACKS                 0

/************************* Commands *************************/
#define R_REGISTER              0
#define W_REGISTER              0b00100000
#define R_RX_PAYLOAD            0b01100001
#define W_TX_PAYLOAD            0b10100000
#define FLUSH_TX                0b11100001
#define FLUSH_RX                0b11100010
#define REUSE_TX_PL             0b11100011
#define ACTIVATE                0b01010000
#define R_RX_PL_WID             0b01100000
#define W_ACK_PAYLOAD           0b10101000
#define W_TX_PAYLOAD_NOACK      0b10110000
#define NOP                     0b11111111
/************************************************************/

/********************* Register Address *********************/
#define CONFIG                  0x00
#define EN_AA                   0x01
#define EN_RXADDR               0x02
#define SETUP_AW                0x03
#define SETUP_RETR              0x04
#define RF_CH                   0x05
#define RF_SETUP                0x06
#define STATUS                  0x07
#define OBSERVE_TX              0x08
#define CD                      0x09
#define RX_ADDR_P0              0x0a
#define RX_ADDR_P1              0x0b
#define RX_ADDR_P2              0x0c
#define RX_ADDR_P3              0x0d
#define RX_ADDR_P4              0x0e
#define RX_ADDR_P5              0x0f
#define TX_ADDR                 0x10
#define RX_PW_P0                0x11
#define RX_PW_P1                0x12
#define RX_PW_P2                0x13
#define RX_PW_P3                0x14
#define RX_PW_P4                0x15
#define RX_PW_P5                0x16
#define FIFO_STATUS             0x17
#define DYNPD                   0x1c
#define FEATURE                 0x1d
/************************************************************/

/*********************** Bit Mapping ************************/
// CONFIG
#define EN_CRC                  (1 << 3)
#define CRCO                    (1 << 2)
#define PWR_UP                  (1 << 1)
#define PRIM_RX                 (1 << 0)

// EN_AA
#define ENAA_P5                 (1 << 5)
#define ENAA_P4                 (1 << 4)
#define ENAA_P3                 (1 << 3)
#define ENAA_P2                 (1 << 2)
#define ENAA_P1                 (1 << 1)
#define ENAA_P0                 (1 << 0)

// EN_RXADDR
#define ERX_P5                  (1 << 5)
#define ERX_P4                  (1 << 4)
#define ERX_P3                  (1 << 3)
#define ERX_P2                  (1 << 2)
#define ERX_P1                  (1 << 1)
#define ERX_P0                  (1 << 0)

// SETUP_AW
#define AW_3                    (1 << 0)
#define AW_4                    (1 << 1)
#define AW_5                    (1 << 0) | (1 << 1)

// SETUP_RETR
#define RETRY_DELAY_250         0
#define RETRY_DELAY_500         1
#define RETRY_DELAY_750         2
#define RETRY_DELAY_1000        3
#define RETRY_DELAY_1250        4
#define RETRY_DELAY_1500        5
#define RETRY_DELAY_1750        6
#define RETRY_DELAY_2000        7
#define RETRY_DELAY_2250        8
#define RETRY_DELAY_2500        9
#define RETRY_DELAY_2750        10
#define RETRY_DELAY_3000        11
#define RETRY_DELAY_3250        12
#define RETRY_DELAY_3500        13
#define RETRY_DELAY_3750        14
#define RETRY_DELAY_4000        15
#define RETRY_CNT_DISABLE       0
#define RETRY_CNT_1             1
#define RETRY_CNT_2             2
#define RETRY_CNT_3             3
#define RETRY_CNT_4             4
#define RETRY_CNT_5             5
#define RETRY_CNT_6             6
#define RETRY_CNT_7             7
#define RETRY_CNT_8             8
#define RETRY_CNT_9             9
#define RETRY_CNT_10            10
#define RETRY_CNT_11            11
#define RETRY_CNT_12            12
#define RETRY_CNT_13            13
#define RETRY_CNT_14            14
#define RETRY_CNT_15            15

// RF_SETUP
#define RF_DR_1MBPS             0
#define RF_DR_2MBPS             (1 << 3)
#define RF_PWR_18DBM            0
#define RF_PWR_12DBM            (1 << 1)
#define RF_PWR_6DBM             (1 << 2)
#define RF_PWR_0DBM             (1 << 1) | (1 << 2)

// STATUS
#define RX_DR                   (1 << 6)
#define TX_DS                   (1 << 5)
#define MAX_RT                  (1 << 4)
#define RX_P_NO                 (1 << 1) | (1 << 2) | (1 << 3)
#define TX_FULL                 (1 << 0)

// DYNPD
#define DPL_P5                  (1 << 5)
#define DPL_P4                  (1 << 4)
#define DPL_P3                  (1 << 3)
#define DPL_P2                  (1 << 2)
#define DPL_P1                  (1 << 1)
#define DPL_P0                  (1 << 0)

// FEATURE
#define EN_DPL                  (1 << 2)
#define EN_ACK_PAY              (1 << 1)
#define EN_DYN_ACK              (1 << 0)
/************************************************************/

typedef struct {

    SPI_HandleTypeDef * hspi;

    void (*delay_us)(uint32_t);
    void (*delay_ms)(uint32_t);

    /************** Pin Definitions ***********/
    GPIO_TypeDef * ce_gpio;
    GPIO_TypeDef * cs_gpio;
    uint16_t ce_pin;
    uint16_t cs_pin;
    /******************************************/

    uint8_t dynamic_payloads;
    uint8_t use_acks;
    
    uint8_t freq_ch;
    uint8_t data_rate;
    uint8_t power_level;

    uint64_t this_addr;
    uint64_t node_addr;

} RadioParams;

void rf_init(RadioParams * radio);
void rf_send_w_no_ack(RadioParams * radio, uint8_t * packet, uint8_t len);
void rf_send_w_ack(RadioParams * radio, uint8_t * packet, uint8_t len, uint8_t * ack, uint8_t * alen);
uint8_t rf_listen_no_ack(RadioParams * radio, uint32_t timeout);
uint8_t rf_listen_ack(RadioParams * radio, uint8_t * response, uint8_t rlen, uint32_t timeout);
void rf_receive(RadioParams * radio, uint8_t * packet, uint8_t * len);
void rf_set_retransmit(RadioParams * radio, uint8_t delay, uint8_t count);
void rf_set_channel(RadioParams * radio, uint8_t ch);
void rf_set_data_rate(RadioParams * radio, uint8_t rate);
void rf_set_power_level(RadioParams * radio, uint8_t power);
// void rf_set_power_level(RadioParams radio, uint8_t power);

#endif