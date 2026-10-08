
#include "nrf24.h"
#include "nrf24_hal.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

/**************************************** Helper Functions *******************************************/
/* Enter standby mode 1 */
/* [input] radio - nRF24L01 structure handle */
static void nrf24_standby(const nrf24_handle_t * radio);

/* Enter standby mode 1 with PRIM_RX enabled */
/* [input] radio - nRF24L01 structure handle */
static void nrf24_standby_rx(const nrf24_handle_t * radio);

/* Activate features R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK */
/* [input] radio - nRF24L01 structure handle */
static void nrf24_activate_features(const nrf24_handle_t * radio);

/* Set address width of RX pipes */
/* [input] radio - nRF24L01 structure handle */
/* [input] width - address width in bytes */
static void nrf24_set_address_width(const nrf24_handle_t * radio, const nrf24_addr_width_t width);

/* Set RX addresses */
/* [input] radio - nRF24L01 structure handle */
static void nrf24_write_rx_addr(const nrf24_handle_t * radio, const nrf24_pipe_t * pipe);

/* After data received, detect which pipe it came from */
/* [input] radio - nRF24L01 structure handle */
/* [output] pipe - pipe data was received from */
static void nrf24_read_rx_pipe_num(const nrf24_handle_t * radio, nrf24_pipe_num_t * pipe);

/* Read payload width based on dynamic/static selection */
/* [input] radio - nRF24L01 structure handle */
/* [input] pipe - RX pipe being read from */
/* [output] width - number of bytes received */
static void nrf24_read_payload_width(const nrf24_handle_t * radio, const nrf24_pipe_num_t pipe, uint8_t * width);

/* Enable selected interrupts */
/* [input] radio - nRF24L01 structure handle */
static void nrf24_set_irq(const nrf24_handle_t * radio);
/*****************************************************************************************************/

nrf24_status_t nrf24_init(nrf24_handle_t * radio)
{
    if (radio->hspi == NULL || radio->delay_ms == NULL || radio->delay_us == NULL)
        return NRF24_NULL_POINTER;

    nrf24_hal_deselect(radio);
    nrf24_hal_disable(radio);

    // power on reset, delay >10.3ms
    radio->delay_ms(20);

    // PWR_UP = 1, wait >1.5ms for start-up
    nrf24_standby(radio);
    radio->delay_ms(2);

    // default configurations
    nrf24_set_channel(radio, radio->freq_ch);
    nrf24_set_power_level(radio, radio->power_level);
    nrf24_set_data_rate(radio, radio->data_rate);
    nrf24_set_retransmit(radio, NRF24_RETRY_DELAY_1000, NRF24_RETRY_CNT_3);

    // set address width to 5 bytes
    nrf24_set_address_width(radio, NRF24_ADDR_W_5);

    // enable selected interrupts
    nrf24_set_irq(radio);

    // flush garbage
    nrf24_flush_rx(radio);
    nrf24_flush_tx(radio);
    // reset status register in case communication has been previously lost
    nrf24_clear_status(radio);

    radio->_initialized = true;

    // radio module should be in standby mode after exiting
    return NRF24_OK;
}

nrf24_status_t nrf24_open_rx_pipe(const nrf24_handle_t * radio, const nrf24_pipe_t * pipe)
{
    if (pipe->payload_type > NRF24_PAYLOAD_DYNAMIC || pipe->ack > NRF24_AUTO_ACK_PAYLOAD || pipe->payload_length > NRF24_MAX_PKT_SIZE_BYTES)
        return NRF24_INVALID_CFG;

    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    // dynamic payloads must be enabled when using acks with payload
    if (pipe->payload_type == NRF24_PAYLOAD_STATIC && pipe->ack == NRF24_AUTO_ACK_PAYLOAD)
        return NRF24_INVALID_CFG;

    if (pipe->num > NRF24_PIPE5)
        return NRF24_TOO_MANY_PIPES;

    // read current configurations and do not overwrite registers
    uint8_t dynpd, en_aa, curr_features;
    nrf24_hal_read_reg(radio, NRF24_REG_DYNPD, &dynpd);
    nrf24_hal_read_reg(radio, NRF24_REG_EN_AA, &en_aa);
    nrf24_hal_read_reg(radio, NRF24_REG_FEATURE, &curr_features);

    if (pipe->payload_type == NRF24_PAYLOAD_DYNAMIC)
    {
        // enable dynamic lengths for this pipe
        dynpd |= (1 << pipe->num);
        // dynamic payloads feature must be enabled
        curr_features |= NRF24_BIT_EN_DPL;
    }
    else
    {
        // disable dynamic payload lengths
        dynpd &= ~(1 << pipe->num);

        // set static length
        uint8_t rx_addr_reg = NRF24_REG_RX_PW_P0 + pipe->num;
        nrf24_hal_write_reg(radio, rx_addr_reg, &(pipe->payload_length), 1);
    }

    switch (pipe->ack)
    {
    case NRF24_AUTO_ACK_DISABLE:
        en_aa &= ~(1 << pipe->num); // disable auto-acks
        curr_features |= NRF24_BIT_EN_DYN_ACK;
        break;
    case NRF24_AUTO_ACK_ENABLE:
        en_aa |= (1 << pipe->num);             // enable auto-acks
        break;
    case NRF24_AUTO_ACK_PAYLOAD:
        en_aa |= (1 << pipe->num);             // enable auto-acks
        curr_features |= NRF24_BIT_EN_ACK_PAY; // enable acks with payload
        break;
    }

    // write new register states back
    nrf24_hal_write_reg(radio, NRF24_REG_DYNPD, &dynpd, 1);
    nrf24_hal_write_reg(radio, NRF24_REG_EN_AA, &en_aa, 1);

    // enable writes to any special function register
    nrf24_activate_features(radio);
    nrf24_hal_write_reg(radio, NRF24_REG_FEATURE, &curr_features, 1);

    // enable reading pipe
    uint8_t erx;
    nrf24_hal_read_reg(radio, NRF24_REG_EN_RXADDR, &erx);
    erx |= (1 << pipe->num);
    nrf24_hal_write_reg(radio, NRF24_REG_EN_RXADDR, &erx, 1);

    // set RX address
    nrf24_write_rx_addr(radio, pipe);

    // copy all pipe data into radio structure
    memcpy((uint8_t *)(&(radio->_pipes[pipe->num])), (uint8_t *)pipe, sizeof(*pipe));

    return NRF24_OK;
}

nrf24_status_t nrf24_open_tx_pipe(const nrf24_handle_t * radio, const uint64_t address)
{
    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    // RX_ADDR_P0 must equal the TX address in order to handle automatic acknowledgements
    uint8_t tx_addr[5] = { (uint8_t)address, (uint8_t)(address >> 8), (uint8_t)(address >> 16), (uint8_t)(address >> 24), (uint8_t)(address >> 32) };

    return nrf24_hal_write_reg(radio, NRF24_REG_TX_ADDR, tx_addr, sizeof(tx_addr));
}

nrf24_status_t nrf24_send(const nrf24_handle_t * radio, const uint8_t * pkt, const uint8_t plen, const bool expect_ack, uint8_t * ack, uint8_t * alen)
{
    if (plen > NRF24_MAX_PKT_SIZE_BYTES)
        return NRF24_INVALID_CFG;

    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    // ensure mode is in standby before placing in tx mode
    nrf24_standby(radio);

    // write data to TX buffer
    if (expect_ack)
        nrf24_hal_write_tx_ack(radio, pkt, plen);
    else
        nrf24_hal_write_tx_nack(radio, pkt, plen);

    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    nrf24_hal_enable(radio);
    radio->delay_us(20);
    nrf24_hal_disable(radio);

    // wait for packet to be transmitted and acknowledgement received / packet sent
    while (1)
    {
        uint8_t status;
        nrf24_read_status(radio, &status);

        // if maximum retries have happened, flush FIFO and clear flag
        if (status & NRF24_BIT_MAX_RT)
        {
            uint8_t reset_max_rt = NRF24_BIT_MAX_RT;
            nrf24_hal_write_reg(radio, NRF24_REG_STATUS, &reset_max_rt, 1);
            nrf24_flush_tx(radio);
            return NRF24_MAX_RETRIES;
        }
        else if (status & NRF24_BIT_TX_DS) // if ack has been received or no ack is expected, exit
        {
            break;
        }
    }

    // clear data sent flag
    uint8_t clear_ds = NRF24_BIT_TX_DS;
    nrf24_hal_write_reg(radio, NRF24_REG_STATUS, &clear_ds, 1);

    // only read acknowledgement if enabled
    *alen = 0;

    uint8_t status;
    nrf24_read_status(radio, &status);

    // if receiving no ack, skip this section after TX_DS is cleared
    // if receiving ack with no payload, DR will not set and this will be skipped
    // if receiving ack with a payload, this section will be executed
    if (expect_ack && (status & NRF24_BIT_RX_DR))
    {
        // read acknowledgement width (auto-acks only on pipe 0)
        nrf24_read_payload_width(radio, NRF24_PIPE0, alen);

        if (*alen > NRF24_MAX_PKT_SIZE_BYTES)
        {
            nrf24_flush_rx(radio);
            return NRF24_INVALID_PKT;
        }
        else if (*alen != 0)
        {
            // read acknowledgement
            nrf24_hal_read_payload(radio, *alen, ack);

            // clear data ready flag reading acknowledgement
            uint8_t clear_dr = NRF24_BIT_RX_DR;
            nrf24_hal_write_reg(radio, NRF24_REG_STATUS, &clear_dr, 1);
        }
    }

    return NRF24_OK;
}

nrf24_status_t nrf24_listen(const nrf24_handle_t * radio, const uint32_t timeout_ms)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    
    // receiver --> PRIM_RX = 1, CE = 1
    nrf24_clear_status(radio);
    nrf24_standby_rx(radio);

    // wait >130us for RX settling
    nrf24_hal_enable(radio);
    radio->delay_us(200);

    // either poll continuously or wait for IRQ and timeout after certain period of time
    // ticks needs to be a measured value
    uint32_t elapsed_us = 0;
    uint32_t timeout_us = timeout_ms * 1000;
    const uint32_t delta = 100;

    while (elapsed_us < timeout_us)
    {
        uint8_t status;
        nrf24_read_status(radio, &status);

        if (status & NRF24_BIT_RX_DR) // if data has been received, exit
        {
            nrf24_hal_disable(radio);
            return NRF24_OK;
        }

        radio->delay_us(delta);
        elapsed_us += delta;
    }

    nrf24_hal_disable(radio);
    return NRF24_TIMEOUT;
}

nrf24_status_t nrf24_listen_it(const nrf24_handle_t * radio)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    // move to receiver mode and then exit (waiting for pin interrupts)
    // receiver --> PRIM_RX = 1, CE = 1
    nrf24_clear_status(radio);
    nrf24_standby_rx(radio);
    nrf24_hal_enable(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_receive(const nrf24_handle_t * radio, const uint8_t * ack, const uint8_t alen, uint8_t * pkt, uint8_t * plen, nrf24_pipe_num_t * pipe_num)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (!radio->_initialized)
        return NRF24_NOT_INIT;

    nrf24_hal_disable(radio);
    nrf24_read_rx_pipe_num(radio, pipe_num);          // get pipe from which data was received

    // if trying to receive data when FIFO is empty...
    if (*pipe_num == NRF24_PIPE_EMPTY)
    {
        *plen = 0;
        nrf24_clear_status(radio);
        return NRF24_INVALID_PKT;
    }

    nrf24_read_payload_width(radio, *pipe_num, plen); // read payload width based on pipe being dynamic/static

    // if an invalid packet, discard it and flush FIFO
    if (*plen > NRF24_MAX_PKT_SIZE_BYTES)
    {
        nrf24_flush_rx(radio);
        nrf24_clear_status(radio);
        *plen = 0;
        return NRF24_INVALID_PKT;
    }

    // read packet
    if (*plen != 0)
    {
        nrf24_hal_read_payload(radio, *plen, pkt);
    }

    // if payload with acks are enabled and attempting to be used...
    if (radio->_pipes[*pipe_num].ack == NRF24_AUTO_ACK_PAYLOAD && alen > 0)
    {
        // write response payload
        nrf24_hal_write_ack(radio, ack, alen);
    }

    // clear flags after reading packet and loading acks
    nrf24_clear_status(radio);

    return NRF24_OK;
}

nrf24_status_t nrf24_set_retransmit(const nrf24_handle_t * radio, const nrf24_retry_delay_t delay, const nrf24_retry_cnt_t count)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (delay > NRF24_RETRY_DELAY_4000 || count > NRF24_RETRY_CNT_15)
        return NRF24_INVALID_CFG;

    uint8_t setup_retry = ((delay & 0x0f) << 4) | (count & 0x0f);
    return nrf24_hal_write_reg(radio, NRF24_REG_SETUP_RETR, &setup_retry, 1);
}

nrf24_status_t nrf24_set_channel(nrf24_handle_t * radio, const uint8_t ch)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (ch > 0x7f)
        return NRF24_INVALID_CFG;

    radio->freq_ch = ch;

    uint8_t rf_ch = ch & 0x7f;
    return nrf24_hal_write_reg(radio, NRF24_REG_RF_CH, &rf_ch, 1);
}

nrf24_status_t nrf24_set_data_rate(nrf24_handle_t * radio, const nrf24_datarate_t rate)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (rate > NRF24_DATARATE_2MBPS)
        return NRF24_INVALID_CFG;

    radio->data_rate = rate;

    // read current power configurations and keep them
    uint8_t rf_setup;
    uint8_t status = nrf24_hal_read_reg(radio, NRF24_REG_RF_SETUP, &rf_setup);

    if (status != NRF24_OK)
        return status;

    // update data rate
    rf_setup = (rf_setup & 0x07) | (rate << NRF24_BIT_RF_DR_SHIFT);
    return nrf24_hal_write_reg(radio, NRF24_REG_RF_SETUP, &rf_setup, 1);
}

nrf24_status_t nrf24_set_power_level(nrf24_handle_t * radio, const nrf24_power_t power)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    if (power > NRF24_TX_PWR_0DBM)
        return NRF24_INVALID_CFG;

    radio->power_level = power;

    // read current data rate configurations and keep them
    uint8_t rf_setup;
    uint8_t status = nrf24_hal_read_reg(radio, NRF24_REG_RF_SETUP, &rf_setup);

    if (status != NRF24_OK)
        return status;

    // update power level
    rf_setup = (rf_setup & 0x09) | (power << NRF24_BIT_RF_PWR_SHIFT);
    return nrf24_hal_write_reg(radio, NRF24_REG_RF_SETUP, &rf_setup, 1);
}

nrf24_status_t nrf24_flush_rx(const nrf24_handle_t * radio)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    uint8_t tmp;
    return nrf24_hal_write_cmd(radio, NRF24_FLUSH_RX, &tmp);
}

nrf24_status_t nrf24_flush_tx(const nrf24_handle_t * radio)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    uint8_t tmp;
    return nrf24_hal_write_cmd(radio, NRF24_FLUSH_TX, &tmp);
}

nrf24_status_t nrf24_read_status(const nrf24_handle_t * radio, uint8_t * status)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    return nrf24_hal_write_cmd(radio, NRF24_NOP, status);
}

nrf24_status_t nrf24_clear_status(const nrf24_handle_t * radio)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    uint8_t clear_status = NRF24_BIT_RX_DR | NRF24_BIT_TX_DS | NRF24_BIT_MAX_RT;
    return nrf24_hal_write_reg(radio, NRF24_REG_STATUS, &clear_status, 1);
}

nrf24_status_t nrf24_read_fifo_status(const nrf24_handle_t * radio, uint8_t * status)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    return nrf24_hal_read_reg(radio, NRF24_REG_FIFO_STATUS, status);
}

nrf24_status_t nrf24_carrier_detect(const nrf24_handle_t * radio, uint8_t * carrier)
{
    if (radio->hspi == NULL)
        return NRF24_NULL_POINTER;

    return nrf24_hal_read_reg(radio, NRF24_REG_CD, carrier);
}

/**************************************** Helper Functions *******************************************/
static void nrf24_standby(const nrf24_handle_t * radio)
{
    nrf24_hal_disable(radio);

    // do not modify other bits in config register
    uint8_t config;
    nrf24_hal_read_reg(radio, NRF24_REG_CONFIG, &config);

    config |= NRF24_BIT_PWR_UP | NRF24_BIT_EN_CRC;
    config &= ~NRF24_BIT_PRIM_RX;
    nrf24_hal_write_reg(radio, NRF24_REG_CONFIG, &config, 1);
}

static void nrf24_standby_rx(const nrf24_handle_t * radio)
{
    nrf24_hal_disable(radio);

    // do not modify other bits in config register
    uint8_t config;
    nrf24_hal_read_reg(radio, NRF24_REG_CONFIG, &config);

    config |= NRF24_BIT_PWR_UP | NRF24_BIT_EN_CRC | NRF24_BIT_PRIM_RX;
    nrf24_hal_write_reg(radio, NRF24_REG_CONFIG, &config, 1);
}

static void nrf24_activate_features(const nrf24_handle_t * radio)
{
    uint8_t activate = 0x73;
    nrf24_hal_write_cmd_data(radio, NRF24_ACTIVATE, &activate, 1);
}

static void nrf24_set_address_width(const nrf24_handle_t * radio, const nrf24_addr_width_t width)
{
    nrf24_hal_write_reg(radio, NRF24_REG_SETUP_AW, &width, 1);
}

static void nrf24_write_rx_addr(const nrf24_handle_t * radio, const nrf24_pipe_t * pipe)
{
    // pipes 0 and 1 require 5 byte addresses
    if (pipe->num == NRF24_PIPE0 || pipe->num == NRF24_PIPE1)
    {
        uint8_t rx_addr[5] = {
            (uint8_t)pipe->address, (uint8_t)(pipe->address >> 8), (uint8_t)(pipe->address >> 16), (uint8_t)(pipe->address >> 24), (uint8_t)(pipe->address >> 32)
        };

        nrf24_hal_write_reg(radio, NRF24_REG_RX_ADDR_P0 + pipe->num, rx_addr, sizeof(rx_addr));
    }
    else
    {
        // pipes 2-5 share 4 MSBytes with pipe 1
        // only write LSB
        uint8_t rx_addr = (uint8_t)pipe->address;
        nrf24_hal_write_reg(radio, NRF24_REG_RX_ADDR_P0 + pipe->num, &rx_addr, sizeof(rx_addr));
    }
}

static void nrf24_read_rx_pipe_num(const nrf24_handle_t * radio, nrf24_pipe_num_t * pipe)
{
    // only used after data has been received...
    uint8_t status;
    nrf24_read_status(radio, &status);

    *pipe = (status >> 1) & 0x07; // extract pipe number from status register
}

static void nrf24_read_payload_width(const nrf24_handle_t * radio, const nrf24_pipe_num_t pipe, uint8_t * width)
{
    // read payload width
    // if dynamic pipe, must get from register
    // if static, use preset valuess
    if (radio->_pipes[pipe].payload_type == NRF24_PAYLOAD_DYNAMIC)
        nrf24_hal_read_reg(radio, NRF24_R_RX_PL_WID, width);
    else
        *width = radio->_pipes[pipe].payload_length;
}

static void nrf24_set_irq(const nrf24_handle_t * radio)
{
    // do not modify other bits in NRF24_REG_CONFIG register
    uint8_t config;
    nrf24_hal_read_reg(radio, NRF24_REG_CONFIG, &config);
    config |= NRF24_BIT_MASK_RX_DR | NRF24_BIT_MASK_TX_DS | NRF24_BIT_MASK_MAX_RT; // disable all interrupts to start

    // enable selected interrupts
    if (radio->nrf24_irqs_t.rx_dr_en)
    {
        config &= ~NRF24_BIT_MASK_RX_DR;
    }
    if (radio->nrf24_irqs_t.tx_ds_en)
    {
        config &= ~NRF24_BIT_MASK_TX_DS;
    }
    if (radio->nrf24_irqs_t.max_rt_en)
    {
        config &= ~NRF24_BIT_MASK_MAX_RT;
    }

    nrf24_hal_write_reg(radio, NRF24_REG_CONFIG, &config, 1);
}
/*****************************************************************************************************/