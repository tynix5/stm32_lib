
#include "nrf24.h"
#include "generic_types.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

/**************************************** Helper Functions *******************************************/
static void rf_set_irq(nrf_handle_t * radio);
/*****************************************************************************************************/

/**************************************** SPI Wrapper Functions **************************************/
static void rf_disable(nrf_handle_t * radio);
static void rf_enable(nrf_handle_t * radio);
static void rf_spi_transmit(nrf_handle_t * radio, uint8_t * data, uint8_t len);
static void rf_spi_transmit_receive(nrf_handle_t * radio, uint8_t * src, uint8_t * dest, uint8_t len);
/*****************************************************************************************************/

nrf_status_t rf_init(nrf_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
    rf_disable(radio);

    // power on reset, delay >10.3ms
    radio->delay_ms(20);

    // PWR_UP = 1, wait >1.5ms for start-up
    rf_disable(radio);
    uint8_t pwr_up[2] = { NRF_REG_CONFIG | NRF_W_REGISTER, NRF_BIT_PWR_UP | NRF_BIT_EN_CRC };
    rf_spi_transmit(radio, pwr_up, 2);
    radio->delay_ms(2);

    // default configurations
    rf_set_channel(radio, radio->freq_ch);
    rf_set_power_level(radio, radio->power_level);
    rf_set_data_rate(radio, radio->data_rate);
    rf_set_retransmit(radio, NRF_RETRY_DELAY_1000, NRF_RETRY_CNT_3);

    uint8_t setup_aw[2] = { NRF_REG_SETUP_AW | NRF_W_REGISTER, NRF_BIT_AW_5 }; // 5 address bytes --> defined from TX_ADDR
    rf_spi_transmit(radio, setup_aw, 2);

    // TX address is address of receiving node
    uint8_t tx_addr[6] = { NRF_REG_TX_ADDR | NRF_W_REGISTER,  (uint8_t)radio->node_addr,         (uint8_t)(radio->node_addr >> 8),
                           (uint8_t)(radio->node_addr >> 16), (uint8_t)(radio->node_addr >> 24), (uint8_t)(radio->node_addr >> 32) };
    rf_spi_transmit(radio, tx_addr, 6);

    // enable date pipe 0 by default
    uint8_t pipe0[2] = { NRF_REG_EN_RXADDR | NRF_W_REGISTER, NRF_BIT_ERX_P0 };
    rf_spi_transmit(radio, pipe0, 2);

    // set receive address data pipe 0
    uint8_t rx_addr[6] = { NRF_REG_RX_ADDR_P0 | NRF_W_REGISTER, (uint8_t)radio->this_addr,         (uint8_t)(radio->this_addr >> 8),
                           (uint8_t)(radio->this_addr >> 16),   (uint8_t)(radio->this_addr >> 24), (uint8_t)(radio->this_addr >> 32) };
    rf_spi_transmit(radio, rx_addr, 6);

    // enable writes to any special function register
    uint8_t activate[2] = { NRF_ACTIVATE, 0x73 };
    rf_spi_transmit(radio, activate, 2);

    uint8_t curr_features = 0;

    if (radio->payload_type == NRF_PAYLOAD_DYNAMIC)
    {
        // enable dynamic payload length for pipe 0
        uint8_t en_dp[2] = { NRF_REG_DYNPD | NRF_W_REGISTER, NRF_BIT_DPL_P0 };
        rf_spi_transmit(radio, en_dp, 2);

        // enable auto-acknowledgement for pipe 0 (required for dynamic payload length)
        uint8_t en_aa[2] = { NRF_REG_EN_AA | NRF_W_REGISTER, NRF_BIT_ENAA_P0 };
        rf_spi_transmit(radio, en_aa, 2);

        curr_features |= NRF_BIT_EN_DPL;
    }
    else
    {
        // disable dynamic payload lengths
        uint8_t en_dp[2] = { NRF_REG_DYNPD | NRF_W_REGISTER, 0 };
        rf_spi_transmit(radio, en_dp, 2);

        // set static payload lengths
        uint8_t payload_lengths[2] = { NRF_REG_RX_PW_P0 | NRF_W_REGISTER, radio->payload_length };
        rf_spi_transmit(radio, payload_lengths, 2);
    }

    if (radio->ack == FEAT_ENABLE)
    {
        // enable auto-acknowledgement for pipe 0 (required for dynamic payload length)
        uint8_t en_aa[2] = { NRF_REG_EN_AA | NRF_W_REGISTER, NRF_BIT_ENAA_P0 };
        rf_spi_transmit(radio, en_aa, 2);

        // enable acknowledgement payloads
        curr_features |= NRF_BIT_EN_ACK_PAY;
    }
    else
    {
        if (radio->payload_type == NRF_PAYLOAD_STATIC)
        {
            // disable auto-acknowledgements when using static payload and disabled acks
            uint8_t disable_aa[2] = { NRF_REG_EN_AA | NRF_W_REGISTER, 0 };
            rf_spi_transmit(radio, disable_aa, 2);
        }

        curr_features |= NRF_BIT_EN_DYN_ACK;
    }

    // using ACK is determined by using W_TX_PAYLOAD_NOACK or W_TX_PAYLOAD
    uint8_t feature[2] = { NRF_REG_FEATURE | NRF_W_REGISTER, curr_features };
    rf_spi_transmit(radio, feature, 2);

    rf_set_irq(radio);

    rf_flush_rx(radio);
    rf_flush_tx(radio);
    // radio module should be in standby mode after exiting

    return RF_SUCCESS;
}

nrf_status_t rf_send(nrf_handle_t * radio, uint8_t * packet, uint8_t plen, uint8_t * ack, uint8_t * alen)
{
    if (plen > MAX_PKT_SIZE)
        return RF_INVALID_CONFIG;

    // ensure mode is in standby before placing in tx mode
    rf_disable(radio);
    uint8_t pwr_up[2] = { NRF_REG_CONFIG | NRF_W_REGISTER, NRF_BIT_PWR_UP | NRF_BIT_EN_CRC };
    rf_spi_transmit(radio, pwr_up, 2);

    // write data to TX buffer
    uint8_t write_tx;
    if (radio->ack == FEAT_ENABLE)
        write_tx = NRF_W_TX_PAYLOAD;
    else
        write_tx = NRF_W_TX_PAYLOAD_NOACK;
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, &write_tx, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(radio->hspi, packet, plen, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    // modify this to use lower on time for transmitter
    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    rf_enable(radio);
    // radio->delay_ms(1);
    radio->delay_us(20);
    rf_disable(radio);

    // wait for packet to be transmitted and acknowledgement received
    while (1)
    {
        uint8_t status = rf_read_status(radio);

        // if maximum retries have happened, flush FIFO and clear flag
        if (status & NRF_BIT_MAX_RT && radio->ack == FEAT_ENABLE)
        {
            uint8_t reset_max_rt[2] = { NRF_REG_STATUS | NRF_W_REGISTER, NRF_BIT_MAX_RT };
            rf_spi_transmit(radio, reset_max_rt, 2);
            rf_flush_tx(radio);
            return RF_MAX_RETRIES;
        }
        else if (status & NRF_BIT_TX_DS)
        {
            break;
        }
    }

    // clear data sent flag
    uint8_t clear_ds[2] = { NRF_REG_STATUS | NRF_W_REGISTER, NRF_BIT_TX_DS };
    rf_spi_transmit(radio, clear_ds, 2);

    // only read acknowledgement if enabled
    *alen = 0;
    if (radio->ack == FEAT_ENABLE && rf_read_status(radio) & NRF_BIT_RX_DR)
    {
        // read acknowledgement width
        uint8_t r_rx_pl[2] = { NRF_R_RX_PL_WID, 0 };
        uint8_t ack_width[2];
        rf_spi_transmit_receive(radio, r_rx_pl, ack_width, 2);
        *alen = ack_width[1];

        if (*alen > MAX_PKT_SIZE)
        {
            rf_flush_rx(radio);
            return RF_INVALID_PACKET;
        }
        else if (*alen != 0)
        {
            // read acknowledgement
            uint8_t r_ack = NRF_R_RX_PAYLOAD;
            HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
            HAL_SPI_Transmit(radio->hspi, &r_ack, 1, SPI_TIMEOUT_MS);
            HAL_SPI_Receive(radio->hspi, ack, *alen, SPI_TIMEOUT_MS);
            HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

            // clear data ready flag reading acknowledgement
            uint8_t clear_dr[2] = { NRF_REG_STATUS | NRF_W_REGISTER, NRF_BIT_RX_DR };
            rf_spi_transmit(radio, clear_dr, 2);
        }
    }

    return RF_SUCCESS;
}

nrf_status_t rf_listen(nrf_handle_t * radio, uint32_t timeout)
{
    // receiver --> PRIM_RX = 1, CE = 1
    rf_disable(radio);

    uint8_t en_rx[2] = { NRF_REG_CONFIG | NRF_W_REGISTER, NRF_BIT_PWR_UP | NRF_BIT_PRIM_RX | NRF_BIT_EN_CRC };
    rf_spi_transmit(radio, en_rx, 2);

    rf_enable(radio);
    // radio->delay_ms(1); // wait >130us for RX settling
    radio->delay_us(200);

    // either poll continuously or wait for IRQ and timeout after certain period of time
    uint32_t ticks = 0;
    while (ticks < timeout)
    {
        if (rf_read_status(radio) & NRF_BIT_RX_DR) // if data has been received, exit
        {
            rf_disable(radio);
            return RF_SUCCESS;
        }

        ticks++;
    }

    return RF_TIMEOUT;
}

nrf_status_t rf_listen_it(nrf_handle_t * radio)
{
    // move to receiver mode and then exit (waiting for pin interrupts)
    // receiver --> PRIM_RX = 1, CE = 1
    rf_disable(radio);

    uint8_t en_rx[2] = { NRF_REG_CONFIG | NRF_W_REGISTER, NRF_BIT_PWR_UP | NRF_BIT_PRIM_RX | NRF_BIT_EN_CRC };
    rf_spi_transmit(radio, en_rx, 2);

    rf_enable(radio);

    return RF_SUCCESS;
}

nrf_status_t rf_receive(nrf_handle_t * radio, uint8_t * packet, uint8_t * plen, uint8_t * ack, uint8_t alen)
{
    // get packet width
    if (radio->payload_type == NRF_PAYLOAD_DYNAMIC)
    {
        uint8_t rx_pl[2] = { NRF_R_RX_PL_WID, 0 };
        uint8_t width[2];
        rf_spi_transmit_receive(radio, rx_pl, width, 2);
        *plen = width[1];
    }
    else
    {
        *plen = radio->payload_length;
    }

    // if an invalid packet, discard it and flush FIFO
    if (*plen > MAX_PKT_SIZE)
    {
        rf_flush_rx(radio);
        *plen = 0;
        return RF_INVALID_PACKET;
    }

    // read packet
    if (*plen != 0)
    {
        uint8_t rx_payload = NRF_R_RX_PAYLOAD;

        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(radio->hspi, &rx_payload, 1, SPI_TIMEOUT_MS);
        HAL_SPI_Receive(radio->hspi, packet, *plen, SPI_TIMEOUT_MS);
        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
    }

    // if payload with acks are enabled and attempting to be used...
    if (radio->ack == FEAT_ENABLE && alen > 0)
    {
        // write response payload
        uint8_t w_ack_payload = NRF_W_ACK_PAYLOAD;
        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(radio->hspi, &w_ack_payload, 1, SPI_TIMEOUT_MS);
        HAL_SPI_Transmit(radio->hspi, ack, alen, SPI_TIMEOUT_MS);
        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
    }

    // clear data ready flag after reading packet and loading acks
    uint8_t clear_dr[2] = { NRF_REG_STATUS | NRF_W_REGISTER, NRF_BIT_RX_DR | NRF_BIT_TX_DS | NRF_BIT_MAX_RT };
    rf_spi_transmit(radio, clear_dr, 2);

    return RF_SUCCESS;
}

void rf_set_retransmit(nrf_handle_t * radio, nrf_retry_delay_t delay, nrf_retry_cnt_t count)
{
    uint8_t setup_retr[2] = { NRF_REG_SETUP_RETR | NRF_W_REGISTER, ((delay & 0x0f) << 4) | (count & 0x0f) };
    rf_spi_transmit(radio, setup_retr, 2);
}

void rf_set_channel(nrf_handle_t * radio, uint8_t ch)
{
    uint8_t rf_ch[2] = { NRF_REG_RF_CH | NRF_W_REGISTER, ch & 0x7f };
    rf_spi_transmit(radio, rf_ch, 2);
}

void rf_set_data_rate(nrf_handle_t * radio, nrf_datarate_t rate)
{
    // read current power configurations and keep them
    uint8_t rf_setup[2] = { NRF_REG_RF_SETUP, 0 };
    uint8_t curr_pwr[2];
    rf_spi_transmit_receive(radio, rf_setup, curr_pwr, 2);

    uint8_t data_rate[2] = { NRF_REG_RF_SETUP | NRF_W_REGISTER, (curr_pwr[1] & 0xf7) | (rate << NRF_BIT_RF_DR_SHIFT) };
    rf_spi_transmit(radio, data_rate, 2);
}

void rf_set_power_level(nrf_handle_t * radio, nrf_power_t power)
{
    // read current data rate configurations and keep them
    uint8_t rf_setup[2] = { NRF_REG_RF_SETUP, 0 };
    uint8_t curr_data_rate[2];
    rf_spi_transmit_receive(radio, rf_setup, curr_data_rate, 2);

    uint8_t power_level[2] = { NRF_REG_RF_SETUP | NRF_W_REGISTER, (curr_data_rate[1] & 0xf9) | (power << NRF_BIT_RF_PWR_SHIFT) };
    rf_spi_transmit(radio, power_level, 2);
}

void rf_flush_rx(nrf_handle_t * radio)
{
    uint8_t flush_rx = NRF_FLUSH_RX;
    rf_spi_transmit(radio, &flush_rx, 1);
}

void rf_flush_tx(nrf_handle_t * radio)
{
    uint8_t flush_tx = NRF_FLUSH_TX;
    rf_spi_transmit(radio, &flush_tx, 1);
}

uint8_t rf_read_status(nrf_handle_t * radio)
{
    uint8_t nop = NRF_NOP;
    uint8_t status;

    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio->hspi, &nop, &status, 1, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    return status;
}

void rf_clear_status(nrf_handle_t * radio)
{
    uint8_t cmd[2] = { NRF_REG_STATUS | NRF_W_REGISTER, NRF_BIT_RX_DR | NRF_BIT_TX_DS | NRF_BIT_MAX_RT };

    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, cmd, 2, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}

uint8_t rf_carrier_detect(nrf_handle_t * radio)
{
    uint8_t cmd[2] = { NRF_REG_CD, 0 };
    uint8_t cd[2];

    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio->hspi, cmd, cd, 2, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    return cd[1];
}

/**************************************** Helper Functions *******************************************/
static void rf_set_irq(nrf_handle_t * radio)
{
    // do not modify other bits in NRF_REG_CONFIG register
    uint8_t cmd[2] = { NRF_REG_CONFIG, 0 };
    uint8_t data[2];

    rf_spi_transmit_receive(radio, cmd, data, 2);

    uint8_t config = data[1];
    config |= NRF_BIT_MASK_RX_DR | NRF_BIT_MASK_TX_DS | NRF_BIT_MASK_MAX_RT; // disable all interrupts to start

    // enable selected interrupts
    if (radio->nrf_irqs_t.rx_dr == FEAT_ENABLE)
    {
        config &= ~NRF_BIT_MASK_RX_DR;
    }
    if (radio->nrf_irqs_t.tx_ds == FEAT_ENABLE)
    {
        config &= ~NRF_BIT_MASK_TX_DS;
    }
    if (radio->nrf_irqs_t.max_rt == FEAT_ENABLE)
    {
        config &= ~NRF_BIT_MASK_MAX_RT;
    }

    cmd[0] = NRF_REG_CONFIG | NRF_W_REGISTER;
    cmd[1] = config;
    rf_spi_transmit(radio, cmd, 2);
}
/*****************************************************************************************************/

/**************************************** SPI Wrapper Functions **************************************/
static void rf_disable(nrf_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_RESET);
}

static void rf_enable(nrf_handle_t * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_SET);
}

static void rf_spi_transmit(nrf_handle_t * radio, uint8_t * data, uint8_t len)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, data, len, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}

static void rf_spi_transmit_receive(nrf_handle_t * radio, uint8_t * src, uint8_t * dest, uint8_t len)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio->hspi, src, dest, len, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}
/*****************************************************************************************************/