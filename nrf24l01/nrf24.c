
#include "nrf24.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elif defined(STM32F446xx)
#include "stm32f4xx_hal.h"
#endif

/**************************************** SPI Wrapper Functions **************************************/
static void rf_disable(RadioParams * radio);
static void rf_enable(RadioParams * radio);
static uint8_t rf_read_status(RadioParams * radio);
static void rf_spi_transmit(RadioParams * radio, uint8_t * data, uint8_t len);
static void rf_spi_transmit_receive(RadioParams * radio, uint8_t * src, uint8_t * dest, uint8_t len);
/*****************************************************************************************************/

// change this init completely
void rf_init(RadioParams * radio)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
    rf_disable(radio);

    // power on reset, delay >10.3ms
    radio->delay_ms(20);

    // PWR_UP = 1, wait >1.5ms for start-up
    rf_disable(radio);
    uint8_t pwr_up[2] = {CONFIG | W_REGISTER, PWR_UP | EN_CRC};
    rf_spi_transmit(radio, pwr_up, 2);
    radio->delay_ms(2);
    
    // default configurations
    uint8_t max_pwr[2] = {RF_SETUP | W_REGISTER, RF_PWR_0DBM};          // max power mode
    rf_spi_transmit(radio, max_pwr, 2);

    uint8_t setup_aw[2] = {SETUP_AW | W_REGISTER, AW_5};                // 5 address bytes --> defined from TX_ADDR
    rf_spi_transmit(radio, setup_aw, 2);

    // TX address is address of receiving node
    uint8_t tx_addr[6] = {
        TX_ADDR | W_REGISTER,
        (uint8_t) radio->node_addr, (uint8_t) (radio->node_addr >> 8), (uint8_t) (radio->node_addr >> 16), (uint8_t) (radio->node_addr >> 24), (uint8_t) (radio->node_addr >> 32)};
    rf_spi_transmit(radio, tx_addr, 6);
    
    // enable date pipe 0 by default
    uint8_t pipe0[2] = {EN_RXADDR | W_REGISTER, ERX_P0};
    rf_spi_transmit(radio, pipe0, 2);
    
    // set receive address data pipe 0
    uint8_t rx_addr[6] = {
        RX_ADDR_P0 | W_REGISTER,
        (uint8_t) radio->this_addr, (uint8_t) (radio->this_addr >> 8), (uint8_t) (radio->this_addr >> 16), (uint8_t) (radio->this_addr >> 24), (uint8_t) (radio->this_addr >> 32)};
    rf_spi_transmit(radio, rx_addr, 6);


    // enable writes to any special function register
    uint8_t activate[2] = {ACTIVATE, 0x73};
    rf_spi_transmit(radio, activate, 2);

    // enable dynamic payload length for pipe 0
    uint8_t en_dp[2] = {DYNPD | W_REGISTER, DPL_P0};
    rf_spi_transmit(radio, en_dp, 2);

    // enable auto-acknowledgement for pipe 0(required for dynamic payload length)
    uint8_t en_aa[2] = {EN_AA | W_REGISTER, ENAA_P0};
    rf_spi_transmit(radio, en_aa, 2);

    // enable dynamic ACK, dynamic payload length, payload with ack
    uint8_t feature[2] = {FEATURE | W_REGISTER, EN_DPL | EN_ACK_PAY | EN_DYN_ACK};
    rf_spi_transmit(radio, feature, 2);

    // radio module should be in standby mode after exiting
}

void rf_send_w_no_ack(RadioParams * radio, uint8_t * packet, uint8_t len)
{
    if (len > MAX_PKT_SIZE)
        return;

    // ensure mode is in standby before placing in tx mode
    rf_disable(radio);
    uint8_t pwr_up[2] = {CONFIG | W_REGISTER, PWR_UP | EN_CRC};
    rf_spi_transmit(radio, pwr_up, 2);

    // write data to TX buffer
    uint8_t w_no_ack = W_TX_PAYLOAD_NOACK;
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, &w_no_ack, 1, HAL_TIMEOUT);
    HAL_SPI_Transmit(radio->hspi, packet, len, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    rf_enable(radio);
    radio->delay_ms(1);
    rf_disable(radio);

    // wait for packet to be transmitted
    while (!(rf_read_status(radio) & TX_DS));

    // clear data sent flag
    uint8_t clear_ds[2] = {STATUS | W_REGISTER, TX_DS};
    rf_spi_transmit(radio, clear_ds, 2);
}

void rf_send_w_ack(RadioParams * radio, uint8_t * packet, uint8_t plen, uint8_t * ack, uint8_t * alen)
{
    if (plen > MAX_PKT_SIZE)
        return;

    // ensure mode is in standby before placing in tx mode
    rf_disable(radio);
    uint8_t pwr_up[2] = {CONFIG | W_REGISTER, PWR_UP | EN_CRC};
    rf_spi_transmit(radio, pwr_up, 2);

    // write data to TX buffer
    uint8_t w_ack = W_TX_PAYLOAD;
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, &w_ack, 1, HAL_TIMEOUT);
    HAL_SPI_Transmit(radio->hspi, packet, plen, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    rf_enable(radio);
    radio->delay_ms(1);
    rf_disable(radio);

    // wait for packet to be transmitted and acknowledgement received
    while (!(rf_read_status(radio) & TX_DS));

    // clear data sent flag
    uint8_t clear_ds[2] = {STATUS | W_REGISTER, TX_DS};
    rf_spi_transmit(radio, clear_ds, 2);

    // read acknowledgement width
    uint8_t r_rx_pl[2] = {R_RX_PL_WID, 0};
    uint8_t ack_width[2];
    rf_spi_transmit_receive(radio, r_rx_pl, ack_width, 2);
    *alen = ack_width[1];

    // read acknowledgement
    uint8_t r_ack = R_RX_PAYLOAD;
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, &r_ack, 1, HAL_TIMEOUT);
    HAL_SPI_Receive(radio->hspi, ack, *alen, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}

uint8_t rf_listen_no_ack(RadioParams * radio, uint32_t timeout)
{
    // receiver --> PRIM_RX = 1, CE = 1
    rf_disable(radio);

    uint8_t en_rx[2] = {CONFIG | W_REGISTER, PWR_UP | PRIM_RX | EN_CRC};
    rf_spi_transmit(radio, en_rx, 2);

    rf_enable(radio);
    radio->delay_ms(1);           // wait >130us for RX settling

    // either poll continuously or wait for IRQ and timeout after certain period of time
    uint32_t ticks = 0;
    while (ticks < timeout)
    {
        if (rf_read_status(radio) & RX_DR)              // if data has been received, exit
        {
            rf_disable(radio);
            uint8_t clear_dr[2] = {STATUS | W_REGISTER, RX_DR};
            rf_spi_transmit(radio, clear_dr, 2);
            return 1;
        }

        ticks++;
    }

    // timeout
    return 0;
}

uint8_t rf_listen_ack(RadioParams * radio, uint8_t * response, uint8_t rlen, uint32_t timeout)
{
    // receiver --> PRIM_RX = 1, CE = 1
    rf_disable(radio);

    uint8_t en_rx[2] = {CONFIG | W_REGISTER, PWR_UP | PRIM_RX | EN_CRC};
    rf_spi_transmit(radio, en_rx, 2);

    uint8_t w_ack_payload = W_ACK_PAYLOAD;
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, &w_ack_payload, 1, HAL_TIMEOUT);
    HAL_SPI_Transmit(radio->hspi, response, rlen, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    rf_enable(radio);
    radio->delay_ms(1);           // wait >130us for RX settling

    // either poll continuously or wait for IRQ and timeout after certain period of time
    uint32_t ticks = 0;
    while (ticks < timeout)
    {
        if (rf_read_status(radio) & RX_DR)              // if data has been received, exit
        {
            rf_disable(radio);
            uint8_t clear_dr[2] = {STATUS | W_REGISTER, RX_DR};
            rf_spi_transmit(radio, clear_dr, 2);
            return 1;
        }

        ticks++;
    }

    // timeout
    return 0;
}


void rf_receive(RadioParams * radio, uint8_t * packet, uint8_t * len)
{
    uint8_t rx_pl[2] = {R_RX_PL_WID, 0};
    uint8_t width[2];
    rf_spi_transmit_receive(radio, rx_pl, width, 2);
    *len = width[1];

    // read packet
    if (*len != 0)
    {
        uint8_t rx_payload = R_RX_PAYLOAD;

        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(radio->hspi, &rx_payload, 1, HAL_TIMEOUT);
        HAL_SPI_Receive(radio->hspi, packet, *len, HAL_TIMEOUT);
        HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
    }
}

/**************************************** SPI Wrapper Functions **************************************/
static void rf_disable(RadioParams * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_RESET);
}

static void rf_enable(RadioParams * radio)
{
    HAL_GPIO_WritePin(radio->ce_gpio, radio->ce_pin, GPIO_PIN_SET);
}

static uint8_t rf_read_status(RadioParams * radio)
{
    uint8_t nop = NOP;
    uint8_t status;

    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio->hspi, &nop, &status, 1, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);

    return status;
}

static void rf_spi_transmit(RadioParams * radio, uint8_t * data, uint8_t len)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->hspi, data, len, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}

static void rf_spi_transmit_receive(RadioParams * radio, uint8_t * src, uint8_t * dest, uint8_t len)
{
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(radio->hspi, src, dest, len, HAL_TIMEOUT);
    HAL_GPIO_WritePin(radio->cs_gpio, radio->cs_pin, GPIO_PIN_SET);
}
/*****************************************************************************************************/