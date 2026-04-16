
#include "nrf24.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#elifdef STM32F446xx
#include "stm32f4xx_hal.h"
#endif

static void rf_disable(GPIO_TypeDef * ce_gpio, uint16_t ce_pin);
static void rf_enable(GPIO_TypeDef * ce_gpio, uint16_t ce_pin);

void rf_init(RadioParams radio)
{
    rf_disable(radio.ce_gpio, radio.ce_pin);

    // power on reset, delay >10.3ms
    radio.delay_ms(20);

    // PWR_UP = 1, wait >1.5ms for start-up
    rf_disable(radio.ce_gpio, radio.ce_pin);
    uint8_t pwr_up[2] = {CONFIG | W_REGISTER, PWR_UP};
    HAL_SPI_Transmit(radio.hspi, pwr_up, 2, 1000);
    radio.delay_ms(2);
    
    // default configurations
    uint8_t max_pwr[2] = {RF_SETUP | W_REGISTER, RF_PWR_0DBM};          // max power mode
    uint8_t setup_aw[2] = {SETUP_AW | W_REGISTER, AW_5};                // 5 address bytes --> defined from TX_ADDR
    HAL_SPI_Transmit(radio.hspi, max_pwr, 2, 1000);
    HAL_SPI_Transmit(radio.hspi, setup_aw, 2, 1000);

    // TX address is address of receiving node
    uint8_t tx_addr[6] = {
        TX_ADDR | W_REGISTER,
        (uint8_t) radio.node_addr, (uint8_t) (radio.node_addr >> 8), (uint8_t) (radio.node_addr >> 16), (uint8_t) (radio.node_addr >> 24), (uint8_t) (radio.node_addr >> 32)};
    HAL_SPI_Transmit(radio.hspi, tx_addr, 6, 1000);
    
    // enable date pipe 0 by default
    uint8_t pipe0[2] = {EN_RXADDR | W_REGISTER, ERX_P0};
    HAL_SPI_Transmit(radio.hspi, pipe0, 2, 1000);
    
    // set receive address data pipe 0
    uint8_t rx_addr[6] = {
        RX_ADDR_P0 | W_REGISTER,
        (uint8_t) radio.this_addr, (uint8_t) (radio.this_addr >> 8), (uint8_t) (radio.this_addr >> 16), (uint8_t) (radio.this_addr >> 24), (uint8_t) (radio.this_addr >> 32)};
    HAL_SPI_Transmit(radio.hspi, rx_addr, 6, 1000);

    // radio module should be in standby mode after exiting
}

void rf_send(RadioParams radio, uint8_t * bytes, uint8_t len)
{
    if (len > MAX_PKT_SIZE)
        return;

    // ensure mode is in standby before placing in tx mode
    rf_disable(radio.ce_gpio, radio.ce_pin);
    uint8_t pwr_up[2] = {CONFIG | W_REGISTER, PWR_UP};
    HAL_SPI_Transmit(radio.hspi, pwr_up, 2, 1000);

    // write with NO_ACK
    uint8_t w_no_ack[2] = {ACTIVATE | W_REGISTER, 0x73};
    HAL_SPI_Transmit(radio.hspi, w_no_ack, 2, 1000);
    
    // write data to TX buffer
    HAL_SPI_Transmit(radio.hspi, bytes, len, 1000);

    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    rf_enable(radio.ce_gpio, radio.ce_pin);
    radio.delay_ms(1);
    rf_disable(radio.ce_gpio, radio.ce_pin);

    // wait for packet to be transmitted
    uint8_t status;
    do {

        uint8_t txd[2] = {STATUS, 0};
        uint8_t rxd[2];
        HAL_SPI_TransmitReceive(radio.hspi, txd, rxd, 2, 1000);
        status = rxd[1];

    } while (!(status & TX_DS));

    // clear data sent flag
    uint8_t clear_ds[2] = {STATUS | W_REGISTER, TX_DS};
    HAL_SPI_Transmit(radio.hspi, clear_ds, 2, 1000);
}

uint8_t rf_listen(RadioParams radio, uint32_t timeout)
{
    // receiver --> PRIM_RX = 1, CE = 1
    rf_disable(radio.ce_gpio, radio.ce_pin);
    uint8_t en_rx[2] = {CONFIG | W_REGISTER, PWR_UP | PRIM_RX};
    HAL_SPI_Transmit(radio.hspi, en_rx, 2, 1000);
    rf_enable(radio.ce_gpio, radio.ce_pin);
    radio.delay_ms(1);           // wait >130us for RX settling

    // either poll continuously or wait for IRQ and timeout after certain period of time
    uint32_t ticks = 0;
    while (ticks < timeout)
    {
        uint8_t status;
        uint8_t txd[2] = {STATUS, 0};
        uint8_t rxd[2];
        HAL_SPI_TransmitReceive(radio.hspi, txd, rxd, 2, 1000);
        status = rxd[1];

        if (status & RX_DR)     // if data has been received, exit
        {
            uint8_t clear_dr[2] = {STATUS | W_REGISTER, RX_DR};
            HAL_SPI_Transmit(radio.hspi, clear_dr, 2, 1000);
            return 1;
        }

        ticks++;
    }

    // timeout
    return 0;
}

void rf_receive(RadioParams radio, uint8_t * packet, uint8_t * len)
{
    // get length of data packet from data pipe
    uint8_t get_len[2] = {RX_PW_P0, 0};
    uint8_t rxd[2];
    HAL_SPI_TransmitReceive(radio.hspi, get_len, rxd, 2, 1000);
    *len = rxd[1];

    // read packet
    if (*len != 0)
    {
        uint8_t rx_payload = R_RX_PAYLOAD;
        HAL_SPI_Transmit(radio.hspi, &rx_payload, 1, 1000);
        HAL_SPI_Receive(radio.hspi, packet, *len, 1000);
    }
}

static void rf_disable(GPIO_TypeDef * ce_gpio, uint16_t ce_pin)
{
    HAL_GPIO_WritePin(ce_gpio, ce_pin, GPIO_PIN_RESET);
}

static void rf_enable(GPIO_TypeDef * ce_gpio, uint16_t ce_pin)
{
    HAL_GPIO_WritePin(ce_gpio, ce_pin, GPIO_PIN_SET);
}