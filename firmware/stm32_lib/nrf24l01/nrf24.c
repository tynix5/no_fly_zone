
#include "nrf24.h"


void nrf_init(RadioParams radio)
{
    // power on reset, delay >10.3ms
    radio.delay_ms(20);

    // PWR_UP = 1, wait >1.5ms for start-up
    radio.ce_low();
    radio.spi_transfer_byte(CONFIG | W_REGISTER, PWR_UP);
    radio.delay_ms(2);

    // default configurations
    radio.spi_transfer_byte(RF_SETUP | W_REGISTER, RF_PWR_0DBM);         // max power mode
    radio.spi_transfer_byte(SETUP_AW | W_REGISTER, AW_5);                // 5 address bytes --> defined from TX_ADDR

    // TX address is address of receiving node
    uint8_t temp[5];
    uint8_t tx_addr[5] = {
        (uint8_t) radio.node_addr, (uint8_t) (radio.node_addr >> 8), (uint8_t) (radio.node_addr >> 16), (uint8_t) (radio.node_addr >> 24), (uint8_t) (radio.node_addr >> 32)};
    radio.spi_transfer(TX_ADDR | W_REGISTER, tx_addr, temp, 5);

    // enable date pipe 0 by default
    radio.spi_transfer_byte(EN_RXADDR | W_REGISTER, ERX_P0);

    // set receive address data pipe 0
     uint8_t rx_addr[5] = {
        (uint8_t) radio.this_addr, (uint8_t) (radio.this_addr >> 8), (uint8_t) (radio.this_addr >> 16), (uint8_t) (radio.this_addr >> 24), (uint8_t) (radio.this_addr >> 32)};
    radio.spi_transfer(RX_ADDR_P0 | W_REGISTER, rx_addr, temp, 5);

    // radio module should be in standby mode after exiting
}

void nrf_send(RadioParams radio, uint8_t * bytes, uint8_t len)
{
    if (len > MAX_PKT_SIZE)
        return;

    // ensure mode is in standby before placing in tx mode
    radio.ce_low();
    radio.spi_transfer_byte(CONFIG | W_REGISTER, PWR_UP);

    // write with NO_ACK
    radio.spi_transfer_byte(ACTIVATE | W_REGISTER, 0x73);

    uint8_t temp[MAX_PKT_SIZE];
    radio.spi_transfer(W_TX_PAYLOAD_NOACK, bytes, temp, len);

    // transmitter --> PRIM_RX = 0, CE = 1 for >10us and <4ms and TX FIFO not empty
    radio.ce_high();
    radio.delay_ms(1);
    radio.ce_low();

    while (!(radio.spi_transfer_byte(STATUS, 0) & TX_DS));      // wait for packet to be sent
    radio.spi_transfer_byte(STATUS | W_REGISTER, TX_DS);        // clear data sent flag
}

uint8_t nrf_listen(RadioParams radio, uint32_t timeout)
{
    // receiver --> PRIM_RX = 1, CE = 1
    radio.ce_low();
    radio.spi_transfer_byte(CONFIG | W_REGISTER, PWR_UP | PRIM_RX);
    radio.ce_high();
    radio.delay_ms(1);           // wait >130us for RX settling

    // either poll continuously or wait for IRQ and timeout after certain period of time
    uint32_t ticks = 0;
    while (ticks < timeout)
    {
        if (radio.spi_transfer_byte(STATUS, 0) & RX_DR)     // if data has been received, exit
        {
            radio.spi_transfer_byte(STATUS | W_REGISTER, RX_DR);    // clear status bit
            return 1;
        }

        ticks++;
    }

    // timeout
    return 0;
}

void nrf_receive(RadioParams radio, uint8_t * packet, uint8_t * len)
{
    // get length of data packet from data pipe
    *len = radio.spi_transfer_byte(RX_PW_P0, 0);

    uint8_t temp[MAX_PKT_SIZE];

    // read packet
    if (*len != 0)
        radio.spi_transfer(R_RX_PAYLOAD, temp, packet, *len);
}