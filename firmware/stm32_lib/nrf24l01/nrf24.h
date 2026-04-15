
#ifndef NRF24_H_
#define NRF24_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define RADIO_MODE_TX           1
#define RADIO_MODE_RX           0

#define MAX_RF_CONN             5

#define MAX_PKT_SIZE            32

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

// RF_SETUP
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
/************************************************************/

typedef struct {

    uint8_t (*spi_transfer_byte)(uint8_t, uint8_t);                        // (address, data)
    void (*spi_transfer)(uint8_t, uint8_t *, uint8_t *, uint8_t);          // (address, data[], data_rcv[], len)
    void (*delay_us)(uint32_t);
    void (*delay_ms)(uint32_t);

    void (*ce_low)(void);
    void (*ce_high)(void);

    uint64_t this_addr;
    uint64_t node_addr;
} RadioParams;

typedef struct {

    uint16_t batt_lvl;
} PacketParams;

void nrf_init(RadioParams radio);
void nrf_send(RadioParams radio, uint8_t * bytes, uint8_t len);
uint8_t nrf_listen(RadioParams radio, uint32_t timeout);
void nrf_receive(RadioParams radio, uint8_t * packet, uint8_t * len);
void nrf_set_power_level(RadioParams radio, uint8_t power);

#endif