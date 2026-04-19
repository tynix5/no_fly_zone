#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"

#define RF_TX_ADDR                 0xE7E7E7E7
#define RF_RX_ADDR                 0xE7E7E7E7

/*
typedef struct {

    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t roll;
} PacketParams;

static void rf_serialize_packet(PacketParams * packet);
*/

static RadioParams tx = {
    
    .this_addr = RF_TX_ADDR,
    .node_addr = RF_RX_ADDR,
    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = HAL_Delay,
    .power_level = RF_PWR_0DBM,
    .data_rate = RF_DR_2MBPS,
    .freq_ch = 2,
    .payload_type = DYNAMIC_PAYLOAD,
    .ack = ENABLE_ACK
};

void app_init(SPI_HandleTypeDef * hspi)
{
    tx.hspi = hspi;
    rf_init(&tx);
}

void app(void)
{
    while (1)
    {
        uint8_t packet[3] = {0x5e, 0x54, 0x21};
        uint8_t response[2];
        uint8_t rlen;
        rf_send(&tx, packet, 3, response, &rlen); 
        
        if (rlen == 2 && response[0] == 0x05 && response[1] == 0x02)
            HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);

        HAL_Delay(250);
    }
}