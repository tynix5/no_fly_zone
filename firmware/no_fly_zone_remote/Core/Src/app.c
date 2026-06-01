#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "ssd1306.h"
#include "oled_helper.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include <math.h>

#define RF_TX_ADDR                  0xE7E7E7E7
#define RF_RX_ADDR                  0xE7E7E7E7

#define NUM_ADC_CONV                5

#define BAT_MAX_V                   4.2
#define BAT_MIN_V                   2.1

#define ENC_UPDATE_T                250


typedef struct {

    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t roll;
    float p;
    float i;
    float d;

} PacketParams;

typedef struct {

    uint8_t batt_lvl;

} AckParams;

static void serialize_rf_packet(PacketParams * packet);


uint16_t adc_conversions[NUM_ADC_CONV];
uint32_t batt_lvl;
uint8_t joysticks[4];

volatile uint8_t adc_ready = 0;
int32_t enc_ticks = 0, last_enc_ticks = 0;
uint32_t last_enc_update = 0;

OLEDPages oled_page = PAGE_1;
uint8_t oled_updated = 0;

// nRF24L01 parameters, should match receiver params
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

static void shutdown();

void app_init(ADC_HandleTypeDef * hadc, DMA_HandleTypeDef * hdma, I2C_HandleTypeDef * hi2c, SPI_HandleTypeDef * hspi, TIM_HandleTypeDef * htim, TIM_HandleTypeDef * henc, PCD_HandleTypeDef * husb)
{
    tx.hspi = hspi;
    rf_init(&tx);

    ssd1306_init(hi2c);

    // enable ADC with DMA transfers
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_conversions, NUM_ADC_CONV);

    // start ADC conversions
    HAL_TIM_Base_Start(htim);

    // start encoder
    HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
    
    ssd1306_clear();
    ssd1306_update();
}

void app(void)
{
    while (1)
    {
        // flight mode
        // 1. read joysticks
        // 2. send over radio
        // 3. print remote battery and quad battery on oled
        // 4. stream all information over usb

        // tune mode
        // 1. read encoder
        // 2. control menu
        // 3. tune PID values
        // 4. send PID values over radio

        enc_ticks = TIM2->CNT;

        // if encoder has moved (use range to prevent unintended page switches)
        if (abs(enc_ticks - last_enc_ticks) > 1 && HAL_GetTick() - last_enc_update > ENC_UPDATE_T)
        {
            if (enc_ticks > last_enc_ticks)
            {
                // knob turned to right, increase page
                if (++oled_page > PAGE_3)
                    oled_page = PAGE_1;
            }
            else
            {
                // knob turned to left, decrease page
                if (--oled_page < PAGE_1)
                    oled_page = PAGE_3;
            }

            last_enc_ticks = enc_ticks;
            last_enc_update = HAL_GetTick();

            // need to update oled
            oled_updated = 0;
        }


        if (!oled_updated)
        {
            if (oled_page == PAGE_1)
                oled_show_battery(batt_lvl , 0);
            else if (oled_page == PAGE_2)
                oled_show_joysticks((uint8_t) ((joysticks[0] / 256.0) * 100.0), (uint8_t) ((joysticks[1] / 256.0) * 100.0), (uint8_t) ((joysticks[2] / 256.0) * 100.0), (uint8_t) ((joysticks[3] / 256.0) * 100.0));
            else
                oled_show_pid();

            oled_updated = 1;
        }

        
        if (adc_ready)
        {

            // move joystick conversions
            for (uint8_t i = 0; i < 4; i++)
            {
                joysticks[i] = (uint8_t) (adc_conversions[i] >> 4);
            }

            // adc level is between 2.1 and 1.15V
            // convert from [1.15, 2.1] to [0, 100]
            float adc_v = ((float) adc_conversions[4] / 4096.0) * 3.3;
            batt_lvl = (uint8_t) (((adc_v - 1.15) / (2.1 - 1.15)) * 100.0);

            if (batt_lvl < 10)
                shutdown();

            // update oled every time ADC is sampled (when not on PID)
            if (oled_page == PAGE_1 || oled_page == PAGE_2)
                oled_updated = 0;

            uint8_t packet[3] = {0x5e, 0x54, 0x21};
            uint8_t response[2];
            uint8_t rlen;
            
            rf_send(&tx, packet, 3, response, &rlen);
            // if (rf_send(&tx, packet, 3, response, &rlen) != RF_SUCCESS)
                // Error_Handler();

            if (rlen == 2 && response[0] == 0x05 && response[1] == 0x02)
                HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);

            adc_ready = 0;
        }    
    }
}

// implement callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    adc_ready = 1;
}

static void shutdown()
{
    ssd1306_clear();
    ssd1306_update();
    while (1)
    {
    }
}