#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "ssd1306.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"

#define RF_TX_ADDR                 0xE7E7E7E7
#define RF_RX_ADDR                 0xE7E7E7E7

#define NUM_ADC_CONV               5

/*
typedef struct {

    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t roll;
} PacketParams;

static void rf_serialize_packet(PacketParams * packet);
*/

uint16_t adc_conversions[NUM_ADC_CONV];
uint16_t batt_lvl;
uint8_t joysticks[4];

volatile uint8_t adc_ready = 0;

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


/************************************************************************************ */
/****************************** Move these to other file **************************** */
/************************************************************************************ */
static void oled_update();
/************************************************************************************ */
/************************************************************************************ */
/************************************************************************************ */

void app_init(ADC_HandleTypeDef * hadc, DMA_HandleTypeDef * hdma, I2C_HandleTypeDef * hi2c, SPI_HandleTypeDef * hspi, TIM_HandleTypeDef * htim, PCD_HandleTypeDef * husb)
{
    tx.hspi = hspi;
    rf_init(&tx);

    ssd1306_init(hi2c);

    HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
    HAL_Delay(1000);


    // enable ADC with DMA transfers
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_conversions, NUM_ADC_CONV);

    // start ADC conversions
    // HAL_TIM_Base_Start(htim);
    ssd1306_clear();
    ssd1306_update();
}

void app(void)
{
    while (1)
    {

        // ssd1306_write_str(0, 0, "Test");
        ssd1306_draw_rect(108, 10, 10, 20);
        ssd1306_draw_filled_rect(10, 10, 10, 20);
        ssd1306_draw_line(3, 63, 122, 57);
        ssd1306_write_str(20, 40, "87%");
        ssd1306_update();   
        HAL_Delay(2000);
        ssd1306_clear();
        ssd1306_update();
        HAL_Delay(2000);

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
        
        /*
        if (adc_ready == 1)
        {

            // move joystick conversions
            for (uint8_t i = 0; i < 4; i++)
            {
                joysticks[i] = (uint8_t) (adc_conversions[i] >> 4);
            }

            uint8_t packet[3] = {0x5e, 0x54, 0x21};
            uint8_t response[2];
            uint8_t rlen;
            rf_send(&tx, packet, 3, response, &rlen); 

            if (rlen == 2 && response[0] == 0x05 && response[1] == 0x02)
                HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);

            batt_lvl = adc_conversions[4];
            adc_ready = 0;
        }    
            */
    }
}

// implement callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    adc_ready = 1;
}

static void oled_update()
{
    // draw remote battery and quadcopter battery
    ssd1306_clear();

    // calculate battery levels as a % (test)
    uint8_t remote_lvl = 50;
    uint8_t quad_lvl = 22;

    const uint8_t batt_start_x = 5;
    const uint8_t batt_start_y = 10;
    const uint8_t batt_width = 10;
    const uint8_t batt_height = 20;

    uint8_t remote_height = (uint8_t) ((float) remote_lvl / 100.0) * batt_height;
    uint8_t quad_height = (uint8_t) ((float) quad_lvl / 100.0) * batt_height;

    // remote
    ssd1306_draw_rect(batt_start_x, batt_start_y, batt_width, batt_height);
    ssd1306_write_str(5, 35, "TX");
    // ssd1306_draw_filled_rect(batt_start_x, batt_start_y + batt_start_y - )

    // quadcopter
    ssd1306_draw_rect(RES_X - 1 - batt_width - batt_start_x, batt_start_y, batt_width, batt_height);
    ssd1306_write_str(113, 35, "RX");
}