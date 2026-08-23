#ifndef OLED_HELPER_H_
#define OLED_HELPER_H_

#include <stdint.h>
#include "ssd1306.h"
#include "app.h"
#include "rf_structs.h"

typedef enum : int32_t
{
    PAGE_1 = 0, // RX, TX battery levels
    PAGE_2,     // joystick locations
    PAGE_3      // PID values
} oled_pages_t;

typedef enum : int32_t
{
    ACTIVE_TUNE_NONE = 0,
    ACTIVE_TUNE_KP,
    ACTIVE_TUNE_KI,
    ACTIVE_TUNE_KD
} oled_active_tune_param_t;

typedef struct
{
    oled_pages_t page;
    uint8_t tx_batt;
    uint8_t rx_batt;
    quad_arm_status_t mode;
    joystick_t * joysticks;
    float kp;
    float ki;
    float kd;
    oled_active_tune_param_t active_tune;
} oled_params_t;

void oled_update(ssd1306_handle_t * holed, oled_params_t * params);
void oled_show_page(ssd1306_handle_t * holed, oled_pages_t page);
void oled_show_lock(ssd1306_handle_t * holed, uint8_t mode);
void oled_show_battery(ssd1306_handle_t * holed, uint8_t tx_batt, uint8_t rx_batt);
void oled_show_joysticks(ssd1306_handle_t * holed, joystick_t * joysticks);
void oled_show_pid(ssd1306_handle_t * holed, float kp, float ki, float kd, oled_active_tune_param_t active_tune);

#endif