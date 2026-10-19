#ifndef OLED_HELPER_H_
#define OLED_HELPER_H_

#include <stdint.h>
#include "ssd1306.h"
#include "rf_structs.h"
#include "joystick_helper.h"
typedef enum : int32_t
{
    PAGE_BATT_LVL = 0, // RX, TX battery levels
    PAGE_JOYSTICK,     // joystick locations
    PAGE_TUNE_PID      // PID values
} oled_pages_t;

typedef struct
{
    oled_pages_t page;

    arm_state_t arm_state;
    uint8_t tx_batt;
    uint8_t rx_batt;

    joystick_t * joysticks;
    float throttle_norm, pitch_norm, roll_norm, yaw_norm;

    float kp, ki, kd;

    active_tune_param_t tuning_param;
    bool is_calibrating;
} oled_params_t;

void oled_update(ssd1306_handle_t * holed, oled_params_t * params);
void oled_show_page(ssd1306_handle_t * holed, oled_pages_t page);
void oled_show_lock(ssd1306_handle_t * holed, uint8_t arm_state);
void oled_show_battery(ssd1306_handle_t * holed, uint8_t tx_batt, uint8_t rx_batt);
void oled_show_joysticks(ssd1306_handle_t * holed, float throttle_norm, float pitch_norm, float roll_norm, float yaw_norm, bool is_calibrating);
void oled_show_pid(ssd1306_handle_t * holed, float kp, float ki, float kd, active_tune_param_t tuning_param);

#endif