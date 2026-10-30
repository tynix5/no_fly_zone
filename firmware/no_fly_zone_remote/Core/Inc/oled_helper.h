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

    joysticks_t * joysticks;
    float throttle_norm, pitch_norm, roll_norm, yaw_norm;

    float kp, ki, kd;

    active_tune_param_t tuning_param;
    bool is_calibrating;
} oled_params_t;

/* Send OLED buffer over I2C to SSD1306 */
/* [input] holed - OLED structure handle */
/* [input] params - OLED parameters handle */
void oled_update(ssd1306_handle_t * holed, oled_params_t * params);

/* Print current OLED page */
/* [input] holed - OLED structure handle */
/* [input] page - current OLED page being printed */
void oled_show_page(ssd1306_handle_t * holed, oled_pages_t page);

/* Show ARM status of remote */
/* [input] holed - OLED structure handle */
/* [input] arm_state - ARM state */
void oled_show_lock(ssd1306_handle_t * holed, uint8_t arm_state);

/* Show battery levels of remote and quadcopter */
/* [input] holed - OLED structure handle */
/* [input] tx_batt - remote battery level */
/* [input] rx_batt - quadcopter battery level */
void oled_show_battery(ssd1306_handle_t * holed, uint8_t tx_batt, uint8_t rx_batt);

/* Show joystick positions and calibration status */
/* [input] holed - OLED structure handle */
/* [input] throttle_norm - normalized throttle input [-1, 1] */
/* [input] pitch_norm - normalized pitch input [-1, 1] */
/* [input] roll_norm - normalized roll input [-1, 1] */
/* [input] yaw_norm - normalized yaw input [-1, 1] */
/* [input] is_calibrating - current calibration status */
void oled_show_joysticks(ssd1306_handle_t * holed, float throttle_norm, float pitch_norm, float roll_norm, float yaw_norm, bool is_calibrating);

/* Show battery levels of remote and quadcopter */
/* [input] holed - OLED structure handle */
/* [input] kp - proportional constant */
/* [input] ki - integral constant */
/* [input] kd - derivative constant */
/* [input] tuning_param - current parameter being tuned */
void oled_show_pid(ssd1306_handle_t * holed, float kp, float ki, float kd, active_tune_param_t tuning_param);

#endif