#include "oled_helper.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "rf_structs.h"
#include "ssd1306.h"

// battery bitmap
static const uint8_t battery[24 * 10] = { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

void oled_update(ssd1306_handle_t * holed, oled_params_t * params)
{
    ssd1306_clear(holed);
    oled_show_lock(holed, params->arm_state);
    oled_show_page(holed, params->page);

    switch (params->page)
    {
    case PAGE_BATT_LVL:
        oled_show_battery(holed, params->tx_batt, params->rx_batt);
        break;

    case PAGE_JOYSTICK:
        oled_show_joysticks(holed, params->throttle_norm, params->pitch_norm, params->roll_norm, params->yaw_norm, params->is_calibrating);
        break;

    case PAGE_TUNE_PID:
        oled_show_pid(holed, params->kp, params->ki, params->kd, params->tuning_param);
        break;
    }

    ssd1306_update(holed);
}

void oled_show_page(ssd1306_handle_t * holed, oled_pages_t page)
{
    switch (page)
    {
    case PAGE_BATT_LVL:
        ssd1306_draw_filled_circle(holed, 48, 58, 4);
        ssd1306_draw_filled_circle(holed, 64, 58, 2);
        ssd1306_draw_filled_circle(holed, 80, 58, 2);
        break;

    case PAGE_JOYSTICK:
        ssd1306_draw_filled_circle(holed, 48, 58, 2);
        ssd1306_draw_filled_circle(holed, 64, 58, 4);
        ssd1306_draw_filled_circle(holed, 80, 58, 2);
        break;

    case PAGE_TUNE_PID:
        ssd1306_draw_filled_circle(holed, 48, 58, 2);
        ssd1306_draw_filled_circle(holed, 64, 58, 2);
        ssd1306_draw_filled_circle(holed, 80, 58, 4);
        break;
    }
}

void oled_show_lock(ssd1306_handle_t * holed, uint8_t arm_state)
{
    switch (arm_state)
    {
    case ARM_STATE_DISARMED:
        ssd1306_write_str(holed, 40, 2, "DISARM");
        break;

    case ARM_STATE_ARMED:
        ssd1306_write_str(holed, 52, 2, "ARM");
        break;

    case ARM_STATE_FAILSAFE:
    default:
        ssd1306_write_str(holed, 48, 2, "FAIL");
    }
}

// REPLACE ALL PAGE NUMBERS WITH APPLE DOTS WITH LARGER DOT INDICATING CURRENT PAGE
void oled_show_battery(ssd1306_handle_t * holed, uint8_t tx_batt, uint8_t rx_batt)
{
    // draw battery outlines for remote and quadcopter
    if (tx_batt > 100)
        tx_batt = 100;
    if (rx_batt > 100)
        rx_batt = 100;

    const uint8_t batt_body_start_x = 5;
    const uint8_t batt_body_start_y = 10;
    const uint8_t batt_body_width = 10;
    const uint8_t batt_body_height = 20;

    ssd1306_draw_bitmap(holed, 5, 7, battery, 10, 24);
    ssd1306_write_str(holed, 2, 35, "TX");

    ssd1306_draw_bitmap(holed, 112, 7, battery, 10, 24);
    ssd1306_write_str(holed, 110, 35, "RX");

    uint8_t tx_batt_fullness = (uint8_t)(((float)tx_batt / 100.0) * batt_body_height);
    uint8_t rx_batt_fullness = (uint8_t)(((float)rx_batt / 100.0) * batt_body_height);

    // draw "full" battery portions for remote
    ssd1306_draw_filled_rect(holed, batt_body_start_x, batt_body_start_y + batt_body_height - tx_batt_fullness, batt_body_width, tx_batt_fullness);
    // write battery percentage to right
    char percent[5];
    snprintf(percent, sizeof(percent), "%d%%", tx_batt);
    ssd1306_write_str(holed, 20, 20, percent);

    // draw "full" battery portions for quadcopter
    ssd1306_draw_filled_rect(
        holed, holed->res_x - 1 - batt_body_width - batt_body_start_x, batt_body_start_y + batt_body_height - rx_batt_fullness, batt_body_width, rx_batt_fullness);
    snprintf(percent, sizeof(percent), "%3d%%", rx_batt);
    ssd1306_write_str(holed, 77, 20, percent);
}

void oled_show_joysticks(ssd1306_handle_t * holed, float throttle_norm, float pitch_norm, float roll_norm, float yaw_norm, bool is_calibrating)
{
    const uint8_t joystick_radius = 18;

    // show current position of left joystick
    const int8_t left_joystick_center_x = 25;
    const int8_t left_joystick_center_y = 32;
    ssd1306_draw_circle(holed, left_joystick_center_x, left_joystick_center_y, 22); // draw joystick perimeter

    // draw where joystick is currently located
    uint8_t left_x = (uint8_t)((int8_t)(yaw_norm * (float)joystick_radius) + left_joystick_center_x);
    uint8_t left_y = (uint8_t)((int8_t)(-throttle_norm * (float)joystick_radius) + left_joystick_center_y); // inverted y because it increases as it goes down
    ssd1306_draw_filled_circle(holed, left_x, left_y, 3);

    // right joystick
    const int8_t right_joystick_center_x = 100;
    const int8_t right_joystick_center_y = 32;
    ssd1306_draw_circle(holed, right_joystick_center_x, right_joystick_center_y, 22);

    uint8_t right_x = (uint8_t)((int8_t)(roll_norm * (float)joystick_radius) + right_joystick_center_x);
    uint8_t right_y = (uint8_t)((int8_t)(-pitch_norm * (float)joystick_radius) + right_joystick_center_y);
    ssd1306_draw_filled_circle(holed, right_x, right_y, 3);

    if (is_calibrating)
        ssd1306_write_str(holed, 52, 28, "CAL");
}

void oled_show_pid(ssd1306_handle_t * holed, float kp, float ki, float kd, active_tune_param_t tuning_param)
{
    // indicate parameter being actively tuned
    if (tuning_param == ACTIVE_TUNE_KP)
        ssd1306_draw_filled_circle(holed, 10, 20, 3);
    else if (tuning_param == ACTIVE_TUNE_KI)
        ssd1306_draw_filled_circle(holed, 10, 30, 3);
    else if (tuning_param == ACTIVE_TUNE_KD)
        ssd1306_draw_filled_circle(holed, 10, 40, 3);

    // convert from floats to integers able to displayed on OLED
    int kp_rounded = (int)roundf(kp * 100.0f);
    int kp_ten = kp_rounded / 1000;
    int kp_one = kp_rounded / 100 % 10;
    int kp_tenth = kp_rounded / 10 % 10;
    int kp_hundredth = kp_rounded % 10;
    char kp_buff[6] = { kp_ten + '0', kp_one + '0', '.', kp_tenth + '0', kp_hundredth + '0', '\0' };
    ssd1306_write_str(holed, 20, 20, "Kp: ");
    ssd1306_write_str(holed, 52, 20, kp_buff);

    int ki_rounded = (int)roundf(ki * 100.0f);
    int ki_ten = ki_rounded / 1000;
    int ki_one = ki_rounded / 100 % 10;
    int ki_tenth = ki_rounded / 10 % 10;
    int ki_hundredth = ki_rounded % 10;
    char ki_buff[6] = { ki_ten + '0', ki_one + '0', '.', ki_tenth + '0', ki_hundredth + '0', '\0' };
    ssd1306_write_str(holed, 20, 30, "Ki: ");
    ssd1306_write_str(holed, 52, 30, ki_buff);

    int kd_rounded = (int)roundf(kd * 100.0f);
    int kd_ten = kd_rounded / 1000;
    int kd_one = kd_rounded / 100 % 10;
    int kd_tenth = kd_rounded / 10 % 10;
    int kd_hundredth = kd_rounded % 10;
    char kd_buff[6] = { kd_ten + '0', kd_one + '0', '.', kd_tenth + '0', kd_hundredth + '0', '\0' };
    ssd1306_write_str(holed, 20, 40, "Kd: ");
    ssd1306_write_str(holed, 52, 40, kd_buff);
}