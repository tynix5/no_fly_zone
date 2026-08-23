#include "oled_helper.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ssd1306.h"

static const uint8_t battery[24 * 10] = { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1,
                                          1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
                                          0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
                                          0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                          1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

void oled_update(ssd1306_handle_t * holed, oled_params_t * params)
{
    ssd1306_clear(holed);
    oled_show_lock(holed, params->mode);
    oled_show_page(holed, params->page);

    if (params->page == PAGE_1)
    {
        oled_show_battery(holed, params->tx_batt, params->rx_batt);
    }
    else if (params->page == PAGE_2)
    {
        oled_show_joysticks(holed, params->joysticks);
    }
    else if (params->page == PAGE_3)
    {
        oled_show_pid(holed, params->kp, params->ki, params->kd, params->active_tune);
    }

    ssd1306_update(holed);
}

void oled_show_page(ssd1306_handle_t * holed, oled_pages_t page)
{
    if (page == PAGE_1)
    {
        ssd1306_draw_filled_circle(holed, 48, 58, 4);
        ssd1306_draw_filled_circle(holed, 64, 58, 2);
        ssd1306_draw_filled_circle(holed, 80, 58, 2);
    }
    else if (page == PAGE_2)
    {
        ssd1306_draw_filled_circle(holed, 48, 58, 2);
        ssd1306_draw_filled_circle(holed, 64, 58, 4);
        ssd1306_draw_filled_circle(holed, 80, 58, 2);
    }
    else if (page == PAGE_3)
    {
        ssd1306_draw_filled_circle(holed, 48, 58, 2);
        ssd1306_draw_filled_circle(holed, 64, 58, 2);
        ssd1306_draw_filled_circle(holed, 80, 58, 4);
    }
}

void oled_show_lock(ssd1306_handle_t * holed, uint8_t mode)
{
    // disarmed
    if (mode == 0)
    {
        ssd1306_write_str(holed, 40, 2, "DISARM");
    }
    else
    {
        ssd1306_write_str(holed, 52, 2, "ARM");
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
    ssd1306_draw_filled_rect(holed,
                             holed->res_x - 1 - batt_body_width - batt_body_start_x,
                             batt_body_start_y + batt_body_height - rx_batt_fullness,
                             batt_body_width,
                             rx_batt_fullness);
    snprintf(percent, sizeof(percent), "%3d%%", rx_batt);
    ssd1306_write_str(holed, 77, 20, percent);
}

void oled_show_joysticks(ssd1306_handle_t * holed, joystick_t * joysticks)
{

    uint8_t throttle_new = (joysticks->throttle / 4096.0) * 100;
    uint8_t pitch_new = (joysticks->pitch / 4096.0) * 100;
    uint8_t roll_new = (joysticks->roll / 4096.0) * 100;
    uint8_t yaw_new = (joysticks->yaw / 4096.0) * 100;

    // show current position of left joystick
    const uint8_t joystick_r = 18;
    ssd1306_draw_circle(holed, 25, 32, 22);

    int8_t left_x = (int8_t)round((double)(yaw_new - 50) / 100.0 * joystick_r * 2);
    int8_t left_y = (int8_t)round((double)(throttle_new - 50) / 100.0 * joystick_r * 2);
    ssd1306_draw_filled_circle(holed, left_x + 25, left_y + 32, 3);

    // right joystick
    ssd1306_draw_circle(holed, 100, 32, 22);

    int8_t right_x = (int8_t)round((double)(roll_new - 50) / 100.0 * joystick_r * 2);
    int8_t right_y = (int8_t)round((double)(pitch_new - 50) / 100.0 * joystick_r * 2);
    ssd1306_draw_filled_circle(holed, right_x + 100, right_y + 32, 3);
}

void oled_show_pid(ssd1306_handle_t * holed, float kp, float ki, float kd, oled_active_tune_param_t active_tune)
{

    // find a better way to do this
    if (active_tune == ACTIVE_TUNE_KP)
        ssd1306_draw_filled_circle(holed, 10, 20, 3);
    else if (active_tune == ACTIVE_TUNE_KI)
        ssd1306_draw_filled_circle(holed, 10, 30, 3);
    else if (active_tune == ACTIVE_TUNE_KD)
        ssd1306_draw_filled_circle(holed, 10, 40, 3);

    ssd1306_write_str(holed, 20, 20, "Kp: ");
    ssd1306_write_str(holed, 20, 30, "Ki: ");
    ssd1306_write_str(holed, 20, 40, "Kd: ");

    int kp_ten = (int)kp / 10.0;
    int kp_one = (int)kp % 10;
    int kp_tenth = (int)(kp * 10.0) % 10;
    int kp_hundredth = (int)round(kp * 100.0) % 10;

    char kp_buff[6] = { kp_ten + '0', kp_one + '0', '.', kp_tenth + '0', kp_hundredth + '0', '\0' };

    int ki_ten = (int)ki / 10;
    int ki_one = (int)ki % 10;
    int ki_tenth = (int)(ki * 10.0) % 10;
    int ki_hundredth = (int)round(ki * 100.0) % 10;

    char ki_buff[6] = { ki_ten + '0', ki_one + '0', '.', ki_tenth + '0', ki_hundredth + '0', '\0' };

    int kd_ten = (int)kd / 10;
    int kd_one = (int)kd % 10;
    int kd_tenth = (int)(kd * 10.0) % 10;
    int kd_hundredth = (int)round(kd * 100.0) % 10;

    char kd_buff[6] = { kd_ten + '0', kd_one + '0', '.', kd_tenth + '0', kd_hundredth + '0', '\0' };
    ssd1306_write_str(holed, 52, 20, kp_buff);
    ssd1306_write_str(holed, 52, 30, ki_buff);
    ssd1306_write_str(holed, 52, 40, kd_buff);
}