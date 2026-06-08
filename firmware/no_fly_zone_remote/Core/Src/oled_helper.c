#include "oled_helper.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ssd1306.h"

static const uint8_t battery[24 * 10] = {   0, 0, 0, 1, 1, 1, 1, 0, 0, 0,
                                            0, 0, 0, 1, 0, 0, 1, 0, 0, 0,
                                            0, 0, 0, 1, 0, 0, 1, 0, 0, 0,
                                            1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            1, 1, 1, 1, 1, 1, 1, 1, 1, 1    };

void oled_show_battery(uint8_t tx_batt, uint8_t rx_batt)
{
    // draw remote battery and quadcopter battery
    ssd1306_clear();

    // page 1
    ssd1306_write_str(48, 52, "1 2 3");
    ssd1306_draw_line(46, 62, 54, 62);

    // draw battery outlines for remote and quadcopter
    if (tx_batt > 100)
        tx_batt = 100;
    if (rx_batt > 100)
        rx_batt = 100;

    const uint8_t batt_body_start_x = 5;
    const uint8_t batt_body_start_y = 10;
    const uint8_t batt_body_width = 10;
    const uint8_t batt_body_height = 20;

    ssd1306_draw_rect(7, 7, 5, 3);         // battery cap
    ssd1306_draw_rect(batt_body_start_x, batt_body_start_y, batt_body_width, batt_body_height);     // battery body
    ssd1306_write_str(2, 35, "TX");

    ssd1306_draw_rect(RES_X - 1 - batt_body_width - batt_body_start_x + 2, 7, 5, 3);      // battery cap
    ssd1306_draw_rect(RES_X - 1 - batt_body_width - batt_body_start_x, batt_body_start_y, batt_body_width, batt_body_height);    // battery body
    ssd1306_write_str(110, 35, "RX");


    uint8_t tx_batt_fullness = (uint8_t) (((float) tx_batt / 100.0) * batt_body_height);
    uint8_t rx_batt_fullness = (uint8_t) (((float) rx_batt / 100.0) * batt_body_height);

    // draw "full" battery portions for remote
    ssd1306_draw_filled_rect(batt_body_start_x, batt_body_start_y + batt_body_height - tx_batt_fullness, batt_body_width, tx_batt_fullness);
    // write battery percentage to right
    char percent[5];
    snprintf(percent, sizeof(percent), "%d%%", tx_batt);
    ssd1306_write_str(20, 20, percent);
    
    // draw "full" battery portions for quadcopter
    ssd1306_draw_filled_rect(RES_X - 1 - batt_body_width - batt_body_start_x, batt_body_start_y + batt_body_height - rx_batt_fullness, batt_body_width, rx_batt_fullness);
    snprintf(percent, sizeof(percent), "%3d%%", rx_batt);
    ssd1306_write_str(77, 20, percent);

    ssd1306_update();
}

void oled_show_joysticks(uint8_t throttle, uint8_t yaw, uint8_t pitch, uint8_t roll)
{
    ssd1306_clear();

    // page 2
    ssd1306_write_str(48, 52, "1 2 3");
    ssd1306_draw_line(62, 62, 70, 62);

    // show current position of left joystick
    const uint8_t joystick_r = 18;
    ssd1306_draw_circle(25, 32, 22);

    int8_t left_x = (int8_t) round((double) (yaw - 50) / 100.0 * joystick_r * 2);
    int8_t left_y = (int8_t) round((double) (throttle - 50) / 100.0 * joystick_r * 2);
    ssd1306_draw_filled_circle(left_x + 25, left_y + 32, 3);
    
    // right joystick
    ssd1306_draw_circle(100, 32, 22);
    
    int8_t right_x = (int8_t) round((double) (roll - 50) / 100.0 * joystick_r * 2);
    int8_t right_y = (int8_t) round((double) (pitch - 50) / 100.0 * joystick_r * 2);
    ssd1306_draw_filled_circle(right_x + 106, right_y + 32, 3);

    ssd1306_update();
}

void oled_show_pid()
{
    ssd1306_clear();

    // page 3
    ssd1306_write_str(48, 52, "1 2 3");
    ssd1306_draw_line(78, 62, 86, 62);

    ssd1306_update();
}