#include "oled_helper.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ssd1306.h"

void oled_show_battery(uint8_t tx_batt, uint8_t rx_batt)
{
    // draw remote battery and quadcopter battery
    ssd1306_clear();

    // page 1
    ssd1306_write_str(48, 52, "1 2 3");
    ssd1306_draw_line(46, 62, 54, 62);

    // calculate battery levels as a % (test)
    uint8_t remote_lvl = tx_batt;
    uint8_t quad_lvl = rx_batt;

    const uint8_t batt_start_x = 5;
    const uint8_t batt_start_y = 10;
    const uint8_t batt_width = 10;
    const uint8_t batt_height = 20;

    uint8_t remote_height = (uint8_t) (((float) remote_lvl / 100.0) * batt_height);
    uint8_t quad_height = (uint8_t) (((float) quad_lvl / 100.0) * batt_height);

    // ssd1306_draw_bitmap(5, 3, battery, 10, 24);

    // remote
    ssd1306_draw_rect(7, 7, 5, 3);      // cap on battery
    ssd1306_draw_rect(batt_start_x, batt_start_y, batt_width, batt_height);
    ssd1306_write_str(2, 35, "TX");
    // draw "full" battery portion
    ssd1306_draw_filled_rect(batt_start_x, batt_start_y + batt_height - remote_height, batt_width, remote_height);
    // write battery percentage to right
    char remote_txt[5];
    snprintf(remote_txt, sizeof(remote_txt), "%d%%", remote_lvl);
    ssd1306_write_str(20, 20, remote_txt);
    
    // quadcopter
    ssd1306_draw_rect(RES_X - 1 - batt_width - batt_start_x + 2, 7, 5, 3);      // cap on battery
    ssd1306_draw_rect(RES_X - 1 - batt_width - batt_start_x, batt_start_y, batt_width, batt_height);
    ssd1306_write_str(110, 35, "RX");
    ssd1306_draw_filled_rect(RES_X - 1 - batt_width - batt_start_x, batt_start_y + batt_height - quad_height, batt_width, quad_height);

    char quad_txt[5];
    snprintf(quad_txt, sizeof(remote_txt), "%3d%%", quad_lvl);
    ssd1306_write_str(77, 20, quad_txt);

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
    // ssd1306_write_ch(left_x + 25, left_y + 32, 'o');
    ssd1306_draw_filled_circle(left_x + 25, left_y + 32, 3);
    
    // right joystick
    ssd1306_draw_circle(100, 32, 22);
    
    int8_t right_x = (int8_t) round((double) (roll - 50) / 100.0 * joystick_r * 2);
    int8_t right_y = (int8_t) round((double) (pitch - 50) / 100.0 * joystick_r * 2);
    ssd1306_draw_filled_circle(right_x + 106, right_y + 32, 3);
    // ssd1306_write_ch(right_x + 106, right_y + 32, 'o');

    ssd1306_update();
}

void oled_show_pid()
{
    ssd1306_clear();

    ssd1306_write_str(48, 52, "1 2 3");
    ssd1306_draw_line(78, 62, 86, 62);

    ssd1306_update();
}