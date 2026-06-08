#ifndef OLED_HELPER_H_
#define OLED_HELPER_H_

#include <stdint.h>

typedef enum : int32_t
{
    PAGE_1 = 0,         // RX, TX battery levels
    PAGE_2,             // joystick locations
    PAGE_3              // PID values
} OLEDPages;

void oled_show_battery(uint8_t tx_batt, uint8_t rx_batt);
void oled_show_joysticks(uint8_t throttle, uint8_t yaw, uint8_t pitch, uint8_t roll);
void oled_show_pid();

#endif