#include "dshot.h"

static void dshot_calculate_crc(uint16_t throttle, uint16_t * pkt);

void dshot_init()
{

}


void dshot_encode(uint16_t throttle, uint8_t tel_req, uint16_t * pkt)
{
    if (throttle > 2047)
        throttle = 2047;
    else if (throttle < 48 && throttle != 0)
        return;

    for (uint8_t i = 0; i < DSHOT_THROTTLE_W; i++)
    {
        if (!!(throttle & (1 << (DSHOT_THROTTLE_W - i - 1))))
            pkt[i] = 210;
        else
            pkt[i] = 105;
    }

    if (tel_req)
        pkt[11] = 210;
    else
        pkt[11] = 105;

    dshot_calculate_crc(throttle, pkt);
}

static void dshot_calculate_crc(uint16_t throttle, uint16_t * pkt)
{
    // XOR all nibbles together
    uint8_t crc = ((throttle ^ (throttle >> 4)) ^ (throttle >> 8)) & 0x0f;

    for (uint8_t i = 0; i < DSHOT_CRC_W; i++)
    {
        if (crc & (1 << (DSHOT_CRC_W - i - 1)))
            pkt[i + 12] = 210;
        else
            pkt[i + 12] = 105;
    }
}