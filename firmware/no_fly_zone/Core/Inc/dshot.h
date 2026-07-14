#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/************************** DShot Packet Specifications ***********************/
#define DSHOT_FRAME_W                       16

#define DSHOT_THROTTLE_W                    11
#define DSHOT_TEL_REQ_W                     1
#define DSHOT_CRC_W                         4

#define DSHOT_THROTTLE_SHIFT                5
#define DSHOT_TEL_REQ_SHIFT                 4
#define DSHOT_CRC_SHIFT                     0
/******************************************************************************/

/************************* DShot Protocol Specifications **********************/
// DShot '1' bit high times
#define DSHOT_150_T1H_US                    5.00
#define DSHOT_300_T1H_US                    2.50
#define DSHOT_600_T1H_US                    1.25
#define DSHOT_1200_T1H_US                   0.625

// DShot '0' bit high times
#define DSHOT_150_T0H_US                    2.50
#define DSHOT_300_T0H_US                    1.25
#define DSHOT_600_T0H_US                    0.625
#define DSHOT_1200_T0H_US                   0.313

// Dshot bit period
#define DSHOT_150_BIT_US                    6.67
#define DSHOT_300_BIT_US                    3.33
#define DSHOT_600_BIT_US                    1.67
#define DSHOT_1200_BIT_US                   0.83
/******************************************************************************/

typedef enum : uint8_t {

    // kbit/s
    DSHOT_BR_150 = 0,
    DSHOT_BR_300,
    DSHOT_BR_600,
    DSHOT_BR_1200

} DShotBitrate;

typedef struct {

    DShotBitrate bitrate;

} DShotParams;

void dshot_init();
void dshot_encode(uint16_t throttle, uint8_t tel_req, uint32_t * pkt);

#endif