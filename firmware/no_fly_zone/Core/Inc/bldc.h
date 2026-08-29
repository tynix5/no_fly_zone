#ifndef BLDC_H_
#define BLDC_H_

#include "dshot.h"

// from a top down view of the quad
//      front_left    front_right
//              \      /
//               \ _ /
//                | |
//                |_|
//               /   \
//              /     \     
//      back_left     back_right

void bldc_init();
void bldc_mix();
void bldc_send();
void bldc_clamp();

#endif