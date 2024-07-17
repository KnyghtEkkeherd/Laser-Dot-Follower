#ifndef _IMAGEPROC_
#define _IMAGE_PROC_
#include "stm32f1xx_hal.h"

void getMiddleDistance(void);

float max(float a, float b, float c);

float min(float a, float b, float c);

uint8_t rgb_to_hsv(float r, float g, float b);

#endif