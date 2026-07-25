/*
*file ultrasonic.h
*brief HC-SR04 超声波测距驱动声明（PA3 Trig, PA2 Echo）
*/

#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "sys.h"
#include "delay.h"

void HC_SR04_Init(void);
float HC_SR04_GetDistance(uint32_t timeout_us);

#endif
