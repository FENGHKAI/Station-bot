/*
*file hc_sr04.h
*brief HC-SR04 超声波测距驱动声明（PA3 Trig, PA2 Echo）
*/

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "sys.h"

void HC_SR04_Init(void);
float HC_SR04_GetDistance(void);

#endif