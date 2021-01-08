/**
 * @file led.h
 * @brief Interface of the LED control module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */

#ifndef LED_H
#define LED_H

#include <Arduino.h>            /* The Arduino framework */

bool ledBlink(uint32_t timeOnMs, uint32_t timeOffMs);
void ledOn();
void ledOff();
void ledSet(bool state);

#endif // LED_H