/**
 * @file trigger.h
 * @brief Interface of a periodic trigger utility
 * @date 2021-01-15
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 */


#include <Arduino.h>

bool periodicTrigger(uint32_t *pTriggerNextMs, const uint32_t triggerPeriodMs, bool retrigger);