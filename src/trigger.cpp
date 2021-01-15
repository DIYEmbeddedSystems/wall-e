/**
 * @file trigger.cpp
 * @brief Implementaion of a periodic trigger utility
 * @date 2021-01-15
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 */

#include "trigger.h"

/**
 * @brief Called frequently, this function returns true once per period
 * @param[in,out] pTriggerNextMs points to a variable holding the next trigger date (in milliseconds)
 * @param triggerPeriodMs the period (in milliseconds) of the trigger
 * @param retrigger if you miss a period (or several periods), do you want to get triggered once per missed period, or juste once?
 * @return true once per period, false otherwise
 * 
 * Usage example:
 * uint32_t nextMs = 10000; // do not start before t = 10s
 * 
 * void loop() {
 *   if (periodicTrigger(&nextMs, 1000, false)) {
 *     Serial.println("Tick!");
 *   }
 * }
 * 
 */
bool periodicTrigger(uint32_t *pTriggerNextMs, const uint32_t triggerPeriodMs, bool retrigger)
{
  bool res = false;
  uint32_t nowMs = millis();

  while ((int32_t)(nowMs - *pTriggerNextMs) >= 0)
  {
    *pTriggerNextMs += triggerPeriodMs;
    res = true;
    if (!retrigger)
    {
      break;
    }
  }
  return res;
}
