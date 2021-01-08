/**
 * @file led.cpp
 * @brief Implementation of the LED control module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */

#include "led.h"
#include <Arduino.h>            /* The Arduino framework */

/**
 * @brief Switch ON the built-in LED
 */
void ledOn()
{
  ledSet(true);
}

/**
 * @brief Switch ON the built-in LED
 */
void ledOff()
{
  ledSet(false);
}

/**
 * @brief Switch ON/OFF the built-in LED
 * @param state: ON/OFF
 */
void ledSet(bool state)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
}

/**
 * @brief Blink the built-in LED with configurable pattern
 * @param timeOnMs: duration of the ON part (in miliseconds)
 * @param timeOffMs: duration of the OFF part (in milliseconds)
 * 
 * Note: user should call this function frequently enough (typically more than once per timeOnMs or TimeOffMs period) 
 * to actually see the blinking pattern.
 * 
 * Example usage:
 * @code
 * void setup() 
 * {
 *   while (!ready()) 
 *   {
 *      ledBlink(900,100); // flash at 1 Hz frequency, 90% time ON and 10% OFF
 *      delay(10);
 *   }
 * }
 * 
 * void loop() 
 * {
 *    ledBlink(100, 900); // flash at 1 Hz frequency, 10% time ON and 90% OFF
 * }
 * @endcode
 */ 
bool ledBlink(uint32_t timeOnMs, uint32_t timeOffMs)
{
  static uint32_t nextMs = 0;
  static bool state = false;
  while ((int32_t)(millis() - nextMs) >= 0)
  {
    state = !state;
    ledSet(state);
    nextMs += (state) ? timeOnMs : timeOffMs;
  }
  return state;
}