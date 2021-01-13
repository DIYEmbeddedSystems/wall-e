/**
 * @file pca9685_servo.h
 * @brief Interface of PCA9685-driven servo with soft speed limit
 * @version 0.1
 * @author Etienne HAMELIN
 * @date 2020-11-24
 * 
 * 
 * Note on servo calibration: 
 * 
 * TowerPro MG90S spec: https://components101.com/sites/default/files/component_datasheet/MG90S-Datasheet.pdf
 *    180° range, 600 °/s, 1ms-2ms pulses.
 *    min_deg = -90, max_deg = 90, min_duty_cycle = 
 * 
 * PWM duty-cycle range:
 * "Standard" range is 1 to 2 ms
 *    1-2ms over 20ms period = 5% to 10% duty-cycle => 5% * 4096 ~ 205; 10% * 4096 = 410
 * Extended PWM range: 0.6 to 2.4ms
 *    3% - 12% => 123 to 491
 * 
 */
#ifndef PCA9685_SERVO
#define PCA9685_SERVO

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdint.h>

#ifndef SERVO_ENABLE_PIN
#define SERVO_ENABLE_PIN D3
#endif


extern Adafruit_PWMServoDriver pwm_driver;

class SlowServo
{
public:
  enum status_e 
  {
    MOVING, STOPPED
  };

  SlowServo(uint8_t pin, 
      float max_speed_dps = 300.0, 
      int32_t min_deg = -90, int32_t max_deg = 90,
      uint32_t min_duty_cycle = 205, uint32_t max_duty_cycle = 410);
  void begin();
  void update();
  void moveTo(int deg);
  void blockingMoveTo(int deg);
  int getStatus();

  uint32_t getDutyCycle();
  int32_t getPos();

//  static void outputEnable();
//  static void outputDisable();

  uint32_t dutyCycleFromDeg(int32_t deg);
  int32_t degFromDutyCycle(uint32_t duty_cycle);

private:
  uint8_t _pin;
  float _ms_per_deg;
  const int32_t _min_deg;
  const int32_t _max_deg;
  const uint32_t _min_duty_cycle;
  const uint32_t _max_duty_cycle;

  uint32_t _start_duty_cycle;
  uint32_t _end_duty_cycle;
  uint32_t _start_ms;
  uint32_t _end_ms;

  static bool _pwm_driver_intialized;

};

#endif