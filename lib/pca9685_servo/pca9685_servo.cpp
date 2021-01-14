/**
 * @file pca9685_servo.cpp
 * @brief Implementation of PCA9685-driven servo with soft speed limit
 * 
 * This library provides smooth movements for servos behind a PCA9685 controller.
 * The PCA9685 is a 16-channel PWM controller on an I2C bus, found for instance on Adafruit's "16-Channel 12-bit 
 * PWM/Servo shield".
 
 * Each servo's angular speed is an instance-specific constant (set in ° per second). When instructed to 
 * moveTo(position_in_degrees), this library computes parameters, and when update() is called frequently,it uses 
 * linear interpolation to compute the servo's expected position at current time, and actuate accordingly. 
 */

#include "pca9685_servo.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm_driver = Adafruit_PWMServoDriver(0x40);   /*!< high-level driver for PCA9685 */
bool SlowServo::_pwm_driver_intialized = false;                       /*!< whether PCA9685 is already initialized */

/**
 * @fn SlowServo
 * @brief Constructor
 */
SlowServo::SlowServo(uint8_t pin, float max_speed_dps, 
      int32_t min_deg, int32_t max_deg,
      uint32_t min_duty_cycle, uint32_t max_duty_cycle)
    : _pin(pin), _min_deg(min_deg), _max_deg(max_deg), 
      _min_duty_cycle(min_duty_cycle), _max_duty_cycle(max_duty_cycle)
{
  // convert speed in °/s ~~> ms/°
  _ms_per_deg = 1000.0 / constrain(max_speed_dps, 1.0, 600.);
  _end_duty_cycle = _start_duty_cycle = _last_duty_cycle = (min_duty_cycle + max_duty_cycle)/2;
  _end_ms = _start_ms = millis();
}

/**
 * @fn begin
 * @brief Starts the PWM driver, enable servo output
 */
void SlowServo::begin()
{
  if (!_pwm_driver_intialized)
  {
    _pwm_driver_intialized = true;

    Serial.printf("Setting up PWM driver\n");

    pwm_driver.begin();
    pwm_driver.setPWMFreq(50);

    Serial.printf("Enabling PWM output\n");
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    

    Serial.printf("PWM driver is initialized now !\n");
  }
}

/**
 * @fn update
 * @brief Compute current interpolated servo position and output
 */
void SlowServo::update()
{
  _last_duty_cycle = getDutyCycle();
  // servo pulses are out of phase, so that (hopefully) all servos don't cause a current inrush at the same time 
  uint32_t dc_on = map(_pin, 0, 15, 0, 4095); // offset at pulse start
  uint32_t dc_off = (dc_on + _last_duty_cycle) % 4095; // offset + duration (duty cycle)
  pwm_driver.setPWM(_pin, dc_on, dc_off);
}

/**
 * @fn getStatus
 * @brief Get servo status
 * @return MOVING or STOPPED
 */
int SlowServo::getStatus()
{
  uint32_t now_ms = millis();
  if ((int32_t)(now_ms - _end_ms) >= 0)
  {
    return SlowServo::STOPPED;
  }
  else 
  {
    return SlowServo::MOVING;
  }
}

/**
 * @fn getDutyCycle
 * @brief Get current interpolated duty cycle value
 * @return duty cycle
 */
uint32_t SlowServo::getDutyCycle()
{
  // Here is the interpolation
  uint32_t now_ms = millis();
  if ((int32_t)(now_ms - _end_ms) >= 0)
  {
    // finished moving
    return _end_duty_cycle;
  }
  else 
  {
    // moving...
    //float alpha = (1.0f * now_ms - _start_ms) / (_end_ms - _start_ms);
    //alpha = constrain(alpha, 0.0f, 1.0f);
    //return alpha * _end_duty_cycle + (1 - alpha) * _start_duty_cycle;
    return map(1.0f * now_ms, _start_ms, _end_ms, _start_duty_cycle, _end_duty_cycle);
  }
}

/**
 * @brief Get current position, in degrees
 * @return position (°)
 */
int32_t SlowServo::getPos()
{
  return degFromDutyCycle(getDutyCycle());
}

/**
 * @brief Move servo towards set position (°), at given speed (°/s)
 * Call update() at regular interval (typically < 20ms) to actually output servo commands
 */
void SlowServo::moveTo(int deg)
{
  uint32_t duty_cycle;
  deg = constrain(deg, _min_deg, _max_deg);
  duty_cycle = dutyCycleFromDeg(deg);
  if (_end_duty_cycle == duty_cycle)
  {
    // commanded position is same as already set: do not change timing
    return;
  }
  uint32_t move_duration_ms = abs(deg - degFromDutyCycle(_last_duty_cycle)) * _ms_per_deg;
  _start_ms = millis();
  _start_duty_cycle = _last_duty_cycle; // start interpolation from last updated position
  _end_duty_cycle = duty_cycle;
  _end_ms = _start_ms + move_duration_ms;
}
 
/**
 * @brief Move servo towards set position (°), at given speed (°/s), return after move is complete
 */
void SlowServo::blockingMoveTo(int deg)
{
  moveTo(deg);
  while (SlowServo::MOVING == getStatus())
  {
    update();
    delay(10);
  }
}

void SlowServo::stop()
{
  _end_ms = millis();
  _end_duty_cycle = _last_duty_cycle;
}

int32_t SlowServo::degFromDutyCycle(uint32_t duty_cycle)
{
  return map(duty_cycle, _min_duty_cycle, _max_duty_cycle, _min_deg, _max_deg);
}

uint32_t SlowServo::dutyCycleFromDeg(int32_t deg) 
{
  return map(deg, _min_deg, _max_deg, _min_duty_cycle, _max_duty_cycle);
}

void SlowServo::outputEnable()
{
  digitalWrite(SERVO_ENABLE_PIN, LOW);
}

void SlowServo::outputDisable()
{
  digitalWrite(SERVO_ENABLE_PIN, HIGH);
}
