/**
 * @file pca9685_servo.cpp
 * @brief Implementation of PCA9685-driven servo with soft speed limit
 */

#include "pca9685_servo.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm_driver = Adafruit_PWMServoDriver(0x40);
bool SlowServo::_pwm_driver_intialized = false;

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
  _end_duty_cycle = _start_duty_cycle = (min_duty_cycle + max_duty_cycle)/2;
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
    digitalWrite(SERVO_ENABLE_PIN, LOW);

    Serial.printf("PWM driver is initialized now !\n");
  }
}

/**
 * @fn update
 * @brief Compute current interpolated servo position and output
 */
void SlowServo::update()
{
  uint32_t dc_on = map(_pin, 0, 15, 0, 4095);
  uint32_t dc_off = (dc_on + getDutyCycle()) % 4095;
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
    float alpha = (1.0f * now_ms - _start_ms) / (_end_ms - _start_ms);
    alpha = constrain(alpha, 0.0f, 1.0f);
    return alpha * _end_duty_cycle + (1 - alpha) * _start_duty_cycle;
    //map(1.0f * now_ms, _start_ms, _end_ms, _start_duty_cycle, _end_duty_cycle);
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
  uint32_t move_duration_ms = abs(deg - getPos()) * _ms_per_deg;
  _start_ms = millis();
  _start_duty_cycle = getDutyCycle();
  _end_duty_cycle = dutyCycleFromDeg(deg);
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

int32_t SlowServo::degFromDutyCycle(uint32_t duty_cycle)
{
  return map(duty_cycle, _min_duty_cycle, _max_duty_cycle, _min_deg, _max_deg);
}

uint32_t SlowServo::dutyCycleFromDeg(int32_t deg) 
{
  return map(deg, _min_deg, _max_deg, _min_duty_cycle, _max_duty_cycle);
}


/*
  void begin();
  void update();
  void moveTo(int deg, float speedDps = 9999);
  void blockingMoveTo(int deg, float speedDps = 9999);

  static void outputEnable();
  static void outputDisable();
  static int dutyCycleFromDeg(int deg);
  static int degFromDutyCycle(int duty_cycle);

private:
  uint8_t _pin;
  float _speed_dps;
  const int _min_deg;
  const int _max_deg;
  const int _min_duty_cycle;
  const int _max_duty_cycle;
};


const int32_t min_deg = -90;
const int32_t max_deg = 90;

const uint32_t min_duty_cycle = 4096 * 5 / 100;
const uint32_t max_duty_cycle = 4096 * 10 / 100;


void servo_begin()
{
  Wire.begin();
  Wire.setClock(100 * 1000);
  pwm.begin();
  pwm.setPWMFreq(50); 

  for (int i = 0; i < 15; ++i)
  {
    servo_move(i, 0);
  }
}

uint32_t duty_cycle_from_degree(int32_t deg)
{
  return map(constrain(deg, min_deg, max_deg), 
      min_deg, max_deg, min_duty_cycle, max_duty_cycle);
}

int32_t degree_from_duty_cycle(uint32_t duty_cycle)
{
  return map(constrain(duty_cycle, min_duty_cycle, max_duty_cycle),
      min_duty_cycle, max_duty_cycle, min_deg, max_deg);
}

void servo_move(int pin, int angle)
{
  uint32_t t_on = 0; //4096 * pin / 16;
  uint32_t t_off = (t_on + duty_cycle_from_degree(angle)) & 0x0FFF;
  pwm.setPWM(pin, t_on, t_off);
}


void servo_output_enable()
{
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, LOW);
}

void servo_output_disable()
{
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, HIGH);
}
*/