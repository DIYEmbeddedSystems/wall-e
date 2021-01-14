/**
 * @file MotorShield.h
 * @brief Driver interface for WEMOS motor shield
 * @date 2021-01-13
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 */

#ifndef MOTORSHIELD_H
#define MOTORSHIELD_H

#include <Arduino.h>

class Motor
{
public:
  enum 
  {
    motor_a = 0,
    motor_b = 1
  };

  enum 
  {
    motor_brake = 0,
    motor_ccw = 1,
    motor_cw = 2,
    motor_stop = 3,
    motor_stdby = 4
  };

  Motor(uint8_t address, uint8_t index);
  void begin();
  uint8_t setPwmFrequency(uint32_t freq);
  uint8_t setSpeed(int32_t speed);
  uint32_t getSpeed();
  uint8_t standby();

private:
  uint8_t i2cAnswer();

  uint8_t _i2c_address;
  uint8_t _index;
  uint32_t _speed;
};

#endif // !MOTORSHIELD_H
