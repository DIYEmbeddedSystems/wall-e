/**
 * @file MotorShield.cpp
 * @brief Driver interface for WEMOS motor shield
 * @date 2021-01-13
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 */
#include "MotorShield.h"

#include <Wire.h>

/**
 * @brief Constructor
 * @param address: I2C address (WEMOS Motor Shield default is 0x30)
 * @param index: motor_a or motor_b 
 */
Motor::Motor(uint8_t address, uint8_t index)
{
  _i2c_address = constrain(address, 0, 127);
  _index = constrain(index, 0, 1);
}

/**
 * @brief Configure motor output
 */
void Motor::begin()
{
  setPwmFrequency(1000); // 1 kHz pwm modulation
  setSpeed(0); // start in stopped state
  delay(100);
}

/**
 * @brief Set the PWM modulation frequency
 * @param freq frequency (in Hz)
 * @return 0: success, other: I2C error code
 */
uint8_t Motor::setPwmFrequency(uint32_t freq)
{
  freq = 1000; //constrain(freq, 1, 80000);
  Wire.beginTransmission(_i2c_address);
  Wire.write((byte)(freq >> 24) & 0x0F);
  Wire.write((byte)(freq >> 16) & 0xFF);
  Wire.write((byte)(freq >> 8) & 0xFF);
  Wire.write((byte)freq & 0xFF);
  uint8_t res = Wire.endTransmission();
  i2cAnswer();
  delay(100);
  return res;
}

/**
 * @brief Read (and discard) the answer from I2C slave, if any
 * @return number of bytes read
 */
uint8_t Motor::i2cAnswer() 
{
  uint8_t res = 0;
  delay(10);
  while (Wire.available()) 
  {
    unsigned char c = Wire.read();
    (void)c;
    ++res;
  }
  delay(10);
  return res;
}

/**
 * @brief Set motor speed 
 * @param speed: clamped to -100 to +100
 * @return 0: success, other: I2C error code
 */
uint8_t Motor::setSpeed(int32_t speed) /* expect speed \in [-100 .. 100] */
{
  uint8_t dir_command;
  speed = constrain(speed, -100, 100);
  _speed = speed;
  uint16_t speedVal16 = abs(speed) * 100; // \in [0 .. 10000]

  if (speed > 0) 
  {
    dir_command = motor_cw;
  }
  else if (speed < 0) 
  {
    dir_command = motor_ccw;
  }
  else 
  {
    dir_command = motor_stop;
  }

  Wire.beginTransmission(_i2c_address);
  Wire.write(0x10 | _index);
  Wire.write(dir_command);
  Wire.write((byte)(speedVal16 >> 8) & 0xFF);
  Wire.write((byte)speedVal16 & 0xFF);
  uint8_t res = Wire.endTransmission();
  i2cAnswer();
  return res;
}

/**
 * @brief Get last written speed command
 * @return speed -100 to 100
 */
uint32_t Motor::getSpeed()
{
  return _speed;
}

/**
 * @brief Temporary disable the motor
 * @return 0: success, other: I2C error code
 */
uint8_t Motor::standby()
{
  _speed = 0;
  Wire.beginTransmission(_i2c_address);
  Wire.write(0x10 | _index);
  Wire.write(motor_stdby);
  Wire.write(0x00);
  Wire.write(0x00);
  uint8_t res = Wire.endTransmission();
  i2cAnswer();
  return res;
}
