#include "GY-80.h"

// Specify sensor parameters
uint8_t Ascale = AFS_2G;
uint8_t Arate = ARTBW_200_100; // 200 Hz ODR, 100 Hz bandwidth

uint8_t Gscale = GFS_250DPS;
uint8_t Grate = GRTBW_200_50; // 200 Hz ODR,  50 Hz bandwidth

uint8_t Mrate = MRT_75; //  75 Hz ODR
uint8_t OSS = OSS_3;    // maximum pressure resolution

float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

// These are constants used to calulate the temperature and pressure from the BMP-085 sensor
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md, b5;
uint16_t ac4, ac5, ac6;
float temperature, pressure;

// Pin definitions
int intPin = 12; // These can be changed, 2 and 3 are the Arduinos ext int pins
int Backlight = 58;

int16_t accelCount[3]; // 16-bit signed accelerometer sensor output
int16_t gyroCount[3];  // 16-bit signed gyro sensor output
int16_t magCount[3];   // 16-bit signed magnetometer sensor output
float magbias[3];      // User-specified magnetometer corrections values

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI *(40.0f / 180.0f) // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift PI *(0.0f / 180.0f)  // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f                         // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
boolean toggle = false;

float pitch, yaw, roll;
float deltat = 0.0f; // integration interval for both filter schemes

uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
//float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

/*
 *                 I2C utility functions
 */

// Wire.h read and write protocols
void writeCommand(uint8_t address, uint8_t command)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(command);             // Put command in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

// Wire.h read and write protocols
void writeReg(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.write(data);                // Put data in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

uint8_t readReg(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                          // `data` will store the register data
  Wire.beginTransmission(address);       // Initialize the Tx buffer
  Wire.write(subAddress);                // Put slave register address in Tx buffer
  Wire.endTransmission(false);           // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1); // Read one byte from slave register address
  data = Wire.read();                    // Fill Rx buffer with result
  return data;                           // Return data read from slave register
}

void readRegs(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  case GFS_250DPS:
    gRes = 250.0 / 32768.0;
    break;
  case GFS_500DPS:
    gRes = 500.0 / 32768.0;
    break;
  case GFS_1000DPS:
    gRes = 1000.0 / 32768.0;
    break;
  case GFS_2000DPS:
    gRes = 2000.0 / 32768.0;
    break;
  }
}

void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  case AFS_2G:
    aRes = 2.0 / (512. * 64.); // 10-bit 2s-complement
    break;
  case AFS_4G:
    aRes = 4.0 / (1024. * 32.); // 11-bit 2s-complement
    break;
  case AFS_8G:
    aRes = 8.0 / (2048. * 16.); // 12-bit 2s-complement
    break;
  case AFS_16G:
    aRes = 16.0 / (4096. * 8.); // 13-bit 2s-complement
    break;
  }
}

void readAccelData(int16_t *destination)
{
  uint8_t rawData[6];                                        // x/y/z accel register data stored here
  readRegs(ADXL345_ADDRESS, ADXL345_DATAX0, 6, &rawData[0]); // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void readGyroData(int16_t *destination)
{
  uint8_t rawData[6];                                                  // x/y/z gyro register data stored here
  readRegs(L3G4200D_ADDRESS, L3G4200D_OUT_X_L | 0x80, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];            // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[1];
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void readMagData(int16_t *destination)
{
  uint8_t rawData[6];                                           // x/y/z gyro register data stored here
  readRegs(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];     // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[4] << 8) | rawData[5];
  destination[2] = ((int16_t)rawData[2] << 8) | rawData[3];
}

void readTempData(byte destination)
{
  destination = readReg(L3G4200D_ADDRESS, L3G4200D_OUT_TEMP); // Read the one raw data register
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              float mx, float my, float mz,
                              float *q)
{
  static uint32_t lastMicros = 0;
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  uint32_t nowMicros = micros();
  float deltat = (nowMicros - lastMicros) * 1.0e-6;
  if (lastMicros == 0)
  { // 1st passed: set deltat to 0.0 to disable the integration part (of the form $ state += der * deltat $ )
    deltat = 0.0;
  }
  lastMicros = nowMicros;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

#if 0
// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f)
    return;           // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f; // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
#endif

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
// These BMP-085 functions were adapted from Jim Lindblom of SparkFun Electronics
void bmp085Calibration()
{
  ac1 = readReg(BMP085_ADDRESS, 0xAA) << 8 | readReg(BMP085_ADDRESS, 0xAB);
  ac2 = readReg(BMP085_ADDRESS, 0xAC) << 8 | readReg(BMP085_ADDRESS, 0xAD);
  ac3 = readReg(BMP085_ADDRESS, 0xAE) << 8 | readReg(BMP085_ADDRESS, 0xAF);
  ac4 = readReg(BMP085_ADDRESS, 0xB0) << 8 | readReg(BMP085_ADDRESS, 0xB1);
  ac5 = readReg(BMP085_ADDRESS, 0xB2) << 8 | readReg(BMP085_ADDRESS, 0xB3);
  ac6 = readReg(BMP085_ADDRESS, 0xB4) << 8 | readReg(BMP085_ADDRESS, 0xB5);
  b1 = readReg(BMP085_ADDRESS, 0xB6) << 8 | readReg(BMP085_ADDRESS, 0xB7);
  b2 = readReg(BMP085_ADDRESS, 0xB8) << 8 | readReg(BMP085_ADDRESS, 0xB9);
  mb = readReg(BMP085_ADDRESS, 0xBA) << 8 | readReg(BMP085_ADDRESS, 0xBB);
  mc = readReg(BMP085_ADDRESS, 0xBC) << 8 | readReg(BMP085_ADDRESS, 0xBD);
  md = readReg(BMP085_ADDRESS, 0xBE) << 8 | readReg(BMP085_ADDRESS, 0xBF);
}

// Temperature returned will be in units of 0.1 deg C
int16_t bmp085GetTemperature()
{
  int16_t ut = 0;
  writeReg(BMP085_ADDRESS, 0xF4, 0x2E); // start temperature measurement
  delay(5);
  uint8_t rawData[2] = {0, 0};
  readRegs(BMP085_ADDRESS, 0xF6, 2, &rawData[0]); // read raw temperature measurement
  ut = (((int16_t)rawData[0] << 8) | rawData[1]);

  long x1, x2;

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

// Calculate pressure read calibration values
// b5 is also required so bmp085GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure()
{
  long up = 0;
  writeReg(BMP085_ADDRESS, 0xF4, 0x34 | OSS << 6); // Configure pressure measurement for highest resolution
  delay(5 + 8 * OSS);                              // delay 5 ms at lowest resolution, 29 ms at highest
  uint8_t rawData[3] = {0, 0, 0};
  readRegs(BMP085_ADDRESS, 0xF6, 3, &rawData[0]); // read raw pressure measurement of 19 bits
  up = (((long)rawData[0] << 16) | ((long)rawData[1] << 8) | rawData[2]) >> (8 - OSS);

  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

void initHMC5883L()
{
  // Set magnetomer ODR; default is 15 Hz
  writeReg(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, Mrate << 2);
  writeReg(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x00); // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
  writeReg(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80);     // enable continuous data mode
}

byte selfTestHMC5883L()
{
  int16_t selfTest[3] = {0, 0, 0};
  //  Perform self-test and calculate temperature compensation bias
  writeReg(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x71); // set 8-average, 15 Hz default, positive self-test measurement
  writeReg(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0xA0); // set gain (bits[7:5]) to 5
  writeReg(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80);     // enable continuous data mode
  delay(150);                                          // wait 150 ms

  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};                      // x/y/z gyro register data stored here
  readRegs(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];        // Turn the MSB and LSB into a signed 16-bit value
  selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
  selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
  writeReg(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x00); // exit self test

  if (selfTest[0] < 575 && selfTest[0] > 243 && selfTest[1] < 575 && selfTest[1] > 243 && selfTest[2] < 575 && selfTest[2] > 243)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void initL3G4200D()
{
  // Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode
  writeReg(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, Grate << 4 | 0x0F);
  writeReg(L3G4200D_ADDRESS, L3G4200D_CTRL_REG3, 0x08);        // Push/pull, active high interrupt, enable data ready interrupt
  writeReg(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4, Gscale << 4); // set gyro full scale
  writeReg(L3G4200D_ADDRESS, L3G4200D_CTRL_REG5, 0x00);        // Disable FIFO
}

void initADXL345()
{
  // wake up device
  writeReg(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
  delay(100);                                         // Let device settle down
  writeReg(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08); // Put device in normal mode

  // Set accelerometer configuration; interrupt active high, left justify MSB
  writeReg(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x04 | Ascale); // Set full scale range for the accelerometer

  // Choose ODR and bandwidth
  writeReg(ADXL345_ADDRESS, ADXL345_BW_RATE, Arate); // Select normal power operation, and ODR and bandwidth

  writeReg(ADXL345_ADDRESS, ADXL345_INT_ENABLE, 0x80); // Enable data ready interrupt
  writeReg(ADXL345_ADDRESS, ADXL345_INT_MAP, 0x00);    // Enable data ready interrupt on INT_1

  writeReg(ADXL345_ADDRESS, ADXL345_FIFO_CTL, 0x00); // Bypass FIFO
}

void calADXL345()
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int abias[3] = {0, 0, 0};
  int16_t accel_bias[3] = {0, 0, 0};
  int samples, ii;

  // wake up device
  writeReg(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
  delay(10);                                          // Let device settle down
  writeReg(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08); // Put device in normal mode
  delay(10);

  // Set accelerometer configuration; interrupt active high, left justify MSB
  writeReg(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x04 | 0x00); // Set full scale range to 2g for the bias calculation
  uint16_t accelsensitivity = 256;                             // = 256 LSB/g at 2g full scale

  // Choose ODR and bandwidth
  writeReg(ADXL345_ADDRESS, ADXL345_BW_RATE, 0x09); // Select normal power operation, and 100 Hz ODR and 50 Hz bandwidth
  delay(10);

  writeReg(ADXL345_ADDRESS, ADXL345_FIFO_CTL, 0x40 | 0x2F); // Enable FIFO stream mode | collect 32 FIFO samples
  delay(1000);                                              // delay 1000 milliseconds to collect FIFO samples

  samples = readReg(ADXL345_ADDRESS, ADXL345_FIFO_STATUS);
  for (ii = 0; ii < samples; ii++)
  {
    readRegs(ADXL345_ADDRESS, ADXL345_DATAX0, 6, &data[0]);
    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]) >> 6;
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]) >> 6;
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) >> 6;
  }

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples;
  accel_bias[2] /= samples;

  // Remove gravity from z-axis accelerometer bias value
  if (accel_bias[2] > 0)
  {
    accel_bias[2] -= accelsensitivity;
  }
  else
  {
    accel_bias[2] += accelsensitivity;
  }

  abias[0] = (int)((-accel_bias[0] / 4) & 0xFF); // offset register are 8 bit 2s-complement, so have sensitivity 1/4 of 2g full scale
  abias[1] = (int)((-accel_bias[1] / 4) & 0xFF);
  abias[2] = (int)((-accel_bias[2] / 4) & 0xFF);

  writeReg(ADXL345_ADDRESS, ADXL345_OFSX, abias[0]);
  writeReg(ADXL345_ADDRESS, ADXL345_OFSY, abias[1]);
  writeReg(ADXL345_ADDRESS, ADXL345_OFSZ, abias[2]);
}

uint8_t initGY80()
{
  // Read the WHO_AM_I register of the ADXL345, this is a good test of communication
  uint8_t c = readReg(ADXL345_ADDRESS, WHO_AM_I_ADXL345); // Read WHO_AM_I register for ADXL345
  Serial.printf("ADXL345 WHO_AM_I register = 0x%02x (expected: 0xE5)\n", c);

  // Read the WHO_AM_I register of the L3G4200D, this is a good test of communication
  uint8_t d = readReg(L3G4200D_ADDRESS, WHO_AM_I_L3G4200D); // Read WHO_AM_I register for L3G4200D
  Serial.printf("L3G4200D WHO_AM_I register = 0x%02x (expected: 0xD3)\n", d);

  // Read the WHO_AM_I register of the HMC5883L, this is a good test of communication
  uint8_t e = readReg(HMC5883L_ADDRESS, HMC5883L_IDA); // Read WHO_AM_I register A for HMC5883L
  uint8_t f = readReg(HMC5883L_ADDRESS, HMC5883L_IDB); // Read WHO_AM_I register B for HMC5883L
  uint8_t g = readReg(HMC5883L_ADDRESS, HMC5883L_IDC); // Read WHO_AM_I register C for HMC5883L
  Serial.printf("HMC5883L WHO_AM_I registers: 0x%02x, 0x%02x, 0x%02x (expected 0x48, 0x34, 0x33)\n",
                e, f, g);

  // Read the first three calibration register of the BMP-085, this is a good test of communication
  uint8_t h = readReg(BMP085_ADDRESS, 0xAA); // Read WHO_AM_I register A for HMC5883L
  uint8_t i = readReg(BMP085_ADDRESS, 0xAB); // Read WHO_AM_I register B for HMC5883L
  uint8_t j = readReg(BMP085_ADDRESS, 0xAC); // Read WHO_AM_I register C for HMC5883L
  Serial.printf("Calibration registers of BMP085: 0x%02x, 0x%02x, 0x%02x (expected != 0x00 and != 0xFF)\n",
                h, i, j);

  if (c == 0xE5 && d == 0xD3 && e == 0x48 && f == 0x34 && g == 0x33 && h != 0xFF && h != 0x00) // WHO_AM_I must match to proceed
  {
    // Initialize devices for active mode read of acclerometer, gyroscope, magnetometer, pressure and temperature
    bmp085Calibration();
    Serial.println("BMP-085 calibration completed....");

    calADXL345();  // Calibrate ADXL345 accelerometers, load biases in bias registers
    initADXL345(); // Initialize and configure accelerometer
    Serial.println("ADXL345 initialized for active data mode....");

    initL3G4200D(); // Initialize and configure gyroscope
    Serial.println("L3G4200D initialized for active data mode....");

    if (selfTestHMC5883L())
    { // perform magnetometer self test
      Serial.println(" HMC5883L passed self test!");
    }
    else
    {
      Serial.println(" HMC5883L failed self test!");
    }
    initHMC5883L(); // Initialize and configure magnetometer
    Serial.println("HMC5883L initialized for active data mode....");

    return true;
  }
  else
  {
    Serial.println("GY80 init failed");
    return false;
  }
}

void updateGY80()
{
  // If intPin goes high or data ready status is TRUE, all data registers have new data
  if (readReg(ADXL345_ADDRESS, ADXL345_INT_SOURCE) & 0x80)
  {                            // When data is ready
    readAccelData(accelCount); // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;
    az = (float)accelCount[2] * aRes;
  }

  if (readReg(L3G4200D_ADDRESS, L3G4200D_STATUS_REG) & 0x08)
  {
    readGyroData(gyroCount); // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
  }

  if (readReg(HMC5883L_ADDRESS, HMC5883L_STATUS) & 0x01)
  {                        // If data ready bit set, then read magnetometer data
    readMagData(magCount); // Read the x/y/z adc values
    mRes = 0.73;           // Conversion to milliGauss, 0.73 mG/LSB in hihgest resolution mode
    // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
    // like the gyro and accelerometer biases
    magbias[0] = -30.; // User environmental x-axis correction in milliGauss
    magbias[1] = +85.; // User environmental y-axis correction in milliGauss
    magbias[2] = -78.; // User environmental z-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes - magbias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes - magbias[1];
    mz = (float)magCount[2] * mRes - magbias[2];
  }
}
