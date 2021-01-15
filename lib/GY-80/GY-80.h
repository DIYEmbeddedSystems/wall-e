#ifndef GY_80_H
#define GY_80_H

#include <Arduino.h>
#include <Wire.h>

/**
 * Derived from https://github.com/kriswiner/GY-80/blob/master/GY80BasicAHRS.ino by: Kris Winer
 * 
 */


// ADXL345 Accelerometer Registers
#define WHO_AM_I_ADXL345        0x00   // Should return 0xE5
#define ADXL345_THRESH_TAP      0x1D   // Tap threshold
#define ADXL345_OFSX            0x1E   // X-axis offset
#define ADXL345_OFSY            0x1F   // Y-axis offset
#define ADXL345_OFSZ            0x20   // Z-axis offset
#define ADXL345_DUR             0x21   // Tap duration
#define ADXL345_LATENT          0x22   // Tap latency
#define ADXL345_WINDOW          0x23   // Tap window
#define ADXL345_THRESH_ACT      0x24   // Activity threshold
#define ADXL345_THRESH_INACT    0x25   // Inactivity threshold
#define ADXL345_TIME_INACT      0x26   // Inactivity time
#define ADXL345_ACT_INACT_CTL   0x27   // Axis enable control for activity/inactivity detection
#define ADXL345_THRESH_FF       0x28   // Free-fall threshold
#define ADXL345_TIME_FF         0x29   // Free-fall time
#define ADXL345_TAP_AXES        0x2A   // Axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS  0x2B   // Source of single/double tap
#define ADXL345_BW_RATE         0x2C   // Data rate and power mode control
#define ADXL345_POWER_CTL       0x2D   // Power-saving features control
#define ADXL345_INT_ENABLE      0x2E   // Interrupt enable control
#define ADXL345_INT_MAP         0x2F   // Interrupt mapping control
#define ADXL345_INT_SOURCE      0x30   // Source of interrupts
#define ADXL345_DATA_FORMAT     0x31   // Data format control
#define ADXL345_DATAX0          0x32   // X-axis data 0
#define ADXL345_DATAX1          0x33   // X-axis data 1
#define ADXL345_DATAY0          0x34   // Y-axis data 0
#define ADXL345_DATAY1          0x35   // Y-axis data 1
#define ADXL345_DATAZ0          0x36   // Z-axis data 0
#define ADXL345_DATAZ1          0x37   // Z-axis data 1
#define ADXL345_FIFO_CTL        0x38   // FIFO control
#define ADXL345_FIFO_STATUS     0x39   // FIFO status
#define ADXL345_ADDRESS         0x53   // Device address when ADO = 0 

// L3G4200D gyro registers
#define WHO_AM_I_L3G4200D       0x0F  // Should return 0xD3
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_REFERENCE      0x25
#define L3G4200D_OUT_TEMP       0x26
#define L3G4200D_STATUS_REG     0x27
#define L3G4200D_OUT_X_L        0x28
#define L3G4200D_OUT_X_H        0x29
#define L3G4200D_OUT_Y_L        0x2A
#define L3G4200D_OUT_Y_H        0x2B
#define L3G4200D_OUT_Z_L        0x2C
#define L3G4200D_OUT_Z_H        0x2D
#define L3G4200D_FIFO_CTRL_REG  0x2E
#define L3G4200D_FIFO_SRC_REG   0x2F
#define L3G4200D_INT1_CFG       0x30
#define L3G4200D_INT1_SRC       0x31
#define L3G4200D_INT1_TSH_XH    0x32
#define L3G4200D_INT1_TSH_XL    0x33
#define L3G4200D_INT1_TSH_YH    0x34
#define L3G4200D_INT1_TSH_YL    0x35
#define L3G4200D_INT1_TSH_ZH    0x36
#define L3G4200D_INT1_TSH_ZL    0x37
#define L3G4200D_INT1_DURATION  0x38
#define L3G4200D_ADDRESS        0x69  // Device address when ADO = 0

// HMC5883L magnetic sensor
#define HMC5883L_ADDRESS      0x1E
#define HMC5883L_CONFIG_A     0x00
#define HMC5883L_CONFIG_B     0x01
#define HMC5883L_MODE         0x02
#define HMC5883L_OUT_X_H      0x03
#define HMC5883L_OUT_X_L      0x04
#define HMC5883L_OUT_Z_H      0x05
#define HMC5883L_OUT_Z_L      0x06
#define HMC5883L_OUT_Y_H      0x07
#define HMC5883L_OUT_Y_L      0x08
#define HMC5883L_STATUS       0x09
#define HMC5883L_IDA          0x0A  // should return 0x48
#define HMC5883L_IDB          0x0B  // should return 0x34
#define HMC5883L_IDC          0x0C  // should return 0x33

// BMP085 digital pressure sensor
#define BMP085_ADDRESS        0x77  // I2C address of BMP085

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

// Set accelerometer ODR and Bandwidth
enum Arate {
  ARTBW_010_005 = 0, // 0.1 Hz ODR, 0.05Hz bandwidth
  ARTBW_020_010,
  ARTBW_039_020,
  ARTBW_078_039,
  ARTBW_156_078,
  ARTBW_313_156,
  ARTBW_125_625,
  ARTBW_25_125,
  ARTBW_50_25,
  ARTBW_100_50,
  ARTBW_200_100,
  ARTBW_400_200,
  ARTBW_800_400,
  ARTBW_1600_800,
  ARTBW_3200_1600  // 3200 Hz ODR, 1600 Hz bandwidth
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Grate { // set gyro ODR and Bandwidth with 4 bits
  GRTBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
  GRTBW_100_25,
  GRTBW_100_25a,
  GRTBW_100_25b,
  GRTBW_200_125,
  GRTBW_200_25,
  GRTBW_200_50,
  GRTBW_200_70,
  GRTBW_400_20,
  GRTBW_400_25,
  GRTBW_400_50,
  GRTBW_400_110,
  GRTBW_800_30,
  GRTBW_800_35,
  GRTBW_800_50,
  GRTBW_800_110  // 800 Hz ODR, 110 Hz bandwidth   
};

enum Mrate { // set magnetometer ODR
  MRT_0075 = 0, // 0.75 Hz ODR
  MRT_015,      // 1.5 Hz
  MRT_030,      // 3.0 Hz
  MRT_075,      // 7.5 Hz
  MRT_15,       // 15 Hz
  MRT_30,       // 30 Hz
  MRT_75,       // 75 Hz ODR    
};

enum OSS {  // BMP-085 sampling rate
  OSS_0 = 0,  // 4.5 ms conversion time
  OSS_1,      // 7.5
  OSS_2,      // 13.5
  OSS_3       // 25.5
};


extern float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
extern float q[4];    // vector to hold quaternion
extern float eInt[3];       // vector to hold integral error for Mahony method


void writeCommand(uint8_t address, uint8_t command);
void writeReg(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readReg(uint8_t address, uint8_t subAddress);
void readRegs(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

void getGres(); 
void getAres(); 
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
void readTempData(byte destination);
void MadgwickQuaternionUpdate(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              float mx, float my, float mz,
                              float *q);
//void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void bmp085Calibration();
int16_t bmp085GetTemperature();
long bmp085GetPressure();
void initHMC5883L();
byte selfTestHMC5883L();
void initL3G4200D();
void initADXL345();
void calADXL345();
uint8_t initGY80();
void updateGY80();







#endif // !GY_80_H

