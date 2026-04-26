#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "types.h"
#include "kalman.h"

// ==================================
// WT901 I2C Configuration
// ==================================
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22
#define WT901_I2C_ADDR 0x50 // Factory default WitMotion I2C Address

// ==================================
// Function Declarations
// ==================================
bool setupIMU();
void handleIMU(BoatStatus& status);

// Web Calibration Commands
bool startMagneticCalibration();
bool endMagneticCalibration();

// Keep these so the webserver doesn't break
bool applyOrientation(int id);
bool detectCurrentOrientation(); 
const char* getOrientationDescription(int id);

// ==================================
// External declarations
// ==================================
extern bool imuConnected;
extern int currentOrientationIndex; 

extern KalmanFilter kf;
extern SemaphoreHandle_t kalmanMutex;
extern SemaphoreHandle_t i2cMutex;

#endif