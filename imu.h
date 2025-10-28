/*
 * Project: Bait Boat Control System (ESP32) - IMU Module
 * Description: Handles BNO08x IMU sensor operations
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>
#include "types.h"

// ==================================
// BNO08x Configuration
// ==================================
#define BNO08X_RESET -1 // Set to -1 to disable reset pin and resolve conflict
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_I2C_ADDRESS 0x4B

// ==================================
// Function Declarations
// ==================================
bool setupIMU();
void setBNO08xReports();
void handleIMU(BoatStatus& status);

// ==================================
// External declarations
// ==================================
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern bool imuConnected;

#endif
