/*
 * Project: Bait Boat Control System (ESP32) - IMU Module
 * Architecture: IMU acts purely as a High-Speed Compass.
 * Update: Pitch and Roll axes swapped for correct physical mounting orientation.
 */

#include <Arduino.h>
#include <math.h>
#include "imu.h"
#include "config.h" 

bool imuConnected = false;
int currentOrientationIndex = 0; 
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t boatStatusMutex;

static unsigned long lastI2cReadTime = 0;

const char* getOrientationDescription(int id) { return "WT901 ENU (Y-Forward)"; }
bool detectCurrentOrientation() { return true; }
bool applyOrientation(int id) { return true; }

bool setupIMU() {
    if (xSemaphoreTake(i2cMutex, 100) == pdTRUE) {
        Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
        Wire.setClock(400000); 
        Wire.beginTransmission(WT901_I2C_ADDR);
        byte error = Wire.endTransmission();
        xSemaphoreGive(i2cMutex);

        if (error == 0) {
            Serial.println("WT901 IMU Detected on Hardware I2C!");
            imuConnected = true;
            return true;
        } else {
            Serial.println("WT901 I2C Not Found. Check SDA/SCL wiring.");
            return false;
        }
    }
    return false;
}

short readRegister(uint8_t reg) {
    Wire.beginTransmission(WT901_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); 
    
    Wire.requestFrom((uint16_t)WT901_I2C_ADDR, (uint8_t)2, true);
    if (Wire.available() >= 2) {
        uint8_t low = Wire.read();
        uint8_t high = Wire.read();
        return (short)((high << 8) | low);
    }
    return 0;
}

void writeRegister(uint8_t reg, uint16_t data) {
    if (xSemaphoreTake(i2cMutex, 50) == pdTRUE) {
        Wire.beginTransmission(WT901_I2C_ADDR);
        Wire.write(reg);
        Wire.write(data & 0xFF);         
        Wire.write((data >> 8) & 0xFF);  
        Wire.endTransmission();
        xSemaphoreGive(i2cMutex);
    }
}

bool startMagneticCalibration() {
    if (!imuConnected) return false;
    writeRegister(0x01, 0x0002); 
    return true;
}

bool endMagneticCalibration() {
    if (!imuConnected) return false;
    writeRegister(0x01, 0x0000); 
    delay(50);                   
    writeRegister(0x00, 0x0000); 
    return true;
}

void handleIMU(BoatStatus& status) {
    if (!imuConnected) return;
    
    // 20Hz update is plenty fast enough for steering
    if (millis() - lastI2cReadTime < 50) return; 
    lastI2cReadTime = millis();

    short rollRaw = 0, pitchRaw = 0, yawRaw = 0;
    bool readSuccess = false;

    if (xSemaphoreTake(i2cMutex, 20) == pdTRUE) {
        rollRaw = readRegister(0x3D);  
        pitchRaw = readRegister(0x3E); 
        yawRaw = readRegister(0x3F);   
        
        readSuccess = true;
        xSemaphoreGive(i2cMutex);
    }

    if (readSuccess) {
        float raw_roll = (float)rollRaw / 32768.0f * 180.0f;
        float raw_pitch = (float)pitchRaw / 32768.0f * 180.0f;
        float raw_yaw = (float)yawRaw / 32768.0f * 180.0f;

        float yaw_deg = 360.0f - raw_yaw; 

        yaw_deg += IMU_HEADING_OFFSET; 
        while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
        while (yaw_deg < 0.0f) yaw_deg += 360.0f;

        float current_declination = 0.0;
        if (xSemaphoreTake(boatStatusMutex, 20) == pdTRUE) {
            current_declination = status.nav.magnetic_declination;
            
            // FIXED: Axes swapped to compensate for physical mounting orientation
            status.nav.roll  = raw_pitch;
            status.nav.pitch = raw_roll;
            
            // IMU serves strictly as the compass. Accelerometers are ignored to prevent teleporting drift.
            status.nav.heading = fmod(yaw_deg + current_declination + 360.0, 360.0);
            status.nav.imu_accuracy = 3;
            xSemaphoreGive(boatStatusMutex);
        }
    }
}