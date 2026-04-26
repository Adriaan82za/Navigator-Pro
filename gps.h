#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WMM_Tinier.h>
#include "types.h"
#include "kalman.h"

// ==================================
// GPS Configuration
// ==================================
const int MIN_SATELLITES = 6;
const float MAX_HDOP = 2.0;
const unsigned long GPS_VALIDITY_TIMEOUT = 30000;
const int GPS_READ_MAX_CHARS = 80;

// ==================================
// Function Declarations
// ==================================
void setupGPS();
void handleGPS(BoatStatus& status);
void updateMagneticDeclination(BoatStatus& status);

// ==================================
// External declarations
// ==================================
extern TinyGPSPlus gps;
extern WMM_Tinier wmm;
extern HardwareSerial gpsSerial;

// Kalman filter instance (defined in main .ino)
extern KalmanFilter kf;

// Mutex for protecting kf (FIXED: Changed to SemaphoreHandle_t)
extern SemaphoreHandle_t kalmanMutex;

// Data mutex (for savedLocations / alertSettings) (FIXED: Changed to SemaphoreHandle_t)
extern SemaphoreHandle_t dataMutex;

#endif