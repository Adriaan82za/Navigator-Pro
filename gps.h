/*
 * Project: Bait Boat Control System (ESP32) - GPS Module
 * Description: Handles GPS sensor operations
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WMM_Tinier.h>
#include "types.h"

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

#endif
