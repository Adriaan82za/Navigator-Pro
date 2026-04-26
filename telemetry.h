#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include "types.h"

// Forward declare WebServer class
class WebServer;

// External Global Object Declarations
extern WebServer server;
extern Preferences preferences;

// Mutexes for thread safety (FIXED: Changed to SemaphoreHandle_t)
extern SemaphoreHandle_t boatStatusMutex;
extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t kalmanMutex;
extern SemaphoreHandle_t pidMutex;
extern SemaphoreHandle_t routeMutex;
extern SemaphoreHandle_t dataMutex;

extern BoatStatus boatStatus;

// External declarations for variables not suited for the status struct
extern SavedLocation savedLocations[5];
extern PIDController steeringPID;
extern PIDController throttlePID;
extern Route currentRoute;
extern AlertSetting alertSettings[NUM_ALERT_TYPES];
extern char ssid[];
extern bool imuConnected; // For IMU status check

// External Function Declarations
void voidMotors();
void setupWebServerRoutes();
void flagSettingsChanged();
void sendJsonResponse(int code, bool success, const char* message);
void handleRestoreUpload();
void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth);

#endif