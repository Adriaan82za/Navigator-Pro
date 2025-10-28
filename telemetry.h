#ifndef TELEMETRY_H
#define TELEMETRY_H

// #include <WebServer.h> // <-- DELETED THIS LINE
#include <ArduinoJson.h>
#include <TinyGPSPlus.h> // MOVED HERE
#include <Preferences.h>
#include <SPIFFS.h>
#include "types.h"

// Forward declare WebServer class
class WebServer;

// External Global Object Declarations
extern WebServer server; // Declaration is okay here
extern TinyGPSPlus gps; // NOW EXTERN
extern Preferences preferences;
extern portMUX_TYPE boatStatusMutex;
extern portMUX_TYPE nvsMutex; // <<< FIX: Added extern for NVS mutex

extern BoatStatus boatStatus;

// External declarations for variables not suited for the status struct
extern SavedLocation savedLocations[5];
extern PIDController steeringPID;
extern Route currentRoute;
extern AlertSetting alertSettings[NUM_ALERT_TYPES];
extern char ssid[];

// External Function Declarations
void voidMotors();
void setupWebServerRoutes();
void flagSettingsChanged();
void sendJsonResponse(int code, bool success, const char* message);

#endif