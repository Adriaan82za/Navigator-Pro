#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <TinyGPSPlus.h>
#include <Preferences.h>
#include "types.h"
#include "config.h"

// ====== External Global Object/Variable Declarations ======
extern TinyGPSPlus gps;
extern Preferences preferences;
extern portMUX_TYPE boatStatusMutex;
extern portMUX_TYPE nvsMutex;
extern SavedLocation savedLocations[5];
extern PIDController steeringPID;
extern PIDController throttlePID; // NEW
extern Route currentRoute;
extern BuzzerPatternControl alertBuzzerPatterns[NUM_ALERT_TYPES];
extern FlashPatternControl alertLightFlashPatterns[NUM_ALERT_TYPES];
extern AlertSetting alertSettings[NUM_ALERT_TYPES];
extern Actuator actuators[];

// ====== Function Declarations ======
void initAutopilot();
void handleAutopilotControl(BoatStatus& status);
void handleRouteNavigation(BoatStatus& status);
void handleLocationSaving(BoatStatus& status);
void handleWaypointArrivalActions(BoatStatus& status);
void saveCurrentLocation(const BoatStatus& status, int slotIndex, bool isHomeSave = false);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double calculateCourseError(const BoatStatus& status, double targetLat, double targetLng);
double calculateCourseError(const BoatStatus& status, double targetHeading); // Overloaded for compass return
int calculateSteeringPID(const BoatStatus& status, double targetHeading); // Overloaded for compass return
int calculateSteeringPID(const BoatStatus& status, double targetLat, double targetLng);
int calculateThrottlePID(const BoatStatus& status, double distance); // NEW

// ====== External Function Declarations from other modules ======
void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings);
void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings);
void startActuator(Actuator &actuator);
void voidMotors();
void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth);

#endif