#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "types.h"
#include "config.h"
#include <Preferences.h>

extern Preferences preferences;
extern PIDController steeringPID;
extern PIDController throttlePID;
extern Route currentRoute;
extern SavedLocation savedLocations[5];
extern Actuator actuators[];

void initAutopilot();
int calculateSteeringPID(const BoatStatus& status, double targetLat, double targetLng);
int calculateSteeringPID(const BoatStatus& status, double targetHeading);
int calculateThrottlePID(const BoatStatus& status, double distance);
void handleRouteNavigation(BoatStatus& status);
void handleLocationSaving(BoatStatus& status);
void handleAutopilotControl(BoatStatus& status);
void handleWaypointArrivalActions(BoatStatus& status);
void saveCurrentLocation(const BoatStatus& status, int slotIndex, bool isHomeSave = false);
void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double calculateCourseError(const BoatStatus& status, double targetLat, double targetLng);
double calculateCourseError(const BoatStatus& status, double targetHeading);
bool checkPlaneCrossing(double originLat, double originLng, double targetLat, double targetLng, double currentLat, double currentLng);

#endif