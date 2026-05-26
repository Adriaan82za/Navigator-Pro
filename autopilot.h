#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "types.h"
#include <Arduino.h>

void initAutopilot();
int calculateSteeringPID(const BoatStatus& status, double targetLat, double targetLng);
int calculateSteeringPID(const BoatStatus& status, double targetHeading);
int calculateThrottlePID(const BoatStatus& status, double distance);
void handleRouteNavigation(BoatStatus& status);
void handleAutopilotControl(BoatStatus& status);
void handleLocationSaving(BoatStatus& status);
void handleWaypointArrivalActions(BoatStatus& status);
void saveCurrentLocation(const BoatStatus& status, int slotIndex, bool isHomeSave = false);
void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth);

double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
bool checkPlaneCrossing(double originLat, double originLng, double targetLat, double targetLng, double currentLat, double currentLng);
double calculateCourseError(const BoatStatus& status, double targetLat, double targetLng);
double calculateCourseError(const BoatStatus& status, double targetHeading);

void handleAutoTune(BoatStatus& status, int& targetSpeedLeft, int& targetSpeedRight);

#endif