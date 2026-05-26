#include "logger.h"
#include <SPIFFS.h>

extern SemaphoreHandle_t logMutex;

void initLogger() {
    // Logging intentionally disabled to prevent SPIFFS flash wear
}

void logTelemetryData(const BoatStatus& status, int leftMotor, int rightMotor, double desiredHeading, double distanceToTarget) {
    // Logging intentionally disabled to prevent SPIFFS flash wear and system load
}