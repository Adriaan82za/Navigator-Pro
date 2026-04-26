#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "types.h"

void initLogger();
void logTelemetryData(const BoatStatus& status, int targetSpeedLeft, int targetSpeedRight, double desiredHeading, double distanceToTarget);

#endif