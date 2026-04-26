#include "logger.h"
#include <SPIFFS.h>

extern SemaphoreHandle_t logMutex;

void initLogger() {
    if (xSemaphoreTake(logMutex, portMAX_DELAY) == pdTRUE) {
        File file = SPIFFS.open("/telemetry.csv", FILE_APPEND);
        if (file) {
            if (file.size() == 0) {
                file.println("Time(ms),Mode,Lat,Lng,Speed(mps),Heading,TargetWP,TargetLat,TargetLng,Distance,DesiredHeading,LeftPWM,RightPWM");
            }
            file.close();
        }
        xSemaphoreGive(logMutex);
    }
}

void logTelemetryData(const BoatStatus& status, int leftMotor, int rightMotor, double desiredHeading, double distanceToTarget) {
    static unsigned long lastLogTime = 0;
    unsigned long now = millis();
    
    if (now - lastLogTime < 250) return;
    if (status.mode.current == MANUAL_MODE || status.mode.current == LOCATION_SAVE_MODE) return;
    if (!status.mode.is_armed) return;
    
    lastLogTime = now;

    double targetLat = 0.0;
    double targetLng = 0.0;
    if (status.mode.current == ANCHOR_MODE) {
        targetLat = status.autopilot.anchor_lat;
        targetLng = status.autopilot.anchor_lng;
    } else {
        extern SavedLocation savedLocations[5];
        extern SemaphoreHandle_t dataMutex;
        if (status.autopilot.target_waypoint_index >= 0 && status.autopilot.target_waypoint_index < 5) {
            if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
                targetLat = savedLocations[status.autopilot.target_waypoint_index].lat;
                targetLng = savedLocations[status.autopilot.target_waypoint_index].lng;
                xSemaphoreGive(dataMutex);
            }
        }
    }

    char logLine[200];
    snprintf(logLine, sizeof(logLine), "%lu,%d,%.6f,%.6f,%.2f,%.2f,%d,%.6f,%.6f,%.2f,%.2f,%d,%d\n",
             now,
             status.mode.current,
             status.nav.latitude,
             status.nav.longitude,
             status.nav.speed_mps,
             status.nav.heading,
             status.autopilot.target_waypoint_index,
             targetLat,
             targetLng,
             distanceToTarget,
             desiredHeading,
             leftMotor,
             rightMotor);

    if (xSemaphoreTake(logMutex, 10) == pdTRUE) {
        File file = SPIFFS.open("/telemetry.csv", FILE_APPEND);
        if (file) {
            file.print(logLine);
            file.close();
        }
        xSemaphoreGive(logMutex);
    }
}