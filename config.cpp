/*
 * Project: Bait Boat Control System (ESP32) - Configuration Module
 * Description: Handles NVS/Preferences access for settings persistence
 * Updated: Removed portMAX_DELAY, disabled NVS log, deferred NVS write, added brake_dist.
 * Author: [Adriaan v.d.Westhuizen]
 * Date: November 04, 2025
 * Version: 12.3.2
 */

#include <Arduino.h>
#include "config.h"
#include <Preferences.h>

extern Preferences preferences;
extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t pidMutex;
extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t boatStatusMutex;

extern SavedLocation savedLocations[5];
extern PIDController steeringPID;
extern AlertSetting alertSettings[NUM_ALERT_TYPES];
extern BoatStatus boatStatus;

void loadAllSettings() {
    if (xSemaphoreTake(nvsMutex, 500 / portTICK_PERIOD_MS) == pdTRUE) {
        preferences.begin("baitboat", true);
        
        float low_batt = preferences.getFloat("low_batt", 10.5);
        bool lb_rth = preferences.getBool("lb_rth_en", false);
        float brake_dist = preferences.getFloat("brake_dist", 3.0);
        
        if (xSemaphoreTake(boatStatusMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
            boatStatus.mode.low_battery_threshold = low_batt;
            boatStatus.autopilot.low_battery_rth_enabled = lb_rth;
            boatStatus.autopilot.braking_distance = brake_dist;
            xSemaphoreGive(boatStatusMutex);
        }

        const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
        char key_buffer[25];
        
        if (xSemaphoreTake(dataMutex, 200 / portTICK_PERIOD_MS) == pdTRUE) {
            for(int i = 0; i < NUM_ALERT_TYPES; i++){
                bool is_lb = (i == AlertType::ALERT_LOW_BATTERY);
                bool is_wr = (i == AlertType::ALERT_WIFI_RESET);
                bool is_armed = (i == AlertType::ALERT_ARMED);

                snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
                int beeps = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i==AlertType::ALERT_HOME_SAVED?4:1))));
                snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
                unsigned long beepDuration = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
                snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
                unsigned long pauseDuration = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 100)));
                snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
                int flashes = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i<4?3:7))));
                snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
                unsigned long flashOnDuration = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
                snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
                unsigned long flashOffDuration = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 150)));
                snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
                byte flashMask = (byte)preferences.getUChar(key_buffer, ALL_LIGHTS_MASK);
                alertSettings[i] = {beeps, beepDuration, pauseDuration, flashes, flashOnDuration, flashOffDuration, flashMask};
            }

            for (int i = 0; i < 5; i++) {
                snprintf(key_buffer, sizeof(key_buffer), "loc%dSet", i);
                savedLocations[i].isSet = preferences.getBool(key_buffer, false);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dLat", i);
                savedLocations[i].lat = preferences.getDouble(key_buffer, 0.0);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dLng", i);
                savedLocations[i].lng = preferences.getDouble(key_buffer, 0.0);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dName", i);
                savedLocations[i].name = preferences.getString(key_buffer, (i==0) ? "Home" : "Waypoint " + String(i));
                snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", i);
                savedLocations[i].dropLeftHopper = preferences.getBool(key_buffer, false);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", i);
                savedLocations[i].dropRightHopper = preferences.getBool(key_buffer, false);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", i);
                savedLocations[i].releaseLeftHook = preferences.getBool(key_buffer, false);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", i);
                savedLocations[i].releaseRightHook = preferences.getBool(key_buffer, false);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", i);
                savedLocations[i].autoReturnToHome = preferences.getBool(key_buffer, false);
            }
            savedLocations[0].name = "Home";
            xSemaphoreGive(dataMutex);
        }

        preferences.end();
        xSemaphoreGive(nvsMutex);
    }
}

void handleSettingsPersistence(BoatStatus& status) {
  bool isDirty = false;
  unsigned long lastChange = 0;
  bool isArmed = false;

  if (xSemaphoreTake(boatStatusMutex, 5) == pdTRUE) {
      isDirty = status.persistence.settings_dirty;
      lastChange = status.persistence.last_change_ms;
      isArmed = status.mode.is_armed;
      xSemaphoreGive(boatStatusMutex);
  }

  if (isDirty && !isArmed && (millis() - lastChange > SETTINGS_SAVE_DEBOUNCE_MS)) {

    if (xSemaphoreTake(nvsMutex, 100) == pdTRUE) {
        preferences.begin("baitboat", false);
        
        if (xSemaphoreTake(pidMutex, 10) == pdTRUE) {
            preferences.putFloat("pid_p", steeringPID.Kp);
            preferences.putFloat("pid_i", steeringPID.Ki);
            preferences.putFloat("pid_d", steeringPID.Kd);
            xSemaphoreGive(pidMutex);
        }

        if (xSemaphoreTake(boatStatusMutex, 10) == pdTRUE) {
            preferences.putFloat("low_batt", status.mode.low_battery_threshold);
            preferences.putBool("lb_rth_en", status.autopilot.low_battery_rth_enabled);
            preferences.putFloat("brake_dist", status.autopilot.braking_distance);
            xSemaphoreGive(boatStatusMutex);
        }

        if (xSemaphoreTake(dataMutex, 20) == pdTRUE) {
            const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
            char key_buffer[25];
            for(int i = 0; i < NUM_ALERT_TYPES; i++){
                snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
                preferences.putInt(key_buffer, alertSettings[i].beeps);
                snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
                preferences.putULong(key_buffer, alertSettings[i].beepDuration);
                snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
                preferences.putULong(key_buffer, alertSettings[i].pauseDuration);
                snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
                preferences.putInt(key_buffer, alertSettings[i].flashes);
                snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
                preferences.putULong(key_buffer, alertSettings[i].flashOnDuration);
                snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
                preferences.putULong(key_buffer, alertSettings[i].flashOffDuration);
                snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
                preferences.putUChar(key_buffer, alertSettings[i].flashMask);
            }

            for(int i=1; i<5; i++) {
                snprintf(key_buffer, sizeof(key_buffer), "loc%dName", i);
                preferences.putString(key_buffer, savedLocations[i].name);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", i);
                preferences.putBool(key_buffer, savedLocations[i].dropLeftHopper);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", i);
                preferences.putBool(key_buffer, savedLocations[i].dropRightHopper);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", i);
                preferences.putBool(key_buffer, savedLocations[i].releaseLeftHook);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", i);
                preferences.putBool(key_buffer, savedLocations[i].releaseRightHook);
                snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", i);
                preferences.putBool(key_buffer, savedLocations[i].autoReturnToHome);
            }
            xSemaphoreGive(dataMutex);
        }

        preferences.end();
        xSemaphoreGive(nvsMutex);

        if (xSemaphoreTake(boatStatusMutex, 5) == pdTRUE) {
            status.persistence.settings_dirty = false;
            xSemaphoreGive(boatStatusMutex);
        }
    }
  }
}