#include "autopilot.h"
#include <Arduino.h>
#include <math.h>

extern SemaphoreHandle_t boatStatusMutex;
extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t pidMutex;
extern SemaphoreHandle_t routeMutex;
extern SemaphoreHandle_t dataMutex;

// FIX: Added missing global declarations for alerts and motors
extern AlertSetting alertSettings[];
extern BuzzerPatternControl alertBuzzerPatterns[];
extern FlashPatternControl alertLightFlashPatterns[];

void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings);
void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings);
void voidMotors();
void startActuator(Actuator &actuator);

static int calculatePID(double error, PIDController& pid, SemaphoreHandle_t& mux, bool isAngle = false) {
    if (xSemaphoreTake(mux, 5) == pdTRUE) {
        unsigned long current_time = millis();
        unsigned long dt = current_time - pid.last_time;
        
        if (dt == 0 || pid.last_time == 0) {
            pid.last_time = current_time;
            pid.previous_error = error;
            xSemaphoreGive(mux);
            return 0;
        }

        double p_term = pid.Kp * error;

        double dt_sec = (double)dt / 1000.0;
        pid.integral += error * dt_sec;
        pid.integral = constrain(pid.integral, -500.0, 500.0); 
        double i_term = pid.Ki * pid.integral;

        double error_change = error - pid.previous_error;

        if (isAngle) {
            if (error_change > 180.0) error_change -= 360.0;
            else if (error_change < -180.0) error_change += 360.0;
        }

        double raw_derivative = error_change / dt_sec;
        double derivative = (0.3 * raw_derivative) + (0.7 * pid.prev_derivative);
        pid.prev_derivative = derivative;
        
        double d_term = pid.Kd * derivative;
        double pid_output = p_term + i_term + d_term;

        pid.previous_error = error;
        pid.last_time = current_time;
        xSemaphoreGive(mux);
        
        return (int)constrain(pid_output, -1000, 1000);
    }
    return 0; 
}

static bool checkStickFlick(uint16_t current_val, uint16_t prev_val, bool high_side) {
    if (high_side) {
        return current_val > STICK_HIGH_THRESHOLD && prev_val <= STICK_HIGH_THRESHOLD;
    } else {
        return current_val < STICK_LOW_THRESHOLD && prev_val >= STICK_LOW_THRESHOLD;
    }
}

void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth) {
    if (!status.mode.is_armed) return;
    status.autopilot.target_waypoint_index = targetWpIndex;
    status.autopilot.engaged = true;
    status.autopilot.rth_active = isRth;
    status.autopilot.arrival_time_ms = 0;
    status.autopilot.actions_triggered = false;
    status.autopilot.drop_complete_time_ms = 0;
    status.autopilot.arrival_state = AP_IDLE;
    
    if (status.nav.has_gps_fix) {
        status.autopilot.origin_lat = status.nav.latitude;
        status.autopilot.origin_lng = status.nav.longitude;
    } else {
        status.autopilot.origin_lat = 0.0;
        status.autopilot.origin_lng = 0.0;
    }
    
    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        if (alertSettings[ALERT_AP_ENGAGED].beeps > 0) {
            startBuzzerPattern(alertBuzzerPatterns[ALERT_AP_ENGAGED], alertSettings[ALERT_AP_ENGAGED]);
            startFlashPattern(alertLightFlashPatterns[ALERT_AP_ENGAGED], alertSettings[ALERT_AP_ENGAGED]);
        }
        xSemaphoreGive(dataMutex);
    }
}

void initAutopilot() {
  if (xSemaphoreTake(nvsMutex, portMAX_DELAY) == pdTRUE) {
      preferences.begin("baitboat", true);
      
      if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
          steeringPID.Kp = preferences.getFloat("pid_p", 4.0);
          steeringPID.Ki = preferences.getFloat("pid_i", 0.1);
          steeringPID.Kd = preferences.getFloat("pid_d", 0.5);
          
          throttlePID.Kp = preferences.getFloat("tpid_p", 20.0);
          throttlePID.Ki = preferences.getFloat("tpid_i", 2.0);
          throttlePID.Kd = preferences.getFloat("tpid_d", 5.0);
          xSemaphoreGive(pidMutex);
      }
      preferences.end();
      xSemaphoreGive(nvsMutex);
  }
  
  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
      if (isnan(steeringPID.Kp) || steeringPID.Kp <= 0) steeringPID.Kp = 4.0;
      if (isnan(steeringPID.Ki) || steeringPID.Ki <= 0) steeringPID.Ki = 0.1;
      if (isnan(steeringPID.Kd) || steeringPID.Kd <= 0) steeringPID.Kd = 0.5;

      if (isnan(throttlePID.Kp) || throttlePID.Kp <= 0) throttlePID.Kp = 20.0;
      if (isnan(throttlePID.Ki) || throttlePID.Ki <= 0) throttlePID.Ki = 2.0;
      if (isnan(throttlePID.Kd) || throttlePID.Kd <= 0) throttlePID.Kd = 5.0;

      steeringPID.integral = 0; steeringPID.previous_error = 0; steeringPID.prev_derivative = 0; steeringPID.last_time = millis();
      throttlePID.integral = 0; throttlePID.previous_error = 0; throttlePID.prev_derivative = 0; throttlePID.last_time = millis();
      xSemaphoreGive(pidMutex);
  }

  if (xSemaphoreTake(routeMutex, portMAX_DELAY) == pdTRUE) {
      currentRoute.waypointCount = 0;
      currentRoute.currentStep = 0;
      currentRoute.status = ROUTE_INACTIVE;
      memset(currentRoute.waypoints, 0, sizeof(currentRoute.waypoints));
      xSemaphoreGive(routeMutex);
  }
}

int calculateSteeringPID(const BoatStatus& status, double targetLat, double targetLng) {
  double error = calculateCourseError(status, targetLat, targetLng);
  int pid_output = calculatePID(error, steeringPID, pidMutex, true);
  return (int)constrain(pid_output, -500, 500); 
}

int calculateSteeringPID(const BoatStatus& status, double targetHeading) {
  double error = calculateCourseError(status, targetHeading);
  int pid_output = calculatePID(error, steeringPID, pidMutex, true);
  return (int)constrain(pid_output, -500, 500); 
}

int calculateThrottlePID(const BoatStatus& status, double distance) {
  double error = distance - status.autopilot.anchor_radius;
  int pid_output = calculatePID(error, throttlePID, pidMutex, false);
  return (int)constrain(pid_output, 0, 300);
}

void handleRouteNavigation(BoatStatus& status) {
    if (!status.mode.is_armed) {
        status.autopilot.engaged = false;
        if (xSemaphoreTake(routeMutex, 10) == pdTRUE) {
            currentRoute.status = ROUTE_INACTIVE;
            xSemaphoreGive(routeMutex);
        }
        return;
    }

    RouteStatus route_status = ROUTE_INACTIVE;
    int route_step = 0;
    int route_wp_count = 0;

    if (xSemaphoreTake(routeMutex, 5) == pdTRUE) {
        route_status = currentRoute.status;
        route_step = currentRoute.currentStep;
        route_wp_count = currentRoute.waypointCount;
        xSemaphoreGive(routeMutex);
    } else {
        return; 
    }

    if (route_status == ROUTE_PAUSED) {
        if (status.autopilot.engaged) {
            status.autopilot.engaged = false;
            voidMotors();
        }
        return;
    }

    if (route_status != ROUTE_ACTIVE || !status.nav.has_gps_fix) {
        status.autopilot.engaged = false;
        return;
    }
    
    bool homeIsSet = false;
    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        homeIsSet = savedLocations[0].isSet;
        xSemaphoreGive(dataMutex);
    }

    if (route_step < route_wp_count) {
        int targetWpIndex = -1;
        if (xSemaphoreTake(routeMutex, 5) == pdTRUE) {
            targetWpIndex = currentRoute.waypoints[route_step];
            xSemaphoreGive(routeMutex);
        }
        
        if (targetWpIndex != -1 && status.autopilot.target_waypoint_index != targetWpIndex) {
            resetForNewTarget(status, targetWpIndex, false);
        }
    } else { 
        if (xSemaphoreTake(routeMutex, 10) == pdTRUE) {
            currentRoute.status = ROUTE_COMPLETE;
            xSemaphoreGive(routeMutex);
        }
        
        if (homeIsSet) {
             if (!status.autopilot.rth_active) {
                resetForNewTarget(status, 0, true);
            }
        } else {
            status.autopilot.engaged = false;
            status.autopilot.target_waypoint_index = -1;
            status.mode.current = MANUAL_MODE;
            voidMotors();
        }
    }
}

void handleLocationSaving(BoatStatus& status) {
  if (!status.nav.has_gps_fix) {
    return;
  }
  if (checkStickFlick(status.rc.channels[CH5], status.rc.prev_stick_ch5, true)) {
    saveCurrentLocation(status, 0, true); 
  }
  if (checkStickFlick(status.rc.channels[CH1], status.rc.prev_stick_ch1, false)) {
    saveCurrentLocation(status, 1); 
  }
  else if (checkStickFlick(status.rc.channels[CH1], status.rc.prev_stick_ch1, true)) {
    saveCurrentLocation(status, 2); 
  }
  if (checkStickFlick(status.rc.channels[CH2], status.rc.prev_stick_ch2, false)) {
    saveCurrentLocation(status, 3);
  }
  else if (checkStickFlick(status.rc.channels[CH2], status.rc.prev_stick_ch2, true)) {
    saveCurrentLocation(status, 4);
  }
}

void handleAutopilotControl(BoatStatus& status) {
  if (status.failsafe.rc_failsafe_start_ms > 0 || status.failsafe.rc_signal_lost_ms > 0) {
      return;
  }
  
  RouteStatus route_status = ROUTE_INACTIVE;
  if (xSemaphoreTake(routeMutex, 5) == pdTRUE) {
      route_status = currentRoute.status;
      xSemaphoreGive(routeMutex);
  }
  
  if (!status.nav.has_gps_fix || route_status == ROUTE_ACTIVE || route_status == ROUTE_PAUSED) {
    return; 
  }
  
  bool homeIsSet = false;
  if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      homeIsSet = savedLocations[0].isSet;
      xSemaphoreGive(dataMutex);
  }

  bool ch8Activated = status.rc.channels[CH8] > RC_MID_POINT; 
  if (status.mode.current == AUTOPILOT_MODE && ch8Activated && homeIsSet) {
      if (!status.autopilot.rth_active) {
          if (route_status != ROUTE_INACTIVE) {
              if (xSemaphoreTake(routeMutex, 10) == pdTRUE) {
                  currentRoute.status = ROUTE_INACTIVE;
                  xSemaphoreGive(routeMutex);
              }
          }
          resetForNewTarget(status, 0, true);
      }
  } else {
      if (status.autopilot.rth_active) {
          status.autopilot.engaged = false;
          voidMotors();
      }
      status.autopilot.rth_active = false;
  }

  if (!status.autopilot.rth_active) {
    int desiredWaypointIndex = -1;
    if (checkStickFlick(status.rc.channels[CH1], status.rc.prev_stick_ch1, false)) { desiredWaypointIndex = 1; }
    else if (checkStickFlick(status.rc.channels[CH1], status.rc.prev_stick_ch1, true)) { desiredWaypointIndex = 2; }
    if (checkStickFlick(status.rc.channels[CH2], status.rc.prev_stick_ch2, false)) { desiredWaypointIndex = 3; }
    else if (checkStickFlick(status.rc.channels[CH2], status.rc.prev_stick_ch2, true)) { desiredWaypointIndex = 4; }

    if (desiredWaypointIndex != -1) {
        bool wpIsSet = false;
        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
            wpIsSet = savedLocations[desiredWaypointIndex].isSet;
            xSemaphoreGive(dataMutex);
        }
        if (wpIsSet) {
            if (status.autopilot.target_waypoint_index != desiredWaypointIndex || !status.autopilot.engaged) {
                resetForNewTarget(status, desiredWaypointIndex, false);
            }
        }
    }
  }
}

// FIX: Restored the handleWaypointArrivalActions function that was accidentally removed
void handleWaypointArrivalActions(BoatStatus& status) {
  unsigned long now = millis();
  int wp_index = status.autopilot.target_waypoint_index;

  if (wp_index == -1 || status.autopilot.arrival_state == AP_IDLE) {
      return;
  }
  
  SavedLocation wp_actions;
  if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      wp_actions = savedLocations[wp_index];
      xSemaphoreGive(dataMutex);
  } else {
      return; 
  }

  switch(status.autopilot.arrival_state) {
      case AP_BRAKING: {
          unsigned long dynamic_braking_duration = (unsigned long)(status.autopilot.braking_start_speed * BRAKING_DURATION_MS_PER_MPS);
          dynamic_braking_duration = constrain(dynamic_braking_duration, MIN_BRAKING_DURATION_MS, MAX_BRAKING_DURATION_MS);
          
          if (now - status.autopilot.arrival_time_ms >= dynamic_braking_duration) {
              voidMotors();
              status.autopilot.arrival_state = AP_ARRIVED_WAITING;
              status.autopilot.arrival_time_ms = now;
          }
          break;
      }

      case AP_ARRIVED_WAITING:
          if (now - status.autopilot.arrival_time_ms >= 1000) {
              bool actions_were_defined = wp_actions.dropLeftHopper || wp_actions.dropRightHopper ||
                                          wp_actions.releaseLeftHook || wp_actions.releaseRightHook;

              if (actions_were_defined && !status.autopilot.actions_triggered) {
                  if (wp_actions.dropLeftHopper) { startActuator(actuators[0]); }
                  if (wp_actions.dropRightHopper) { startActuator(actuators[1]); }
                  if (wp_actions.releaseLeftHook) { startActuator(actuators[2]); }
                  if (wp_actions.releaseRightHook) { startActuator(actuators[3]); }
                  status.autopilot.actions_triggered = true;
                  status.autopilot.drop_complete_time_ms = now;
                  status.autopilot.arrival_state = AP_POST_ACTION_WAIT;
              } else {
                  status.autopilot.arrival_state = AP_COMPLETE;
              }
          }
          break;

      case AP_POST_ACTION_WAIT:
          if (now - status.autopilot.drop_complete_time_ms >= 5000) {
              status.autopilot.arrival_state = AP_COMPLETE;
          }
          break;

      case AP_COMPLETE: {
          auto stopAndResetToManual = [&]() {
              status.autopilot.engaged = false;
              status.autopilot.rth_active = false;
              status.autopilot.target_waypoint_index = -1;
              status.autopilot.arrival_state = AP_IDLE;
              status.mode.current = MANUAL_MODE; 
              voidMotors();
          };

          RouteStatus route_status = ROUTE_INACTIVE;
          if (xSemaphoreTake(routeMutex, 5) == pdTRUE) {
              route_status = currentRoute.status;
              xSemaphoreGive(routeMutex);
          }

          if (status.autopilot.rth_active) { 
              stopAndResetToManual();
          } 
          else if (route_status == ROUTE_ACTIVE) { 
              if (xSemaphoreTake(routeMutex, 10) == pdTRUE) {
                  currentRoute.currentStep++;
                  xSemaphoreGive(routeMutex);
              }
              status.autopilot.engaged = false; 
              status.autopilot.target_waypoint_index = -1; 
              status.autopilot.arrival_state = AP_IDLE;
          }
          else if (wp_actions.autoReturnToHome && wp_actions.isSet) { 
              resetForNewTarget(status, 0, true);
          } 
          else { 
              stopAndResetToManual();
          }
          break;
      }
    
      default:
        status.autopilot.arrival_state = AP_IDLE;
        break;
  }
}

void saveCurrentLocation(const BoatStatus& status, int slotIndex, bool isHomeSave) {
  if (slotIndex < 0 || slotIndex >= 5 || !status.nav.has_gps_fix) {
    return;
  }
  
  if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      savedLocations[slotIndex].lat = status.nav.latitude;
      savedLocations[slotIndex].lng = status.nav.longitude;
      savedLocations[slotIndex].isSet = true;
      savedLocations[slotIndex].dropLeftHopper = false;
      savedLocations[slotIndex].dropRightHopper = false;
      savedLocations[slotIndex].releaseLeftHook = false;
      savedLocations[slotIndex].releaseRightHook = false;
      savedLocations[slotIndex].autoReturnToHome = false;

      if (isHomeSave) {
        startBuzzerPattern(alertBuzzerPatterns[ALERT_HOME_SAVED], alertSettings[ALERT_HOME_SAVED]);
        startFlashPattern(alertLightFlashPatterns[ALERT_HOME_SAVED], alertSettings[ALERT_HOME_SAVED]);
      } else {
        startBuzzerPattern(alertBuzzerPatterns[ALERT_WP_SAVED], alertSettings[ALERT_WP_SAVED]);
        startFlashPattern(alertLightFlashPatterns[ALERT_WP_SAVED], alertSettings[ALERT_WP_SAVED]);
      }
      xSemaphoreGive(dataMutex);
  }

  if (xSemaphoreTake(nvsMutex, 100) == pdTRUE) {
      preferences.begin("baitboat", false);
      preferences.putDouble(("loc" + String(slotIndex) + "Lat").c_str(), status.nav.latitude);
      preferences.putDouble(("loc" + String(slotIndex) + "Lng").c_str(), status.nav.longitude);
      preferences.putBool(("loc" + String(slotIndex) + "Set").c_str(), true);
      preferences.end();
      xSemaphoreGive(nvsMutex);
  }
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371e3;
  double phi1 = lat1 * M_PI / 180.0;
  double phi2 = lat2 * M_PI / 180.0;
  double deltaPhi = (lat2 - lat1) * M_PI / 180.0;
  double deltaLambda = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) * sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = atan2(y, x) * 180.0 / M_PI;
  return fmod(bearing + 360.0, 360.0);
}

bool checkPlaneCrossing(double originLat, double originLng, double targetLat, double targetLng, double currentLat, double currentLng) {
    if (originLat == 0.0 && originLng == 0.0) return false;
    double latMid = (originLat + targetLat) / 2.0 * M_PI / 180.0;
    double deg2m = 111320.0;
    double dx = (targetLng - originLng) * deg2m * cos(latMid);
    double dy = (targetLat - originLat) * deg2m;
    double px = (currentLng - originLng) * deg2m * cos(latMid);
    double py = (currentLat - originLat) * deg2m;
    double dotProduct = (px * dx) + (py * dy);
    double segmentLengthSq = (dx * dx) + (dy * dy);
    return (segmentLengthSq > 0) && (dotProduct >= segmentLengthSq);
}

double calculateCourseError(const BoatStatus& status, double targetLat, double targetLng) {
    double distToTarget = calculateDistance(status.nav.latitude, status.nav.longitude, targetLat, targetLng);
    double bearingToTarget = calculateBearing(status.nav.latitude, status.nav.longitude, targetLat, targetLng);
    
    if (status.autopilot.origin_lat == 0.0 || status.autopilot.origin_lng == 0.0 || distToTarget < 3.0) {
        double courseDiff = bearingToTarget - status.nav.heading;
        if (courseDiff > 180) courseDiff -= 360;
        if (courseDiff < -180) courseDiff += 360;
        return courseDiff;
    }

    double trackBearing = calculateBearing(status.autopilot.origin_lat, status.autopilot.origin_lng, targetLat, targetLng);
    double bearingToCurrent = calculateBearing(status.autopilot.origin_lat, status.autopilot.origin_lng, status.nav.latitude, status.nav.longitude);
    double distToCurrent = calculateDistance(status.autopilot.origin_lat, status.autopilot.origin_lng, status.nav.latitude, status.nav.longitude);

    double angleDiffRad = (bearingToCurrent - trackBearing) * M_PI / 180.0;
    double xte = distToCurrent * sin(angleDiffRad);

    double lookaheadDist = max(4.0f, status.nav.speed_mps * 3.0f);

    double correctionAngle = atan2(-xte, lookaheadDist) * 180.0 / M_PI;
    double desiredBearing = fmod(trackBearing + correctionAngle + 360.0, 360.0);

    double courseDiff = desiredBearing - status.nav.heading;
    if (courseDiff > 180) courseDiff -= 360;
    if (courseDiff < -180) courseDiff += 360;

    return courseDiff;
}

double calculateCourseError(const BoatStatus& status, double targetHeading) {
  double courseDiff = targetHeading - status.nav.heading;
  if (courseDiff > 180) courseDiff -= 360;
  if (courseDiff < -180) courseDiff += 360;
  return courseDiff;
}