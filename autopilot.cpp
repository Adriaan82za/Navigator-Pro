/*
 * Project: Bait Boat Control System (ESP32) - Autopilot Module
 * Description: Handles all autopilot logic, including PID steering,
 * waypoint navigation, and IMU heading calculation.
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 18, 2025
 * Version: 8.1.5 (Bug Fixes and Stability Improvements)
 */

#include "autopilot.h"
#include <Arduino.h>
#include <math.h>

// Internal helper function to avoid code duplication in PID calculations
static int calculatePID(double error, PIDController& pid) {
    unsigned long current_time = millis();
    unsigned long dt = current_time - pid.last_time;
    if (dt == 0) return 0;

    double p_term = pid.Kp * error;

    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -500, 500); // Anti-windup
    double i_term = pid.Ki * pid.integral;

    double derivative = (error - pid.previous_error) / (double)dt;
    double d_term = pid.Kd * derivative;

    double pid_output = p_term + i_term + d_term;

    pid.previous_error = error;
    pid.last_time = current_time;

    return (int)constrain(pid_output, -1000, 1000);
}


// Helper to detect a flick of the RC stick to one side.
static bool checkStickFlick(uint16_t current_val, uint16_t prev_val, bool high_side) {
    if (high_side) {
        return current_val > STICK_HIGH_THRESHOLD && prev_val <= STICK_HIGH_THRESHOLD;
    } else {
        return current_val < STICK_LOW_THRESHOLD && prev_val >= STICK_LOW_THRESHOLD;
    }
}

// Helper to reset autopilot state for a new destination
void resetForNewTarget(BoatStatus& status, int targetWpIndex, bool isRth) {
    if (!status.mode.is_armed) return;
    status.autopilot.target_waypoint_index = targetWpIndex;
    status.autopilot.engaged = true;
    status.autopilot.rth_active = isRth;
    status.autopilot.arrival_time_ms = 0;
    status.autopilot.actions_triggered = false;
    status.autopilot.drop_complete_time_ms = 0;
    status.autopilot.arrival_state = AP_IDLE;
    
    if (alertSettings[ALERT_AP_ENGAGED].beeps > 0) {
        startBuzzerPattern(alertBuzzerPatterns[ALERT_AP_ENGAGED], alertSettings[ALERT_AP_ENGAGED]);
        startFlashPattern(alertLightFlashPatterns[ALERT_AP_ENGAGED], alertSettings[ALERT_AP_ENGAGED]);
    }
}

// ==============================
// Autopilot Initialization
// ==============================
void initAutopilot() {
  portENTER_CRITICAL(&nvsMutex);
  preferences.begin("baitboat", true);
  
  steeringPID.Kp = preferences.getFloat("pid_p", 4.0);
  steeringPID.Ki = preferences.getFloat("pid_i", 0.1);
  steeringPID.Kd = preferences.getFloat("pid_d", 0.5);
  
  throttlePID.Kp = preferences.getFloat("tpid_p", 20.0);
  throttlePID.Ki = preferences.getFloat("tpid_i", 2.0);
  throttlePID.Kd = preferences.getFloat("tpid_d", 5.0);
  
  preferences.end();
  portEXIT_CRITICAL(&nvsMutex);

  // Allow Ki/Kd to be zero, only correct NaN
  if (isnan(steeringPID.Kp) || steeringPID.Kp < 0) steeringPID.Kp = 4.0;
  if (isnan(steeringPID.Ki) || steeringPID.Ki < 0) steeringPID.Ki = 0.1;
  if (isnan(steeringPID.Kd) || steeringPID.Kd < 0) steeringPID.Kd = 0.5;

  if (isnan(throttlePID.Kp) || throttlePID.Kp < 0) throttlePID.Kp = 20.0;
  if (isnan(throttlePID.Ki) || throttlePID.Ki < 0) throttlePID.Ki = 2.0;
  if (isnan(throttlePID.Kd) || throttlePID.Kd < 0) throttlePID.Kd = 5.0;

  steeringPID.integral = 0;
  steeringPID.previous_error = 0;
  steeringPID.last_time = millis();

  throttlePID.integral = 0;
  throttlePID.previous_error = 0;
  throttlePID.last_time = millis();
  
  currentRoute.waypointCount = 0;
  currentRoute.currentStep = 0;
  currentRoute.status = ROUTE_INACTIVE;
  
  memset(currentRoute.waypoints, 0, sizeof(currentRoute.waypoints));
}

// ==============================
// PID Steering Calculation (for GPS)
// ==============================
int calculateSteeringPID(const BoatStatus& status, double targetLat, double targetLng) {
  double error = calculateCourseError(status, targetLat, targetLng);
  int pid_output = calculatePID(error, steeringPID);
  // Increased constraint to allow for pivot turns
  return (int)constrain(pid_output, -500, 500); 
}

// Overloaded version for Compass Failsafe
int calculateSteeringPID(const BoatStatus& status, double targetHeading) {
  double error = calculateCourseError(status, targetHeading);
  int pid_output = calculatePID(error, steeringPID);
  // Increased constraint to allow for pivot turns
  return (int)constrain(pid_output, -500, 500); 
}


// ==============================
// PID Throttle Calculation for Anchor Mode
// ==============================
int calculateThrottlePID(const BoatStatus& status, double distance) {
  double error = distance - status.autopilot.anchor_radius;
  // Note: We use the helper but constrain the output differently for throttle
  int pid_output = calculatePID(error, throttlePID);
  return (int)constrain(pid_output, 0, 300); // Output is 0-300, added to mid-point
}

// ==============================
// Route Navigation Handler
// ==============================
void handleRouteNavigation(BoatStatus& status) {
    if (!status.mode.is_armed) {
        status.autopilot.engaged = false;
        currentRoute.status = ROUTE_INACTIVE;
        return;
    }

    if (currentRoute.status == ROUTE_PAUSED) {
        if (status.autopilot.engaged) {
            status.autopilot.engaged = false;
            voidMotors();
        }
        return;
    }

    if (currentRoute.status != ROUTE_ACTIVE || !status.nav.has_gps_fix) {
        status.autopilot.engaged = false;
        return;
    }

    if (currentRoute.currentStep < currentRoute.waypointCount) {
        int targetWpIndex = currentRoute.waypoints[currentRoute.currentStep];
        if (status.autopilot.target_waypoint_index != targetWpIndex) {
            resetForNewTarget(status, targetWpIndex, false);
        }
    } else { // Route finished, now do RTH
        currentRoute.status = ROUTE_COMPLETE;
        if (savedLocations[0].isSet) {
             if (!status.autopilot.rth_active) {
                resetForNewTarget(status, 0, true);
            }
        } else { // No home point, just stop
            status.autopilot.engaged = false;
            status.autopilot.target_waypoint_index = -1;
            status.mode.current = MANUAL_MODE;
            voidMotors();
        }
    }
}


// ==============================
// Location Saving
// ==============================
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

// ==============================
// Autopilot Control (for single waypoints)
// ==============================
void handleAutopilotControl(BoatStatus& status) {
  // If a failsafe is active, do not allow any manual autopilot control to interfere.
  if (status.failsafe.rc_failsafe_start_ms > 0 || status.failsafe.rc_signal_lost_ms > 0) {
      return;
  }

  if (!status.nav.has_gps_fix || currentRoute.status == ROUTE_ACTIVE || currentRoute.status == ROUTE_PAUSED) {
    return; 
  }
  
  bool ch8Activated = status.rc.channels[CH8] > RC_MID_POINT; 
  if (status.mode.current == AUTOPILOT_MODE && ch8Activated && savedLocations[0].isSet) {
      if (!status.autopilot.rth_active) {
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

    if (desiredWaypointIndex != -1 && savedLocations[desiredWaypointIndex].isSet) {
        if (status.autopilot.target_waypoint_index != desiredWaypointIndex || !status.autopilot.engaged) {
            resetForNewTarget(status, desiredWaypointIndex, false);
        }
    }
  }
}

// ==============================
// Handle Waypoint Arrival Actions (State Machine)
// ==============================
void handleWaypointArrivalActions(BoatStatus& status) {
  unsigned long now = millis();
  int wp_index = status.autopilot.target_waypoint_index;

  if (wp_index == -1 || status.autopilot.arrival_state == AP_IDLE) {
      return;
  }

  switch(status.autopilot.arrival_state) {
      case AP_BRAKING: {
          unsigned long dynamic_braking_duration = (unsigned long)(status.autopilot.braking_start_speed * BRAKING_DURATION_MS_PER_MPS);
          dynamic_braking_duration = constrain(dynamic_braking_duration, MIN_BRAKING_DURATION_MS, MAX_BRAKING_DURATION_MS);
          
          if (now - status.autopilot.arrival_time_ms >= dynamic_braking_duration) {
              voidMotors(); // Stop braking
              status.autopilot.arrival_state = AP_ARRIVED_WAITING;
              status.autopilot.arrival_time_ms = now; // Reset timer for waiting period
          }
          break;
      }

      case AP_ARRIVED_WAITING:
          if (now - status.autopilot.arrival_time_ms >= 1000) { // Reduced wait time
              bool actions_were_defined = savedLocations[wp_index].dropLeftHopper ||
                                          savedLocations[wp_index].dropRightHopper ||
                                          savedLocations[wp_index].releaseLeftHook ||
                                          savedLocations[wp_index].releaseRightHook;

              if (actions_were_defined && !status.autopilot.actions_triggered) {
                  if (savedLocations[wp_index].dropLeftHopper) { startActuator(actuators[0]); }
                  if (savedLocations[wp_index].dropRightHopper) { startActuator(actuators[1]); }
                  if (savedLocations[wp_index].releaseLeftHook) { startActuator(actuators[2]); }
                  if (savedLocations[wp_index].releaseRightHook) { startActuator(actuators[3]); }
                  status.autopilot.actions_triggered = true;
                  status.autopilot.drop_complete_time_ms = now;
                  status.autopilot.arrival_state = AP_POST_ACTION_WAIT;
              } else {
                  // No actions defined, or they have already been triggered
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

          if (status.autopilot.rth_active) { // Arrived at home
              currentRoute.status = ROUTE_INACTIVE;
              stopAndResetToManual();
          } 
          else if (currentRoute.status == ROUTE_ACTIVE) { // Arrived at a route waypoint
              currentRoute.currentStep++;
              status.autopilot.engaged = false; 
              status.autopilot.target_waypoint_index = -1; 
              status.autopilot.arrival_state = AP_IDLE;
          }
          else if (savedLocations[wp_index].autoReturnToHome && savedLocations[0].isSet) { // Arrived at a single WP with auto-RTH enabled
              resetForNewTarget(status, 0, true);
          } 
          else { // Arrived at a single WP with no further actions
              stopAndResetToManual();
          }
          break;
      }
    
      default:
        status.autopilot.arrival_state = AP_IDLE;
        break;
  }
}


// ==============================
// Save Current Location
// ==============================
void saveCurrentLocation(const BoatStatus& status, int slotIndex, bool isHomeSave) {
  if (slotIndex < 0 || slotIndex >= 5) {
    return;
  }
  
  // Read lat/lon locally from status.nav under mutex
  portENTER_CRITICAL(&boatStatusMutex);
  bool has_fix = status.nav.has_gps_fix;
  double lat = status.nav.latitude;
  double lng = status.nav.longitude;
  portEXIT_CRITICAL(&boatStatusMutex);
  
  if (!has_fix) {
    return;
  }
  
  // Write to Preferences under nvsMutex
  portENTER_CRITICAL(&nvsMutex);
  preferences.begin("baitboat", false);
  preferences.putDouble(("loc" + String(slotIndex) + "Lat").c_str(), lat);
  preferences.putDouble(("loc" + String(slotIndex) + "Lng").c_str(), lng);
  preferences.putBool(("loc" + String(slotIndex) + "Set").c_str(), true);
  preferences.end();
  portEXIT_CRITICAL(&nvsMutex);
  
  // Update savedLocations[] in-memory under boatStatusMutex
  portENTER_CRITICAL(&boatStatusMutex);
  savedLocations[slotIndex].lat = lat;
  savedLocations[slotIndex].lng = lng;
  savedLocations[slotIndex].isSet = true;
  savedLocations[slotIndex].dropLeftHopper = false;
  savedLocations[slotIndex].dropRightHopper = false;
  savedLocations[slotIndex].releaseLeftHook = false;
  savedLocations[slotIndex].releaseRightHook = false;
  savedLocations[slotIndex].autoReturnToHome = false;
  portEXIT_CRITICAL(&boatStatusMutex);
  
  // Trigger buzzer/flash alerts outside mutex
  if (isHomeSave) {
    startBuzzerPattern(alertBuzzerPatterns[ALERT_HOME_SAVED], alertSettings[ALERT_HOME_SAVED]);
    startFlashPattern(alertLightFlashPatterns[ALERT_HOME_SAVED], alertSettings[ALERT_HOME_SAVED]);
  } else {
    startBuzzerPattern(alertBuzzerPatterns[ALERT_WP_SAVED], alertSettings[ALERT_WP_SAVED]);
    startFlashPattern(alertLightFlashPatterns[ALERT_WP_SAVED], alertSettings[ALERT_WP_SAVED]);
  }
}

// ==============================
// Distance Calculation (Haversine Formula)
// ==============================
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

// ==============================
// Bearing Calculation
// ==============================
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = atan2(y, x) * 180.0 / M_PI;
  return fmod(bearing + 360.0, 360.0);
}

// ==============================
// Course Error Calculation
// ==============================
double calculateCourseError(const BoatStatus& status, double targetLat, double targetLng) {
  double targetBearing = calculateBearing(status.nav.latitude, status.nav.longitude, targetLat, targetLng);
  double courseDiff = targetBearing - status.nav.heading;
  if (courseDiff > 180) courseDiff -= 360;
  if (courseDiff < -180) courseDiff += 360;
  return courseDiff;
}

// Overloaded version for Compass Failsafe
double calculateCourseError(const BoatStatus& status, double targetHeading) {
  double courseDiff = targetHeading - status.nav.heading;
  if (courseDiff > 180) courseDiff -= 360;
  if (courseDiff < -180) courseDiff += 360;
  return courseDiff;
}