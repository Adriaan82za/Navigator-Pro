#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

#define MAX_ROUTE_WAYPOINTS 10
#define NUM_IBUS_CHANNELS 10
const int NUM_ALERT_TYPES = 11;

struct xyzFloat {
  float x, y, z;
};

enum Channels { CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10 };

enum BoatMode {
  MANUAL_MODE,
  AUTOPILOT_MODE,
  LOCATION_SAVE_MODE,
  ANCHOR_MODE
};

enum ArmingState {
  ARM_IDLE,
  ARM_THROTTLE_HIGH_DETECTED,
  ARM_THROTTLE_LOW_DETECTED,
  ARMED
};

enum RouteStatus {
  ROUTE_INACTIVE,
  ROUTE_ACTIVE,
  ROUTE_PAUSED,
  ROUTE_COMPLETE
};

enum ArrivalState {
  AP_IDLE,
  AP_BRAKING,
  AP_ARRIVED_WAITING,
  AP_POST_ACTION_WAIT,
  AP_COMPLETE
};

enum AlertType {
    ALERT_HOME_SAVED,
    ALERT_WP_SAVED,
    ALERT_GPS_FIX,
    ALERT_AP_ENGAGED,
    ALERT_LH_DROP,
    ALERT_RH_DROP,
    ALERT_LK_REL,
    ALERT_RK_REL,
    ALERT_LOW_BATTERY,
    ALERT_WIFI_RESET,
    ALERT_ARMED
};

enum ShiftRegisterBits {
  BUZZER = 2,
  REAR_LEFT_LIGHT = 3,
  FRONT_LEFT_LIGHT = 4, 
  HEADLIGHTS = 5,       
  FRONT_RIGHT_LIGHT = 6, 
  REAR_RIGHT_LIGHT = 7
};

struct BuzzerPatternControl {
  bool active;
  unsigned long startTime, nextToggleTime;
  int currentBeep, totalBeeps;
  unsigned long beepDuration, pauseDuration;
  bool currentlyOn;
};

struct FlashPatternControl {
  bool active;
  unsigned long startTime, nextToggleTime;
  int currentToggle, totalToggles;
  unsigned long onDuration, offDuration;
  byte flashMask;
  bool currentlyOn;
};

struct Actuator {
  const uint8_t pin;
  bool active;
  unsigned long activatedAt;
  const bool inverted;
  const unsigned long onTime;
};

struct SavedLocation {
  double lat, lng;
  bool isSet;
  String name;
  bool dropLeftHopper;
  bool dropRightHopper;
  bool releaseLeftHook;
  bool releaseRightHook;
  bool autoReturnToHome;
};

struct PIDController {
  double Kp, Ki, Kd;
  double integral;
  double previous_error;
  double prev_derivative; 
  unsigned long last_time;
};

struct Route {
  int waypoints[MAX_ROUTE_WAYPOINTS];
  int waypointCount;
  int currentStep;
  RouteStatus status;
};

struct AlertSetting {
  int beeps;
  unsigned long beepDuration;
  unsigned long pauseDuration;
  int flashes;
  unsigned long flashOnDuration;
  unsigned long flashOffDuration;
  byte flashMask;
};

struct BoatStatus {
  struct {
    uint16_t channels[NUM_IBUS_CHANNELS];
    uint16_t prev_stick_ch1, prev_stick_ch2, prev_stick_ch5;
    volatile bool new_data_ready;
    volatile unsigned long last_update_ms;
  } rc;

  struct {
    BoatMode current;
    BoatMode previous;
    float low_battery_threshold;
    bool is_armed;
    ArmingState arming_state;
  } mode;

  struct {
    float heading, pitch, roll;
    float magnetic_declination;
    bool has_gps_fix;
    bool sbas_active;   // <--- FIXED: ADDED SBAS BOOLEAN HERE
    unsigned long last_gps_signal_ms;
    uint8_t imu_accuracy;
    double latitude;
    double longitude;
    float speed_mps;
  } nav;

  struct {
    bool engaged;
    bool rth_active;
    int target_waypoint_index;
    bool low_battery_rth_enabled;
    unsigned long arrival_time_ms;
    bool actions_triggered;
    unsigned long drop_complete_time_ms;
    ArrivalState arrival_state;
    float braking_start_speed;
    
    double anchor_lat;
    double anchor_lng;
    float anchor_radius;
    float anchor_heading;  
    double origin_lat;
    double origin_lng;
  } autopilot;

  struct {
    float battery_voltage;
    unsigned long last_voltage_read_ms;
  } vitals;

  struct {
    bool settings_dirty;
    unsigned long last_change_ms;
  } persistence;
  
  struct {
    unsigned long rc_signal_lost_ms;
    unsigned long rc_failsafe_start_ms;
    float compass_return_heading; 
  } failsafe;

  BoatStatus() {
    for(int i=0; i<NUM_IBUS_CHANNELS; ++i) rc.channels[i] = (i==2) ? 1000:1500;
    rc.prev_stick_ch1 = 1500;
    rc.prev_stick_ch2 = 1500;
    rc.prev_stick_ch5 = 1500;
    rc.new_data_ready = false;
    rc.last_update_ms = 0;

    mode.current = MANUAL_MODE;
    mode.previous = MANUAL_MODE;
    mode.low_battery_threshold = 10.5;
    mode.is_armed = false;
    mode.arming_state = ARM_IDLE;

    nav.heading = 0.0; nav.pitch = 0.0; nav.roll = 0.0;
    nav.magnetic_declination = 0.0;
    nav.has_gps_fix = false;
    nav.sbas_active = false; // <--- FIXED: INITIALIZED TO FALSE
    nav.last_gps_signal_ms = 0;
    nav.imu_accuracy = 0;
    nav.latitude = 0.0;
    nav.longitude = 0.0;
    nav.speed_mps = 0.0;

    autopilot.engaged = false;
    autopilot.rth_active = false;
    autopilot.target_waypoint_index = -1;
    autopilot.low_battery_rth_enabled = false;
    autopilot.arrival_time_ms = 0;
    autopilot.actions_triggered = false;
    autopilot.drop_complete_time_ms = 0;
    autopilot.arrival_state = AP_IDLE;
    autopilot.braking_start_speed = 0.0f;
    autopilot.anchor_lat = 0.0;
    autopilot.anchor_lng = 0.0;
    autopilot.anchor_radius = 2.0;
    autopilot.anchor_heading = 0.0f;
    autopilot.origin_lat = 0.0;
    autopilot.origin_lng = 0.0;

    vitals.battery_voltage = 0.0;
    vitals.last_voltage_read_ms = 0;
    
    persistence.settings_dirty = false;
    persistence.last_change_ms = 0;

    failsafe.rc_signal_lost_ms = 0;
    failsafe.rc_failsafe_start_ms = 0;
    failsafe.compass_return_heading = -1.0; 
  }
};

#endif