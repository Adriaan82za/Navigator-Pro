#ifndef CONFIG_H
#define CONFIG_H

// ==================================
// RC Control Settings
// ==================================

// Stick thresholds for activating commands
const int STICK_LOW_THRESHOLD = 1200;
const int STICK_HIGH_THRESHOLD = 1800;

// Center point for RC channels
const int RC_MID_POINT = 1500;

// Thresholds for special functions on boot or via switches
const int WIFI_RESET_THRESHOLD = 1900;
const int LIGHT_SWITCH_THRESHOLD = 1700;

// Channel value ranges for mode selection (CH7)
const int MODE_MANUAL_MAX = 1200;
const int MODE_AP_MIN = 1400;
const int MODE_AP_MAX = 1600;
const int MODE_LOC_SAVE_MIN = 1800;

// Reverse RC channel direction if needed
const bool REVERSE_RC_THROTTLE = false;
const bool REVERSE_RC_STEERING = false;

// Autopilot dynamic braking configuration
const float BRAKING_FACTOR = 1.2;          // Multiplier for speed to determine braking distance (e.g., 1.2 seconds out)
const float MIN_BRAKING_DISTANCE = 2.0;    // Minimum distance to start braking, in meters
const int BRAKING_REVERSE_PULSE = 1300;    // Throttle pulse for reverse braking (1500 is neutral)

// NEW: Adaptive Braking Settings
const float BRAKING_DURATION_MS_PER_MPS = 250.0; // Milliseconds of braking per meter/second of speed. (e.g., 2 m/s speed = 500ms braking)
const unsigned long MIN_BRAKING_DURATION_MS = 200; // A short burst even at low speed.
const unsigned long MAX_BRAKING_DURATION_MS = 1000; // Max braking time to prevent stalling.


// ESC Pulse Width Range
const int ESC_MIN_PULSE = 900;
const int ESC_MAX_PULSE = 2100;

#endif