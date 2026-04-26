#ifndef CONFIG_H
#define CONFIG_H

#include "types.h"

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
const float BRAKING_FACTOR = 1.2;          // Multiplier for speed to determine braking distance
const float MIN_BRAKING_DISTANCE = 0.0;    // Minimum distance to start braking
const int BRAKING_REVERSE_PULSE = 1300;    // Throttle pulse for reverse braking

// Adaptive Braking Settings
const float BRAKING_DURATION_MS_PER_MPS = 250.0;
const unsigned long MIN_BRAKING_DURATION_MS = 200;
const unsigned long MAX_BRAKING_DURATION_MS = 1000;

// Settings persistence debounce
const unsigned long SETTINGS_SAVE_DEBOUNCE_MS = 10000;

// Light masks
const byte HEADLIGHTS_MASK        = (1 << HEADLIGHTS);
const byte FRONT_LEFT_LIGHT_MASK  = (1 << FRONT_LEFT_LIGHT);
const byte FRONT_RIGHT_LIGHT_MASK = (1 << FRONT_RIGHT_LIGHT);
const byte REAR_LEFT_LIGHT_MASK   = (1 << REAR_LEFT_LIGHT);
const byte REAR_RIGHT_LIGHT_MASK  = (1 << REAR_RIGHT_LIGHT);
const byte BUZZER_MASK            = (1 << BUZZER);
const byte ALL_LIGHTS_MASK = HEADLIGHTS_MASK | FRONT_LEFT_LIGHT_MASK | FRONT_RIGHT_LIGHT_MASK | REAR_LEFT_LIGHT_MASK | REAR_RIGHT_LIGHT_MASK;
const byte LEFT_LIGHTS_MASK = FRONT_LEFT_LIGHT_MASK | REAR_LEFT_LIGHT_MASK;
const byte RIGHT_LIGHTS_MASK = FRONT_RIGHT_LIGHT_MASK | REAR_RIGHT_LIGHT_MASK;

// ESC Pulse Width Range
const int ESC_MIN_PULSE = 900;
const int ESC_MAX_PULSE = 2100;

// --- NEW: COMPASS OFFSET ---
// Add +90.0 if boat reads North when facing East.
// Add -90.0 if boat reads North when facing West.
const float IMU_HEADING_OFFSET = 0.0; 

// ==================================
// Function Declarations
// ==================================
void loadAllSettings();
void handleSettingsPersistence(BoatStatus& status);

#endif