/*
 * Project: Bait Boat Control System (ESP32) - Main
 * Architecture: GPS is Boss, IMU is Compass.
 * Live Logging Disabled. CH9 PID Tuning Removed.
 */

#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WMM_Tinier.h>
#include <esp_task_wdt.h>

#include "types.h"
#include "config.h"
#include "imu.h"
#include "gps.h"
#include "nav_webserver.h"
#include "telemetry.h"
#include "autopilot.h"
#include "kalman.h"
#include "logger.h"

#define VOLTAGE_PIN 36
const float R1 = 30100.0, R2 = 7500.0, ADC_REF = 3.3, CALIBRATION_FACTOR = 1.097;
const int ADC_RES = 4095, VOLTAGE_SAMPLES = 10;

#define IBUS_PIN 25
#define IBUS_BAUD 115200
#define IBUS_PACKET_SIZE 32
#define ESC_LEFT_PIN 26
#define ESC_RIGHT_PIN 27

#define OPEN_LEFT_BAIT_HOPPER_PIN 18
#define OPEN_RIGHT_BAIT_HOPPER_PIN 4
#define RELEASE_LEFT_HOOK_PIN 19
#define RELEASE_RIGHT_HOOK_PIN 2
#define AUXILIARY_PIN 23

#define SR_SER 13
#define SR_RCLK 33
#define SR_SRCLK 32

const unsigned long IBUS_TIMEOUT = 250; 
const unsigned long LOW_BATT_DEBOUNCE_MS = 3000;
const int FAILSAFE_CHANNEL_THRESHOLD = 1700;
const unsigned long ACTUATOR_COOLDOWN_MS = 2000;
const float MOTOR_SMOOTHING_ALPHA = 0.2;
const unsigned long FAILSAFE_RTH_TRIGGER_MS = 5000;
const unsigned long LOOP_INTERVAL_MS = 5;

Servo escLeft;
Servo escRight;
int smoothedLeft = 1500;
int smoothedRight = 1500;

TinyGPSPlus gps;
WMM_Tinier wmm;
HardwareSerial gpsSerial(1);
HardwareSerial ibusSerial(2);
Preferences preferences;
WebServer server(80);
KalmanFilter kf;

BoatStatus boatStatus;
SavedLocation savedLocations[5];
PIDController steeringPID;
PIDController throttlePID;
Route currentRoute;
char ssid[33];

Actuator actuators[] = {
  {OPEN_LEFT_BAIT_HOPPER_PIN, false, 0, false, 500},
  {OPEN_RIGHT_BAIT_HOPPER_PIN, false, 0, false, 500},
  {RELEASE_LEFT_HOOK_PIN, false, 0, false, 350},
  {RELEASE_RIGHT_HOOK_PIN, false, 0, false, 350},
  {AUXILIARY_PIN, false, 0, false, 0}
};
const int NUM_ACTUATORS = sizeof(actuators) / sizeof(actuators[0]);
unsigned long lastActuatorTime[5] = {0, 0, 0, 0, 0};

AlertSetting alertSettings[NUM_ALERT_TYPES];
BuzzerPatternControl alertBuzzerPatterns[NUM_ALERT_TYPES];
FlashPatternControl alertLightFlashPatterns[NUM_ALERT_TYPES];
byte desiredBaseLightState = ALL_LIGHTS_MASK;
byte lastShiftRegisterState = 0x00;

SemaphoreHandle_t boatStatusMutex = NULL;
SemaphoreHandle_t nvsMutex = NULL;
SemaphoreHandle_t ibusMux = NULL;
SemaphoreHandle_t kalmanMutex = NULL;
SemaphoreHandle_t pidMutex = NULL;
SemaphoreHandle_t routeMutex = NULL;
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;
SemaphoreHandle_t logMutex = NULL;

volatile uint16_t ibus_channel_data[NUM_IBUS_CHANNELS];
volatile unsigned long last_ibus_packet_ms = 0;
volatile bool ibus_alive = false; 

void ibus_task(void *pvParameters);
void processIbusData();
void handleModes(BoatStatus& status);
void updateMotors(BoatStatus& status);
void handleManualActuators(BoatStatus& status);
void updateActuators();
void handleSystemChecks(BoatStatus& status);
void updateLights();
float readBatteryVoltage();
void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings);
void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings);
void startActuator(Actuator &actuator);
void setupIO();
void setupESCs();
void setupLightsAndBuzzer();
void voidMotors();
void handleArming(BoatStatus& status);
void handleCompassReturn(BoatStatus& status);
void updateSingleBuzzerPattern(BuzzerPatternControl& pattern);
void manageAllBuzzerAlerts();
void updateSingleFlashPattern(FlashPatternControl& pattern);
void manageAllLightFlashAlerts();

void setup() {
  if(!SPIFFS.begin(true)){
    // File system failed to mount
  }

  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 3000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 

  boatStatusMutex = xSemaphoreCreateMutex();
  nvsMutex = xSemaphoreCreateMutex();
  ibusMux = xSemaphoreCreateMutex();
  kalmanMutex = xSemaphoreCreateMutex();
  pidMutex = xSemaphoreCreateMutex();
  routeMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  
  initLogger();

  setupGPS();
  setupIO();
  setupLightsAndBuzzer();
  
  xTaskCreatePinnedToCore(ibus_task, "iBusTask", 4096, NULL, 3, NULL, 0);
  delay(500);
  processIbusData();

  if (boatStatus.rc.channels[CH6] > WIFI_RESET_THRESHOLD) {
     if(xSemaphoreTake(nvsMutex, portMAX_DELAY)) {
         preferences.begin("baitboat", false);
         preferences.remove("wifi_ssid");
         preferences.remove("wifi_pass");
         preferences.end();
         xSemaphoreGive(nvsMutex);
     }
     for(int i=0; i<5; i++) {
        byte flashOnState = BUZZER_MASK;
        byte flashOffState = ~ALL_LIGHTS_MASK;
        digitalWrite(SR_RCLK, LOW);
        shiftOut(SR_SER, SR_SRCLK, MSBFIRST, flashOnState);
        digitalWrite(SR_RCLK, HIGH);
        delay(100);
        digitalWrite(SR_RCLK, LOW);
        shiftOut(SR_SER, SR_SRCLK, MSBFIRST, flashOffState);
        digitalWrite(SR_RCLK, HIGH);
        delay(100);
     }
  }

  setupIMU(); 
  loadAllSettings();
  setupESCs();
  initAutopilot();
  boatStatus.vitals.battery_voltage = readBatteryVoltage();

  xTaskCreatePinnedToCore(webserver_task, "WebServerTask", 12288, NULL, 1, NULL, 0);
}

void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long now = millis();
  
  if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = now;

    if (!imuConnected) {
      static unsigned long lastImuRetry = 0;
      if (now - lastImuRetry > 2000) {
        lastImuRetry = now;
        imuConnected = setupIMU();
      }
    }
    if (imuConnected) {
      handleIMU(boatStatus);
    }

    processIbusData();
    handleGPS(boatStatus);
    handleModes(boatStatus);
    handleSystemChecks(boatStatus);
    handleArming(boatStatus);

    bool explicit_failsafe_active = (boatStatus.rc.channels[CH1] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH2] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH3] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH4] > FAILSAFE_CHANNEL_THRESHOLD);

    static unsigned long lastHeartbeatCheck = 0;
    bool task_frozen = false;

    if (now - lastHeartbeatCheck > 500) {
        if (!ibus_alive) {
            task_frozen = true;
        }
        ibus_alive = false; 
        lastHeartbeatCheck = now;
    }

    if (explicit_failsafe_active || (now - boatStatus.rc.last_update_ms > IBUS_TIMEOUT) || task_frozen) {
        if (explicit_failsafe_active) {
            if (boatStatus.failsafe.rc_failsafe_start_ms == 0) boatStatus.failsafe.rc_failsafe_start_ms = now;
        } else {
            if (boatStatus.failsafe.rc_signal_lost_ms == 0) boatStatus.failsafe.rc_signal_lost_ms = now;
        }

        unsigned long failsafe_duration = explicit_failsafe_active ?
            (now - boatStatus.failsafe.rc_failsafe_start_ms) : (now - boatStatus.failsafe.rc_signal_lost_ms);
        
        bool homeIsSet = false;
        if(xSemaphoreTake(dataMutex, 10)) {
            homeIsSet = savedLocations[0].isSet;
            xSemaphoreGive(dataMutex);
        }

        if (failsafe_duration > FAILSAFE_RTH_TRIGGER_MS) {
            bool has_fix = false;
            if(xSemaphoreTake(boatStatusMutex, 5) == pdTRUE) {
                has_fix = boatStatus.nav.has_gps_fix;
                xSemaphoreGive(boatStatusMutex);
            }

            if (boatStatus.mode.is_armed && homeIsSet && has_fix) {
                if (!boatStatus.autopilot.rth_active) {
                    resetForNewTarget(boatStatus, 0, true);
                }
                updateMotors(boatStatus);
            } else if (boatStatus.mode.is_armed && !has_fix && boatStatus.mode.current == MANUAL_MODE && !boatStatus.autopilot.rth_active) {
                handleCompassReturn(boatStatus);
            } else {
                voidMotors();
            }
        } else {
            voidMotors();
        }
        
        updateLights();
        esp_task_wdt_reset();
        return;

    } else {
        boatStatus.failsafe.rc_failsafe_start_ms = 0;
        boatStatus.failsafe.rc_signal_lost_ms = 0;
        boatStatus.failsafe.compass_return_heading = -1.0;
    }

    if (now < 2000) return;

    updateMotors(boatStatus);
    handleManualActuators(boatStatus);
    updateActuators();
    handleSettingsPersistence(boatStatus);
    
    updateLights();

    boatStatus.rc.prev_stick_ch1 = boatStatus.rc.channels[CH1];
    boatStatus.rc.prev_stick_ch2 = boatStatus.rc.channels[CH2];
    boatStatus.rc.prev_stick_ch5 = boatStatus.rc.channels[CH5];
    
    esp_task_wdt_reset();
  }
}

void ibus_task(void *pvParameters) {
  esp_task_wdt_add(NULL);
  ibusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_PIN, -1);

  static uint8_t buffer[IBUS_PACKET_SIZE];
  static int bufferIndex = 0;
  uint16_t temp_channels[NUM_IBUS_CHANNELS];

  while (true) {
    esp_task_wdt_reset(); 
    ibus_alive = true;

    if (ibusSerial.available()) {
      uint8_t byte = ibusSerial.read();

      if (bufferIndex == 0 && byte != 0x20) continue;
      if (bufferIndex == 1 && byte != 0x40) { bufferIndex = 0; continue; }

      if (bufferIndex < IBUS_PACKET_SIZE) {
        buffer[bufferIndex++] = byte;
      }

      if (bufferIndex == IBUS_PACKET_SIZE) {
        uint16_t calculated_checksum = 0xFFFF;
        for (int i = 0; i < 30; i++) {
          calculated_checksum -= buffer[i];
        }
        uint16_t received_checksum = (buffer[31] << 8) | buffer[30];

        if (calculated_checksum == received_checksum) {
          for (int i = 0; i < NUM_IBUS_CHANNELS; i++) {
            temp_channels[i] = (buffer[3 + i*2] << 8) | buffer[2 + i*2];
          }

          if(xSemaphoreTake(ibusMux, 5)) { 
              memcpy((void*)ibus_channel_data, (const void*)temp_channels, sizeof(temp_channels));
              last_ibus_packet_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
              xSemaphoreGive(ibusMux);
          }
        }
        bufferIndex = 0;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void processIbusData() {
    if(xSemaphoreTake(ibusMux, 5)) {
        memcpy((void*)boatStatus.rc.channels, (const void*)ibus_channel_data, sizeof(ibus_channel_data));
        boatStatus.rc.last_update_ms = last_ibus_packet_ms;
        xSemaphoreGive(ibusMux);
    }

    if (REVERSE_RC_THROTTLE) { boatStatus.rc.channels[CH3] = 3000 - boatStatus.rc.channels[CH3]; }
    if (REVERSE_RC_STEERING) { boatStatus.rc.channels[CH4] = 3000 - boatStatus.rc.channels[CH4]; }

    for(int i=0; i<NUM_IBUS_CHANNELS; ++i){
        boatStatus.rc.channels[i] = constrain(boatStatus.rc.channels[i], 1000, 2000);
    }
}

void handleArming(BoatStatus& status) {
  if (status.mode.is_armed) return;

  bool flicked = (status.rc.channels[CH5] > LIGHT_SWITCH_THRESHOLD && status.rc.prev_stick_ch5 <= LIGHT_SWITCH_THRESHOLD) ||
                 (status.rc.channels[CH5] < LIGHT_SWITCH_THRESHOLD && status.rc.prev_stick_ch5 >= LIGHT_SWITCH_THRESHOLD);
  if (flicked) {
    status.mode.is_armed = true;
    if(xSemaphoreTake(dataMutex, 10)){
        startBuzzerPattern(alertBuzzerPatterns[ALERT_ARMED], alertSettings[ALERT_ARMED]);
        startFlashPattern(alertLightFlashPatterns[ALERT_ARMED], alertSettings[ALERT_ARMED]);
        xSemaphoreGive(dataMutex);
    }
  }
}

void setupIO() {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(actuators[i].pin, OUTPUT);
    digitalWrite(actuators[i].pin, actuators[i].inverted ? HIGH : LOW);
  }
  pinMode(VOLTAGE_PIN, INPUT);
  analogReadResolution(12);
  pinMode(SR_SER, OUTPUT);
  pinMode(SR_RCLK, OUTPUT);
  pinMode(SR_SRCLK, OUTPUT);
}

void setupESCs() {
  escLeft.attach(ESC_LEFT_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
  escRight.attach(ESC_RIGHT_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
  escLeft.writeMicroseconds(RC_MID_POINT);
  escRight.writeMicroseconds(RC_MID_POINT);
  delay(2000);
}

void setupLightsAndBuzzer() {
  byte initialByte = ~ALL_LIGHTS_MASK;
  initialByte &= ~BUZZER_MASK;
  digitalWrite(SR_RCLK, LOW);
  shiftOut(SR_SER, SR_SRCLK, MSBFIRST, initialByte);
  digitalWrite(SR_RCLK, HIGH);
}

void handleModes(BoatStatus& status) {
  int ch7Value = status.rc.channels[CH7];
  status.mode.previous = status.mode.current;

  // AutoTune Override Logic
  if (status.mode.current == AUTOTUNE_MODE) {
      int throttle = status.rc.channels[CH3];
      int steering = status.rc.channels[CH4];
      if (abs(throttle - RC_MID_POINT) > 50 || abs(steering - RC_MID_POINT) > 50) {
          status.mode.current = MANUAL_MODE; // Cancel on stick movement
      }
      return; 
  }

  if (ch7Value <= MODE_MANUAL_MAX) {
      if(status.mode.current != ANCHOR_MODE) {
          status.mode.current = MANUAL_MODE;
      }
  } else if (ch7Value >= MODE_AP_MIN && ch7Value <= MODE_AP_MAX) {
      status.mode.current = AUTOPILOT_MODE;
  } else if (ch7Value >= MODE_LOC_SAVE_MIN) {
      status.mode.current = LOCATION_SAVE_MODE;
  }

  bool ch8Activated = status.rc.channels[CH8] > RC_MID_POINT;
  if (status.mode.previous == MANUAL_MODE && status.mode.current == MANUAL_MODE && ch8Activated) {
      status.mode.current = ANCHOR_MODE;
  } else if (status.mode.current == ANCHOR_MODE && !ch8Activated) {
      status.mode.current = MANUAL_MODE;
  }

  if (status.mode.current != status.mode.previous) {
    voidMotors();
    status.autopilot.engaged = false;
    status.autopilot.rth_active = false;
    status.autopilot.target_waypoint_index = -1;
    status.autopilot.arrival_state = AP_IDLE;
    
    if(xSemaphoreTake(routeMutex, 10)) {
        currentRoute.status = ROUTE_INACTIVE;
        xSemaphoreGive(routeMutex);
    }

    if (status.mode.current == ANCHOR_MODE && status.nav.has_gps_fix) {
        status.autopilot.anchor_lat = status.nav.latitude;
        status.autopilot.anchor_lng = status.nav.longitude;
        status.autopilot.anchor_heading = status.nav.heading; 
        
        if(xSemaphoreTake(pidMutex, 10)){
            throttlePID.integral = 0;
            steeringPID.integral = 0;
            xSemaphoreGive(pidMutex);
        }
    }
    
    if(xSemaphoreTake(dataMutex, 10)){
        for(int i = 0; i < NUM_ALERT_TYPES; i++) {
            alertBuzzerPatterns[i].active = false;
            alertLightFlashPatterns[i].active = false;
        }
        xSemaphoreGive(dataMutex);
    }
  }

  if (status.mode.current == LOCATION_SAVE_MODE) {
      handleLocationSaving(status);
  }
  else if (status.mode.current == AUTOPILOT_MODE) {
    RouteStatus route_status = ROUTE_INACTIVE;

    if(xSemaphoreTake(routeMutex, 5)) {
        route_status = currentRoute.status;
        xSemaphoreGive(routeMutex);
    }

    if (route_status == ROUTE_ACTIVE || route_status == ROUTE_PAUSED) {
      handleRouteNavigation(status);
    } else {
      handleAutopilotControl(status);
    }
  }
}

void handleCompassReturn(BoatStatus& status) {
  if (status.failsafe.compass_return_heading < 0) {
    status.failsafe.compass_return_heading = fmod(status.nav.heading + 180.0, 360.0);
  }
  int steeringAdjust = calculateSteeringPID(status, status.failsafe.compass_return_heading);
  int baseSpeed = 1650; 
  int targetSpeedLeft = baseSpeed - steeringAdjust;
  int targetSpeedRight = baseSpeed + steeringAdjust;

  escLeft.writeMicroseconds(constrain(targetSpeedLeft, ESC_MIN_PULSE, ESC_MAX_PULSE));
  escRight.writeMicroseconds(constrain(targetSpeedRight, ESC_MIN_PULSE, ESC_MAX_PULSE));
}

void updateMotors(BoatStatus& status) {
  if (!status.mode.is_armed) {
    voidMotors();
    return;
  }

  int targetSpeedLeft = RC_MID_POINT;
  int targetSpeedRight = RC_MID_POINT;
  
  if (status.mode.current == MANUAL_MODE) {
    int throttle = status.rc.channels[CH3];
    int steering = status.rc.channels[CH4];

    const int stickDeadband = 20;
    if (abs(throttle - RC_MID_POINT) < stickDeadband) throttle = RC_MID_POINT;
    if (abs(steering - RC_MID_POINT) < stickDeadband) steering = RC_MID_POINT;

    int motorTrim = map(status.rc.channels[CH10], 1000, 2000, -50, 50);
    int steering_diff = (steering - RC_MID_POINT);

    targetSpeedLeft = throttle + steering_diff - motorTrim;
    targetSpeedRight = throttle - steering_diff + motorTrim;
  }
  else if (status.mode.current == ANCHOR_MODE && status.nav.has_gps_fix) {
      double distance = calculateDistance(status.nav.latitude, status.nav.longitude, status.autopilot.anchor_lat, status.autopilot.anchor_lng);

      if (distance > status.autopilot.anchor_radius) {
          double bearingToAnchor = calculateBearing(status.nav.latitude, status.nav.longitude, status.autopilot.anchor_lat, status.autopilot.anchor_lng);

          int steeringAdjust = calculateSteeringPID(status, bearingToAnchor);
          int position_throttle_adjust = calculateThrottlePID(status, distance);
          int combined_throttle_adjust = constrain(position_throttle_adjust, 0, 400);

          targetSpeedLeft  = RC_MID_POINT + combined_throttle_adjust - steeringAdjust;
          targetSpeedRight = RC_MID_POINT + combined_throttle_adjust + steeringAdjust;

      } else {
          int steeringAdjust = calculateSteeringPID(status, status.autopilot.anchor_heading);
          targetSpeedLeft = RC_MID_POINT + steeringAdjust;
          targetSpeedRight = RC_MID_POINT - steeringAdjust;
      }
  }
  else if (status.mode.current == AUTOPILOT_MODE && status.autopilot.engaged && status.autopilot.target_waypoint_index != -1) {
    if (status.autopilot.arrival_state != AP_IDLE) {
      handleWaypointArrivalActions(status);

      if (status.autopilot.arrival_state == AP_BRAKING) {
          unsigned long braking_duration = millis() - status.autopilot.arrival_time_ms;

          if (braking_duration < 150) {
              escLeft.writeMicroseconds(RC_MID_POINT);
              escRight.writeMicroseconds(RC_MID_POINT);
          } else {
              escLeft.writeMicroseconds(1300);
              escRight.writeMicroseconds(1300);
          }
      }
      return;
    }
    
    double targetLat = 0, targetLng = 0;
    bool has_actions = false;

    if(xSemaphoreTake(dataMutex, 10)){
        targetLat = savedLocations[status.autopilot.target_waypoint_index].lat;
        targetLng = savedLocations[status.autopilot.target_waypoint_index].lng;
        has_actions = savedLocations[status.autopilot.target_waypoint_index].dropLeftHopper ||
                      savedLocations[status.autopilot.target_waypoint_index].dropRightHopper ||
                      savedLocations[status.autopilot.target_waypoint_index].releaseLeftHook ||
                      savedLocations[status.autopilot.target_waypoint_index].releaseRightHook;
        xSemaphoreGive(dataMutex);
    }

    RouteStatus route_status = ROUTE_INACTIVE;
    int route_step = 0;
    int wp_count = 0;
    
    if(xSemaphoreTake(routeMutex, 5)){
        route_status = currentRoute.status;
        route_step = currentRoute.currentStep;
        wp_count = currentRoute.waypointCount;
        xSemaphoreGive(routeMutex);
    }

    double distance = calculateDistance(status.nav.latitude, status.nav.longitude, targetLat, targetLng);
    float dynamic_braking_dist = status.autopilot.braking_distance;
    
    bool is_continuous_mapping = (route_status == ROUTE_ACTIVE && route_step < (wp_count - 1) && !has_actions);
    bool plane_crossed = false;

    if (status.autopilot.origin_lat != 0.0) {
        plane_crossed = checkPlaneCrossing(status.autopilot.origin_lat, status.autopilot.origin_lng, targetLat, targetLng, status.nav.latitude, status.nav.longitude);
    }

    if (is_continuous_mapping && (plane_crossed || distance < 3.0)) { 
        if (xSemaphoreTake(routeMutex, 10) == pdTRUE) {
            currentRoute.currentStep++;
            xSemaphoreGive(routeMutex);
        }
        status.autopilot.target_waypoint_index = -1; 
        return;
    } 
    else if (!is_continuous_mapping && distance < dynamic_braking_dist) {
      status.autopilot.arrival_time_ms = millis();
      status.autopilot.arrival_state = AP_BRAKING;
      status.autopilot.braking_start_speed = status.nav.speed_mps;
      escLeft.writeMicroseconds(RC_MID_POINT);
      escRight.writeMicroseconds(RC_MID_POINT);
      return;
    }

    status.autopilot.arrival_state = AP_IDLE;
    double courseError = abs(calculateCourseError(status, targetLat, targetLng));
    int steeringAdjust = calculateSteeringPID(status, targetLat, targetLng);

    int baseSpeed = 2000;
    if (courseError > 15.0) {
        baseSpeed = map(constrain(courseError, 15, 90), 15, 90, 2000, 1650);
    }
    
    targetSpeedLeft = baseSpeed + steeringAdjust;
    targetSpeedRight = baseSpeed - steeringAdjust;

  } 
  else if (status.mode.current == AUTOTUNE_MODE) {
      handleAutoTune(status, targetSpeedLeft, targetSpeedRight);
  } 
  else {
      voidMotors();
  }

  smoothedLeft = MOTOR_SMOOTHING_ALPHA * targetSpeedLeft + (1 - MOTOR_SMOOTHING_ALPHA) * smoothedLeft;
  smoothedRight = MOTOR_SMOOTHING_ALPHA * targetSpeedRight + (1 - MOTOR_SMOOTHING_ALPHA) * smoothedRight;

  escLeft.writeMicroseconds(constrain(smoothedLeft, ESC_MIN_PULSE, ESC_MAX_PULSE));
  escRight.writeMicroseconds(constrain(smoothedRight, ESC_MIN_PULSE, ESC_MAX_PULSE));
}

void handleManualActuators(BoatStatus& status) {
  if (status.mode.current != MANUAL_MODE) return;
  if (millis() - status.rc.last_update_ms > IBUS_TIMEOUT) return;

  unsigned long now = millis();
  if(xSemaphoreTake(dataMutex, 10)) {
      if (now - lastActuatorTime[0] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH1] < STICK_LOW_THRESHOLD && status.rc.prev_stick_ch1 >= STICK_LOW_THRESHOLD) {
          startActuator(actuators[0]);
          startBuzzerPattern(alertBuzzerPatterns[ALERT_LH_DROP], alertSettings[ALERT_LH_DROP]);
          startFlashPattern(alertLightFlashPatterns[ALERT_LH_DROP], alertSettings[ALERT_LH_DROP]);
          lastActuatorTime[0] = now;
      }
      if (now - lastActuatorTime[1] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH1] > STICK_HIGH_THRESHOLD && status.rc.prev_stick_ch1 <= STICK_HIGH_THRESHOLD) {
          startActuator(actuators[1]);
          startBuzzerPattern(alertBuzzerPatterns[ALERT_RH_DROP], alertSettings[ALERT_RH_DROP]);
          startFlashPattern(alertLightFlashPatterns[ALERT_RH_DROP], alertSettings[ALERT_RH_DROP]);
          lastActuatorTime[1] = now;
      }
      if (now - lastActuatorTime[2] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH2] < STICK_LOW_THRESHOLD && status.rc.prev_stick_ch2 >= STICK_LOW_THRESHOLD) {
          startActuator(actuators[2]);
          startBuzzerPattern(alertBuzzerPatterns[ALERT_LK_REL], alertSettings[ALERT_LK_REL]);
          startFlashPattern(alertLightFlashPatterns[ALERT_LK_REL], alertSettings[ALERT_LK_REL]);
          lastActuatorTime[2] = now;
      }
      if (now - lastActuatorTime[3] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH2] > STICK_HIGH_THRESHOLD && status.rc.prev_stick_ch2 <= STICK_HIGH_THRESHOLD) {
          startActuator(actuators[3]);
          startBuzzerPattern(alertBuzzerPatterns[ALERT_RK_REL], alertSettings[ALERT_RK_REL]);
          startFlashPattern(alertLightFlashPatterns[ALERT_RK_REL], alertSettings[ALERT_RK_REL]);
          lastActuatorTime[3] = now;
      }
      xSemaphoreGive(dataMutex);
  }
  
  if (status.rc.channels[CH6] > RC_MID_POINT && !actuators[4].active) {
      startActuator(actuators[4]);
  } else if (status.rc.channels[CH6] <= RC_MID_POINT && actuators[4].active) {
      actuators[4].active = false;
      digitalWrite(actuators[4].pin, actuators[4].inverted ? HIGH : LOW);
  }
}

void updateActuators() {
    unsigned long now = millis();

    for (int i = 0; i < NUM_ACTUATORS; i++) {
        if (actuators[i].active && actuators[i].onTime > 0 && (now - actuators[i].activatedAt >= actuators[i].onTime)) {
            actuators[i].active = false;
            digitalWrite(actuators[i].pin, actuators[i].inverted ? HIGH : LOW);
        }
    }
}

void handleSystemChecks(BoatStatus& status) {
    static bool isLowBattery = false;
    static unsigned long lowBatteryStartTime = 0;
    static bool low_batt_alert_active = false;
    unsigned long now = millis();

    if (now - status.vitals.last_voltage_read_ms >= 1000) {
        status.vitals.battery_voltage = readBatteryVoltage();
        status.vitals.last_voltage_read_ms = now;

        if (status.vitals.battery_voltage < status.mode.low_battery_threshold && status.vitals.battery_voltage > 1.0) {
            if (!isLowBattery) {
                isLowBattery = true;
                lowBatteryStartTime = now;
            }
        } else {
            isLowBattery = false;
            lowBatteryStartTime = 0; 
            if (low_batt_alert_active) { 
                if(xSemaphoreTake(dataMutex, 10)) {
                    alertBuzzerPatterns[ALERT_LOW_BATTERY].active = false;
                    alertLightFlashPatterns[ALERT_LOW_BATTERY].active = false;
                    xSemaphoreGive(dataMutex);
                }
                low_batt_alert_active = false;
            }
        }
        
        bool homeIsSet = false;
        if(xSemaphoreTake(dataMutex, 10)){
             homeIsSet = savedLocations[0].isSet;
             xSemaphoreGive(dataMutex);
        }

        if (isLowBattery && !low_batt_alert_active && (now - lowBatteryStartTime > LOW_BATT_DEBOUNCE_MS)) {
            if (status.autopilot.low_battery_rth_enabled && homeIsSet && !status.autopilot.rth_active) {
                resetForNewTarget(status, 0, true);
            } else {
                if(xSemaphoreTake(dataMutex, 10)) {
                    startBuzzerPattern(alertBuzzerPatterns[ALERT_LOW_BATTERY], alertSettings[ALERT_LOW_BATTERY]);
                    startFlashPattern(alertLightFlashPatterns[ALERT_LOW_BATTERY], alertSettings[ALERT_LOW_BATTERY]);
                    xSemaphoreGive(dataMutex);
                }
            }
            low_batt_alert_active = true;
        }
    }

    desiredBaseLightState = (status.rc.channels[CH5] < LIGHT_SWITCH_THRESHOLD || status.mode.current == LOCATION_SAVE_MODE) ?
        ALL_LIGHTS_MASK : 0x00;
}

void updateSingleBuzzerPattern(BuzzerPatternControl& pattern) {
    if (!pattern.active) { pattern.currentlyOn = false; return; }
    unsigned long now = millis();
    if (now >= pattern.nextToggleTime) {
        if (!pattern.currentlyOn) {
            pattern.currentBeep++;
            if (pattern.currentBeep > pattern.totalBeeps && pattern.totalBeeps < 9999) { 
                pattern.active = false;
                pattern.currentlyOn = false; return;
            }
        }
        pattern.currentlyOn = !pattern.currentlyOn;
        pattern.nextToggleTime = now + (pattern.currentlyOn ? pattern.beepDuration : pattern.pauseDuration);
    }
}

void manageAllBuzzerAlerts() {
    if(xSemaphoreTake(dataMutex, 5)){
        for(int i = 0; i < NUM_ALERT_TYPES; i++){
            updateSingleBuzzerPattern(alertBuzzerPatterns[i]);
        }
        xSemaphoreGive(dataMutex);
    }
}

void updateSingleFlashPattern(FlashPatternControl& pattern) {
    if (!pattern.active) { pattern.currentlyOn = false; return; }
    unsigned long now = millis();
    if (now >= pattern.nextToggleTime) {
         if (!pattern.currentlyOn) {
            pattern.currentToggle++;
            if (pattern.currentToggle > pattern.totalToggles && pattern.totalToggles < 9999) {
                pattern.active = false;
                pattern.currentlyOn = false; return;
            }
        }
        pattern.currentlyOn = !pattern.currentlyOn;
        pattern.nextToggleTime = now + (pattern.currentlyOn ? pattern.onDuration : pattern.offDuration);
    }
}

void manageAllLightFlashAlerts() {
    if(xSemaphoreTake(dataMutex, 5)){
        for(int i = 0; i < NUM_ALERT_TYPES; i++) {
            updateSingleFlashPattern(alertLightFlashPatterns[i]);
        }
        xSemaphoreGive(dataMutex);
    }
}

void updateLights() {
  manageAllBuzzerAlerts();
  manageAllLightFlashAlerts();
  
  byte logicalOnState = desiredBaseLightState;
  bool isBuzzerOn = false;

  if(xSemaphoreTake(dataMutex, 5)){
      for (int i = 0; i < NUM_ALERT_TYPES; i++) {
        if (alertLightFlashPatterns[i].active) {
          if (alertLightFlashPatterns[i].currentlyOn) {
            logicalOnState |= alertLightFlashPatterns[i].flashMask;
          } else {
            logicalOnState &= ~alertLightFlashPatterns[i].flashMask;
          }
        }
      }
      for (int i = 0; i < NUM_ALERT_TYPES; i++) {
        if (alertBuzzerPatterns[i].currentlyOn) {
          isBuzzerOn = true;
          break;
        }
      }
      xSemaphoreGive(dataMutex);
  }

  byte bitsToSend = ~logicalOnState;
  if (isBuzzerOn) {
    bitsToSend |= BUZZER_MASK;
  } else {
    bitsToSend &= ~BUZZER_MASK;
  }

  if (bitsToSend != lastShiftRegisterState) {
    digitalWrite(SR_RCLK, LOW);
    shiftOut(SR_SER, SR_SRCLK, MSBFIRST, bitsToSend);
    digitalWrite(SR_RCLK, HIGH);
    lastShiftRegisterState = bitsToSend;
  }
}

float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) { 
      sum += analogRead(VOLTAGE_PIN);
      delayMicroseconds(100);
  }
  float avgVoltage = (float)sum / VOLTAGE_SAMPLES;
  return (avgVoltage * ADC_REF / ADC_RES) * ((R1 + R2) / R2) * CALIBRATION_FACTOR;
}

void startActuator(Actuator &actuator) {
  actuator.active = true;
  actuator.activatedAt = millis();
  digitalWrite(actuator.pin, actuator.inverted ? LOW : HIGH);
}

void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings) {
  pattern.active = true;
  pattern.startTime = millis();
  pattern.nextToggleTime = millis(); 
  pattern.currentBeep = 0;
  pattern.totalBeeps = settings.beeps;
  pattern.beepDuration = settings.beepDuration;
  pattern.pauseDuration = settings.pauseDuration;
  pattern.currentlyOn = false;
}

void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings) {
    pattern.active = true;
    pattern.startTime = millis();
    pattern.nextToggleTime = millis();
    pattern.currentToggle = 0;
    pattern.totalToggles = settings.flashes;
    pattern.onDuration = settings.flashOnDuration;
    pattern.offDuration = settings.flashOffDuration;
    pattern.flashMask = settings.flashMask;
    pattern.currentlyOn = false;
}

void voidMotors() {
  escLeft.writeMicroseconds(RC_MID_POINT);
  escRight.writeMicroseconds(RC_MID_POINT);
  smoothedLeft = RC_MID_POINT;
  smoothedRight = RC_MID_POINT;
}