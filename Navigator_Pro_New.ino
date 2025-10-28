/*
 * Project: Bait Boat Control System (ESP32) - Main
 * Description: This file handles core setup, main loop, and hardware interfaces.
 * Now using a dedicated core for iBus and WebServer to ensure system stability.
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 24, 2025
 * Version: 12.1.4 (Robust Anchor Mode with Drift Compensation)
 */

#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WMM_Tinier.h>

#include "types.h"
#include "telemetry.h"
#include "autopilot.h"
#include "config.h"
#include "kalman.h"

// ==============================
// BNO08x Configuration
// ==============================
#define BNO08X_RESET -1 // Set to -1 to disable reset pin and resolve conflict
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO08X_I2C_ADDRESS 0x4B

// ==============================
// Configuration Constants
// ==============================
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

const int MIN_SATELLITES = 6;
const float MAX_HDOP = 2.0;
const unsigned long GPS_VALIDITY_TIMEOUT = 30000;
const unsigned long IBUS_TIMEOUT = 100;
const int GPS_READ_MAX_CHARS = 80;
const unsigned long LOW_BATT_DEBOUNCE_MS = 3000;
const int FAILSAFE_CHANNEL_THRESHOLD = 1700;
const byte HEADLIGHTS_MASK        = (1 << HEADLIGHTS);
const byte FRONT_LEFT_LIGHT_MASK  = (1 << FRONT_LEFT_LIGHT);
const byte FRONT_RIGHT_LIGHT_MASK = (1 << FRONT_RIGHT_LIGHT);
const byte REAR_LEFT_LIGHT_MASK   = (1 << REAR_LEFT_LIGHT);
const byte REAR_RIGHT_LIGHT_MASK  = (1 << REAR_RIGHT_LIGHT);
const byte BUZZER_MASK            = (1 << BUZZER);
const byte ALL_LIGHTS_MASK = HEADLIGHTS_MASK | FRONT_LEFT_LIGHT_MASK | FRONT_RIGHT_LIGHT_MASK | REAR_LEFT_LIGHT_MASK | REAR_RIGHT_LIGHT_MASK;
const byte LEFT_LIGHTS_MASK = FRONT_LEFT_LIGHT_MASK | REAR_LEFT_LIGHT_MASK;
const byte RIGHT_LIGHTS_MASK = FRONT_RIGHT_LIGHT_MASK | REAR_RIGHT_LIGHT_MASK;

const unsigned long ACTUATOR_COOLDOWN_MS = 2000;
const float MOTOR_SMOOTHING_ALPHA = 0.2;
const unsigned long SETTINGS_SAVE_DEBOUNCE_MS = 10000;
const unsigned long FAILSAFE_RTH_TRIGGER_MS = 5000;
const unsigned long LOOP_INTERVAL_MS = 5;

// ==============================
// Global Objects
// ==============================
TinyGPSPlus gps;
WMM_Tinier wmm;
HardwareSerial gpsSerial(1);
HardwareSerial ibusSerial(2);
Preferences preferences;
WebServer server(80);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
Servo escLeft, escRight;
KalmanFilter kf;
// ==============================
// Central State and Data
// ==============================
BoatStatus boatStatus;
portMUX_TYPE boatStatusMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE nvsMutex = portMUX_INITIALIZER_UNLOCKED; // <<< FIX: Added mutex for NVS/Preferences
SavedLocation savedLocations[5];
PIDController steeringPID;
PIDController throttlePID;
Route currentRoute;
char ssid[33];
bool imuConnected = false;
static int smoothedLeft = 1500, smoothedRight = 1500;
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

// ISR-safe variables for iBus
portMUX_TYPE ibusMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t ibus_channel_data[NUM_IBUS_CHANNELS];
volatile unsigned long last_ibus_packet_ms = 0;

// Forward Declarations
void ibus_task(void *pvParameters);
void webserver_task(void *pvParameters);
void loadAllSettings();
void setupIO();
void setupESCs();
void setupLightsAndBuzzer();
void processIbusData();
void handleGPS(BoatStatus& status);
void handleIMU(BoatStatus& status);
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
void handleSettingsPersistence(BoatStatus& status);
void setupWifi();
void voidMotors();
void handleManualPidTuning(BoatStatus& status);
void handleDynamicPID(BoatStatus& status);
void setBNO08xReports();
void updateMagneticDeclination();
void handleArming(BoatStatus& status);
void handleCompassReturn(BoatStatus& status);
void updateSingleBuzzerPattern(BuzzerPatternControl& pattern);
void manageAllBuzzerAlerts();
void updateSingleFlashPattern(FlashPatternControl& pattern);
void manageAllLightFlashAlerts();

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    while(1) delay(1000);
  }

  if (wmm.begin()) {
    // WMM_Tinier Initialized
  } else {
    Serial.println("WMM_Tinier Initialization failed!");
  }

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  setupIO();
  setupLightsAndBuzzer();

  xTaskCreatePinnedToCore(ibus_task, "iBusTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(webserver_task, "WebServerTask", 8192, NULL, 1, NULL, 0);

  delay(500);
  processIbusData();
  if (boatStatus.rc.channels[CH6] > WIFI_RESET_THRESHOLD) {
     portENTER_CRITICAL(&nvsMutex); // <<< FIX: Protect NVS access
     preferences.begin("baitboat", false);
     preferences.remove("wifi_ssid");
     preferences.remove("wifi_pass");
     preferences.end();
     portEXIT_CRITICAL(&nvsMutex); // <<< FIX: Release NVS access

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

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno08x.begin_I2C(BNO08X_I2C_ADDRESS)) {
      imuConnected = false;
      Serial.println("BNO08x connection failed.");
  } else {
      imuConnected = true;
      setBNO08xReports();
  }

  loadAllSettings();
  setupESCs();
  initAutopilot();
  boatStatus.vitals.battery_voltage = readBatteryVoltage();
}

// ==============================
// Main Loop (runs on Core 1)
// ==============================
void loop() {
  static unsigned long lastLoopTime = 0;
  if (millis() - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = millis();
    // --- 1. SENSOR READS AND STATE UPDATES (ALWAYS RUN) ---
    if (!imuConnected) {
      if (bno08x.begin_I2C(BNO08X_I2C_ADDRESS)) {
        imuConnected = true;
        setBNO08xReports();
      }
    }
    if (imuConnected) {
      handleIMU(boatStatus);
    }

    processIbusData();
    handleGPS(boatStatus);
    updateMagneticDeclination();
    handleModes(boatStatus);
    handleSystemChecks(boatStatus);
    handleArming(boatStatus);
    // --- 2. FAILSAFE CHECK ---
    // This check takes priority for motor control.
    bool explicit_failsafe_active = (boatStatus.rc.channels[CH1] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH2] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH3] > FAILSAFE_CHANNEL_THRESHOLD &&
                                     boatStatus.rc.channels[CH4] > FAILSAFE_CHANNEL_THRESHOLD);
    if (explicit_failsafe_active || (millis() - boatStatus.rc.last_update_ms > IBUS_TIMEOUT)) {
        if (explicit_failsafe_active) {
            if (boatStatus.failsafe.rc_failsafe_start_ms == 0) boatStatus.failsafe.rc_failsafe_start_ms = millis();
        } else {
            if (boatStatus.failsafe.rc_signal_lost_ms == 0) boatStatus.failsafe.rc_signal_lost_ms = millis();
        }

        voidMotors(); // Failsafe takes control of motors.
        unsigned long failsafe_duration = explicit_failsafe_active ?
            (millis() - boatStatus.failsafe.rc_failsafe_start_ms) : (millis() - boatStatus.failsafe.rc_signal_lost_ms);
        if (failsafe_duration > FAILSAFE_RTH_TRIGGER_MS) {
            if (boatStatus.mode.is_armed && savedLocations[0].isSet && !boatStatus.autopilot.rth_active && boatStatus.nav.has_gps_fix) {
                resetForNewTarget(boatStatus, 0, true);
            } else if (boatStatus.mode.is_armed && !boatStatus.nav.has_gps_fix && boatStatus.mode.current == MANUAL_MODE && !boatStatus.autopilot.rth_active) {
                handleCompassReturn(boatStatus);
            }
        }
        updateLights(); // Still update lights in failsafe.
        return; // Skip normal motor control logic.
    } else {
        // Reset failsafe timers if signal is good
        boatStatus.failsafe.rc_failsafe_start_ms = 0;
        boatStatus.failsafe.rc_signal_lost_ms = 0;
        boatStatus.failsafe.compass_return_heading = -1.0;
    }

    // --- 3. NORMAL OPERATION (if not in failsafe) ---
    if (millis() < 2000) return; // Short delay on boot to let systems stabilize

    handleManualPidTuning(boatStatus);
    handleDynamicPID(boatStatus);
    updateMotors(boatStatus);
    handleManualActuators(boatStatus);
    updateActuators();
    handleSettingsPersistence(boatStatus);
    // --- 4. FINAL UPDATES ---
    updateLights(); // Update lights based on normal operation state

    boatStatus.rc.prev_stick_ch1 = boatStatus.rc.channels[CH1];
    boatStatus.rc.prev_stick_ch2 = boatStatus.rc.channels[CH2];
    boatStatus.rc.prev_stick_ch5 = boatStatus.rc.channels[CH5];
  }
}

// ==========================================
// iBus Task (runs exclusively on Core 0)
// ==========================================
void ibus_task(void *pvParameters) {
  ibusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_PIN, -1);
  static uint8_t buffer[IBUS_PACKET_SIZE];
  static int bufferIndex = 0;
  uint16_t temp_channels[NUM_IBUS_CHANNELS];
  while (true) {
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

          portENTER_CRITICAL(&ibusMux);
          memcpy((void*)ibus_channel_data, (const void*)temp_channels, sizeof(temp_channels));
          last_ibus_packet_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
          portEXIT_CRITICAL(&ibusMux);
        }
        bufferIndex = 0;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// Web Server Task (runs exclusively on Core 0)
// ==========================================
void webserver_task(void *pvParameters) {
    setupWifi();
    setupWebServerRoutes();
    while(true) {
        server.handleClient();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


// ==============================
// Functions (running on Core 1)
// ==============================

void processIbusData() {
    portENTER_CRITICAL(&ibusMux);
    memcpy((void*)boatStatus.rc.channels, (const void*)ibus_channel_data, sizeof(ibus_channel_data));
    boatStatus.rc.last_update_ms = last_ibus_packet_ms;
    portEXIT_CRITICAL(&ibusMux);

    if (REVERSE_RC_THROTTLE) { boatStatus.rc.channels[CH3] = 3000 - boatStatus.rc.channels[CH3]; }
    if (REVERSE_RC_STEERING) { boatStatus.rc.channels[CH4] = 3000 - boatStatus.rc.channels[CH4]; }

    for(int i=0; i<NUM_IBUS_CHANNELS; ++i){
        boatStatus.rc.channels[i] = constrain(boatStatus.rc.channels[i], 1000, 2000);
    }
}

void handleArming(BoatStatus& status) {
  if (status.mode.is_armed) {
    return;
  }

  bool flicked = (status.rc.channels[CH5] > LIGHT_SWITCH_THRESHOLD && status.rc.prev_stick_ch5 <= LIGHT_SWITCH_THRESHOLD) ||
                 (status.rc.channels[CH5] < LIGHT_SWITCH_THRESHOLD && status.rc.prev_stick_ch5 >= LIGHT_SWITCH_THRESHOLD);
  if (flicked) {
    status.mode.is_armed = true;
    startBuzzerPattern(alertBuzzerPatterns[ALERT_ARMED], alertSettings[ALERT_ARMED]);
    startFlashPattern(alertLightFlashPatterns[ALERT_ARMED], alertSettings[ALERT_ARMED]);
  }
}

void updateMagneticDeclination() {
  if (gps.location.isValid() && gps.date.isValid()) {
    uint16_t current_year = gps.date.year();
    uint8_t year_for_calc;
    if (current_year < 2025) {
        year_for_calc = 25;
    } else {
        year_for_calc = current_year % 100;
    }

    boatStatus.nav.magnetic_declination = wmm.magneticDeclination(
        gps.location.lat(),
        gps.location.lng(),
        year_for_calc,
        gps.date.month(),
        gps.date.day()
    );
  }
}

void setBNO08xReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable linear acceleration");
  }
}

// =================================================================
// START OF CHANGE: Updated handleIMU from user-provided file
// =================================================================
void handleIMU(BoatStatus& status) {
  static unsigned long lastPredictTime = 0;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Compute IMU-derived values locally first
      uint8_t imu_accuracy = sensorValue.status & 0b00000011;
      float q_i = sensorValue.un.rotationVector.i;
      float q_j = sensorValue.un.rotationVector.j;
      float q_k = sensorValue.un.rotationVector.k;
      float q_real = sensorValue.un.rotationVector.real;
      float yaw_rad = atan2(2.0 * (q_i * q_j + q_real * q_k),
                            q_real * q_real + q_i * q_i - q_j * q_j - q_k * q_k);
      float yaw_deg = yaw_rad * (180.0 / PI);

      // Get magnetic declination for heading calculation
      float mag_decl = status.nav.magnetic_declination;
      
      float final_heading = yaw_deg + mag_decl;
      if(final_heading >= 360.0) final_heading -= 360.0;
      if(final_heading < 0.0) final_heading += 360.0;
      
      float sinr_cosp = 2 * (q_real * q_i + q_j * q_k);
      float cosr_cosp = 1 - 2 * (q_i * q_i + q_j * q_j);
      float roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

      float sinp = 2 * (q_real * q_j - q_k * q_i);
      float pitch_val;
      if (abs(sinp) >= 1)
        pitch_val = copysign(M_PI / 2, sinp) * (180.0 / PI);
      else
        pitch_val = asin(sinp) * (180.0 / PI);
      float pitch = pitch_val * -1.0;

      // Update nav fields under mutex
      portENTER_CRITICAL(&boatStatusMutex);
      status.nav.imu_accuracy = imu_accuracy;
      status.nav.heading = final_heading;
      status.nav.roll = roll;
      status.nav.pitch = pitch;
      portEXIT_CRITICAL(&boatStatusMutex);

    } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        if (lastPredictTime > 0) {
            float dt = (micros() - lastPredictTime) / 1000000.0f;
            float acc_x = sensorValue.un.linearAcceleration.x;
            float acc_y = sensorValue.un.linearAcceleration.y;

            // Read heading under mutex
            portENTER_CRITICAL(&boatStatusMutex);
            float heading = status.nav.heading;
            portEXIT_CRITICAL(&boatStatusMutex);

            // Run kalman_predict outside mutex
            float heading_rad = heading * PI / 180.0;
            float cos_h = cos(heading_rad);
            float sin_h = sin(heading_rad);
            float acc_n = acc_x * cos_h - acc_y * sin_h;
            float acc_e = acc_x * sin_h + acc_y * cos_h;
            kalman_predict(kf, dt, acc_n, acc_e);

            // Copy predicted values locally
            double pred_lat = kf.x[0];
            double pred_lon = kf.x[1];
            double vel_lat_mps = kf.x[2] * (M_PI / 180.0) * 6371000.0;
            double vel_lon_mps = kf.x[3] * (M_PI / 180.0) * 6371000.0 * cos(kf.x[0] * M_PI / 180.0);
            float pred_speed = sqrt(pow(vel_lat_mps, 2) + pow(vel_lon_mps, 2));

            // Update nav fields under mutex
            portENTER_CRITICAL(&boatStatusMutex);
            status.nav.latitude = pred_lat;
            status.nav.longitude = pred_lon;
            status.nav.speed_mps = pred_speed;
            portEXIT_CRITICAL(&boatStatusMutex);
        }
        lastPredictTime = micros();
    }
  }
}
// =================================================================
// END OF CHANGE: Updated handleIMU
// =================================================================

void handleManualPidTuning(BoatStatus& status) {
    if (status.mode.current == MANUAL_MODE || status.mode.current == LOCATION_SAVE_MODE) {
        steeringPID.Kp = map(status.rc.channels[CH9], 1000, 2000, 100, 700) / 100.0;
    }
}

void handleDynamicPID(BoatStatus& status) {
    if (status.mode.current != AUTOPILOT_MODE || !status.autopilot.engaged) {
        return;
    }

    float baseKp = map(status.rc.channels[CH9], 1000, 2000, 100, 800) / 100.0;

    float currentSpeed = status.nav.speed_mps;
    const float lowSpeedThreshold = 1.0;
    const float highSpeedThreshold = 2.5;
    const float highSpeedKpReduction = 0.5;
    if (currentSpeed <= lowSpeedThreshold) {
        steeringPID.Kp = baseKp;
    } else if (currentSpeed >= highSpeedThreshold) {
        steeringPID.Kp = baseKp * (1.0 - highSpeedKpReduction);
    } else {
        float factor = (currentSpeed - lowSpeedThreshold) / (highSpeedThreshold - lowSpeedThreshold);
        steeringPID.Kp = baseKp * (1.0 - (highSpeedKpReduction * factor));
    }
}

void setupWifi() {
  portENTER_CRITICAL(&nvsMutex); // <<< FIX: Protect NVS access
  preferences.begin("baitboat", true);
  String ap_ssid = preferences.getString("wifi_ssid", "Navigator-Pro");
  String ap_pass = preferences.getString("wifi_pass", "Nav@1234!");
  preferences.end();
  portEXIT_CRITICAL(&nvsMutex); // <<< FIX: Release NVS access

  if (ap_pass.length() > 0 && ap_pass.length() < 8) {
      ap_pass = "Nav@1234!";
  }

  strncpy(ssid, ap_ssid.c_str(), sizeof(ssid)-1);
  ssid[sizeof(ssid)-1] = '\0';

  WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());
}

void handleSettingsPersistence(BoatStatus& status) {
  if (status.persistence.settings_dirty && (millis() - status.persistence.last_change_ms > SETTINGS_SAVE_DEBOUNCE_MS)) {

    portENTER_CRITICAL(&nvsMutex); // <<< FIX: Protect NVS access
    preferences.begin("baitboat", false);
    preferences.putFloat("pid_p", steeringPID.Kp);
    preferences.putFloat("pid_i", steeringPID.Ki);
    preferences.putFloat("pid_d", steeringPID.Kd);
    preferences.putFloat("low_batt", status.mode.low_battery_threshold);
    preferences.putBool("lb_rth_en", status.autopilot.low_battery_rth_enabled);
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

    preferences.end();
    portEXIT_CRITICAL(&nvsMutex); // <<< FIX: Release NVS access

    status.persistence.settings_dirty = false;
  }
}

void loadAllSettings(){
    portENTER_CRITICAL(&nvsMutex); // <<< FIX: Protect NVS access
    preferences.begin("baitboat", true);
    boatStatus.mode.low_battery_threshold = preferences.getFloat("low_batt", 10.5);
    boatStatus.autopilot.low_battery_rth_enabled = preferences.getBool("lb_rth_en", false);
    const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
    char key_buffer[25];
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

    preferences.end();
    portEXIT_CRITICAL(&nvsMutex); // <<< FIX: Release NVS access
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
}

void setupLightsAndBuzzer() {
  byte initialByte = ~ALL_LIGHTS_MASK;
  initialByte &= ~BUZZER_MASK;

  digitalWrite(SR_RCLK, LOW);
  shiftOut(SR_SER, SR_SRCLK, MSBFIRST, initialByte);
  digitalWrite(SR_RCLK, HIGH);
}

void handleGPS(BoatStatus& status) {
  int chars_processed = 0;
  while (gpsSerial.available() && chars_processed++ < GPS_READ_MAX_CHARS) {
    gps.encode(gpsSerial.read());
  }

  bool hasGpsFixNow = gps.location.isValid() && gps.satellites.value() >= MIN_SATELLITES && gps.hdop.hdop() <= MAX_HDOP;
  
  // Compute local values first
  double current_lat = 0.0, current_lon = 0.0;
  float current_speed = 0.0;
  bool trigger_alert = false;
  bool trigger_mode_change = false;
  
  if (hasGpsFixNow) {
    unsigned long now = millis();
    bool had_gps_fix = false;
    
    portENTER_CRITICAL(&boatStatusMutex);
    had_gps_fix = status.nav.has_gps_fix;
    status.nav.last_gps_signal_ms = now;
    portEXIT_CRITICAL(&boatStatusMutex);
    
    if (!had_gps_fix) {
      // First fix initialization
      trigger_alert = true;
      kalman_init(kf, gps.location.lat(), gps.location.lng(), 2.0);
      current_lat = kf.x[0];
      current_lon = kf.x[1];
      current_speed = 0.0;
    } else {
      // Subsequent updates
      kalman_update(kf, gps.location.lat(), gps.location.lng());
      current_lat = kf.x[0];
      current_lon = kf.x[1];
      double vel_lat_mps = kf.x[2] * (M_PI / 180.0) * 6371000.0;
      double vel_lon_mps = kf.x[3] * (M_PI / 180.0) * 6371000.0 * cos(kf.x[0] * M_PI / 180.0);
      current_speed = sqrt(pow(vel_lat_mps, 2) + pow(vel_lon_mps, 2));
    }
  } else {
    unsigned long now = millis();
    unsigned long last_signal;
    bool had_gps_fix = false;
    BoatMode current_mode;
    
    portENTER_CRITICAL(&boatStatusMutex);
    last_signal = status.nav.last_gps_signal_ms;
    had_gps_fix = status.nav.has_gps_fix;
    current_mode = status.mode.current;
    portEXIT_CRITICAL(&boatStatusMutex);
    
    if (now - last_signal > GPS_VALIDITY_TIMEOUT && had_gps_fix) {
      if(current_mode == AUTOPILOT_MODE || current_mode == ANCHOR_MODE){
        trigger_mode_change = true;
      }
    }
    // Still update with potentially stale data
    current_lat = gps.location.lat();
    current_lon = gps.location.lng();
  }
  
  // Update status under mutex
  portENTER_CRITICAL(&boatStatusMutex);
  status.nav.has_gps_fix = hasGpsFixNow;
  status.nav.latitude = current_lat;
  status.nav.longitude = current_lon;
  if (hasGpsFixNow) {
    status.nav.speed_mps = current_speed;
  }
  if (trigger_mode_change) {
    status.mode.current = MANUAL_MODE;
    status.autopilot.engaged = false;
  }
  portEXIT_CRITICAL(&boatStatusMutex);
  
  // Trigger alerts and motors outside mutex
  if (trigger_alert) {
    startBuzzerPattern(alertBuzzerPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
    startFlashPattern(alertLightFlashPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
  }
  if (trigger_mode_change) {
    voidMotors();
  }
}

void handleModes(BoatStatus& status) {
  int ch7Value = status.rc.channels[CH7];
  status.mode.previous = status.mode.current;
  if (ch7Value <= MODE_MANUAL_MAX) {
      if(status.mode.current == ANCHOR_MODE) {
          // Allow switching out of anchor only via CH8
      } else {
          status.mode.current = MANUAL_MODE;
      }
  } else if (ch7Value >= MODE_AP_MIN && ch7Value <= MODE_AP_MAX) {
      status.mode.current = AUTOPILOT_MODE;
  } else if (ch7Value >= MODE_LOC_SAVE_MIN) {
      status.mode.current = LOCATION_SAVE_MODE;
  }

  bool ch8Activated = status.rc.channels[CH8] > RC_MID_POINT;

  // Anchor mode activation/deactivation via CH8 (only if currently in MANUAL)
  if (status.mode.previous == MANUAL_MODE && status.mode.current == MANUAL_MODE && ch8Activated) {
      status.mode.current = ANCHOR_MODE;
  } else if (status.mode.current == ANCHOR_MODE && !ch8Activated) {
      status.mode.current = MANUAL_MODE;
  }

  // Handle state transitions
  if (status.mode.current != status.mode.previous) {
    voidMotors();
    status.autopilot.engaged = false;
    status.autopilot.rth_active = false;
    status.autopilot.target_waypoint_index = -1;
    status.autopilot.arrival_state = AP_IDLE;
    currentRoute.status = ROUTE_INACTIVE;

    if (status.mode.current == ANCHOR_MODE && status.nav.has_gps_fix) {
        status.autopilot.anchor_lat = status.nav.latitude; // Use Kalman filtered position
        status.autopilot.anchor_lng = status.nav.longitude;
        // Reset PID integrals when entering anchor mode
        throttlePID.integral = 0;
        steeringPID.integral = 0;
    }

    for(int i = 0; i < NUM_ALERT_TYPES; i++) {
        alertBuzzerPatterns[i].active = false;
        alertLightFlashPatterns[i].active = false;
    }
  }

  // Handle mode-specific actions
  if (status.mode.current == LOCATION_SAVE_MODE) {
      handleLocationSaving(status);
  }
  else if (status.mode.current == AUTOPILOT_MODE) {
    if (currentRoute.status == ROUTE_ACTIVE || currentRoute.status == ROUTE_PAUSED) {
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
  int baseSpeed = 1650; // Slow forward

  int targetSpeedLeft = baseSpeed - steeringAdjust;
  int targetSpeedRight = baseSpeed + steeringAdjust;
  escLeft.writeMicroseconds(constrain(targetSpeedLeft, ESC_MIN_PULSE, ESC_MAX_PULSE));
  escRight.writeMicroseconds(constrain(targetSpeedRight, ESC_MIN_PULSE, ESC_MAX_PULSE));
}

// =================================================================
// START OF CHANGE: Robust Anchor Mode Implementation
// =================================================================
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
    if (abs(throttle - RC_MID_POINT) < stickDeadband) { throttle = RC_MID_POINT; }
    if (abs(steering - RC_MID_POINT) < stickDeadband) { steering = RC_MID_POINT; }

    int motorTrim = map(status.rc.channels[CH10], 1000, 2000, -50, 50);
    int steering_diff = (steering - RC_MID_POINT);
    targetSpeedLeft = throttle + steering_diff - motorTrim;
    targetSpeedRight = throttle - steering_diff + motorTrim;
  }
  // --- Robust Anchor Mode Logic ---
  else if (status.mode.current == ANCHOR_MODE && (status.nav.has_gps_fix || kf.x[0] != 0.0)) { // Use Kalman position even if GPS fix is momentarily lost

      // --- Tuning Parameter ---
      const float DRIFT_COMPENSATION_FACTOR = 150.0; // Proportional gain for drift. Increase to fight drift harder. Adjust this value!

      // 1. Get Position Error
      double distance = calculateDistance(status.nav.latitude, status.nav.longitude, status.autopilot.anchor_lat, status.autopilot.anchor_lng);
      double bearingToAnchor = calculateBearing(status.nav.latitude, status.nav.longitude, status.autopilot.anchor_lat, status.autopilot.anchor_lng);
      double courseErrorToAnchor = calculateCourseError(status, bearingToAnchor); // Error: target_bearing - current_heading

      // 2. Estimate Drift Velocity (using Kalman filter state)
      // Convert Kalman velocity (degrees/sec) to m/s
      double lat_rad = status.nav.latitude * M_PI / 180.0;
      double vel_lat_mps = kf.x[2] * (M_PI / 180.0) * 6371000.0;
      double vel_lon_mps = kf.x[3] * (M_PI / 180.0) * 6371000.0 * cos(lat_rad);

      // Rotate velocity into boat's frame (forward/sideways) - optional but useful for understanding
      // float heading_rad = status.nav.heading * PI / 180.0;
      // float cos_h = cos(heading_rad);
      // float sin_h = sin(heading_rad);
      // float vel_forward = vel_lat_mps * cos_h + vel_lon_mps * sin_h; // Velocity along boat's direction
      // float vel_sideways = -vel_lat_mps * sin_h + vel_lon_mps * cos_h;// Velocity perpendicular to boat's direction

      // Calculate total drift speed and direction *against* the drift
      double drift_speed_mps = sqrt(pow(vel_lat_mps, 2) + pow(vel_lon_mps, 2));
      double drift_direction_rad = atan2(vel_lon_mps, vel_lat_mps); // Direction the drift IS going (0=North, PI/2=East)
      double counter_drift_heading_rad = drift_direction_rad + M_PI; // Direction to point INTO the drift
      double counter_drift_heading_deg = fmod(counter_drift_heading_rad * 180.0 / M_PI + 360.0, 360.0);

      // 3. Calculate Control Outputs

      // --- Steering ---
      // Decide target heading: point towards anchor or into drift?
      // Simple approach: Prioritize fighting drift if significant, otherwise point to anchor.
      double target_heading_deg;
      const float MIN_DRIFT_SPEED_FOR_HEADING = 0.1; // m/s - threshold to prioritize drift heading
      if (drift_speed_mps > MIN_DRIFT_SPEED_FOR_HEADING) {
          target_heading_deg = counter_drift_heading_deg;
      } else {
          target_heading_deg = bearingToAnchor;
      }
      int steeringAdjust = calculateSteeringPID(status, target_heading_deg); // Steer towards the chosen target heading

      // --- Throttle ---
      // Combine position correction and drift correction
      int position_throttle_adjust = 0;
      if (distance > status.autopilot.anchor_radius) {
          // Only apply position throttle if outside radius
          position_throttle_adjust = calculateThrottlePID(status, distance);
      }
      // Proportional counter-drift throttle (always active to hold against drift)
      int drift_throttle_adjust = (int)(drift_speed_mps * DRIFT_COMPENSATION_FACTOR);
      drift_throttle_adjust = constrain(drift_throttle_adjust, 0, 300); // Limit counter-drift thrust

      int combined_throttle_adjust = position_throttle_adjust + drift_throttle_adjust;
      combined_throttle_adjust = constrain(combined_throttle_adjust, 0, 400); // Limit combined thrust

      // Decide forward/reverse based on anchor position relative to boat heading
      if (abs(courseErrorToAnchor) < 90) {
          // Anchor generally in front: Use combined forward thrust
          targetSpeedLeft  = RC_MID_POINT + combined_throttle_adjust - steeringAdjust;
          targetSpeedRight = RC_MID_POINT + combined_throttle_adjust + steeringAdjust;
      } else {
          // Anchor generally behind: Use combined reverse thrust, invert steering for reverse
          targetSpeedLeft  = RC_MID_POINT - combined_throttle_adjust + steeringAdjust;
          targetSpeedRight = RC_MID_POINT - combined_throttle_adjust - steeringAdjust;
      }

  }
  // =================================================================
  // END OF CHANGE: Robust Anchor Mode Implementation
  // =================================================================
  else if (status.mode.current == AUTOPILOT_MODE && status.autopilot.engaged && status.autopilot.target_waypoint_index != -1) {

    // --- Standard Autopilot Waypoint Navigation ---
    if (status.autopilot.arrival_state != AP_IDLE) {
      handleWaypointArrivalActions(status);
      if (status.autopilot.arrival_state == AP_BRAKING) {
          escLeft.writeMicroseconds(BRAKING_REVERSE_PULSE);
          escRight.writeMicroseconds(BRAKING_REVERSE_PULSE);
      }
      return; // Don't apply further motor commands if braking/arrived
    }

    double targetLat = savedLocations[status.autopilot.target_waypoint_index].lat;
    double targetLng = savedLocations[status.autopilot.target_waypoint_index].lng;
    double distance = calculateDistance(status.nav.latitude, status.nav.longitude, targetLat, targetLng);

    // Dynamic braking check
    float dynamic_braking_dist = max(MIN_BRAKING_DISTANCE, status.nav.speed_mps * BRAKING_FACTOR);
    if (distance < dynamic_braking_dist) {
      status.autopilot.arrival_time_ms = millis();
      status.autopilot.arrival_state = AP_BRAKING;
      status.autopilot.braking_start_speed = status.nav.speed_mps;
      // Write braking pulse immediately
      escLeft.writeMicroseconds(BRAKING_REVERSE_PULSE);
      escRight.writeMicroseconds(BRAKING_REVERSE_PULSE);
      return; // Exit motor update for this cycle
    }

    // If not braking, continue with normal navigation steering/throttle
    status.autopilot.arrival_state = AP_IDLE; // Ensure state is idle if not braking

    double courseError = abs(calculateCourseError(status, targetLat, targetLng));
    int steeringAdjust = calculateSteeringPID(status, targetLat, targetLng);

    // Aggressive Pivot Steering Logic (from previous implementation)
    if (courseError > 45.0) { // Pivot turn if error is large
        targetSpeedLeft = RC_MID_POINT + steeringAdjust;
        targetSpeedRight = RC_MID_POINT - steeringAdjust;
    } else { // Aggressive differential steering otherwise
        int baseSpeed = 2000; // Max forward throttle for autopilot
        if (courseError > 20.0) { // Slow down slightly for sharper turns
            baseSpeed = map(courseError, 20, 45, 2000, 1750);
        }
        int steeringEffect = steeringAdjust * 2; // Multiply effect for strong turning
        targetSpeedLeft = baseSpeed + steeringEffect;
        targetSpeedRight = baseSpeed - steeringEffect;
    }
  } else {
      // Default case if no other mode applies (e.g., LOCATION_SAVE mode, or unarmed)
      voidMotors();
  }

  // Apply motor smoothing and constraints
  smoothedLeft = MOTOR_SMOOTHING_ALPHA * targetSpeedLeft + (1 - MOTOR_SMOOTHING_ALPHA) * smoothedLeft;
  smoothedRight = MOTOR_SMOOTHING_ALPHA * targetSpeedRight + (1 - MOTOR_SMOOTHING_ALPHA) * smoothedRight;

  escLeft.writeMicroseconds(constrain(smoothedLeft, ESC_MIN_PULSE, ESC_MAX_PULSE));
  escRight.writeMicroseconds(constrain(smoothedRight, ESC_MIN_PULSE, ESC_MAX_PULSE));
}


void voidMotors() {
  escLeft.writeMicroseconds(RC_MID_POINT);
  escRight.writeMicroseconds(RC_MID_POINT);
  smoothedLeft = RC_MID_POINT;
  smoothedRight = RC_MID_POINT;
}

void handleManualActuators(BoatStatus& status) {
  if (status.mode.current != MANUAL_MODE) {
    return;
  }

  if (millis() - status.rc.last_update_ms > IBUS_TIMEOUT) {
      return; // No RC signal
  }

    unsigned long now = millis();
    // Left Hopper (CH1 Down)
    if (now - lastActuatorTime[0] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH1] < STICK_LOW_THRESHOLD && status.rc.prev_stick_ch1 >= STICK_LOW_THRESHOLD) {
        startActuator(actuators[0]);
        startBuzzerPattern(alertBuzzerPatterns[ALERT_LH_DROP], alertSettings[ALERT_LH_DROP]);
        startFlashPattern(alertLightFlashPatterns[ALERT_LH_DROP], alertSettings[ALERT_LH_DROP]);
        lastActuatorTime[0] = now;
    }
    // Right Hopper (CH1 Up)
    if (now - lastActuatorTime[1] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH1] > STICK_HIGH_THRESHOLD && status.rc.prev_stick_ch1 <= STICK_HIGH_THRESHOLD) {
        startActuator(actuators[1]);
        startBuzzerPattern(alertBuzzerPatterns[ALERT_RH_DROP], alertSettings[ALERT_RH_DROP]);
        startFlashPattern(alertLightFlashPatterns[ALERT_RH_DROP], alertSettings[ALERT_RH_DROP]);
        lastActuatorTime[1] = now;
    }
    // Left Hook (CH2 Down)
    if (now - lastActuatorTime[2] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH2] < STICK_LOW_THRESHOLD && status.rc.prev_stick_ch2 >= STICK_LOW_THRESHOLD) {
        startActuator(actuators[2]);
        startBuzzerPattern(alertBuzzerPatterns[ALERT_LK_REL], alertSettings[ALERT_LK_REL]);
        startFlashPattern(alertLightFlashPatterns[ALERT_LK_REL], alertSettings[ALERT_LK_REL]);
        lastActuatorTime[2] = now;
    }
    // Right Hook (CH2 Up)
    if (now - lastActuatorTime[3] >= ACTUATOR_COOLDOWN_MS && status.rc.channels[CH2] > STICK_HIGH_THRESHOLD && status.rc.prev_stick_ch2 <= STICK_HIGH_THRESHOLD) {
        startActuator(actuators[3]);
        startBuzzerPattern(alertBuzzerPatterns[ALERT_RK_REL], alertSettings[ALERT_RK_REL]);
        startFlashPattern(alertLightFlashPatterns[ALERT_RK_REL], alertSettings[ALERT_RK_REL]);
        lastActuatorTime[3] = now;
    }
    // Auxiliary (CH6 Switch) - Assuming momentary action isn't needed, just on/off
    if (status.rc.channels[CH6] > RC_MID_POINT && !actuators[4].active) {
        startActuator(actuators[4]); // Activate if switched on
    }
    else if (status.rc.channels[CH6] <= RC_MID_POINT && actuators[4].active) {
        actuators[4].active = false; // Deactivate if switched off
        digitalWrite(actuators[4].pin, actuators[4].inverted ? HIGH : LOW);
    }
}

void updateActuators() {
    unsigned long now = millis();
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        // Only turn off timed actuators (onTime > 0)
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
        if (status.vitals.battery_voltage < status.mode.low_battery_threshold && status.vitals.battery_voltage > 1.0) { // Check voltage > 1.0 to avoid false trigger on startup/disconnect
            if (!isLowBattery) {
                isLowBattery = true;
                lowBatteryStartTime = now;
            }
        } else {
            isLowBattery = false;
            lowBatteryStartTime = 0; // Reset timer if voltage recovers
            if (low_batt_alert_active) { // Stop alert if voltage recovers
                alertBuzzerPatterns[ALERT_LOW_BATTERY].active = false;
                alertLightFlashPatterns[ALERT_LOW_BATTERY].active = false;
                low_batt_alert_active = false;
            }
        }

        // Trigger alert/RTH only after debounce period
        if (isLowBattery && !low_batt_alert_active && (now - lowBatteryStartTime > LOW_BATT_DEBOUNCE_MS)) {
            if (status.autopilot.low_battery_rth_enabled && savedLocations[0].isSet && !status.autopilot.rth_active) {
                resetForNewTarget(status, 0, true); // Initiate RTH
            } else {
                // Start continuous alert if RTH not enabled/possible
                startBuzzerPattern(alertBuzzerPatterns[ALERT_LOW_BATTERY], alertSettings[ALERT_LOW_BATTERY]);
                startFlashPattern(alertLightFlashPatterns[ALERT_LOW_BATTERY], alertSettings[ALERT_LOW_BATTERY]);
            }
            low_batt_alert_active = true; // Mark alert as active
        }
    }

    // Update desired base light state based on CH5 switch
    desiredBaseLightState = (status.rc.channels[CH5] < LIGHT_SWITCH_THRESHOLD || status.mode.current == LOCATION_SAVE_MODE) ?
                            ALL_LIGHTS_MASK : 0x00; // Lights on if switch down OR in save mode
}

void updateSingleBuzzerPattern(BuzzerPatternControl& pattern) {
    if (!pattern.active) { pattern.currentlyOn = false; return; }
    unsigned long now = millis();
    if (now >= pattern.nextToggleTime) {
        if (!pattern.currentlyOn) { // If it was off, starting a new beep
            pattern.currentBeep++;
            // Check if finite pattern is complete
            if (pattern.currentBeep > pattern.totalBeeps && pattern.totalBeeps < 9999) { // 9999+ indicates continuous
                pattern.active = false;
                pattern.currentlyOn = false; return;
            }
        }
        // Toggle state
        pattern.currentlyOn = !pattern.currentlyOn;
        // Set next toggle time
        pattern.nextToggleTime = now + (pattern.currentlyOn ? pattern.beepDuration : pattern.pauseDuration);
    }
}

void manageAllBuzzerAlerts() {
    for(int i = 0; i < NUM_ALERT_TYPES; i++){
        updateSingleBuzzerPattern(alertBuzzerPatterns[i]);
    }
}

void updateSingleFlashPattern(FlashPatternControl& pattern) {
    if (!pattern.active) { pattern.currentlyOn = false; return; }
    unsigned long now = millis();
    if (now >= pattern.nextToggleTime) {
         if (!pattern.currentlyOn) { // If it was off, starting a new flash cycle
            pattern.currentToggle++;
            // Check if finite pattern is complete
            if (pattern.currentToggle > pattern.totalToggles && pattern.totalToggles < 9999) { // 9999+ indicates continuous
                pattern.active = false;
                pattern.currentlyOn = false; return;
            }
        }
        // Toggle state
        pattern.currentlyOn = !pattern.currentlyOn;
        // Set next toggle time
        pattern.nextToggleTime = now + (pattern.currentlyOn ? pattern.onDuration : pattern.offDuration);
    }
}

void manageAllLightFlashAlerts() {
    for(int i = 0; i < NUM_ALERT_TYPES; i++) {
        updateSingleFlashPattern(alertLightFlashPatterns[i]);
    }
}

void updateLights() {
  manageAllBuzzerAlerts();
  manageAllLightFlashAlerts();

  byte logicalOnState = desiredBaseLightState; // Start with the base state from the switch

  // Override with flashing patterns
  for (int i = 0; i < NUM_ALERT_TYPES; i++) {
    if (alertLightFlashPatterns[i].active) {
      if (alertLightFlashPatterns[i].currentlyOn) {
        logicalOnState |= alertLightFlashPatterns[i].flashMask; // Turn ON lights in the mask
      } else {
        logicalOnState &= ~alertLightFlashPatterns[i].flashMask; // Turn OFF lights in the mask
      }
    }
  }

  // Determine final buzzer state (any active pattern turns it on)
  bool isBuzzerOn = false;
  for (int i = 0; i < NUM_ALERT_TYPES; i++) {
    if (alertBuzzerPatterns[i].currentlyOn) {
      isBuzzerOn = true;
      break;
    }
  }

  // Combine light state and buzzer state for shift register
  // Note: Shift register logic seems inverted (0 = ON, 1 = OFF for lights)
  byte bitsToSend = ~logicalOnState; // Invert for lights
  if (isBuzzerOn) {
    bitsToSend |= BUZZER_MASK; // Buzzer ON (assuming 1 = ON for buzzer bit)
  } else {
    bitsToSend &= ~BUZZER_MASK; // Buzzer OFF
  }

  // Send to shift register only if state changed
  if (bitsToSend != lastShiftRegisterState) {
    digitalWrite(SR_RCLK, LOW);
    shiftOut(SR_SER, SR_SRCLK, MSBFIRST, bitsToSend);
    digitalWrite(SR_RCLK, HIGH);
    lastShiftRegisterState = bitsToSend;
  }
}


float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) { sum += analogRead(VOLTAGE_PIN); }
  float avgVoltage = (float)sum / VOLTAGE_SAMPLES;
  // Apply voltage divider formula and calibration
  return (avgVoltage * ADC_REF / ADC_RES) * ((R1 + R2) / R2) * CALIBRATION_FACTOR;
}

void startActuator(Actuator &actuator) {
  actuator.active = true;
  actuator.activatedAt = millis();
  digitalWrite(actuator.pin, actuator.inverted ? LOW : HIGH); // Set pin HIGH or LOW based on inversion
}

void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings) {
  pattern.active = true;
  pattern.startTime = millis();
  pattern.nextToggleTime = millis(); // Start immediately
  pattern.currentBeep = 0;
  pattern.totalBeeps = settings.beeps;
  pattern.beepDuration = settings.beepDuration;
  pattern.pauseDuration = settings.pauseDuration;
  pattern.currentlyOn = false; // Start in OFF state
}

void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings) {
    pattern.active = true;
    pattern.startTime = millis();
    pattern.nextToggleTime = millis(); // Start immediately
    pattern.currentToggle = 0;
    pattern.totalToggles = settings.flashes;
    pattern.onDuration = settings.flashOnDuration;
    pattern.offDuration = settings.flashOffDuration;
    pattern.flashMask = settings.flashMask;
    pattern.currentlyOn = false; // Start in OFF state
}