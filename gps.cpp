/*
 * Project: Bait Boat Control System (ESP32) - GPS Module
 * Description: Handles GPS sensor operations
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#include <Arduino.h>
#include "gps.h"
#include "kalman.h"

extern TinyGPSPlus gps;
extern WMM_Tinier wmm;
extern HardwareSerial gpsSerial;
extern KalmanFilter kf;
extern BuzzerPatternControl alertBuzzerPatterns[];
extern FlashPatternControl alertLightFlashPatterns[];
extern AlertSetting alertSettings[];

// Forward declarations from main
void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings);
void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings);
void voidMotors();

void setupGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  
  if (wmm.begin()) {
    // WMM_Tinier Initialized
  } else {
    Serial.println("WMM_Tinier Initialization failed!");
  }
}

void updateMagneticDeclination(BoatStatus& status) {
  if (gps.location.isValid() && gps.date.isValid()) {
    uint16_t current_year = gps.date.year();
    uint8_t year_for_calc;
    if (current_year < 2025) {
        year_for_calc = 25;
    } else {
        year_for_calc = current_year % 100;
    }

    status.nav.magnetic_declination = wmm.magneticDeclination(
        gps.location.lat(),
        gps.location.lng(),
        year_for_calc,
        gps.date.month(),
        gps.date.day()
    );
  }
}

void handleGPS(BoatStatus& status) {
  int chars_processed = 0;
  while (gpsSerial.available() && chars_processed++ < GPS_READ_MAX_CHARS) {
    gps.encode(gpsSerial.read());
  }

  bool hasGpsFixNow = gps.location.isValid() && gps.satellites.value() >= MIN_SATELLITES && gps.hdop.hdop() <= MAX_HDOP;
  if (hasGpsFixNow) {
    status.nav.last_gps_signal_ms = millis();
    if (!status.nav.has_gps_fix) {
      startBuzzerPattern(alertBuzzerPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
      startFlashPattern(alertLightFlashPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
      kalman_init(kf, gps.location.lat(), gps.location.lng(), 2.0);
    } else {
      kalman_update(kf, gps.location.lat(), gps.location.lng());
      status.nav.latitude = kf.x[0];
      status.nav.longitude = kf.x[1];
      double vel_lat_mps = kf.x[2] * (M_PI / 180.0) * 6371000.0;
      double vel_lon_mps = kf.x[3] * (M_PI / 180.0) * 6371000.0 * cos(kf.x[0] * M_PI / 180.0);
      status.nav.speed_mps = sqrt(pow(vel_lat_mps, 2) + pow(vel_lon_mps, 2));
    }
  } else {
    if (millis() - status.nav.last_gps_signal_ms > GPS_VALIDITY_TIMEOUT && status.nav.has_gps_fix) {
      if(status.mode.current == AUTOPILOT_MODE || status.mode.current == ANCHOR_MODE){
        status.mode.current = MANUAL_MODE;
        status.autopilot.engaged = false;
        voidMotors();
      }
    }
  }
  status.nav.has_gps_fix = hasGpsFixNow;
  if (!hasGpsFixNow) {
      status.nav.latitude = gps.location.lat(); // Still update with potentially stale data if needed elsewhere
      status.nav.longitude = gps.location.lng();
      // Speed comes from Kalman filter, which continues predicting
  }
}
