/*
 * Project: Navigator_Pro_New Control System (ESP32) - GPS Module
 * Architecture: High-Speed (5Hz) GPS Authority with Auto-Config, ACK Verification & SBAS Detection.
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

extern SemaphoreHandle_t kalmanMutex;
extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t boatStatusMutex; 

// SBAS/WAAS Extractors: Field 6 of the GGA NMEA sentence (0=No Fix, 1=Standard, 2=SBAS/DGPS)
TinyGPSCustom fixQualityGNGGA(gps, "GNGGA", 6); 
TinyGPSCustom fixQualityGPGGA(gps, "GPGGA", 6);

void startBuzzerPattern(BuzzerPatternControl& pattern, const AlertSetting& settings);
void startFlashPattern(FlashPatternControl& pattern, const AlertSetting& settings);
void voidMotors();

// Upgraded UBX Sender with Acknowledgment (ACK) Parsing
void sendUBX(uint8_t msgClass, uint8_t msgID, uint8_t* payload, uint16_t length) {
    while (gpsSerial.available()) gpsSerial.read();

    gpsSerial.write(0xB5);
    gpsSerial.write(0x62);
    gpsSerial.write(msgClass);
    gpsSerial.write(msgID);
    gpsSerial.write(length & 0xFF);
    gpsSerial.write((length >> 8) & 0xFF);
    
    uint8_t ckA = 0, ckB = 0;
    ckA += msgClass; ckB += ckA;
    ckA += msgID; ckB += ckA;
    ckA += (length & 0xFF); ckB += ckA;
    ckA += ((length >> 8) & 0xFF); ckB += ckA;
    
    for (uint16_t i = 0; i < length; i++) {
        gpsSerial.write(payload[i]);
        ckA += payload[i];
        ckB += ckA;
    }
    gpsSerial.write(ckA);
    gpsSerial.write(ckB);

    Serial.print("Sending UBX [");
    Serial.print(msgClass, HEX); Serial.print(" "); Serial.print(msgID, HEX);
    Serial.print("]... ");

    unsigned long startTime = millis();
    uint8_t ackBuf[10];
    int ackIndex = 0;
    bool acknowledged = false;
    bool nak_received = false;

    while (millis() - startTime < 500) {
        if (gpsSerial.available()) {
            uint8_t c = gpsSerial.read();
            
            if (ackIndex == 0 && c == 0xB5) ackBuf[ackIndex++] = c;
            else if (ackIndex == 1 && c == 0x62) ackBuf[ackIndex++] = c;
            else if (ackIndex == 2 && c == 0x05) ackBuf[ackIndex++] = c; 
            else if (ackIndex == 3 && (c == 0x00 || c == 0x01)) ackBuf[ackIndex++] = c; 
            else if (ackIndex > 3 && ackIndex < 10) {
                ackBuf[ackIndex++] = c;
                if (ackIndex == 10) {
                    if (ackBuf[6] == msgClass && ackBuf[7] == msgID) {
                        if (ackBuf[3] == 0x01) {
                            Serial.println("ACK RECEIVED (Success)");
                            acknowledged = true;
                            break;
                        } else {
                            Serial.println("NAK RECEIVED (Rejected)");
                            nak_received = true;
                            break;
                        }
                    }
                    ackIndex = 0; 
                }
            } else {
                ackIndex = 0; 
            }
        }
    }
    
    if (!acknowledged && !nak_received) {
         Serial.println("TIMEOUT (No response)");
    }
    delay(20); 
}

void configureGPSFor5Hz() {
    Serial.println("\n--- Starting GPS Auto-Configuration ---");

    // Force Measurement Rate to 200ms (5Hz)
    uint8_t ratePayload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
    sendUBX(0x06, 0x08, ratePayload, sizeof(ratePayload));

    // Disable GLL NMEA Spam (F0 01)
    uint8_t gllPayload[] = {0xF0, 0x01, 0x00};
    sendUBX(0x06, 0x01, gllPayload, sizeof(gllPayload));

    // Disable GSA NMEA Spam (F0 02)
    uint8_t gsaPayload[] = {0xF0, 0x02, 0x00};
    sendUBX(0x06, 0x01, gsaPayload, sizeof(gsaPayload));

    // Disable GSV NMEA Spam (F0 03)
    uint8_t gsvPayload[] = {0xF0, 0x03, 0x00};
    sendUBX(0x06, 0x01, gsvPayload, sizeof(gsvPayload));

    // Disable VTG NMEA Spam (F0 05)
    uint8_t vtgPayload[] = {0xF0, 0x05, 0x00};
    sendUBX(0x06, 0x01, vtgPayload, sizeof(vtgPayload));

    Serial.println("--- GPS Configuration Complete ---\n");
}

void setupGPS() {
  gpsSerial.setRxBufferSize(4096);
  
  // Safe, stable 38400 baud. 
  gpsSerial.begin(38400, SERIAL_8N1, 16, 17);
  
  // Give the M10N exactly one second to fully boot up before sending commands
  delay(1000); 
  configureGPSFor5Hz();
  
  if (!wmm.begin()) {
    Serial.println("WMM_Tinier Initialization failed!");
  }
}

void updateMagneticDeclination(BoatStatus& status) {
  if (gps.location.isValid() && gps.date.isValid() && gps.date.year() > 2024) {
    uint8_t current_year = gps.date.year() % 100; 
    uint8_t current_month = gps.date.month();
    uint8_t current_day = gps.date.day();
    
    float dec = wmm.magneticDeclination(gps.location.lat(), gps.location.lng(), current_year, current_month, current_day);
    
    if (xSemaphoreTake(boatStatusMutex, 10) == pdTRUE) {
        status.nav.magnetic_declination = dec;
        xSemaphoreGive(boatStatusMutex);
    }
  }
}

void handleGPS(BoatStatus& status) {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    status.nav.last_gps_signal_ms = millis();
    
    static unsigned long last_wmm_update = 0;
    static bool wmm_calculated = false;
    if (!wmm_calculated || millis() - last_wmm_update > 60000) { 
        updateMagneticDeclination(status);
        if (status.nav.magnetic_declination != 0.0) {
            wmm_calculated = true;
        }
        last_wmm_update = millis();
    }

    if (gps.satellites.value() >= MIN_SATELLITES && gps.hdop.hdop() <= MAX_HDOP) {
      status.nav.has_gps_fix = true;
    } else {
      status.nav.has_gps_fix = false;
    }

    if (xSemaphoreTake(boatStatusMutex, 10) == pdTRUE) {
        status.nav.latitude = gps.location.lat();
        status.nav.longitude = gps.location.lng();
        status.nav.speed_mps = gps.speed.mps();
        
        // --- NEW: SBAS/WAAS DETECTION ---
        status.nav.sbas_active = false; 
        if (fixQualityGNGGA.isValid() && atoi(fixQualityGNGGA.value()) == 2) {
            status.nav.sbas_active = true;
        } else if (fixQualityGPGGA.isValid() && atoi(fixQualityGPGGA.value()) == 2) {
            status.nav.sbas_active = true;
        }

        xSemaphoreGive(boatStatusMutex);
    }

    if (!isKalmanInitialized) { 
      if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          startBuzzerPattern(alertBuzzerPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
          startFlashPattern(alertLightFlashPatterns[ALERT_GPS_FIX], alertSettings[ALERT_GPS_FIX]);
          xSemaphoreGive(dataMutex);
      }
      
      if (xSemaphoreTake(kalmanMutex, 10) == pdTRUE) {
          kalman_init(kf, gps.location.lat(), gps.location.lng(), 2.0);
          isKalmanInitialized = true; 
          xSemaphoreGive(kalmanMutex);
      }
    }
  } else {
    if (millis() - status.nav.last_gps_signal_ms > GPS_VALIDITY_TIMEOUT && status.nav.has_gps_fix) {
        status.nav.has_gps_fix = false;
        
        if (status.mode.current == AUTOPILOT_MODE || status.mode.current == ANCHOR_MODE) {
            status.autopilot.engaged = false;
            status.autopilot.rth_active = false;
            status.autopilot.target_waypoint_index = -1;
            status.mode.current = MANUAL_MODE;
            voidMotors();
        }
    }
  }
}