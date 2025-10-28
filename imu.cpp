/*
 * Project: Bait Boat Control System (ESP32) - IMU Module
 * Description: Handles BNO08x IMU sensor operations
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#include <Arduino.h>
#include "imu.h"
#include "kalman.h"
#include <Wire.h>

extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;
extern bool imuConnected;
extern KalmanFilter kf;

bool setupIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno08x.begin_I2C(BNO08X_I2C_ADDRESS)) {
      imuConnected = false;
      Serial.println("BNO08x connection failed.");
      return false;
  } else {
      imuConnected = true;
      setBNO08xReports();
      return true;
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

void handleIMU(BoatStatus& status) {
  static unsigned long lastPredictTime = 0;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      status.nav.imu_accuracy = sensorValue.status & 0b00000011;
      float q_i = sensorValue.un.rotationVector.i;
      float q_j = sensorValue.un.rotationVector.j;
      float q_k = sensorValue.un.rotationVector.k;
      float q_real = sensorValue.un.rotationVector.real;
      float yaw_rad = atan2(2.0 * (q_i * q_j + q_real * q_k),
                            q_real * q_real + q_i * q_i - q_j * q_j - q_k * q_k);
      float yaw_deg = yaw_rad * (180.0 / PI);

      // --- FIXED: Removed yaw_deg = 90.0 - yaw_deg; to correct 90° offset ---
      // If mounting requires rotation, add a prefs-based offset instead, e.g.:
      // yaw_deg += preferences.getFloat("heading_offset", 0.0);

      float final_heading = yaw_deg + status.nav.magnetic_declination;
      if(final_heading >= 360.0) final_heading -= 360.0;
      if(final_heading < 0.0) final_heading += 360.0;
      status.nav.heading = final_heading;
      float sinr_cosp = 2 * (q_real * q_i + q_j * q_k);
      float cosr_cosp = 1 - 2 * (q_i * q_i + q_j * q_j);
      status.nav.roll = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

      float sinp = 2 * (q_real * q_j - q_k * q_i);
      float pitch_val;
      if (abs(sinp) >= 1)
        pitch_val = copysign(M_PI / 2, sinp) * (180.0 / PI);
      else
        pitch_val = asin(sinp) * (180.0 / PI);
      // --- User's Transformation for Pitch ---
      status.nav.pitch = pitch_val * -1.0;
      // --- End Transformation ---

    } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        // <<< FIX: Removed 'status.nav.has_gps_fix' check.
        // Prediction should run even without GPS (dead reckoning).
        if (lastPredictTime > 0) {
            float dt = (micros() - lastPredictTime) / 1000000.0f;
            float acc_x = sensorValue.un.linearAcceleration.x;
            float acc_y = sensorValue.un.linearAcceleration.y;

            // Use the final calculated heading (which includes user transformations)
            // to rotate accelerations for the Kalman filter.
            float heading_rad = status.nav.heading * PI / 180.0;
            float cos_h = cos(heading_rad);
            float sin_h = sin(heading_rad);
            float acc_n = acc_x * cos_h - acc_y * sin_h;
            float acc_e = acc_x * sin_h + acc_y * cos_h;
            kalman_predict(kf, dt, acc_n, acc_e);

            status.nav.latitude = kf.x[0];
            status.nav.longitude = kf.x[1];
            double vel_lat_mps = kf.x[2] * (M_PI / 180.0) * 6371000.0;
            double vel_lon_mps = kf.x[3] * (M_PI / 180.0) * 6371000.0 * cos(kf.x[0] * M_PI / 180.0);
            status.nav.speed_mps = sqrt(pow(vel_lat_mps, 2) + pow(vel_lon_mps, 2));
        }
        lastPredictTime = micros();
    }
  }
}
