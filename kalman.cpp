/*
 * Project: Bait Boat Control System (ESP32) - Kalman Filter
 * Description: A simple 2D Kalman filter for fusing GPS and IMU data.
 */

#include "kalman.h"
#include <math.h>
#include <string.h>

// Define the global initialization flag
bool isKalmanInitialized = false;

// Earth radius in meters
const double EARTH_RADIUS = 6371000.0;

// Initialize the Kalman filter
void kalman_init(KalmanFilter& kf, double lat, double lon, double gps_noise_m) {
    // Initial state
    kf.x[0] = lat;
    kf.x[1] = lon;
    kf.x[2] = 0.0; // Initial velocity lat (deg/s)
    kf.x[3] = 0.0; // Initial velocity lon (deg/s)

    // Convert meters to degrees for the matrix math
    double m_to_deg = 1.0 / 111320.0;

    // Initial state covariance 
    memset(kf.P, 0, sizeof(kf.P));
    double init_variance = pow(5.0 * m_to_deg, 2); // 5 meter initial uncertainty
    kf.P[0][0] = init_variance;
    kf.P[1][1] = init_variance;
    kf.P[2][2] = init_variance;
    kf.P[3][3] = init_variance;

    // Process noise (IMU uncertainty - scaled to degrees)
    double acc_noise_deg = 0.2 * m_to_deg; 
    memset(kf.Q, 0, sizeof(kf.Q));
    kf.Q[0][0] = 0.25 * pow(acc_noise_deg, 2);
    kf.Q[1][1] = 0.25 * pow(acc_noise_deg, 2);
    kf.Q[2][2] = pow(acc_noise_deg, 2);
    kf.Q[3][3] = pow(acc_noise_deg, 2);

    // Measurement noise (GPS uncertainty - scaled to degrees)
    double gps_noise_deg = gps_noise_m * m_to_deg;
    memset(kf.R, 0, sizeof(kf.R));
    kf.R[0][0] = pow(gps_noise_deg, 2); 
    kf.R[1][1] = pow(gps_noise_deg, 2);
}

// Prediction step
void kalman_predict(KalmanFilter& kf, double dt, double acc_n, double acc_e) {
    // THE FIX: "Water Drag"
    // Blunts mathematical velocity every tick to prevent indoor GPS runaway.
    double drag_factor = 0.90; 

    double F[4][4] = {
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, drag_factor, 0}, 
        {0, 0, 0, drag_factor}  
    };
    
    double cos_lat = cos(kf.x[0] * M_PI / 180.0);
    if (abs(cos_lat) < 0.01) {
        cos_lat = (cos_lat < 0) ? -0.01 : 0.01;
    }

    double dv_lat = acc_n * dt * (180.0 / M_PI) / EARTH_RADIUS;
    double dv_lon = acc_e * dt * (180.0 / M_PI) / (EARTH_RADIUS * cos_lat);

    kf.x[0] += kf.x[2] * dt;
    kf.x[1] += kf.x[3] * dt;
    kf.x[2] = (kf.x[2] * drag_factor) + dv_lat;
    kf.x[3] = (kf.x[3] * drag_factor) + dv_lon;

    // THE FIX: Max Speed Clamp
    double max_vel_deg = 5.0 * (180.0 / M_PI) / EARTH_RADIUS;
    if (kf.x[2] > max_vel_deg) kf.x[2] = max_vel_deg;
    if (kf.x[2] < -max_vel_deg) kf.x[2] = -max_vel_deg;
    if (kf.x[3] > max_vel_deg) kf.x[3] = max_vel_deg;
    if (kf.x[3] < -max_vel_deg) kf.x[3] = -max_vel_deg;

    double P_pred[4][4];
    double FP[4][4];
    
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            FP[i][j] = F[i][0]*kf.P[0][j] + F[i][1]*kf.P[1][j] + F[i][2]*kf.P[2][j] + F[i][3]*kf.P[3][j];
        }
    }
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            P_pred[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2] + FP[i][3]*F[j][3] + kf.Q[i][j];
        }
    }
    memcpy(kf.P, P_pred, sizeof(kf.P));
}

// Update step
void kalman_update(KalmanFilter& kf, double lat, double lon) {
    double y[2] = {lat - kf.x[0], lon - kf.x[1]};

    double S[2][2] = {
        {kf.P[0][0] + kf.R[0][0], kf.P[0][1] + kf.R[0][1]},
        {kf.P[1][0] + kf.R[1][0], kf.P[1][1] + kf.R[1][1]}
    };

    double S_inv_det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    
    if (abs(S_inv_det) < 1e-18) { 
        return; 
    }
    double S_inv[2][2] = {
        {S[1][1] / S_inv_det, -S[0][1] / S_inv_det},
        {-S[1][0] / S_inv_det, S[0][0] / S_inv_det}
    };
    
    double K[4][2];
    for(int i=0; i<4; ++i) {
        K[i][0] = kf.P[i][0]*S_inv[0][0] + kf.P[i][1]*S_inv[1][0];
        K[i][1] = kf.P[i][0]*S_inv[0][1] + kf.P[i][1]*S_inv[1][1];
    }
    
    kf.x[0] += K[0][0]*y[0] + K[0][1]*y[1];
    kf.x[1] += K[1][0]*y[0] + K[1][1]*y[1];
    
    // Apply the drag factor to the coordinate update jumps as well
    kf.x[2] = (kf.x[2] + K[2][0]*y[0] + K[2][1]*y[1]) * 0.90;
    kf.x[3] = (kf.x[3] + K[3][0]*y[0] + K[3][1]*y[1]) * 0.90;

    double I_KH[4][4];
    double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            I_KH[i][j] = I[i][j] - (K[i][0] * (j==0) + K[i][1] * (j==1));
        }
    }

    double P_new[4][4];
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            P_new[i][j] = 0;
            for(int k=0; k<4; ++k) {
                P_new[i][j] += I_KH[i][k] * kf.P[k][j];
            }
        }
    }
    memcpy(kf.P, P_new, sizeof(kf.P));
}