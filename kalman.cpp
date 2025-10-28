#include "kalman.h"
#include <math.h>
#include <string.h>

// Earth radius in meters
const double EARTH_RADIUS = 6371000.0;

// Initialize the Kalman filter
void kalman_init(KalmanFilter& kf, double lat, double lon, double gps_noise) {
    // Initial state
    kf.x[0] = lat;
    kf.x[1] = lon;
    kf.x[2] = 0.0; // Initial velocity lat
    kf.x[3] = 0.0; // Initial velocity lon

    // Initial state covariance (start with some uncertainty)
    memset(kf.P, 0, sizeof(kf.P));
    kf.P[0][0] = 1.0;
    kf.P[1][1] = 1.0;
    kf.P[2][2] = 1.0;
    kf.P[3][3] = 1.0;

    // Process noise (tune these values)
    // Represents uncertainty in the model (e.g., from waves, wind)
    double acc_noise = 0.1;
    memset(kf.Q, 0, sizeof(kf.Q));
    kf.Q[0][0] = 0.25 * pow(acc_noise, 2);
    kf.Q[1][1] = 0.25 * pow(acc_noise, 2);
    kf.Q[2][2] = pow(acc_noise, 2);
    kf.Q[3][3] = pow(acc_noise, 2);

    // Measurement noise (from GPS)
    memset(kf.R, 0, sizeof(kf.R));
    kf.R[0][0] = gps_noise;
    kf.R[1][1] = gps_noise;
}

// Prediction step
void kalman_predict(KalmanFilter& kf, double dt, double acc_n, double acc_e) {
    // State transition matrix
    double F[4][4] = {
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    
    // Convert accelerations (m/s^2) to changes in lat/lon velocity
    double dv_lat = acc_n * dt * (180.0 / M_PI) / EARTH_RADIUS;
    double dv_lon = acc_e * dt * (180.0 / M_PI) / (EARTH_RADIUS * cos(kf.x[0] * M_PI / 180.0));

    // Predict state
    kf.x[0] += kf.x[2] * dt;
    kf.x[1] += kf.x[3] * dt;
    kf.x[2] += dv_lat;
    kf.x[3] += dv_lon;

    // Predict covariance: P = F * P * F' + Q
    double P_pred[4][4];
    double FP[4][4];
    // FP = F * P
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            FP[i][j] = F[i][0]*kf.P[0][j] + F[i][1]*kf.P[1][j] + F[i][2]*kf.P[2][j] + F[i][3]*kf.P[3][j];
        }
    }
    // P_pred = FP * F' + Q
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            P_pred[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2] + FP[i][3]*F[j][3] + kf.Q[i][j];
        }
    }
    memcpy(kf.P, P_pred, sizeof(kf.P));
}

// Update step
void kalman_update(KalmanFilter& kf, double lat, double lon) {
    // Measurement residual (y = z - H*x)
    double y[2] = {lat - kf.x[0], lon - kf.x[1]};

    // Residual covariance (S = H*P*H' + R)
    // H is [[1,0,0,0],[0,1,0,0]], so S is just the top-left 2x2 of P + R
    double S[2][2] = {
        {kf.P[0][0] + kf.R[0][0], kf.P[0][1] + kf.R[0][1]},
        {kf.P[1][0] + kf.R[1][0], kf.P[1][1] + kf.R[1][1]}
    };

    // Inverse of S
    double S_inv_det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (abs(S_inv_det) < 1e-6) {
        return; // Avoid division by zero
    }
    double S_inv[2][2] = {
        {S[1][1] / S_inv_det, -S[0][1] / S_inv_det},
        {-S[1][0] / S_inv_det, S[0][0] / S_inv_det}
    };
    
    // Kalman gain (K = P*H' * S_inv)
    double K[4][2];
    for(int i=0; i<4; ++i) {
        K[i][0] = kf.P[i][0]*S_inv[0][0] + kf.P[i][1]*S_inv[1][0];
        K[i][1] = kf.P[i][0]*S_inv[0][1] + kf.P[i][1]*S_inv[1][1];
    }
    
    // Update state (x = x + K*y)
    kf.x[0] += K[0][0]*y[0] + K[0][1]*y[1];
    kf.x[1] += K[1][0]*y[0] + K[1][1]*y[1];
    kf.x[2] += K[2][0]*y[0] + K[2][1]*y[1];
    kf.x[3] += K[3][0]*y[0] + K[3][1]*y[1];

    // Update covariance (P = (I - K*H)*P)
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