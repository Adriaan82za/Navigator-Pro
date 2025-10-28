#ifndef KALMAN_H
#define KALMAN_H

// A simple 2D Kalman filter for fusing GPS and IMU data.
// State vector: [latitude, longitude, velocity_lat, velocity_lon]

struct KalmanFilter {
    // State vector [lat, lon, vel_lat, vel_lon]
    double x[4];
    
    // State covariance matrix
    double P[4][4];
    
    // Process noise covariance matrix
    double Q[4][4];
    
    // Measurement noise covariance matrix
    double R[2][2];
};

void kalman_init(KalmanFilter& kf, double lat, double lon, double gps_noise);
void kalman_predict(KalmanFilter& kf, double dt, double acc_n, double acc_e);
void kalman_update(KalmanFilter& kf, double lat, double lon);

#endif