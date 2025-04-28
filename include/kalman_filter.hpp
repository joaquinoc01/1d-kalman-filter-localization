#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(const Eigen::VectorXf& initial_state,
                 const Eigen::MatrixXf& error_covariance,
                 const Eigen::MatrixXf& process_noise,
                 const Eigen::MatrixXf& measurement_noise);

    // Interface function
    Eigen::VectorXf predictAndUpdate(const Eigen::VectorXf& prev_state, const Eigen::VectorXf& measurement, float control_input);

    // Getter function
    Eigen::Vector2f getInitialState() const {return state_;}
    Eigen::Matrix2f getCovariance() const {return P_;}
    
    // Add noise to real measurements
    float getNoisyMeasurement(float true_measurement, float noise_stddev);

private:
    Eigen::VectorXf state_; // Initial guess of the state
    Eigen::MatrixXf P_;     // error covariance
    Eigen::MatrixXf R_;     // process noise covariance
    Eigen::MatrixXf Q_;     // measurement noise covariance

    // A is the State Transition Matrix (2x2)
    // It defines how the previous state affects the current state
    // [1  dt]
    // [0   1]
    Eigen::Matrix2f A_;

    // B is the Control Input Matrix (2x1)
    // Defines how control input (e.g. acceleration) affects the state
    // [0.5 * dt^2]
    // [     dt    ]
    Eigen::Vector2f B_;

    // C is the Measurement Matrix (1x2)
    // Defines how the state maps to the measured value (e.g., position only)
    // [1 0] means we only observe position, not velocity
    Eigen::RowVector2f C_;

    // Helper functions
    Eigen::Vector2f predictState(const Eigen::VectorXf& prev_state, float control_input);
    Eigen::MatrixXf predictUncertainty();
    Eigen::MatrixXf computeKalmanGain(const Eigen::MatrixXf& predicted_cov);
    Eigen::VectorXf updateStep(const Eigen::VectorXf& predicted_state, const Eigen::MatrixXf& kalman_gain, const Eigen::VectorXf& measurement);
    Eigen::MatrixXf updateErrorCovariance(const Eigen::MatrixXf& predicted_cov, const Eigen::MatrixXf& kalman_gain);
};