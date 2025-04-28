#include <iostream>
#include <random>

#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(const Eigen::VectorXf& initial_state, const Eigen::MatrixXf& error_covariance, const Eigen::MatrixXf& process_noise, const Eigen::MatrixXf& measurement_noise)
: state_(initial_state), P_(error_covariance), R_(process_noise), Q_(measurement_noise) {
    float dt{0.1f}; // Time step (e.g., 100 ms)

    // Resize matrices
    A_ = Eigen::Matrix2f(); // 2x2 for state: [pos, vel]
    B_ = Eigen::Vector2f(); // 2x1 for control input: [acceleration]
    C_ = Eigen::RowVector2f(); // 1x2 for measurement: [position]

    // Initialize transition matrix A
    A_ << 1.0f, dt,
          0.0f, 1.0f;

    // Control input model B
    B_ << 0.5f * dt * dt, dt;

    // Measurement model C (we only measure position)
    C_ << 1.0f, 0.0f;

    // State vector [pos, vel] — initialized with position only
    state_ = initial_state;

    // Error covariance matrix (uncertainty in the state)
    float initial_belief{100.0f};
    P_ = Eigen::Matrix2f::Identity() * initial_belief;

    // Process noise (uncertainty in the model)
    R_ = Eigen::Matrix2f::Identity() * process_noise;

    // Measurement noise (uncertainty in sensor)
    Q_ = Eigen::Matrix<float, 1, 1>::Identity() * measurement_noise;
}

template<typename T>
void printVector(const T& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        std::cout << vec[i] << " ";
    }
    std::cout << std::endl;
}

float KalmanFilter::getNoisyMeasurement(float true_measurement, float noise_stddev) {
    // Generate Gaussian noise
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise_distribution(0.0f, noise_stddev);
    
    // Add noise to the true measurement
    return true_measurement + noise_distribution(gen);
}

Eigen::Vector2f KalmanFilter::predictState(const Eigen::VectorXf& prev_state, float control_input)
{
    // μ̄_t = A_t * μ_{t-1} + B_t * u_t
    // Predict the state at time t, given the previous state estimate and control input
    return A_ * prev_state + B_ * control_input;
}

Eigen::MatrixXf KalmanFilter::predictUncertainty()
{
    // Σ̄_t = A * Σ_{t-1} * A^T + R
    // Predict the error covariance at time t
    return A_ * P_ * A_.transpose() + R_;
}

Eigen::MatrixXf KalmanFilter::computeKalmanGain(const Eigen::MatrixXf& predicted_cov)
{
    // K_t = Σ̄_t * C_t^T * (C_t * Σ̄_t * C_t^T + Q_t)^-1
    // Compute the Kalman Gain, which determines the weight of the measurements in the state update
    Eigen::MatrixXf S {C_ * predicted_cov * C_.transpose() + Q_};
    return predicted_cov * C_.transpose() * S.inverse();
}

Eigen::VectorXf KalmanFilter::updateStep(const Eigen::VectorXf& predicted_state, const Eigen::MatrixXf& kalman_gain, const Eigen::VectorXf& measurement)
{
    // \mu_t = \mū_t + K_t * (z_t − C_t * \mū_t)
    // Updates the state estimate using the Kalman gain and the sensor measurement
    return predicted_state + kalman_gain * (measurement - C_ * predicted_state);
}

Eigen::MatrixXf KalmanFilter::updateErrorCovariance(const Eigen::MatrixXf& predicted_cov, const Eigen::MatrixXf& kalman_gain)
{
    // Σ_t = (I - K_t * C_t) * Σ̄_t
    // Updates the error covariance based on the Kalman gain and predicted uncertainty
    Eigen::MatrixXf I {Eigen::MatrixXf::Identity(P_.rows(), P_.cols())};
    return (I - kalman_gain * C_) * predicted_cov;
}

Eigen::VectorXf KalmanFilter::predictAndUpdate(const Eigen::VectorXf& prev_state, const Eigen::VectorXf& measurement, float control_input)
{
    // Step 1: Prediction 
    Eigen::Vector2f predicted_state_estimate{predictState(prev_state, control_input)};
    Eigen::MatrixXf predicted_state_covariance{predictUncertainty()};

    // Step 2: Compute Kalman Gain
    Eigen::MatrixXf kalman_gain{computeKalmanGain(predicted_state_covariance)};

    // Step 3: Update Step
    Eigen::VectorXf next_state_estimate{updateStep(predicted_state_estimate, kalman_gain, measurement)};

    // Step 4: Update the error covariance
    P_ = updateErrorCovariance(predicted_state_covariance, kalman_gain);

    return next_state_estimate;
}

int main()
{
    Eigen::VectorXf initial_state(2);  // 2D state: [position, velocity]
    initial_state << 0.0f, 0.0f;  // Initial guess for position and velocity

    // Error covariance matrix (initial uncertainty in state)
    Eigen::MatrixXf error_covariance(2, 2);
    error_covariance << 100.0f, 0.0f,
                        0.0f, 100.0f;  // High uncertainty in both position and velocity

    // Process noise covariance (uncertainty in the model)
    Eigen::MatrixXf process_noise(2, 2);
    process_noise << 0.01f, 0.0f,
                    0.0f, 0.01f;  // Small process noise

    // Measurement noise covariance (uncertainty in measurements)
    Eigen::Matrix<float, 1, 1> measurement_noise;
    measurement_noise << 0.1f;

    // Initialize the KalmanFilter with these matrices
    KalmanFilter filter(initial_state, error_covariance, process_noise, measurement_noise);

    Eigen::Vector2f state_estimate = filter.getInitialState();
    Eigen::Vector2f true_state(0.0f, 0.0f);
    std::vector<float> control_inputs = {0.5f, 0.5f, 0.0f, -0.2f, 0.1f, 0.3f, -0.1f, 0.2f, 0.0f, -0.2f};
    std::vector<float> measurements =   {0.4f, 0.6f, 0.65f, 0.5f, 0.7f, 0.9f, 1.0f, 1.1f, 1.05f, 1.0f};
    std::vector<Eigen::Vector2f> results(control_inputs.size(), Eigen::Vector2f(0.0f, 0.0f));

    // Set noise standard deviation
    float noise_stddev = 0.1f; // Standard deviation for the Gaussian noise

    for (size_t i = 0; i < control_inputs.size(); ++i) {
        float noisy_measurement = filter.getNoisyMeasurement(measurements[i], noise_stddev);

        Eigen::VectorXf measurement(1);
        measurement << noisy_measurement;

        state_estimate = filter.predictAndUpdate(state_estimate, measurement, control_inputs[i]);

        // Simulate true state (without noise)
        float dt = 0.1f; // Same time step as used in your model
        true_state(0) += true_state(1) * dt + 0.5f * control_inputs[i] * dt * dt;  // Update position
        true_state(1) += control_inputs[i] * dt;  // Update velocity

        std::cout << "Step " << i << ":\n";
        std::cout << "  True position: " << true_state(0) << "\n";
        std::cout << "  Measured position (noisy): " << noisy_measurement << "\n";
        std::cout << "  Estimated position: " << state_estimate(0) << "\n";
        std::cout << "  Estimated velocity: " << state_estimate(1) << "\n";
        std::cout << "  Current covariance matrix P_:\n" << filter.getCovariance() << "\n\n";
    }
    return 0;
}