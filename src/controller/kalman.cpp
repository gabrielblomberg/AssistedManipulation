#include "controller/kalman.hpp"

#include <iostream>

std::unique_ptr<KalmanFilter> KalmanFilter::create(
    const KalmanFilter::Configuration &configuration
) {
    static auto check_dimensions = [](
        const char *name,
        const MatrixXd &matrix,
        std::int64_t rows,
        std::int64_t cols
    ){
        bool is_valid = matrix.rows() == rows && matrix.cols() == cols;

        if (!is_valid) {
            std::cerr
                << "invalid " << name << " dimensions ("
                << matrix.rows() << ", " << matrix.cols()
                << ") expected (" << rows << ", " << cols << ")"
                << std::endl;
        }

        return is_valid;
    };

    bool valid = check_dimensions(
        "state transition matrix",
        configuration.state_transition_matrix,
        configuration.states,
        configuration.states
    );

    // valid &= check_dimensions(
    //     "control transition matrix",
    //     configuration.control_transition_matrix,
    //     configuration.states,
    //     configuration.controls
    // );

    valid &= check_dimensions(
        "transition covariance matrix",
        configuration.transition_covariance,
        configuration.states,
        configuration.states
    );

    valid &= check_dimensions(
        "observation matrix",
        configuration.observation_matrix,
        configuration.observed_states,
        configuration.states
    );

    valid &= check_dimensions(
        "observation covariance matrix",
        configuration.observation_covariance,
        configuration.observed_states,
        configuration.observed_states
    );

    valid &= check_dimensions(
        "initial state",
        configuration.initial_state,
        configuration.states,
        1
    );

    valid &= check_dimensions(
        "initial state covariance",
        configuration.initial_covariance,
        configuration.states,
        configuration.states
    );

    if (!valid) {
        std::cerr << "invalid kalman filter configuration" << std::endl;
        return nullptr;
    }

    auto filter = std::unique_ptr<KalmanFilter>(new KalmanFilter(configuration));
    filter->m_state = configuration.initial_state;
    filter->m_next_state = (
        configuration.state_transition_matrix * filter->m_next_state
    );

    return std::unique_ptr<KalmanFilter>(new KalmanFilter(configuration));
}

KalmanFilter::KalmanFilter(const Configuration &config)
    : m_observed_state_size(config.observed_states)
    , m_estimated_state_size(config.states)
    , m_state_transition_matrix(config.state_transition_matrix)
    , m_transition_covariance(config.transition_covariance)
    , m_observation_matrix(config.observation_matrix)
    , m_observation_covariance(config.observation_covariance)
    , m_identity(MatrixXd::Identity(config.states, config.states))
    , m_covariance(config.initial_covariance)
    , m_state(config.initial_state)
    , m_next_state(m_state_transition_matrix * config.initial_state)
{}

void KalmanFilter::update(VectorXd &observation)
{
    // Calculate the optimal kalman gain.
    MatrixXd optimal_kalman_gain = (
        m_covariance * m_observation_matrix.transpose() * (
            m_observation_matrix * m_covariance * m_observation_matrix.transpose() +
            m_observation_covariance
        ).inverse()
    );

    assert(!optimal_kalman_gain.hasNaN());

    // Correct the previously predicted state estimation, by interpolating
    // between the estimated state and the observed state.
    m_state = m_next_state + optimal_kalman_gain * (
        observation - m_observation_matrix * m_next_state
    );

    // Update the noise covariance of the estimated state. Simplified update
    // when the kalman gain is optimal. See
    // https://en.wikipedia.org/wiki/Kalman_filter#Derivations
    m_covariance = (
        m_identity - optimal_kalman_gain * m_observation_matrix
    ) * m_covariance;

    // Predict the next state from the current state and control.
    m_next_state = (
        m_state_transition_matrix * m_state
    );

    // Extrapolate the noise to the next state.
    m_covariance = (
        m_state_transition_matrix * m_covariance * m_state_transition_matrix.transpose() + 
        m_transition_covariance
    );
}

void KalmanFilter::predict(bool update_covariance)
{
    m_state = m_next_state;

    m_next_state = m_state_transition_matrix * m_state;

    if (update_covariance) {
        m_covariance = (
            m_state_transition_matrix * m_covariance * m_state_transition_matrix.transpose() + 
            m_transition_covariance
        );
    }
}
