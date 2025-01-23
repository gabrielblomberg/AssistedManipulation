#pragma once

#include <variant>
#include <mutex>
#include <shared_mutex>
#include <deque>

#include "controller/json.hpp"
#include "controller/kalman.hpp"

/**
 * @brief A predictor that observes forces and estimates future forces.
 */
class Forecast
{
public:

    // Forward declare configuration before defining forecast types.
    struct Configuration;

    /**
     * @brief Create a forecast.
     * 
     * @param configuration The configuration of the forecast.
     */
    static std::unique_ptr<Forecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Update the forecast with an observed measurement.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    virtual void update(VectorXd measurement, double time) = 0;

    /**
     * @brief Update the forecast without an observed measurement.
     * 
     * @pre `time > previous_update_time`
     * 
     * @param time The time of the update.
     */
    virtual void update(double time) = 0;

    /**
     * @brief Forecast the state at a time in the future.
     * 
     * @param time The time of the prediction.
     * @returns The predicted state.
     */
    virtual VectorXd forecast(double time) = 0;
};

/**
 * @brief A trivial last observation carried forward (LOCF) forecast that
 * returns the most recent observation.
 */
class LOCFForecast : public Forecast
{
public:

    struct Configuration {

        /// The most recent observation.
        VectorXd observation;

        /// The valid horison.
        double horison;

        /// JSON conversion for average forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, observation, horison);
    };

    /**
     * @brief Create a last observation carried forward forecast.
     * 
     * @param configuration The forecast configuration.
     * @return Pointer to the forecast on success or nullptr on failure.
     */
    static inline std::unique_ptr<LOCFForecast> create(
        const Configuration &configuration
    ) {
        return std::unique_ptr<LOCFForecast>(
            new LOCFForecast(configuration)
        );
    }

    /**
     * @brief Update the last observation.
     * 
     * @param measurement The measurement or observation.
     * @param time The time of the measurement, unused.
     */
    inline void update(VectorXd measurement, double time) override {
        std::unique_lock lock(m_mutex);
        m_valid_until = time + m_horison;
        m_observation = measurement;
    }

    /**
     * @brief Updating without a reference does nothing.
     * @param time The time of the observation, unused.
     */
    inline void update(double time) override {}

    /**
     * @brief Get the last observation carried forward.
     * 
     * @param time The time of the observation, unused.
     * @returns The last observation.
     */
    inline VectorXd forecast(double time) override {
        std::shared_lock lock(m_mutex);
        if (time > m_valid_until)
            return VectorXd::Zero(m_observation.rows(), m_observation.cols());
        return m_observation;
    }

private:

    LOCFForecast(const Configuration &configuration)
        : m_horison(configuration.horison)
        , m_valid_until(0.0)
        , m_observation(configuration.observation)
    {}

    /// Mutex protecting concurrent access to the last observation.
    std::shared_mutex m_mutex;

    double m_horison;

    double m_valid_until;

    /// The last observation.
    VectorXd m_observation;
};

/**
 * @brief An average forecast that returns the average observation over an
 * averaging window, always including the most recent measurement even when
 * outside of the window (constant forecast).
 */
class AverageForecast : public Forecast
{
public:

    struct Configuration {

        /// The number of states.
        unsigned int states;

        /// The period of time over which to calculate the average..
        double window;

        /// JSON conversion for average forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, states, window);
    };

    /**
     * @brief Create an average forecast.
     * 
     * @param configuration 
     * @return std::unique_ptr<AverageForecast> 
     */
    static std::unique_ptr<AverageForecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Does nothing.
     */
    void update(double time) override;

    /**
     * @brief Update the predictor with an observed measurement.
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    void update(VectorXd measurement, double time) override;

    /**
     * @brief Predict the state as the last observed measurement, regardless of
     * time.
     * 
     * @param time The time of the forecasted state, unused.
     * @returns The average observation.
     */
    VectorXd forecast(double /* time */) override;

private:

    friend class Handle;

    /**
     * @brief Initialise the average forecast.
     * 
     * @param window The period of time in seconds to average measurements over.
     * @param states The number of states of each measurement.
     */
    AverageForecast(double window, unsigned int states);

    /**
     * @brief Remove all samples older than the current time window.
     */
    void clear_old_measurements(double time);

    /**
     * @brief Update the calculated average in the buffer.
     */
    void update_average();

    /// Mutex protecting concurrent read / write.
    std::shared_mutex m_mutex;

    /// Window over which to average.
    double m_window;

    double m_last;

    /// The window of forces to average over. The most recent observation is
    /// always  kept regardless of its age.
    std::vector<std::pair<double, VectorXd>> m_buffer;

    /// The average observation of the window.
    VectorXd m_average;
};

/**
 * @brief A forecast based on a kalman filter.
 * 
 * Note that update() must be called every configured time_step, with or without
 * an observation.
 */
class KalmanForecast : public Forecast
{
public:

    struct Configuration {

        /// The number of states to forecast.
        unsigned int observed_states;

        /// The time step of the state transition matrix.
        double time_step;

        /// The horison over which the forecast should be predicted.
        double horison;

        /// The order of the euclidean model.
        unsigned int order;

        VectorXd variance;

        /// The initial state of the system. E.g. {x, y, z}
        VectorXd initial_state;

        /// JSON conversion for kalman forecast configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            observed_states, time_step, horison, order, variance, initial_state
        );
    };

    /**
     * @brief Create a new kalman forecast.
     * 
     * @param configuration The configuration of the predictor.
     * @returns A pointer to the predictor on success or nullptr on failure.
     */
    static std::unique_ptr<KalmanForecast> create(
        const Configuration &configuration
    );

    /**
     * @brief Create a state transition matrix where each state accumulates its
     * standard derivatives.
     * 
     * For example, a 2 state transition matrix of order 3 has the state
     * transition matrix for `x += dx * dt + 1/2 * ddx^2 * dt` and
     * `y += dy * dt + 1/2 * ddy^2 * dt`. 
     * 
     * The order is the highest derivative in every observation.
     * 
     * @param time_step The time step of the transition matrix.
     * @param states The number of dimensions of the observed state.
     * @param order The order of the state transition matrix.
     * 
     * @returns The state transition matrix.
     */
    static MatrixXd create_euler_state_transition_matrix(
        double time_step,
        unsigned int observed_states,
        unsigned int order
    );

    static MatrixXd create_euler_state_transition_covariance_matrix(
        const VectorXd &variance,
        unsigned int observed_states,
        unsigned int order
    );

    /**
     * @brief Update the forecast with an observation.
     * 
     * @param measurement The observed measurement.
     * @param time The time of the observation.
     */
    void update(VectorXd measurement, double time) override;

    /**
     * @brief Update without an observation.
     */
    void update(double time) override;

    /**
     * @brief Predict the force as the last observed force, regardless of
     * time.
     * 
     * @param time The time of the force prdiction
     * @returns The last observed force.
     */
    VectorXd forecast(double /* time */) override;

private:

    friend class Handle;

    /**
     * @brief Initialise the kalman forecaster.
     */
    KalmanForecast(
        const Configuration &configuration,
        unsigned int steps,
        double last_update,
        std::unique_ptr<KalmanFilter> &&filter,
        std::unique_ptr<KalmanFilter> &&predictor,
        VectorXd initial_state
    );

    /**
     * @brief Simple recursive factorial function.
     */
    static unsigned int factorial(unsigned int n);

    /// The order of forecast.
    unsigned int m_order;

    /// The number of observed states in the forecaster.
    unsigned int m_observed_states;

    /// The number of states including derivatives.
    unsigned int m_estaimted_states;

    /// The period of time over which the prediction is made.
    double m_horison;

    /// The expected kalman time step.
    double m_time_step;

    /// The number of steps to make in each forecast.
    unsigned int m_steps;

    /// The time of the last forecast.
    double m_last_update;

    std::shared_mutex m_mutex;

    /// Location to the observed state, filled with zeros for derivatives.
    VectorXd m_measurement;

    /// The kalman filter used to estimate the current wrench.
    std::unique_ptr<KalmanFilter> m_filter;

    /// The kalman filter used to predict future wrench.
    std::unique_ptr<KalmanFilter> m_predictor;

    /// The latest updated horison.
    MatrixXd m_prediction;
};

/**
 * @brief Configuration of a forecast.
 * 
 * Postpone configuration until types are fully declared.
 */
struct Forecast::Configuration
{
    enum Type {
        LOCF,
        AVERAGE,
        KALMAN
    };

    /// The configured forecast.
    Type type;

    /// Configuration for last observation carried forward.
    std::optional<LOCFForecast::Configuration> locf;

    /// Configuration for average forecast
    std::optional<AverageForecast::Configuration> average;

    /// Configuration for average kalman.
    std::optional<KalmanForecast::Configuration> kalman;

    /// JSON conversion for forecast configuration.
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Forecast::Configuration,
        type, locf, average, kalman
    )
};
