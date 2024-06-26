#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <iostream>

#include <Eigen/Eigen>

namespace mppi {

/**
 * @brief Configuration parameters for the controller.
 */
struct Configuration
{
    /// The number of rollouts to perform on each time.
    std::int64_t rollouts;

    /// The number of best control trajectory rollouts to keep to warmstart the
    // next control trajectory sampling phase (before rolling out).
    std::int64_t keep_best_rollouts;

    /// The time increment passed to the dynamics simulation when performing
    /// rollouts in seconds.
    double step_size;

    /// The duration of time of each rollout in seconds.
    double horison;

    /// The factor by which the optimal policy is updated.
    double gradient_step;

    /// The gradient is clipped to [-gradient_minmax, gradient_minmax].
    double gradient_minmax;

    /// Cost to likelihood mapping scaling.
    double cost_scale;

    /// Discount factor of cost calculation.
    double cost_discount_factor;

    /// The covariance matrix to generate rollout noise from.
    Eigen::MatrixXd covariance;

    /// True to use the last control when a trajectory finishes, or false to use
    // the default value.
    bool control_default_last;

    /// If not control_default_last, the control to return when the trajectory
    // finishes.
    Eigen::VectorXd control_default_value;
};

/**
 * @brief The dynamics class stores and updates the system state.
 * 
 * This class should be subclassed to implement dynamics.
 */
class Dynamics
{
public:

    virtual ~Dynamics() = default;

    /**
     * @brief Get the degrees of freedom of the system control input.
     * @returns The degrees of freedom of the system control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the degrees of freedom of the system state.
     * @returns The degrees of freedom of the system state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     */
    virtual void set(const Eigen::VectorXd &state) = 0;

    /**
     * @brief Step the dynamics simulation.
     * 
     * This function updates the internal state returned by Dynamics::state().
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     */
    virtual Eigen::Ref<Eigen::VectorXd> step(const Eigen::VectorXd &control, double dt) = 0;
};

/**
 * @brief The cost class stores and updates the cost of a rollout.
 * 
 * This class should be subclassed to implement the objective function.
 */
class Cost
{
public:

    virtual ~Cost() = default;

    /**
     * @brief Get the expected degrees of freedom of the system control input.
     * @returns The expected degrees of freedom of the system control input.
     */
    virtual constexpr int control_dof() = 0;

    /**
     * @brief Get the expected degrees of freedom of the system state.
     * @returns The expected degrees of freedom of the system state.
     */
    virtual constexpr int state_dof() = 0;

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param dt The change in time.
     * 
     * @returns The cost of the step.
     */
    virtual double get(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        double dt
    ) = 0;
};

/**
 * @brief A multivariate gaussian sampler.
 * 
 * Based on https://github.com/ethz-asl/sampling_based_control/blob/master/mppi/include/mppi/utils/multivariate_normal_eigen.h
 * that is based on https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
 */
class Gaussian
{
public:

    /**
     * @brief Create a new multivariate gaussian.
     * 
     * The covariance matrix must be square, and the mean must have length
     * equal to the number of covariance rows.
     * 
     * @param mean The mean of each gaussian.
     * @param covariance The covariance matrix of the distribution.
     */
    inline Gaussian(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
        : m_mean(mean)
    {
        set_covariance(covariance);
    }

    /**
     * @brief Create a new multivariate gaussian with zero mean.
     * 
     * The covariance matrix must be square.
     * 
     * @param covariance The covariance matrix of the distribution.
     */
    inline Gaussian(const Eigen::MatrixXd &covariance)
        : Gaussian(Eigen::VectorXd::Zero(covariance.rows()), covariance)
    {}

    /**
     * @brief Set the covariance of the distribution.
     * @param covariance The covariance matrix of the distribution.
     */
    inline void set_covariance(Eigen::MatrixXd const &covariance)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        m_transform = (
            solver.eigenvectors() *
            solver.eigenvalues().cwiseSqrt().asDiagonal()
        );
    }

    /**
     * @brief Sample the distribution.
     * @returns A vector of values sampled from each gaussian.
     */
    inline Eigen::VectorXd operator()() const {
        return m_mean + m_transform * Eigen::VectorXd(m_mean.size()).unaryExpr(
            [&](double) { return s_distribution(s_generator); }
        );
    }

private:

    /// The mean of each gaussian.
    Eigen::VectorXd m_mean;

    /// Transformation matrix from N(0, 1) noise to the multivariate noise.
    Eigen::MatrixXd m_transform;

    /// The pseudo-random number generator.
    static std::mt19937 s_generator;

    /// A N(0, 1) gaussian distribution single values from.
    static std::normal_distribution<double> s_distribution;
};

/**
 * @brief A model predictive path integral (MPPI) optimal trajectory
 * generator.
 * 
 * Generates sequences of control outputs over a period of time (a control
 * trajectory) by optimising for the least cost randomly generated system state
 * evolution and control trajectory.
 * 
 * The generated trajectory should be linearly interpolated with respect to
 * time.
 * 
 * Data is stored in a column major fashion since that is the default in Eigen.
 * That is each number in a column is stored contiguously. Hence each control
 * vector and state vector are stored column wise. Time increases with the
 * column index.
 */
class Trajectory
{
public:

    /// The number of always available precomputed rollouts. These are the zero
    /// control sample and negative of the previous optimal trajectory.
    static const constexpr std::int64_t s_static_rollouts = 2;

    /**
     * @brief Create a new Trajectory optimal trajectory generator.
     * 
     * @param dynamics A pointer to the dynamics object responsible that keeps
     * track of the current system state, and predicts future system states.
     * @param cost A pointer to a cost object, that tracks to cumulative cost
     * of dynamics simulation rollouts.
     * @param configuration The configuration of the trajectory.
     * @param state The initial system state.
     * 
     * @returns A pointer to the trajectory on success, or nullptr on failure.
     */
    static std::unique_ptr<Trajectory> create(
        Dynamics *dynamics,
        Cost *cost,
        const Configuration &configuration,
        const Eigen::Ref<Eigen::VectorXd> state
    );

    /**
     * @brief Update the trajectory from a state and time.
     * 
     * @param state The current state of the dynamics.
     * @param time The current time in seconds.
     */
    void update(const Eigen::Ref<Eigen::VectorXd> state, double time);

    /**
     * @brief Get the noise of a rollout.
     * 
     * @param rollout The rollout to get the noise of.
     * @return The noise of the rollout.
     */
    inline auto get_rollout(std::int64_t rollout) {
        return m_rollouts.block(
            rollout * m_dynamics->control_dof(), // start row
            0, // start col
            m_dynamics->control_dof(), // rows
            m_steps // cols
        );
    };

    /**
     * @brief Get the cost of a rollout.
     * 
     * @param rollout The rollout to get the cost of.
     * @returns The cost of the rollout.
     */
    inline double get_cost(std::int64_t rollout) const {
        return m_costs[rollout];
    }

    /**
     * @brief Get the optimal trajectory starting at the last update time.
     * @returns The optimal control trajectory.
     */
    inline const Eigen::VectorXd &trajectory() const {
        return m_optimal_control;
    }

    /**
     * @brief Evaluate the optimal control trajectory at a given time.
     * 
     * @param control Reference to the control vector to fill.
     * @param t The time to evaluate the control trajectory.
     */
    void get(Eigen::Ref<Eigen::VectorXd> control, double time);

    /**
     * @brief Evaluate the current optimal control trajectory at a given time.
     * 
     * @param time The time to evaluate the control trajectory.
     * @returns The control parameters for the current time.
     */
    inline Eigen::VectorXd operator()(double time) {
        Eigen::VectorXd control;
        get(control, time);
        return control;
    }

private:

    /**
     * @brief Initialise the trajectory generator.
     */
    Trajectory(
        Dynamics *dynamics,
        Cost *cost,
        const Configuration &configuration,
        const Eigen::Ref<Eigen::VectorXd> state,
        int steps
    ) noexcept;

    /**
     * @brief Sample the rollouts to simulate.
     * 
     * Samples noise from a multivariate gaussian distribution for each time
     * step of each rollout.
     * 
     * Shifts the previous rollouts forward in time to align with the current
     * sample time. Otherwise the optimal rollout would be applied after a time
     * delay.
     * 
     * The best k rollouts from the previous sample are not resampled. Only the
     * part of the horison that has come into view are sampled.
     * 
     * Each rollout noise is added to the current optimal rollout and simulated
     * in rollout(), and analysed in optimise() to get the optimal trajectory.
     */
    void sample(double time);

    /**
     * @brief Rollout a sampled trajectory.
     * 
     * Adds the noise of the given rollout to the optimal trajectory, and
     * simulates it. The cumulative cost of the rollout is stored.
     * 
     * @param index The rollout to perform.
     */
    void rollout(std::int64_t index);

    /**
     * @brief Updates the optimal control trajectory.
     * 
     * Transforms the costs of each simulated rollouts into normalised
     * likelihoods.
     * 
     * Creates a linear combination of all rollout noise, using the normalised
     * likelihoods as the weights, to produce the optimal rollout noise.
     * 
     * Uses the optimal rollout noise to gradient step the optimal control
     * trajectory.
     */
    void optimise();

    /// The configuration of the trajectory generation.
    Configuration m_configuration;

    /// Keeps track of system state and simulates responses to control actions.
    Dynamics *m_dynamics;

    /// Keeps track of individual cumulative cost of dynamics simulation rollouts.
    Cost *m_cost;

    /// The random number generator to use in the normal distribution.
    Gaussian m_gaussian;

    /// The number of time steps per rollout.
    int m_steps;

    /// The number of degrees of freedom for the system state.
    int m_state_dof;

    /// The number of degrees of freedom for the control input.
    int m_control_dof;

    /// The current state from which the controller is generating trajectories.
    Eigen::VectorXd m_rollout_state;

    /// The current time of trajectory generation.
    double m_rollout_time;

    /// The number of time steps to shift control by to align with current time.
    std::int64_t m_shift_by;

    /// The number of columns that was shifted to align with current time.
    std::int64_t m_shifted;

    /// The time of the last trajectory generation.
    double m_last_rollout_time;

    /// The control parameters applied at each step in the rollouts.
    Eigen::MatrixXd m_rollouts;

    /// The cost of each rollout.
    Eigen::VectorXd m_costs;

    /// The weight of each rollout. The higher the better.
    Eigen::VectorXd m_weights;

    /// The gradient applied to the optimal control trajectory.
    Eigen::MatrixXd m_gradient;

    /// The previous optimal control, shifted to align with the current time.
    Eigen::MatrixXd m_optimal_control_shifted;

    /// The optimal control.
    Eigen::MatrixXd m_optimal_control;

    /// Mutex protecting concurrent access to the optimal control double buffer.
    std::mutex m_optimal_control_mutex;

    /// Buffer to store rollout indexes before sorting them by cost.
    std::vector<std::int64_t> m_ordered_rollouts;
};

} // namespace mppi
