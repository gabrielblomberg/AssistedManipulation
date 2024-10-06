#pragma once

#include "controller/json.hpp"
#include "controller/mppi.hpp"
#include "controller/cost.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "frankaridgeback/state.hpp"

/**
 * @brief Objective function of the franka research 3 ridgeback assisted
 * manipulation task.
 */
class AssistedManipulation : public mppi::Cost
{
public:

    struct Configuration {

        /// If joint limit costs are enabled.
        bool enable_joint_limit = true;

        /// If reach costs are enabled.
        bool enable_reach_limit = false;

        /// If end effector manipulability is maximised.
        bool enable_maximise_manipulability = false;

        /// If the power used by the trajectory is minimised.
        bool enable_minimise_power = false;

        /// If variable damping should be enabled.
        bool enable_variable_damping = false;

        /// Lower joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> lower_joint_limit;

        /// Upper joint limits if enabled.
        std::array<QuadraticCost, FrankaRidgeback::DoF::JOINTS> upper_joint_limit;

        /// Maximum reach if enabled.
        QuadraticCost maximum_reach;

        /// Minimum reach if enabled.
        QuadraticCost minimum_reach;

        /// Manipulability limits if enabled. Relative to `sqrt(det(J * J^T))`
        /// that is proportional to the volume of the manipulability ellipsoid,
        /// clipped above 1e-10. Jacobian in spatial frame. Greater values are
        /// better. Limit is a lower bound on this value.
        QuadraticCost minimum_manipulability;

        /// Maximum power (joules per second) usage if enabled. 
        QuadraticCost maximum_power;

        /// The maximum damping that occurs when the end effector has zero
        /// velocity. The A in c(v) = Ae^{lambda * v}
        double variable_damping_maximum;

        /// The exponential drop-off from variable_damping_maximum with respect
        /// to velocity. The lambda in c(v) = Ae^{lambda * v}
        double variable_damping_dropoff;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            enable_joint_limit, enable_reach_limit, 
            enable_maximise_manipulability, enable_minimise_power,
            enable_variable_damping, lower_joint_limit, upper_joint_limit,
            maximum_reach, minimum_reach, minimum_manipulability, maximum_power,
            variable_damping_maximum, variable_damping_dropoff
        )
    };

    /**
     * @brief The default configuration of the assisted manipulation objective
     * 
     * The fidelity of the joint limits probably doesn't need to be this high.
     * 
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .enable_joint_limit = true,
        .enable_reach_limit = false,
        .enable_maximise_manipulability = false,
        .enable_minimise_power = false,
        .enable_variable_damping = false,
        .lower_joint_limit = {{
            {-2.0,    1'000, 100'00}, // Base rotation
            {-2.0,    1'000, 100'00}, // Base x
            {-6.28,   1'000, 100'00}, // Base y
            {-2.8973, 1'000, 100'00}, // Joint1
            {-1.7628, 1'000, 100'00}, // Joint2
            {-2.8973, 1'000, 100'00}, // Joint3
            {-3.0718, 1'000, 100'00}, // Joint4
            {-2.8973, 1'000, 100'00}, // Joint5
            {-0.0175, 1'000, 100'00}, // Joint6
            {-2.8973, 1'000, 100'00}, // Joint7
            {0.5,     1'000, 100'00}, // Gripper x
            {0.5,     1'000, 100'00}  // Gripper y
        }},
        .upper_joint_limit = {{
            {2.0,    1'000, 100'00}, // Base rotation
            {2.0,    1'000, 100'00}, // Base x
            {6.28,   1'000, 100'00}, // Base y
            {2.8973, 1'000, 100'00}, // Joint1
            {1.7628, 1'000, 100'00}, // Joint2
            {2.8973, 1'000, 100'00}, // Joint3
            {3.0718, 1'000, 100'00}, // Joint4
            {2.8973, 1'000, 100'00}, // Joint5
            {0.0175, 1'000, 100'00}, // Joint6
            {2.8973, 1'000, 100'00}, // Joint7
            {0.5,    1'000, 100'00}, // Gripper x
            {0.5,    1'000, 100'00}  // Gripper y
        }}
    };

    /**
     * @brief Get the number of state degrees of freedom.
     */
    inline constexpr int get_state_dof() override {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the number of control degrees of freedom.
     */
    inline constexpr int get_control_dof() override {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Create an assisted manipulation objective function.
     * 
     * @param configuration The configuration of the objective function.
     * @returns A pointer to the objective on success, or nullptr on failure.
     */
    static std::unique_ptr<AssistedManipulation> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the cumulative power cost.
     */
    inline double get_power_cost() {
        return m_power_cost;
    }

    /**
     * 
    */
    inline double get_manipulability_cost() {
        return m_manipulability_cost;
    }

    /**
     * 
    */
    inline double get_joint_cost() {
        return m_joint_cost;
    }

    /**
     * 
    */
    inline double get_reach_cost() {
        return m_reach_cost;
    }

    /**
     * 
     */
    inline double get_variable_damping_cost() {
        return m_variable_damping_cost;
    }

    /**
     * @brief Get the cost of a state and control input over dt.
     * 
     * @param state The state of the system.
     * @param control The control parameters applied to the state.
     * @param dynamics Pointer to the dynamics at the time step.
     * @param time The current time.
     * 
     * @returns The cost of the step.
     */
    double get_cost(
        const Eigen::VectorXd &state,
        const Eigen::VectorXd &control,
        mppi::Dynamics *dynamics,
        double time
    ) override;

    /**
     * @brief Reset the objective function cost.
     */
    void reset() override {};

    /**
     * @brief Make a copy of the objective function.
     */
    inline std::unique_ptr<mppi::Cost> copy() override {
        return std::unique_ptr<AssistedManipulation>(
            new AssistedManipulation(m_configuration)
        );
    }

private:

    /**
     * @brief Initialise the assisted manipulation cost.
     * 
     * @param model Pointer to the robot model.
     * @param configuration The configuration of the objective function.
     */
    AssistedManipulation(const Configuration &configuration);

    double power_cost(FrankaRidgeback::Dynamics *dynamics);

    double manipulability_cost(FrankaRidgeback::Dynamics *dynamics);

    double joint_limit_cost(const FrankaRidgeback::State &state);

    double reach_cost(FrankaRidgeback::Dynamics *dynamics);

    double variable_damping_cost(const FrankaRidgeback::State &state);

    /// The configuration of the objective function.
    Configuration m_configuration;

    /// Spatial jacobian.
    Eigen::Matrix<double, 6, 6> m_space_jacobian;

    double m_cost;

    double m_power_cost;

    double m_manipulability_cost;

    double m_joint_cost;

    double m_reach_cost;

    double m_variable_damping_cost;
};
