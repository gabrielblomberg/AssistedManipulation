#pragma once

#include <optional>

#include "frankaridgeback/dynamics.hpp"
#include "controller/mppi.hpp"
#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "simulation/simulator.hpp"

namespace FrankaRidgeback {

/**
 * @brief MPPI frankaridgeback dynamics implemented with RaiSim.
 * 
 * This class is responsible for both:
 * - Simulating the FrankaRidgeback robot in the main simulation. The resulting
 *   state is passed to the FrankaRidgeback::Actor and visualised.
 * - Performing forward simulated rollouts of the dynamics in the MPPI routine.
 */
class RaisimDynamics : public Dynamics
{
public:

    struct Configuration {

        /// The simulator configuration if the simulator is not passed
        std::optional<Simulator::Configuration> simulator;

        /// The file name of the robot definition.
        std::string filename;

        /// The frame of the end effector, to measure forces from.
        std::string end_effector_frame;

        /// The initial state.
        FrankaRidgeback::State initial_state;

        /// The proportional gain of the joint PD controller.
        FrankaRidgeback::Control proportional_gain;

        /// The differential gain of the joint PD controller.
        FrankaRidgeback::Control differential_gain;

        /// Override of the initial available energy for the robot.
        std::optional<double> energy;

        // JSON conversion for franka-ridgeback raisim dynamics configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            simulator, filename, end_effector_frame, initial_state,
            proportional_gain, differential_gain, energy
        )
    };

    static inline const Configuration DEFAULT_CONFIGURATION {
        .simulator = Simulator::Configuration {
            .time_step = 0.01,
            .gravity = {0.0, 0.0, 9.81}
        },
        .filename = "",
        .end_effector_frame = "panda_grasp_joint",
        .initial_state = make_state(FrankaRidgeback::Preset::HUDDLED),
        .proportional_gain = FrankaRidgeback::Control{
            0.0, 0.0, 0.0, // base
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
            100.0, 100.0
        },
        .differential_gain = FrankaRidgeback::Control{
            1000.0, 1000.0, 1000.0, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
            50.0, 50.0
        },
        .energy = 1000.0
    };

    /**
     * @brief Create a new franka-ridgeback raisim dynamics instance.
     * 
     * Copies the configuration.
     * 
     * @param configuration The configuration of the raisim dynamics.
     * @param dynamics_forecast_handle Handle to the dynamics forecast.
     * @param simulator Pointer to the simulator to use for the world, if the
     * configuration for the world is std::nullopt.
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<RaisimDynamics> create(
        Configuration configuration,
        std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle = nullptr,
        Simulator *simulator = nullptr
    );

    /**
     * @brief Copy the dynamics.
     * @returns A copy of the dynamics.
     */
    inline std::unique_ptr<mppi::Dynamics> copy() override
    {
        if (m_forecast)
            return create(m_configuration, std::move(m_forecast->copy()));
        return create(m_configuration, nullptr);
    }

    /**
     * @brief Step the dynamics.
     * 
     * Called when used for mppi trajectory generation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     * 
     * @returns The subsequent state.
     */
    Eigen::Ref<VectorXd> step(const VectorXd &control, double dt) override;

    /**
     * @brief Set the control actions for the dynamics.
     * 
     * Called when used for simulating in an external world.
     * 
     * @param control The controls applied at the current state (before dt).
     */
    void act(const Control &control);

    /**
     * @brief Update the state of the dynamics.
     * 
     * Called when used for simulating in an external world.
     */
    void update();

    /**
     * @brief Set the state of the dynamics.
     * 
     * The wrench of the state is ignored, and is instead handled by the force
     * forecast.
     * 
     * @param state The system state.
     * @param time The time of the state.
     */
    void set_state(const VectorXd &state, double time) override;

    /**
     * @brief Get the dynamics state.
     * @returns The state of the dynamics.
     */
    Eigen::Ref<VectorXd> get_state() override
    {
        return m_state;
    }

    /**
     * @brief Get the degrees of freedom of the franka-ridgeback control.
     */
    inline constexpr int get_control_dof() override
    {
        return FrankaRidgeback::DoF::CONTROL;
    }

    /**
     * @brief Get the degrees of freedom of the franka-ridgeback state.
     */
    inline constexpr int get_state_dof() override
    {
        return FrankaRidgeback::DoF::STATE;
    }

    /**
     * @brief Get the current joint position.
     */
    inline Eigen::Ref<const VectorXd> get_joint_position() const override
    {
        return m_position;
    }

    /**
     * @brief Get the current joint velocity.
     */
    inline Eigen::Ref<const VectorXd> get_joint_velocity() const override
    {
        return m_velocity;
    }

    /**
     * @brief Get the position of a frame.
     * 
     * @todo Make the frame parameter an enumeration.
     * 
     * @param frame The frame.
     * @returns The position of the frame.
     */
    inline Vector3d get_frame_position(Frame frame) override
    {
        raisim::Vec<3> position;
        m_robot->getFramePosition(FRAME_NAMES[(std::size_t)frame], position);
        return position.e();
    }

    /**
     * @brief Get the orientation of a frame.
     * 
     * @todo Make the frame parameter an enumeration.
     * 
     * @param frame The frame.
     * @returns The orientation of the frame.
     */
    inline Quaterniond get_frame_orientation(Frame frame) override
    {
        raisim::Mat<3, 3> orientation;
        m_robot->getFrameOrientation(FRAME_NAMES[(std::size_t)frame], orientation);
        return Quaterniond(orientation.e());
    }

    /**
     * @brief Get the origin of a link in the world frame
     * 
     * @param link The link to get the origin of.
     * @returns The position of the link in the world frame.
     */
    virtual Vector3d get_link_position(Link link) override
    {
        return m_robot->getBodyCOM_W()[
            m_robot->getBodyIdx(LINK_NAMES[(std::size_t)link])
        ].e();
    }

    /**
     * @brief Get the kinematics of the end effector.
     */
    inline const EndEffectorState &get_end_effector_state() const
    {
        return m_end_effector_state;
    }

    /**
     * @brief Get the current power from applied joint controls.
     * 
     * This is given by the sum of generalised joint force multiplied by their
     * generalised velocities. This is torque * angular velocity for revolute
     * joints and force * linear velocity for prismatic joints.
     * 
     * @return The current power usage in joules/s.
     */
    virtual double get_joint_power() const override
    {
        return m_joint_power;
    }

    /**
     * @brief Get the current power from external forces.
     * @return The current power usage in joules/s.
     */
    virtual double get_external_power() const override
    {
        return m_external_power;
    }

    /**
     * @brief Get the current energy left in the energy tank.
     */
    inline double get_tank_energy() const override
    {
        return m_energy_tank.get_energy();
    }

    /**
     * @brief Get a pointer to the dynamics forecast if it exists.
     * 
     * @warning May be nullptr.
     * @returns A pointer to the dynamics forecast on success or nullptr if
     * there is no dynamics forecast handle.
     */
    const DynamicsForecast::Handle *get_forecast() const override
    {
        if (m_forecast)
            return m_forecast.get();
        return nullptr;
    }

    /**
     * @brief Get the actual wrench of the end effector.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    inline Vector6d get_end_effector_simulated_wrench() const override
    {
        return m_end_effector_simulated_wrench;
    }

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    void add_end_effector_simulated_wrench(Vector6d wrench) override;

private:

    /**
     * @brief Initialise the raisim franka-ridgeback dynamics.
     * 
     * @param configuration 
     * @param world The world in which the franka-ridgeback exists.
     * @param dynamics_forecast_handle Handle to the dynamics forecast.
     * @param robot Pointer to the robot in the world.
     * @param end_effector_frame_index Index of the end effector frame.
     * @param end_effector_frame_name The name of the end effector frame.
     */
    RaisimDynamics(
        const Configuration &configuration,
        std::shared_ptr<raisim::World> &&world,
        std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle,
        raisim::ArticulatedSystem *robot,
        std::int64_t end_effector_frame_index,
        std::string end_effector_frame_name
    );

    /**
     * @brief Calculate dynamics information on stepping or setting the state.
     */
    void calculate();

    /// The configuration of the frankaridgeback dynamics.
    Configuration m_configuration;

    /// The simulated world.
    std::shared_ptr<raisim::World> m_world;

    /// The simulated articulated object, generated from the urdf file.
    raisim::ArticulatedSystem *m_robot;

    /// The current joint positions.
    VectorXd m_position_command;

    /// The current joint velocities.
    VectorXd m_velocity_command;

    /// The current joint positions.
    VectorXd m_position;

    /// The current joint velocities.
    VectorXd m_velocity;

    /// The index of the end effector frame into raisim data structure.
    std::int64_t m_end_effector_frame_index;

    /// The name of the end effector frame of reference.
    std::string m_end_effector_frame_name;

    /// The current kinematics of the end effector frame.
    EndEffectorState m_end_effector_state;

    /// Linear part of the jacobian, returned by raisim.
    MatrixXd m_end_effector_linear_jacobian;

    /// Angular part of the jacobian, returned by raisim.
    MatrixXd m_end_effector_angular_jacobian;

    /// The most recently calculated internal joint power.
    double m_joint_power;

    /// The most recently calculated external power.
    double m_external_power;

    /// The available energy.
    EnergyTank m_energy_tank;

    /// The current state of the dynamics.
    State m_state;

    /// Handle to the forecast dynamics, unused, passed to the objective.
    std::unique_ptr<DynamicsForecast::Handle> m_forecast;

    /// Cumulative wrench added to the end effector, only for simulation.
    Vector6d m_end_effector_simulated_wrench;
};

} // FrankaRidgeback
