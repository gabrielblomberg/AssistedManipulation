#pragma once

#include <utility>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include "controller/energy.hpp"
#include "controller/json.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

/**
 * @brief Model predictive path integral control dynamics for the
 * frankaridgeback implemented with pinocchio.
 * 
 * @note See simulation/raisim_dynamics.hpp for a raisim based dynamics
 * implementation.
 * 
 * @warning This class is broken due to the pinocchio articulated body algorithm
 * function diverging. This class also expects torque control (not velocity
 * control).
 */
class PinocchioDynamics : public Dynamics
{
public:

    struct Configuration {

        /// The filename of the URDF robot definition file to instantiate the
        // model from.
        std::string filename;

        /// The name of the end effector frame.
        std::string end_effector_frame;

        /// The initial state of the dynamics.
        State initial_state;

        /// The initial energy in the energy tank.
        double energy;

        /// JSON conversion for dynamics configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            filename, end_effector_frame, energy
        );
    };

    static inline const Configuration DEFAULT_CONFIGURATION {
        .filename = "",
        .end_effector_frame = "panda_grasp_joint",
        .initial_state = FrankaRidgeback::State::Zero(),
        .energy = 10.0
    };

    /**
     * @brief Create a new franka ridgeback dynamics object.
     * 
     * Copies the configuration.
     * 
     * @param configuration The configuration of the dynamics.
     * @param dynamics_forecast_handle An optional dynamics forecast to
     * forward to the objective function.
     * 
     * @returns A pointer to the dynamics on success or nullptr on failure.
     */
    static std::unique_ptr<PinocchioDynamics> create(
        Configuration configuration,
        std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle = nullptr
    );

    /**
     * @brief Copy the dynamics.
     */
    std::unique_ptr<mppi::Dynamics> copy() override;

    /**
     * @brief Get the configuration of the dynamics.
     */
    inline const Configuration &get_configuration()
    {
        return m_configuration;
    }

    /**
     * @brief Step the dynamics simulation.
     * 
     * @param control The controls applied at the current state (before dt).
     * @param dt The change in time.
     * 
     * @returns The FrankaRidgeback::State after stepping.
     */
    Eigen::Ref<VectorXd> step(const VectorXd &control, double dt) override;

    /**
     * @brief Set the dynamics simulation to a given state.
     * @param state The system state.
     * @param time The time of the dynamincs state.
     */
    void set_state(const VectorXd &state, double time) override;

    /**
     * @brief Get the dynamics state.
     */
    inline Eigen::Ref<VectorXd> get_state() override
    {
        return m_state;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * control.
     */
    inline constexpr int get_control_dof() override
    {
        return DoF::CONTROL;
    }

    /**
     * @brief Get the number of degrees of freedom for the frankaridgeback
     * state.
     */
    inline constexpr int get_state_dof() override
    {
        return DoF::STATE;
    }

    /**
     * @brief Get the current joint position.
     */
    inline Eigen::Ref<const VectorXd> get_joint_position() const override
    {
        return m_joint_position;
    }

    /**
     * @brief Get the current joint velocity.
     */
    inline Eigen::Ref<const VectorXd> get_joint_velocity() const override
    {
        return m_joint_velocity;
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
        return m_data->oMf[
            m_model->getFrameId(FRAME_NAMES[(std::size_t)frame])
        ].translation();
    }

    /**
     * @brief Get the orientation of a frame.
     * 
     * @todo Make the frame parameter an enumeration.
     * 
     * @param frame The frame.
     * @returns The orientation of the frame.
     */
    Quaterniond get_frame_orientation(Frame frame) override
    {
        return Quaterniond(m_data->oMf[
            m_model->getFrameId(FRAME_NAMES[(std::size_t)frame])
        ].rotation());
    }

    /**
     * @brief Get the origin of a link in the world frame
     * 
     * @todo Needs implementation.
     * 
     * @param link The link to get the origin of.
     * @returns The position of the link in the world frame.
     */
    Vector3d get_link_position(Link link) override
    {
        return Vector3d::Zero();
    }

    /**
     * @brief Get the kinematics of the end effector.
     */
    inline const EndEffectorState &get_end_effector_state() const override
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
        return 0.0;
    }

    /**
     * @brief Get the current power from external forces.
     * @return The current power usage in joules/s.
     */
    virtual double get_external_power() const override
    {
        return 0.0;
    }

    /**
     * @brief Get the current energy left in the energy tank.
     */
    double get_tank_energy() const override
    {
        return m_energy_tank.get_energy();
    }

    /**
     * @brief Get a pointer to the dynamics forecast if it exists.
     * 
     * @warning May be nullptr.
     * @returns A pointer to the dynamics forecast on success or std::nullopt if
     * there is no dynamics forecast handle.
     */
    const DynamicsForecast::Handle *get_forecast() const override
    {
        if (m_forecast)
            return m_forecast.get();
        return nullptr;
    }

    /**
     * @brief Get the actual wrench applied of the end effector by calls to
     * add_end_effector_true_wrench().
     * 
     * @todo Implement simulating external end effector wrench for pinocchio
     * dynamics.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    inline Vector6d get_end_effector_simulated_wrench() const override
    {
        return Vector6d::Zero();
    }

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @todo Implement simulating external end effector wrench for pinocchio
     * dynamics.
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    inline void add_end_effector_simulated_wrench(Vector6d wrench) override {}

    /**
     * @brief Get the underlying pinocchio model.
     * @returns The underlying pinocchio model.
     */
    pinocchio::Model *get_model() const {
        return m_model.get();
    }

    /**
     * @brief Get the underlying pinocchio data.
     * 
     * See https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/topic/doc-v2/doxygen-html/structse3_1_1Data.html
     * for all available model data.
     * 
     * @return const pinocchio::Data* 
     */
    pinocchio::Data *get_data() const {
        return m_data.get();
    }

    /**
     * @brief Get the offset in the world space between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset between the frames.
     */
    inline Vector3d get_frame_offset(
        const std::string &from_frame,
        const std::string &to_frame
    ){
        return m_data->oMf[m_model->getFrameId(to_frame)].translation() -
        m_data->oMf[m_model->getFrameId(from_frame)].translation();
    }

    /**
     * @brief Get the error screw error between two frames.
     * 
     * @param from_frame The first frame.
     * @param to_frame The second frame.
     * 
     * @returns The offset and orientation difference between the frames.
     */
    inline Vector6d get_frame_error(
        const std::string &from_frame,
        const std::string &to_frame
    ) {
        using namespace pinocchio;

        return log6(
            m_data->oMf[m_model->getFrameId(to_frame)].actInv(
                m_data->oMf[m_model->getFrameId(from_frame)]
            )
        ).toVector();
    }

    /**
     * @brief Get the pose of a frame.
     * 
     * @param frame 
     * @return std::tuple<Vector3d, Quaterniond> 
     */
    inline std::tuple<Vector3d, Quaterniond> get_frame_pose(
        const std::string &frame
    ){
        return std::make_tuple(
            m_data->oMf[m_model->getFrameId(frame)].translation(),
            (Quaterniond)m_data->oMf[m_model->getFrameId(frame)].rotation()
        );
    }

protected:

    /**
     * @brief Calculates kinematic information after position and velocity
     * updates.
     */
    void calculate();

    /**
     * @brief Initialise the dynamics parameters.
     * 
     * @param configuration The configuration of the dynamics.
     * @param model Pointer to the pinocchio model.
     * @param data Pointer to the pinocchio data.
     * @param geometry Pointer to the pinocchio geometry model.
     * @param end_effector_index Index into the vector of model joint frames for
     * the end effector.
     * @param dynamics_forecast_handle An optional handle to the forecasted
     * dynamics, forwarded to the objective function.
     */
    PinocchioDynamics(
        const Configuration &configuration,
        std::unique_ptr<pinocchio::Model> &&model,
        std::unique_ptr<pinocchio::Data> &&data,
        std::unique_ptr<pinocchio::GeometryModel> &&geometry_model,
        std::unique_ptr<pinocchio::GeometryData> &&geometry_data,
        std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle,
        std::size_t end_effector_index
    );

    /// The configuration of the pinocchio.
    Configuration m_configuration;

    /// The robot model.
    std::unique_ptr<pinocchio::Model> m_model;

    /// Data used for model calculations.
    std::unique_ptr<pinocchio::Data> m_data;

    /// Model used for collision detection.
    std::unique_ptr<pinocchio::GeometryModel> m_geometry_model;

    /// Data used for collision detection.
    std::unique_ptr<pinocchio::GeometryData> m_geometry_data;

    /// Index of the end effector frame in the frame vector.
    std::size_t m_end_effector_frame_index;

    /// The current joint positions.
    Eigen::Vector<double, DoF::JOINTS> m_joint_position;

    /// The current joint velocities.
    Eigen::Vector<double, DoF::JOINTS> m_joint_velocity;

    /// The current torques applied by the controller to the joints.
    Eigen::Vector<double, DoF::JOINTS> m_joint_torque;

    /// The acceleration of the joints from forward dynamics.
    Eigen::Vector<double, DoF::JOINTS> m_joint_acceleration;

    /// The kinematic state of the end effector.
    EndEffectorState m_end_effector_state;

    /// Optional pointer to the force predictor.
    std::unique_ptr<DynamicsForecast::Handle> m_forecast;

    /// The current power consumption.
    double m_power;

    /// The energy tank.
    EnergyTank m_energy_tank;

    /// The state to yield to the mppi trajectory generator.
    State m_state;

    /// Time of the last dynamics set.
    double m_time;
};

} // namespace FrankaRidgeback
