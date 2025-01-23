#pragma once

#include <array>

#include "controller/eigen.hpp"
#include "controller/mppi.hpp"
#include "controller/forecast.hpp"
#include "frankaridgeback/dof.hpp"
#include "frankaridgeback/control.hpp"
#include "frankaridgeback/state.hpp"

namespace FrankaRidgeback {

// Forward declaration of the abstrat class dynamics. See interface at the
// bottom of this file.
class Dynamics;

/**
 * @brief Matrix mapping of the end effector velocity to the joint velocities.
 */
using Jacobian = Eigen::Matrix<double, 6, DoF::JOINTS>;

/**
 * @brief Enumeration of all the robot frames.
 */
enum class Frame {
    WORLD_JOINT,
    X_BASE_JOINT,
    Y_BASE_JOINT,
    PIVOT_JOINT,
    PANDA_JOINT1,
    OMNI_BASE_FLANGE,
    BASE_LINK_JOINT,
    MID_MOUNT_JOINT,
    RIGHT_SIDE_COVER_LINK_JOINT,
    LEFT_SIDE_COVER_LINK_JOINT,
    FRONT_COVER_LINK_JOINT,
    REAR_COVER_LINK_JOINT,
    FRONT_LIGHTS_LINK_JOINT,
    REAR_LIGHTS_LINK_JOINT,
    TOP_LINK_JOINT,
    AXLE_JOINT,
    IMU_JOINT,
    RIDGEBACK_SENSOR_MOUNT_JOINT,
    REFERENCE_LINK_JOINT,
    ARM_MOUNT_JOINT,
    PANDA_JOINT_FRANKA_MOUNT_LINK,
    PANDA_JOINT2,
    PANDA_JOINT3,
    PANDA_JOINT4,
    PANDA_JOINT5,
    PANDA_JOINT6,
    PANDA_JOINT7,
    PANDA_FINGER_JOINT1,
    PANDA_FINGER_JOINT2,
    PANDA_JOINT8,
    PANDA_HAND_JOINT,
    PANDA_GRASP_JOINT,
    _SIZE
};

/**
 * @brief Enumeration of all the robot links.
 */
enum class Link {
    OMNI_BASE_ROOT_LINK,
    X_SLIDER,
    Y_SLIDER,
    PIVOT,
    PANDA_LINK1,
    PANDA_LINK2,
    PANDA_LINK3,
    PANDA_LINK4,
    PANDA_LINK5,
    PANDA_LINK6,
    PANDA_LINK7,
    PANDA_LEFTFINGER,
    PANDA_RIGHTFINGER,
    _SIZE
};

/**
 * @brief Mapping of frames to their named identifiers.
 */
extern std::array<std::string, (std::size_t)Frame::_SIZE> FRAME_NAMES;

/**
 * @brief Mapping of links to their named identifiers.
 */
extern std::array<std::string, (std::size_t)Link::_SIZE> LINK_NAMES;

/**
 * @brief Data structure containing frame kinematic information at a given time.
 */
struct EndEffectorState {

    /// Position (x, y, z).
    Vector3d position = Vector3d::Zero();

    /// Orientation.
    Quaterniond orientation = Quaterniond::Identity();

    /// The linear velocity (vx, vy, vz).
    Vector3d linear_velocity = Vector3d::Zero();

    /// The angular velocity (wx, wy, wz).
    Vector3d angular_velocity = Vector3d::Zero();

    /// The linear acceleration (ax, ay, az).
    Vector3d linear_acceleration = Vector3d::Zero();

    /// The angular acceleration (alpha_x, alpha_y, alpha_z).
    Vector3d angular_acceleration = Vector3d::Zero();

    /// The jacobian of the end effector in the world frame.
    Jacobian jacobian = Jacobian::Zero();
};

/**
 * @brief A forecast of the dynamics.
 */
class DynamicsForecast
{
public:

    /**
     * @brief Get a handle to the dynamics forecast.
     * 
     * @todo This is overkill, should be passing around a shared pointer to
     * DynamicsForecast instead. A handle could provide a read only interface to the
     * dynamics forecast for the objective function.
     */
    class Handle
    {
    public:

        /**
         * @brief Forecast at a time in the future.
         * @returns The predicted state.
         */
        inline DynamicsForecast *get()
        {
            return m_parent;
        };

        inline const DynamicsForecast *get() const
        {
            return m_parent;
        };

        /**
         * @brief Make a copy of the handle.
         */
        inline std::unique_ptr<Handle> copy()
        {
            return std::unique_ptr<Handle>(
                new Handle(m_parent)
            );
        };

    private:

        friend class DynamicsForecast;

        Handle(DynamicsForecast *parent)
            : m_parent(parent)
        {}

        /// Pointer to the owning dynamics forecast instance.
        DynamicsForecast *m_parent;
    };

    struct Configuration {

        /// The time step of the forecasted dynamics trajectory.
        double time_step;

        /// The time horison over which to forecast the dynamics trajectory.
        double horison;

        /// The strategy to use to forecast the external wrench.
        Forecast::Configuration end_effector_wrench_forecast;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            time_step, horison, end_effector_wrench_forecast
        )
    };

    /**
     * @brief Create a new instance of the dynamics.
     * 
     * @param configuration The configuration of the dynamics forecast.
     * @param dynamics The dynamics instance to use fore forecasting.
     * 
     * @returns A pointer to the dynamics forecast on success or nullptr on
     * failure.
     */
    static std::unique_ptr<DynamicsForecast> create(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics
    );

    /**
     * @brief Create a handle to the dynamics forecast.
     * @returns A pointer to the handle on success or nullptr on failure.
     */
    inline std::unique_ptr<Handle> create_handle()
    {
        return std::unique_ptr<Handle>(
            new Handle(this)
        );
    }

    /**
     * @brief Update the forecasted wrench with an observation.
     * 
     * @param wrench The wrench on the end effector observed.
     * @param time The time of the observation.
     */
    inline void observe_wrench(Vector6d &wrench, double time)
    {
        m_end_effector_wrench_forecast->update(wrench, time);
    }

    /**
     * @brief Update the forecasted wrench with an observation of time.
     * 
     * @param time The time of the observation.
     */
    inline void observe_time(double time)
    {
        m_end_effector_wrench_forecast->update(time);
    }

    /**
     * @brief Update the dynamics forecast over the time horison based on
     * the observed wrench trajectory.
     * 
     * @param state The initial state to forecast the dynamics from.
     * @param time The time of the initial state.
     */
    void forecast(State state, double time);

    /**
     * @brief Get the time of the last forecast.
     */
    inline double get_last_forecast_time() const
    {
        return m_last_forecast;
    }

    inline const std::vector<Eigen::Vector<double, DoF::JOINTS>> &get_joint_position() const
    {
        return m_joint_position;
    }

    /**
     * @brief Get the kinematics of the end effector.
     * @param time The time of the end effector state.
     */
    inline const EndEffectorState &get_end_effector_state(double time) const
    {
        return m_end_effector[parameterise(time)];
    }

    /**
     * @brief Get the end effector forecast wrench.
     * 
     * A prediction of the wrench applied to the end effector during rollout.
     * 
     * @returns The wrench (fx, fy, fz, tau_x, tau_y, tau_z) expected at the end
     * effector.
     */
    inline Vector6d get_end_effector_wrench(double time) const
    {
        return m_end_effector_wrench_forecast->forecast(time);
    }

    /**
     * @brief Get the wrench dynamics time step in seconds.
     */
    double get_time_step() const
    {
        return m_configuration.time_step;
    }

    /**
     * @brief Get the forecast dynamics horison in seconds.
     */
    double get_horison() const
    {
        return m_configuration.horison;
    }

    /**
     * @brief Get the full end effector trajectory over the horison every time
     * step.
     */
    const std::vector<EndEffectorState> &get_end_effector_trajectory() const
    {
        return m_end_effector;
    }

    /**
     * @brief Get the full wrench trajectory over the horison every time step.
     */
    const std::vector<Vector6d> &get_wrench_trajectory() const
    {
        return m_end_effector_wrench;
    }

    /**
     * @brief Get the power trajectory.
     */
    const std::vector<double> &get_joint_power_trajectory() const
    {
        return m_joint_power;
    }

    /**
     * @brief Get the power trajectory.
     */
    const std::vector<double> &get_external_power_trajectory() const
    {
        return m_external_power;
    }

    /**
     * @brief Get the energy trajectory.
     */
    const std::vector<double> &get_energy_trajectory() const
    {
        return m_energy;
    }

protected:

    /**
     * @brief Initialise the dynamics forecast.
     * 
     * @param configuration The configuration of the forecast.
     * @param dynamics The dynamics to use to forecast.
     * @param wrench_forecast The forecast strategy used to forecast the wrench
     * applied to the end effector.
     * @param steps The number of time steps in the time horison.
     */
    DynamicsForecast(
        const Configuration &configuration,
        std::unique_ptr<Dynamics> &&dynamics,
        std::unique_ptr<Forecast> &&wrench_forecast,
        unsigned int steps
    );

    /**
     * @brief Parameterises time to forecast index.
     * 
     * @param time The time to parameterise.
     * @returns The index into the forecast buffer at that time.
     */
    inline std::int64_t parameterise(double time) const
    {
        // Extrapolate initial wrench backwards.
        if (time < m_last_forecast)
            return 0;

        // Extrapolate last wrench forwards.
        if (time >= m_configuration.horison)
            return m_steps - 1;

        // Steps into the horison.
        double t = (time - m_last_forecast) / m_configuration.time_step;

        // The step less than or equal to t.
        return (std::int64_t)t;
    }

    /// The configuration of the dynamics forecast.
    Configuration m_configuration;

    /// The number of time steps in the horison.
    const unsigned int m_steps;

    /// The time of the last forecast.
    double m_last_forecast;

    /// The dynamics used to rollout the trajectory given the forecasted wrench.
    std::unique_ptr<Dynamics> m_dynamics;

    /// Pointer to the forecast wrench.
    std::unique_ptr<Forecast> m_end_effector_wrench_forecast;

    /// The forecast joint positions.
    std::vector<Eigen::Vector<double, DoF::JOINTS>> m_joint_position;

    /// The forecasted end effector trajectory.
    std::vector<EndEffectorState> m_end_effector;

    std::vector<double> m_joint_power;

    std::vector<double> m_external_power;

    /// The forecasted energy.
    std::vector<double> m_energy;

    /// The forecasted wrench.
    std::vector<Vector6d> m_end_effector_wrench;
};

/**
 * @brief Base class for all frankaridgeback mppi dynamics implementations.
 * 
 * Used to ensure the derived dynamics class provides methods used by objective
 * functions.
 */
class Dynamics : public mppi::Dynamics
{
public:

    /**
     * @brief Static function that returns the path to the dynamics urdf file.
     * @returns Path to the robot definition.
     */
    static inline std::filesystem::path find_path() {
        return (std::filesystem::current_path() / "model/robot.urdf").string();
    }

    /**
     * @brief Get the current joint position.
     */
    virtual Eigen::Ref<const VectorXd> get_joint_position() const = 0;

    /**
     * @brief Get the current joint velocity.
     */
    virtual Eigen::Ref<const VectorXd> get_joint_velocity() const = 0;

    /**
     * @brief Get the position of a frame.
     * 
     * @todo Make the frame parameter an enumeration.
     * 
     * @param frame The frame.
     * @returns The position of the frame.
     */
    virtual Vector3d get_frame_position(Frame frame) = 0;

    /**
     * @brief Get the orientation of a frame.
     * 
     * @todo Make the frame parameter an enumeration.
     * 
     * @param frame The frame.
     * @returns The orientation of the frame.
     */
    virtual Quaterniond get_frame_orientation(Frame frame) = 0;

    /**
     * @brief Get the origin of a link in the world frame
     * 
     * @param link The link to get the origin of.
     * @returns The position of the link in the world frame.
     */
    virtual Vector3d get_link_position(Link link) = 0;

    /**
     * @brief Get the offset between two frames.
     * 
     * @param from The first frame.
     * @param to The second frame.
     * 
     * @returns The offset.
     */
    inline Vector3d get_frame_offset(Frame from, Frame to)
    {
        return get_frame_position(to) - get_frame_position(from); 
    }

    /**
     * @brief Get the kinematics of the end effector.
     */
    virtual const EndEffectorState &get_end_effector_state() const = 0;

    /**
     * @brief Get the current power from applied joint controls.
     * 
     * This is given by the sum of generalised joint force multiplied by their
     * generalised velocities. This is torque * angular velocity for revolute
     * joints and force * linear velocity for prismatic joints.
     * 
     * @return The current power usage in joules/s.
     */
    virtual double get_joint_power() const = 0;

    /**
     * @brief Get the current power from external forces.
     * @return The current power usage in joules/s.
     */
    virtual double get_external_power() const = 0;

    /**
     * @brief Get the current energy left in the energy tank.
     */
    virtual double get_tank_energy() const = 0;

    /**
     * @brief Get a pointer to the dynamics forecast if it exists.
     * 
     * @warning May be nullptr.
     * @returns A pointer to the dynamics forecast on success or std::nullopt if
     * there is no dynamics forecast handle.
     */
    virtual const DynamicsForecast::Handle *get_forecast() const = 0;

    /**
     * @brief Get the actual wrench applied of the end effector by calls to
     * add_end_effector_true_wrench().
     * 
     * @note This is only used for simulating the robot actor.
     * 
     * @returns The actually applied wrench (fx, fy, fz, tau_x, tau_y, tau_z) at
     * the end effector in the world frame.
     */
    virtual Vector6d get_end_effector_simulated_wrench() const = 0;

    /**
     * @brief Add cumulative wrench to the end effector, to be simulated on the
     * next step, after which it is set to zero.
     * 
     * @note This is used for the dynamics forecast, simulating external
     * end effector wrench.
     * 
     * @param wrench The wrench to cumulative add to the end effector to be
     * simulated.
     */
    virtual void add_end_effector_simulated_wrench(Vector6d wrench) = 0;
};

} // namespace FrankaRidgeback
