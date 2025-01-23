#include "logging/frankaridgeback.hpp"

namespace logger {

std::unique_ptr<FrankaRidgebackDynamics> FrankaRidgebackDynamics::create(
    const Configuration &configuration
) {
    auto logger = std::unique_ptr<FrankaRidgebackDynamics>(
        new FrankaRidgebackDynamics()
    );

    if (configuration.log_joints) {
        logger->m_joint_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "joints.csv",
            .header = CSV::make_header(
                "time",
                "x", "y", "yaw",
                "arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7",
                "gripper_x", "gripper_y"
            )
        });
    }

    if (configuration.log_control) {
        logger->m_control_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "control.csv",
            .header = CSV::make_header(
                "time",
                "x", "y", "yaw",
                "arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7",
                "gripper_x", "gripper_y"
            )
        });
    }

    if (configuration.log_end_effector_position) {
        logger->m_position_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_position.csv",
            .header = CSV::make_header("time", "x", "y", "z")
        });
    }

    if (configuration.log_end_effector_orientation) {
        logger->m_orientation_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_orientation.csv",
            .header = CSV::make_header("time", "x", "y", "z", "w")
        });
    }

    if (configuration.log_end_effector_velocity) {
        logger->m_linear_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_velocity.csv",
            .header = CSV::make_header("time", "vx", "vy", "vz")
        });
        logger->m_angular_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_velocity.csv",
            .header = CSV::make_header("time", "wx", "wy", "wz")
        });
    }

    if (configuration.log_end_effector_acceleration) {
        logger->m_linear_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_acceleration.csv",
            .header = CSV::make_header("time", "ax", "ay", "az")
        });
        logger->m_angular_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_acceleration.csv",
            .header = CSV::make_header("time", "alpha_x", "alpha_y", "alpha_z")
        });
    }

    if (configuration.log_power) {
        logger->m_power_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "power.csv",
            .header = CSV::make_header("time", "power")
        });
    }

    if (configuration.log_tank_energy) {
        logger->m_energy_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "tank_energy.csv",
            .header = CSV::make_header("time", "energy")
        });
    }

    bool error = (
        (configuration.log_end_effector_position && !logger->m_position_logger) ||
        (configuration.log_end_effector_orientation && !logger->m_orientation_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_linear_velocity_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_angular_velocity_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_linear_acceleration_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_angular_acceleration_logger) ||
        (configuration.log_power && !logger->m_power_logger) ||
        (configuration.log_tank_energy && !logger->m_energy_logger)
    );

    if (error) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    return logger;
}

void FrankaRidgebackDynamics::log(
    double time,
    const FrankaRidgeback::Dynamics &dynamics
) {
    const auto &end_effector = dynamics.get_end_effector_state();

    if (m_joint_logger)
        m_joint_logger->write(time, dynamics.get_joint_position());

    if (m_position_logger)
        m_position_logger->write(time, end_effector.position);

    if (m_orientation_logger)
        m_orientation_logger->write(time, end_effector.orientation.coeffs());

    if (m_linear_velocity_logger)
        m_linear_velocity_logger->write(time, end_effector.linear_velocity);

    if (m_angular_velocity_logger)
        m_angular_velocity_logger->write(time, end_effector.angular_velocity);

    if (m_linear_acceleration_logger)
        m_linear_acceleration_logger->write(time, end_effector.linear_acceleration);

    if (m_angular_acceleration_logger)
        m_angular_acceleration_logger->write(time, end_effector.angular_acceleration);

    if (m_power_logger) {
        m_power_logger->write(time, dynamics.get_joint_power());
        m_power_logger->write(time, dynamics.get_external_power());
    }

    if (m_energy_logger)
        m_energy_logger->write(time, dynamics.get_tank_energy());
}

void FrankaRidgebackDynamics::log_control(double time, const VectorXd &control)
{
    if (m_control_logger)
        m_control_logger->write(time, control);
}

std::unique_ptr<FrankaRidgebackDynamicsForecast> FrankaRidgebackDynamicsForecast::create(
    const Configuration &configuration
) {
    auto logger = std::unique_ptr<FrankaRidgebackDynamicsForecast>(
        new FrankaRidgebackDynamicsForecast()
    );

    if (configuration.log_joints) {
        logger->m_joint_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "joints.csv",
            .header = CSV::make_header(
                "time",
                "x", "y", "yaw",
                "arm1", "arm2", "arm3", "arm4", "arm5", "arm6", "arm7",
                "gripper_x", "gripper_y"
            )
        });
    }

    if (configuration.log_end_effector_position) {
        logger->m_position_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_position.csv",
            .header = CSV::make_header("update_time", "time", "x", "y", "z")
        });
    }

    if (configuration.log_end_effector_orientation) {
        logger->m_orientation_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_orientation.csv",
            .header = CSV::make_header("update_time", "time", "x", "y", "z", "w")
        });
    }

    if (configuration.log_end_effector_velocity) {
        logger->m_linear_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_velocity.csv",
            .header = CSV::make_header("update_time", "time", "vx", "vy", "vz")
        });
        logger->m_angular_velocity_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_velocity.csv",
            .header = CSV::make_header("update_time", "time", "wx", "wy", "wz")
        });
    }

    if (configuration.log_end_effector_acceleration) {
        logger->m_linear_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_linear_acceleration.csv",
            .header = CSV::make_header("update_time", "time", "ax", "ay", "az")
        });
        logger->m_angular_acceleration_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "end_effector_angular_acceleration.csv",
            .header = CSV::make_header("update_time", "time", "alpha_x", "alpha_y", "alpha_z")
        });
    }

    if (configuration.log_power) {
        logger->m_power_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "power.csv",
            .header = CSV::make_header("update_time", "time", "power")
        });
    }

    if (configuration.log_tank_energy) {
        logger->m_energy_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "tank_energy.csv",
            .header = CSV::make_header("update_time", "time", "energy")
        });
    }

    if (configuration.log_wrench) {
        logger->m_wrench_logger = CSV::create(CSV::Configuration{
            .path = configuration.folder / "wrench.csv",
            .header = CSV::make_header("update_time", "time", "fx", "fy", "fz", "tau_x", "tau_y", "tau_z")
        });
    }

    bool error = (
        (configuration.log_end_effector_position && !logger->m_position_logger) ||
        (configuration.log_end_effector_orientation && !logger->m_orientation_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_linear_velocity_logger) ||
        (configuration.log_end_effector_velocity && !logger->m_angular_velocity_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_linear_acceleration_logger) ||
        (configuration.log_end_effector_acceleration && !logger->m_angular_acceleration_logger) ||
        (configuration.log_power && !logger->m_power_logger) ||
        (configuration.log_tank_energy && !logger->m_energy_logger) ||
        (configuration.log_wrench && !logger->m_wrench_logger)
    );

    if (error) {
        std::cerr << "failed to create csv logger" << std::endl;
        return nullptr;
    }

    return logger;
}

void FrankaRidgebackDynamicsForecast::log(
    const FrankaRidgeback::DynamicsForecast &forecast_dynamics
) {
    double time = forecast_dynamics.get_last_forecast_time();
    if (time == last_forecast_time)
        return;

    double time_step = forecast_dynamics.get_time_step();
    auto &trajectory = forecast_dynamics.get_end_effector_trajectory();
    // auto &wrench = forecast_dynamics.get_wrench_trajectory();

    for (std::size_t i = 0; i < trajectory.size(); i++) {
        auto &state = trajectory[i];
        double t = time + i * time_step;

        if (m_joint_logger)
            m_joint_logger->write(time, forecast_dynamics.get_joint_position()[i]);

        if (m_position_logger)
            m_position_logger->write(time, t, state.position);

        if (m_orientation_logger)
            m_orientation_logger->write(time, t, state.orientation.coeffs());

        if (m_linear_velocity_logger)
            m_linear_velocity_logger->write(time, t, state.linear_velocity);

        if (m_angular_velocity_logger)
            m_angular_velocity_logger->write(time, t, state.angular_velocity);

        if (m_linear_acceleration_logger)
            m_linear_acceleration_logger->write(time, t, state.linear_acceleration);

        if (m_angular_acceleration_logger)
            m_angular_acceleration_logger->write(time, t, state.angular_acceleration);

        if (m_power_logger) {
            m_power_logger->write(time, t, forecast_dynamics.get_joint_power_trajectory()[i]);
            m_power_logger->write(time, t, forecast_dynamics.get_external_power_trajectory()[i]);
        }

        if (m_energy_logger)
            m_energy_logger->write(time, t, forecast_dynamics.get_energy_trajectory()[i]);

        if (m_wrench_logger)
            m_wrench_logger->write(time, t, forecast_dynamics.get_wrench_trajectory()[i]);
    }

    last_forecast_time = time;
}

} // namespace logger
