#include "frankaridgeback/objective/assisted_manipulation.hpp"

#include <iostream>

#include "frankaridgeback/dynamics.hpp"

std::unique_ptr<AssistedManipulation> AssistedManipulation::create(
    const Configuration &configuration
) {
    return std::unique_ptr<AssistedManipulation>(
        new AssistedManipulation(configuration)
    );
}

AssistedManipulation::AssistedManipulation(
    const Configuration &configuration
  ) : m_configuration(configuration)
    , m_cost(0.0)
    , m_power_cost(0.0)
    , m_manipulability_cost(0.0)
    , m_joint_cost(0.0)
    , m_reach_cost(0.0)
{}

double AssistedManipulation::get_cost(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & c,
    mppi::Dynamics *d,
    double /*time */
) {
    const FrankaRidgeback::State &state = s;
    const FrankaRidgeback::Control &control = c;

    auto dynamics = static_cast<FrankaRidgeback::Dynamics*>(d);

    if (m_configuration.enable_minimise_power)
        m_power_cost = power_cost(dynamics);

    if (m_configuration.enable_maximise_manipulability)
        m_manipulability_cost = manipulability_cost(dynamics);

    if (m_configuration.enable_joint_limit)
        m_joint_cost = joint_limit_cost(state);

    if (m_configuration.enable_reach_limit)
        m_reach_cost = reach_cost(dynamics);

    if (m_configuration.enable_variable_damping)
        m_variable_damping_cost = variable_damping_cost(state);

    m_cost = (
        m_power_cost +
        m_manipulability_cost +
        m_joint_cost +
        m_reach_cost
    );

    return m_cost;
}

double AssistedManipulation::power_cost(FrankaRidgeback::Dynamics *dynamics)
{
    // Minimise power.
    const auto &maximum = m_configuration.maximum_power;
    double power = dynamics->get_power();

    if (power < maximum.limit)
        return 0.0;

    return (
        maximum.constant_cost +
        std::max(0.0, maximum.quadratic_cost * (power - maximum.limit))
    );
}

double AssistedManipulation::manipulability_cost(FrankaRidgeback::Dynamics *dynamics)
{
    const auto &minimum = m_configuration.minimum_manipulability;
    const auto &jacobian = dynamics->get_end_effector_state().jacobian;

    m_space_jacobian = jacobian * jacobian.transpose();

    // Value proportional to the volumne of the manipulability ellipsoid.
    double ellipsoid_volume = std::sqrt(m_space_jacobian.determinant());

    // Clip to a small value to prevent division by zero on singularity.
    if (ellipsoid_volume < 1e-10)
        ellipsoid_volume = 1e-10;

    return minimum.quadratic_cost * std::pow(1 / ellipsoid_volume, 2);
}

double AssistedManipulation::joint_limit_cost(const FrankaRidgeback::State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < FrankaRidgeback::DoF::JOINTS; i++) {
        const auto &lower = m_configuration.lower_joint_limit[i];
        const auto &upper = m_configuration.upper_joint_limit[i];

        double position = state.position()(i);

        if (position < lower.limit) {
            cost += (
                lower.constant_cost +
                lower.quadratic_cost * std::pow(lower.limit - position, 2)
            );
        }
        else if (position > upper.limit) {
            cost += (
                upper.constant_cost +
                lower.quadratic_cost * std::pow(position - upper.limit, 2)
            );
        }
    }

    return cost;
}

double AssistedManipulation::reach_cost(FrankaRidgeback::Dynamics *dynamics)
{
    double cost = 0.0;
    const auto &min = m_configuration.minimum_reach;
    const auto &max = m_configuration.maximum_reach;

    double offset = dynamics->get_frame_offset(
        FrankaRidgeback::Frame::BASE_LINK_JOINT,
        FrankaRidgeback::Frame::PANDA_GRASP_JOINT
    ).norm();

    if (offset < min.limit) {
        cost += (
            min.constant_cost +
            min.quadratic_cost * std::pow(offset - min.limit, 2)
        );
    }
    else if (offset > max.limit) {
        cost += (
            max.constant_cost +
            max.quadratic_cost * std::pow(offset - max.limit, 2)
        );
    }

    return 0.0;
}

double AssistedManipulation::variable_damping_cost(const FrankaRidgeback::State &state)
{
    // double velocity = m_model->end_effector_velocity().norm();

    // double expected_damping = (
    //     m_configuration.variable_damping_maximum *
    //     std::exp(-m_configuration.variable_damping_dropoff * velocity)
    // );

    // // The expected resistive force against the external force.
    // double expected_inertia = expected_damping * velocity;

    // double inertia = m_model->get_data().Jm_model->get_data()->M
    // // Subtract the 
    // double residual_force = (
    //     state.end_effector_force().norm() - expected_damping * velocity
    // );

    // // // F = c * x_dot therefore c = F \ x_dot
    // double damping = state.end_effector_torque().norm() / velocity;

    // m_variable_damping_cost.quadratic * ;

    return 0.0;
}
