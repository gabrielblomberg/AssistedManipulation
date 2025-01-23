#include "frankaridgeback/objective/assisted_manipulation.hpp"

#include <iostream>

#include "frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

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
{
    reset(0.0);
}

void AssistedManipulation::reset(double time)
{
    m_initial_time = time;
    m_joint_cost = 0.0;
    m_self_collision_cost = 0.0;
    m_workspace_cost = 0.0;
    m_energy_cost = 0.0;
    m_velocity_cost = 0.0;
    m_trajectory_cost = 0.0;
    m_manipulability_cost = 0.0;
    m_cost = 0.0;
}

double AssistedManipulation::get_cost(
    const Eigen::VectorXd &s,
    const Eigen::VectorXd &c,
    mppi::Dynamics *d,
    double time
) {
    const State &state = s;
    // const Control &control = c;

    auto dynamics = static_cast<Dynamics*>(d);

    double cost = 0.0;

    if (m_configuration.enable_joint_limit)
        cost += joint_limit_cost(state);

    if (m_configuration.enable_self_collision_limit)
        cost += self_collision_cost(dynamics);

    if (m_configuration.enable_workspace_limit)
        cost += workspace_cost(dynamics);

    if (m_configuration.enable_energy_limit)
        cost += energy_cost(dynamics);

    if (m_configuration.enable_velocity_cost)
        cost += velocity_cost(state);

    if (m_configuration.enable_trajectory_cost)
        cost += trajectory_cost(dynamics, time);

    if (m_configuration.enable_manipulability_cost)
        cost += manipulability_cost(dynamics);

    return cost;
}

double AssistedManipulation::joint_limit_cost(const State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < DoF::JOINTS; i++) {
        const auto &lower = m_configuration.lower_joint_limit[i];
        const auto &upper = m_configuration.upper_joint_limit[i];
        double position = state.position()(i);
        double c = lower(position) + upper(position);
        cost += c;
    }

    m_joint_cost += cost;
    return cost;
}

double AssistedManipulation::self_collision_cost(Dynamics *dynamics)
{
    const static std::vector<std::tuple<Link, std::vector<Link>>> CHECK_COLLSION = {{
        {Link::PIVOT, {
            Link::PANDA_LINK3,
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK1, {
            Link::PANDA_LINK3,
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK2, {
            Link::PANDA_LINK4,
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK3, {
            Link::PANDA_LINK5,
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK4, {
            Link::PANDA_LINK6,
            Link::PANDA_LINK7,
        }},
        {Link::PANDA_LINK5, {
            Link::PANDA_LINK7,
        }}
    }};

    // const auto &self_collision = m_configuration.self_collision_limit;
    double cost = 0.0;

    for (std::size_t i = 0; i < CHECK_COLLSION.size(); i++) {
        const auto &[first_link, against] = CHECK_COLLSION[i];

        for (std::size_t j = 0; j < against.size(); j++) {
            Link second_link = against[j];

            // Distance between sphere origins.
            double distance = (
                dynamics->get_link_position(first_link) -
                dynamics->get_link_position(second_link)
            ).norm();

            // Sum of sphere radii.
            double radii = (
                m_configuration.self_collision_radii[(std::size_t)first_link - 3] +
                m_configuration.self_collision_radii[(std::size_t)second_link - 3]
            );

            // Collision vector. Positive when colliding.
            double collision = distance - radii;

            // Barrier function cost.
            cost += m_configuration.self_collision_limit(collision);
        }
    }

    m_self_collision_cost += cost;
    return cost;
}

double AssistedManipulation::workspace_cost(Dynamics *dynamics)
{
    double cost = 0.0;

    // Position of the end effector.
    Vector3d end_effector = dynamics->get_end_effector_state().position;

    // Rotation from world frame to infront of the robot.
    AngleAxisd rotate_forward = AngleAxisd(dynamics->get_state()[2], Vector3d::UnitZ());

    // Unit vector in the forward direction of the robot, normal to the
    // infront / behind plane.
    Vector3d forward = rotate_forward * Vector3d::UnitX();

    // Position of the plane in front of the robot.
    Vector3d robot = (
        dynamics->get_frame_position(Frame::ARM_MOUNT_JOINT) +
        rotate_forward * Vector3d(0.1, 0, 0.15) // offset from base link
    );

    // Vector from plane origin to end effector.
    Vector3d to_end_effector = end_effector - robot;

    // Distance from the infront / behind plane to the end effector.
    double projection = (
        to_end_effector.dot(forward) / forward.dot(forward)
    );

    // Keep end effector in front of the robot.
    cost += m_configuration.workspace_limit_infront(projection);

    // Limit reach.
    double reach = to_end_effector.norm();
    cost += m_configuration.workspace_limit_reach(reach);

    // Calculate yaw between body and end effector.
    Vector2d v1 = to_end_effector.head<2>();
    Vector2d v2 = forward.head<2>();
    double yaw = std::acos(v1.dot(v2) / v1.norm() / v2.norm());
    if (!std::isnan(yaw)) {
        cost += m_configuration.workspace_cost_yaw(std::fabs(yaw));
    }

    // Keep the end effector above the robot.
    double height = end_effector[2] - robot[2];
    cost += m_configuration.workspace_limit_above(height);

    m_workspace_cost += cost;
    return cost;
}

double AssistedManipulation::energy_cost(Dynamics *dynamics)
{
    double energy = dynamics->get_tank_energy();

    double cost = (
        m_configuration.energy_limit_below(energy) +
        m_configuration.energy_limit_above(energy)
    );

    m_energy_cost += cost;
    return cost;
}

double AssistedManipulation::velocity_cost(const State &state)
{
    double cost = 0.0;

    for (size_t i = 0; i < DoF::JOINTS; i++) {
        const auto &objective = m_configuration.velocity_cost[i];
        cost += objective.quadratic_cost * std::pow(std::fabs(state.velocity()(i)), 2);
    }

    m_velocity_cost += cost;
    return cost;
}

double AssistedManipulation::trajectory_cost(const Dynamics *dynamics, double time)
{
    if (!dynamics->get_forecast())
        return 0.0;

    const auto &state = dynamics->get_end_effector_state();
    const auto forecast = dynamics->get_forecast()->get();

    // Vector3d force = forecast->get_wrench_trajectory()[0].head<3>();
    Vector3d force = forecast->get_end_effector_wrench(time).head<3>();

    Vector3d target_vector = (
        m_configuration.trajectory_target_scale * force
    ).cwiseMin(
        m_configuration.trajectory_target_maximum
    ).cwiseMax(
        -m_configuration.trajectory_target_maximum
    );

    // target_vector = (
    //     forecast->get_end_effector_trajectory()[0].position + target_vector - state.position
    // );

    double distance = target_vector.norm();

    double cost = 0.0;
    if (distance > m_configuration.trajectory_position_threshold) {
        cost += m_configuration.trajectory_position_cost(distance);

        // Project linear velocity onto the target vector.
        double projection = (
            state.linear_velocity.dot(target_vector) /
            target_vector.dot(target_vector)
        );

        // Projected component onto the target vector.
        projection = std::copysign(1.0, projection) * (target_vector * projection).norm();

        // Calculate ideal velocity field.
        double velocity_target = std::clamp(
            std::exp(m_configuration.trajectory_velocity_dropoff * distance) - 1,
            m_configuration.trajectory_velocity_minimum,
            m_configuration.trajectory_velocity_maximum
        );

        double velocity_error = std::fabs(velocity_target - projection);

        // Make relative to the velocity limit.
        cost += m_configuration.trajectory_velocity_cost(velocity_error);
    }

    m_trajectory_cost += cost;
    return cost;
}

double AssistedManipulation::manipulability_cost(Dynamics *dynamics)
{
    double cost = 0.0;

    const auto &jacobian = dynamics->get_end_effector_state().jacobian.rightCols(
        DoF::JOINTS - DoF::BASE
    ).topLeftCorner(3, DoF::ARM);

    m_manipulability_matrix = jacobian * jacobian.transpose();

    // auto eigenvalues = m_manipulability_matrix.eigenvalues().real().eval();
    // double min = eigenvalues.minCoeff();
    // double max = eigenvalues.maxCoeff();
    // auto mu_2 = max / min;
    // cost = m_configuration.manipulability_cost(mu_2 - 1);

    // Value proportional to the volume of the manipulability ellipsoid.
    double volume = std::sqrt(m_manipulability_matrix.determinant());
    if (std::isnan(volume))
        volume = 1e-5;
    else
        volume = std::clamp(volume, 1e-5, 1e5);

    cost = m_configuration.manipulability_cost(1 / volume);

    m_manipulability_cost += cost;
    return cost;
}

} // namespace FrankaRidgeback
