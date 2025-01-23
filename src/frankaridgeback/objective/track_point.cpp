#include "frankaridgeback/objective/track_point.hpp"

#include "frankaridgeback/dynamics.hpp"

#include <iostream>
#include <random>

namespace FrankaRidgeback {

double TrackPoint::get_cost(
    const VectorXd & s,
    const VectorXd & /*control */,
    mppi::Dynamics *d,
    double /*dt */
) {
    const State &state = s;
    auto dynamics = static_cast<Dynamics*>(d);

    double cost = point_cost(dynamics);

    if (m_configuration.enable_joint_limits) {
        cost += joint_limit_cost(state);
    }

    if (m_configuration.enable_self_collision_avoidance) {
        cost += self_collision_cost(dynamics);
    }

    if (m_configuration.enable_reach_limits) {
        cost += reach_cost(dynamics);
    }

    return cost;
}

double TrackPoint::point_cost(Dynamics *dynamics)
{
    double distance = (
        dynamics->get_end_effector_state().position - m_configuration.point
    ).norm();

    return 100.0 * std::pow(distance, 2);
}

double TrackPoint::joint_limit_cost(const State &state)
{
    // Statically initialise the lower limits.
    static VectorXd lower_limit = []{
        VectorXd lower_limit = VectorXd(DoF::JOINTS);
        lower_limit <<
            -2.0, -2.0, -6.28,
            -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
            0.5, 0.5;
        return lower_limit;
    }();

    // Statically initialise the upper limits.
    static VectorXd upper_limit = []{
        VectorXd upper_limit(DoF::JOINTS);
        upper_limit <<
            2.0, 2.0, 6.28,
            2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973,
            0.5, 0.5;
        return upper_limit;
    }();

    double cost = 0.0;

    // Joint limits.
    for (size_t i = 0; i < 10; i++) {
        if (state(i) < lower_limit[i])
            cost += 1000.0 + 100000.0 * std::pow(lower_limit[i] - state(i), 2);

        if (state(i) > upper_limit[i])
            cost += 1000.0 + 100000.0 * std::pow(state(i) - upper_limit[i], 2);
    }

    return cost;
}

double TrackPoint::self_collision_cost(Dynamics *dynamics)
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
            double collision = radii - distance;

            // Barrier function cost.
            cost += m_configuration.self_collision_limit(collision);
        }
    }

    return cost;
}

double TrackPoint::reach_cost(Dynamics *dynamics)
{
    // Position of the end effector.
    Vector3d end_effector = dynamics->get_end_effector_state().position;

    // Rotation from world frame to infront of the robot.
    AngleAxisd rotate_forward = AngleAxisd(dynamics->get_state()[2], Vector3d::UnitZ());

    // Unit vector in the forward direction of the robot, normal to the
    // infront / behind plane.
    // Vector3d forward = rotate_forward * Vector3d::UnitX();

    // Position of the plane in front of the robot.
    Vector3d robot = (
        dynamics->get_frame_position(Frame::ARM_MOUNT_JOINT) +
        rotate_forward * Vector3d(0.3, 0, 0.15) // offset from base link
    );

    // Vector from plane origin to end effector.
    Vector3d to_end_effector = end_effector - robot;

    // Limit reach.
    double reach = to_end_effector.norm();
    return m_configuration.maximum_reach_limit(reach);
}

} // namespace FrankaRidgeback
