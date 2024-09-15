#include "frankaridgeback/objective/track_point.hpp"

#include "frankaridgeback/dynamics.hpp"

#include <iostream>
#include <random>

double TrackPoint::get_cost(
    const Eigen::VectorXd & s,
    const Eigen::VectorXd & /*control */,
    mppi::Dynamics *d,
    double /*dt */
) {
    const FrankaRidgeback::State &state = s;
    auto dynamics = static_cast<FrankaRidgeback::Dynamics*>(d);

    auto position = dynamics->get_end_effector_position();

    // Target end effector at point (1.0, 1.0, 1.0)    
    Eigen::Vector3d target = Eigen::Vector3d(1.0, 1.0, 1.0);

    double cost = 100.0 * std::pow((position - target).norm(), 2);

    Eigen::VectorXd lower_limit(FrankaRidgeback::DoF::JOINTS);
    lower_limit <<
        -2.0, -2.0, -6.28,
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
        0.5, 0.5;

    Eigen::VectorXd upper_limit(FrankaRidgeback::DoF::JOINTS);
    upper_limit <<
        2.0, 2.0, 6.28,
        2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973,
        0.5, 0.5;

    // Joint limits.
    for (size_t i = 0; i < 10; i++) {
        if (state(i) < lower_limit[i])
            cost += 1000 + 100000 * std::pow(lower_limit[i] - state(i), 2);

        if (state(i) > upper_limit[i])
            cost += 1000 + 100000 * std::pow(state(i) - upper_limit[i], 2);
    }

    // Eigen::Vector3d collision_vector = dynamics->offset("panda_link0", "panda_link7");
    // cost += 1000 * std::pow(std::max(0.0, 0.35 - collision_vector.norm()), 2);

    return cost;
}
