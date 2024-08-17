#include "test/case/reach.hpp"

static ReachForPoint::Configuration default_configuration {
    .folder = "reach",
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .objective = {
        .point = Eigen::Vector3d(1.0, 1.0, 1.0),
        .model = {
            .filename = "",
            .end_effector_frame = "panda_grasp"
        }
    },
    .mppi = {
        .initial_state = FrankaRidgeback::State::Zero(),
        .rollouts = 20,
        .keep_best_rollouts = 10,
        .time_step = 0.1,
        .horison = 1.0,
        .gradient_step = 1.0,
        .cost_scale = 10.0,
        .cost_discount_factor = 1.0,
        .covariance = FrankaRidgeback::Control{
            0.0, 0.0, 0.2, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
            0.0, 0.0 // gripper
        }.asDiagonal(),
        .control_bound = false,
        .control_min = FrankaRidgeback::Control{
            -0.2, -0.2, -0.2, // base
            -5.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, //arm
            -0.05, -0.05 // gripper
        },
        .control_max = FrankaRidgeback::Control{
            0.2, 0.2, 0.2, // base
            5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // arm
            0.05, 0.05 // gripper
        },
        .control_default = FrankaRidgeback::Control::Zero(),
        .smoothing = std::nullopt,
        .threads = 12
    },
    .actor = {
        .controller_rate = 0.3,
        .controller_substeps = 10,
        .urdf_filename = "",
        .end_effector_frame = "panda_grasp_joint",
        .initial_state = FrankaRidgeback::State::Zero(),
        .proportional_gain = FrankaRidgeback::Control{
            0.0, 0.0, 0.0, // base
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
            100.0, 100.0 //gripper
        },
        .differential_gain = FrankaRidgeback::Control{
            1000.0, 1000.0, 1.0, // base
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
            50.0, 50.0 // gripper
        },
    },
    .mppi_logger = {
        .folder = "",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = 0
    }
};

std::unique_ptr<Test> ReachForPoint::create(json &patch)
{
    Configuration configuration = default_configuration;

    // If configuration overrides were provided, apply them based on the json
    // patch specification.
    try {
        if (!patch.is_null())
            configuration = ((json)default_configuration).merge_patch(patch);
    }
    catch (const json::exception &err) {
        std::cerr << "error when parsing json configuration: " << err.what() << std::endl;
        std::cerr << "patched configuration was " << patch.dump(4) << std::endl;
        return nullptr;
    }

    // Urdf file locations are runtime only.
    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();
    configuration.objective.model.filename = urdf;
    configuration.actor.urdf_filename = urdf;

    std::unique_ptr<Simulator> simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto cost = TrackPoint::create(configuration.objective);
    if (!cost) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    auto dynamics = FrankaRidgeback::Dynamics::create();
    if (!dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto robot = FrankaRidgebackActor::create(
        configuration.actor,
        configuration.mppi,
        simulator.get(),
        std::move(dynamics),
        std::move(cost)
    );

    if (!robot) {
        std::cerr << "failed to create frankaridgeback actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(robot);

    configuration.mppi_logger.folder = cwd / "mppi";
    configuration.mppi_logger.rollouts = robot->get_trajectory().get_rollout_count();

    auto logger = logger::MPPI::create(configuration.mppi_logger);
    if (!logger) {
        return nullptr;
    }

    auto test = std::unique_ptr<ReachForPoint>(new ReachForPoint());
    test->m_simulator = std::move(simulator);
    test->m_robot = std::move(robot);
    test->m_mppi_logger = std::move(logger);

    return test; 
}

bool ReachForPoint::run()
{
    while (m_simulator->get_time() < 30) {
        // raisim::TimedLoop(m_simulator->get_time_step() * 1e6);
        m_simulator->step();
        m_mppi_logger->log(m_robot->get_trajectory());
    }

    return true;
}
