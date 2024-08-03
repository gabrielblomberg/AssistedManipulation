#include "tests/reach.hpp"

std::unique_ptr<TestBase> ReachForPoint::create()
{
    std::unique_ptr<Simulator> simulator = Simulator::create({
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    });

    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto cwd = std::filesystem::current_path();
    std::string urdf = (cwd / "model/robot.urdf").string();

    FrankaRidgebackActor::Configuration configuration {
        .mppi = {
            .dynamics = FrankaRidgeback::Dynamics::create(),
            .cost = TrackPoint::create({
                .point = Eigen::Vector3d(1.0, 1.0, 1.0),
                .model = {
                    .filename = urdf,
                    .end_effector_frame = "panda_grasp"
                }
            }),
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
            .filter = std::nullopt,
            .threads = 12
        },
        .controller_rate = 0.3,
        .controller_substeps = 10,
        .urdf_filename = urdf,
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
    };

    if (!configuration.mppi.cost) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    if (!configuration.mppi.dynamics) {
        std::cerr << "failed to create mppi dynamics" << std::endl;
        return nullptr;
    }

    auto robot = FrankaRidgebackActor::create(
        std::move(configuration),
        simulator.get()
    );

    if (!robot) {
        std::cerr << "failed to create FrankaRidgebackActor actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(robot);

    auto logger = logger::MPPI::create(logger::MPPI::Configuration{
        .folder = cwd / "mppi",
        .trajectory = &robot->get_trajectory()
    });

    if (!logger) {
        return nullptr;
    }

    auto test = std::unique_ptr<ReachForPoint>(new ReachForPoint());
    test->m_simulator = std::move(simulator);
    test->m_robot = std::move(robot);
    test->m_mppi_logger = std::move(logger);

    return test; 
}

void ReachForPoint::run()
{
    for (;;) {
        raisim::TimedLoop(simulator->get_time_step() * 1e6);
        simulator->step();
        logger->log(robot->get_trajectory());
    }
}
