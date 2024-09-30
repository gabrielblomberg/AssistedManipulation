#include "test/case/base.hpp"

#include "logging/file.hpp"

const BaseTest::Configuration BaseTest::DEFAULT_CONFIGURATION {
    .folder = "default",
    .duration = 30.0,
    .simulator = {
        .time_step = 0.005,
        .gravity = {0.0, 0.0, 9.81}
    },
    .actor = {
        .dynamics = {
            .type = FrankaRidgeback::SimulatorAdaptor::Type::PINOCCHIO,
            .raisim = FrankaRidgeback::RaisimDynamics::Configuration {
                .simulator = Simulator::Configuration {
                    .time_step = 0.005,
                    .gravity = {0.0, 0.0, 9.81}
                },
                .filename = "",
                .end_effector_frame = "panda_grasp_joint",
                .initial_state = FrankaRidgeback::State::Zero(),
                .proportional_gain = FrankaRidgeback::Control{
                    0.0, 0.0, 0.0, // base
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
                    100.0, 100.0
                },
                .differential_gain = FrankaRidgeback::Control{
                    1000.0, 1000.0, 1.0, // base
                    10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
                    50.0, 50.0
                },
                .energy = 10.0
            },
            .pinocchio = FrankaRidgeback::PinocchioDynamics::Configuration {
                .filename = "",
                .end_effector_frame = "panda_grasp_joint",
                .initial_state = FrankaRidgeback::State::Zero(),
                .energy = 10.0
            }
        },
        .mppi_type = FrankaRidgeback::Actor::Type::RAISIM,
        .mppi_raisim = FrankaRidgeback::RaisimDynamics::Configuration {
            .simulator = Simulator::Configuration {
                .time_step = 0.005,
                .gravity = {0.0, 0.0, 9.81}
            },
            .filename = "",
            .end_effector_frame = "panda_grasp_joint",
            .initial_state = FrankaRidgeback::State::Zero(),
            .proportional_gain = FrankaRidgeback::Control{
                0.0, 0.0, 0.0, // base
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // arm
                100.0, 100.0
            },
            .differential_gain = FrankaRidgeback::Control{
                1000.0, 1000.0, 1.0, // base
                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, // arm
                50.0, 50.0
            },
            .energy = 10.0
        },
        .mppi_pinocchio = FrankaRidgeback::PinocchioDynamics::Configuration {
            .filename = "",
            .end_effector_frame = "panda_grasp_joint",
            .initial_state = FrankaRidgeback::State::Zero(),
            .energy = 10.0
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
                0.1, 0.1, 0.2, // base
                2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, // arm
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
            // .smoothing = mppi::Configuration::Smoothing {
            //     .window = 10,
            //     .order = 1
            // },
            .threads = 12
        },
        .controller_rate = 0.15,
        .controller_substeps = 5
    },
    .objective = {
        .type = BaseTest::Objective::Type::TRACK_POINT,
        .assisted_manipulation = AssistedManipulation::DEFAULT_CONFIGURATION,
        .track_point = TrackPoint::DEFAULT_CONFIGURATION
    },
    .wrench_forecast = std::nullopt,
    .mppi_logger = {
        .folder = "",
        .state_dof = FrankaRidgeback::DoF::STATE,
        .control_dof = FrankaRidgeback::DoF::CONTROL,
        .rollouts = 0
    },
    .dynamics_logger = {
        .folder = "",
        .log_end_effector_position = true,
        .log_end_effector_velocity = true,
        .log_end_effector_acceleration = true,
        .log_power = true,
        .log_tank_energy = true,
    }
};

std::unique_ptr<BaseTest> BaseTest::create(Options &options)
{
    Configuration configuration = DEFAULT_CONFIGURATION;
    configuration.duration = options.duration;
    configuration.folder = options.folder;

    // If configuration overrides were provided, apply them based on the json
    // patch specification.
    try {
        if (!options.patch.is_null()) {
            json json_configuration = configuration;
            json_configuration.merge_patch(options.patch);
            configuration = json_configuration;
        }
    }
    catch (const json::exception &err) {
        std::cerr << "error when patching json configuration: " << err.what() << std::endl;
        std::cerr << "configuration was " << ((json)DEFAULT_CONFIGURATION).dump(4) << std::endl;
        std::cerr << "patch was " << options.patch.dump(4) << std::endl;
        return nullptr;
    }

    return create(configuration);
}

std::unique_ptr<BaseTest> BaseTest::create(const Configuration &configuration)
{
    std::unique_ptr<Simulator> simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    std::unique_ptr<mppi::Cost> objective;

    switch (configuration.objective.type)
    {
        case Objective::Type::ASSISTED_MANIPULATION: {
            if (!configuration.objective.assisted_manipulation) {
                std::cerr << "assisted manipulation objective selected with no configuration" << std::endl;
                return nullptr;
            }

            objective = AssistedManipulation::create(
                *configuration.objective.assisted_manipulation
            );
            break;
        }
        case Objective::Type::TRACK_POINT: {
            if (!configuration.objective.track_point) {
                std::cerr << "assisted manipulation objective selected with no configuration" << std::endl;
                return nullptr;
            }

            objective = TrackPoint::create(*configuration.objective.track_point);
            break;
        }
        default: {
            std::cerr << "unknown objective type " << configuration.objective.type << " provided" << std::endl;
            return nullptr;
        }
    }

    if (!objective) {
        std::cerr << "failed to create mppi cost" << std::endl;
        return nullptr;
    }

    std::unique_ptr<Forecast> wrench_forecast = nullptr;
    std::unique_ptr<Forecast::Handle> wrench_forecast_handle = nullptr;
    if (configuration.wrench_forecast) {
        wrench_forecast = Forecast::create(*configuration.wrench_forecast);
        wrench_forecast_handle = wrench_forecast->create_handle();
    }

    auto frankaridgeback = FrankaRidgeback::Actor::create(
        configuration.actor,
        simulator.get(),
        std::move(objective),
        std::move(wrench_forecast_handle),
        nullptr
    );

    if (!frankaridgeback) {
        std::cerr << "failed to create frankaridgeback actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(frankaridgeback);

    // Override rollout count of logger.
    auto mppi_log_configuration = configuration.mppi_logger;
    mppi_log_configuration.rollouts = frankaridgeback->get_controller().get_rollout_count();
    mppi_log_configuration.folder = configuration.folder / "mppi";

    auto mppi_logger = logger::MPPI::create(mppi_log_configuration);
    if (!mppi_logger) {
        std::cerr << "failed to create mppi logger" << std::endl;
        return nullptr;
    }

    auto dynamics_log_configuration = configuration.dynamics_logger;
    dynamics_log_configuration.folder = configuration.folder / "dynamics";

    auto dynamics_logger = logger::FrankaRidgebackDynamics::create(dynamics_log_configuration);
    if (!dynamics_logger) {
        std::cerr << "failed to create dynamics logger" << std::endl;
        return nullptr;
    }

    // Log the configuration used in the test.
    {
        auto file = logger::File::create(configuration.folder / "configuration.json");
        if (!file) {
            std::cerr << "failed create configuration file" << std::endl;
            return nullptr;
        }
        file->get_stream() << ((json)configuration).dump(4);
    }

    return std::unique_ptr<BaseTest>(
        new BaseTest(
            configuration.duration,
            std::move(simulator),
            std::move(frankaridgeback),
            std::move(mppi_logger),
            std::move(dynamics_logger)
        )
    );
}

BaseTest::BaseTest(
    double duration,
    std::unique_ptr<Simulator> &&simulator,
    std::shared_ptr<FrankaRidgeback::Actor> &&frankaridgeback,
    std::unique_ptr<logger::MPPI> &&mppi_logger,
    std::unique_ptr<logger::FrankaRidgebackDynamics> &&dynamics_logger
 ) : m_duration(duration)
   , m_simulator(std::move(simulator))
   , m_frankaridgeback(std::move(frankaridgeback))
   , m_mppi_logger(std::move(mppi_logger))
   , m_dynamics_logger(std::move(dynamics_logger))
{}

void BaseTest::step()
{
    m_simulator->step();
    m_mppi_logger->log(m_frankaridgeback->get_controller());
    m_dynamics_logger->log(m_simulator->get_time(), m_frankaridgeback->get_dynamics());
}

bool BaseTest::run()
{
    while (m_simulator->get_time() < m_duration) {

        // Stores the current time and a duration. If the duration has not
        // elapsed by the time the destructor is called, waits for the remaining
        // duration. Caps the maximum loop speed to realtime.
        auto delay = raisim::TimedLoop(m_simulator->get_time_step() * 1e6);

        step();
    }

    return true;
}
