#include "test/case/base.hpp"
#include "logging/file.hpp"

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
    std::cout << "here1" << std::endl;
    std::unique_ptr<Simulator> simulator = Simulator::create(configuration.simulator);
    if (!simulator) {
        std::cerr << "failed to create simulator" << std::endl;
        return nullptr;
    }

    auto frankaridgeback = FrankaRidgeback::Actor::create(
        configuration.actor,
        simulator.get()
    );

    if (!frankaridgeback) {
        std::cerr << "failed to create frankaridgeback actor" << std::endl;
        return nullptr;
    }

    simulator->add_actor(frankaridgeback);

    // Override rollout count of logger.
    auto mppi_log_configuration = configuration.mppi_logger;
    auto dynamics_log_configuration = configuration.dynamics_logger;
    auto forecast_log_configuration = configuration.forecast_logger;
    auto objective_log_configuration = configuration.objective_logger;

    mppi_log_configuration.rollouts = frankaridgeback->get_controller().get_rollout_count();
    mppi_log_configuration.folder = configuration.folder / "mppi";
    dynamics_log_configuration.folder = configuration.folder / "dynamics";
    forecast_log_configuration.folder = configuration.folder / "forecast";
    objective_log_configuration.folder = configuration.folder / "objective";

    auto mppi_logger = logger::MPPI::create(mppi_log_configuration);
    if (!mppi_logger) {
        std::cerr << "failed to create mppi logger" << std::endl;
        return nullptr;
    }

    auto dynamics_logger = logger::FrankaRidgebackDynamics::create(dynamics_log_configuration);
    if (!dynamics_logger) {
        std::cerr << "failed to create dynamics logger" << std::endl;
        return nullptr;
    }

    auto forecast_logger = logger::FrankaRidgebackDynamicsForecast::create(forecast_log_configuration);
    if (!forecast_logger) {
        std::cerr << "failed to create forecast logger" << std::endl;
        return nullptr;
    }

    std::unique_ptr<logger::AssistedManipulation> objective_logger;
    if (configuration.actor.objective.type == FrankaRidgeback::ObjectiveType::ASSISTED_MANIPULATION) {
        objective_logger = logger::AssistedManipulation::create(objective_log_configuration);
        if (!objective_logger) {
            std::cerr << "failed to create objective logger" << std::endl;
            return nullptr;
        }
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
            std::move(dynamics_logger),
            std::move(forecast_logger),
            std::move(objective_logger)
        )
    );
}

BaseTest::BaseTest(
    double duration,
    std::unique_ptr<Simulator> &&simulator,
    std::shared_ptr<FrankaRidgeback::Actor> &&frankaridgeback,
    std::unique_ptr<logger::MPPI> &&mppi_logger,
    std::unique_ptr<logger::FrankaRidgebackDynamics> &&dynamics_logger,
    std::unique_ptr<logger::FrankaRidgebackDynamicsForecast> &&forecast_logger,
    std::unique_ptr<logger::AssistedManipulation> &&objective_logger
 ) : m_duration(duration)
   , m_simulator(std::move(simulator))
   , m_frankaridgeback(std::move(frankaridgeback))
   , m_mppi_logger(std::move(mppi_logger))
   , m_dynamics_logger(std::move(dynamics_logger))
   , m_forecast_logger(std::move(forecast_logger))
   , m_objective_logger(std::move(objective_logger))
{}

void BaseTest::step()
{
    m_simulator->step();

    m_mppi_logger->log(m_frankaridgeback->get_controller());

    m_dynamics_logger->log(m_simulator->get_time(), m_frankaridgeback->get_dynamics());
    m_dynamics_logger->log_control(m_simulator->get_time(), m_frankaridgeback->get_control());

    if (m_frankaridgeback->get_forecast())
        m_forecast_logger->log(*m_frankaridgeback->get_forecast());

    if (m_objective_logger) {
        m_objective_logger->log(
            m_frankaridgeback->get_controller().get_update_last(),
            dynamic_cast<const FrankaRidgeback::AssistedManipulation&>(
                m_frankaridgeback->get_controller().get_optimal_cost()
            )
        );
    }
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
