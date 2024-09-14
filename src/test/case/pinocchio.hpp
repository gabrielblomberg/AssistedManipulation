#pragma once

#include "test/test.hpp"
#include "frankaridgeback/dynamics.hpp"
#include "controller/forecast.hpp"
#include "simulator/simulator.hpp"

class PinocchioDynamicsTest : public RegisteredTest<PinocchioDynamicsTest>
{
public:

    struct Configuration {

        double duration;

        Simulator::Configuration simulator;

        FrankaRidgeback::Dynamics::Configuration dynamics;

        Forecast::LOCFForecast::Configuration force;

        FrankaRidgeback::State initial_state;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, dynamics);
    };

    static inline constexpr const char *TEST_NAME = "pinocchio";

    /**
     * @brief Create a test reaching for a point.
     * 
     * @param options The test options. The configuration overrides from the
     * default configuration.
     * 
     * @return A pointer to the test on success or nullptr on failure.
     */
    static inline std::unique_ptr<Test> create(Options &options) {
        return create(options.)
    }

    static std::unique_ptr<Test> create(const Configuration &configuration);

    /**
     * @brief Run the test.
     * @returns If the test was successful.
     */
    bool run() override;

private:

    PinocchioDynamicsTest() = default;

    double m_duration;

    std::unique_ptr<Simulator> m_simulator;

    std::unique_ptr<FrankaRidgeback::Dynamics> m_dynamics;

    std::unique_ptr<LOCFForecast> m_force;

    ArticulatedSystemVisual *m_visual;
};
