#pragma once

#include "controller/trajectory.hpp"
#include "test/case/external_wrench.hpp"

/**
 * @brief Convenience test for maintaining a circular trajectory with external
 * wrench.
 */
class CircleTest : public RegisteredTest<CircleTest>
{
public:

    static inline constexpr const char *TEST_NAME = "circle";

    struct Configuration {

        /// Configuration of the base external wrench test.
        ExternalWrenchTest::Configuration base;

        /// Patch to apply for the the circle trajectory.
        CircularTrajectory::Configuration circle;

        // JSON conversion for circle test configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            base, circle
        )
    };

    inline const static Configuration DEFAULT_CONFIGURATION {
        .base = ExternalWrenchTest::DEFAULT_CONFIGURATION,
        .circle = CircularTrajectory::DEFAULT_CONFIGURATION
    };

    inline static std::unique_ptr<ExternalWrenchTest> create(
        Options &options
    ) {
        if (options.folder.empty())
            options.folder = "circle";

        // Patch to set the reach object with the default configuration.
        json default_patch = {
            {"trajectory", {
                {"position", {
                    {"type", PositionTrajectory::Configuration::Type::CIRCLE},
                    {"circle",  DEFAULT_CONFIGURATION.circle},
                }}
            }}
        };

        // Unless the options overrides the default itself.
        default_patch.merge_patch(options.patch);
        options.patch = default_patch;

        (void)CircleTest();

        return ExternalWrenchTest::create(options);
    }

private:

    /**
     * @brief Disabled.
     */
    inline bool run() override
    {
        return false;
    }
};
