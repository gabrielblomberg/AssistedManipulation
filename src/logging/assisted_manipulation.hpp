#pragma once

#include <limits>
#include <filesystem>

#include "logging/csv.hpp"
#include "controller/mppi.hpp"
#include "frankaridgeback/objective/assisted_manipulation.hpp"

namespace logger {

/**
 * @brief Logging class for AssistedManipulation.
 */
class AssistedManipulation
{
public:

    struct Configuration {

        /// Path to the folder to log into.
        std::filesystem::path folder;

        bool log_joint_limit = true;

        bool log_self_collision_limit = true;

        bool log_workspace_limit = true;

        bool log_energy_limit = true;

        bool log_velocity_cost = true;

        bool log_trajectory_cost = true;

        bool log_manipulability_cost = true;

        bool log_total = true;

        // JSON conversion for mppi logger configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            folder, log_joint_limit, log_self_collision_limit,
            log_workspace_limit, log_energy_limit, log_velocity_cost,
            log_trajectory_cost, log_manipulability_cost, log_total
        )
    };

    /**
     * @brief Create a new AssistedManipulation logger.
     * 
     * @param configuration The logger configuration.
     * @returns A pointer to the AssistedManipulation logger on success or nullptr on failure.
     */
    static std::unique_ptr<AssistedManipulation> create(Configuration configuration);

    /**
     * @brief Log the objective function.
     * 
     * @param time The time of the objective.
     * @param objective The assisted manipulation objective.
     */
    void log(
        double time,
        const FrankaRidgeback::AssistedManipulation &objective
    );

private:

    inline AssistedManipulation(const Configuration &configuration)
        : m_configuration(configuration)
        , m_last_update(std::numeric_limits<double>::min())
    {}

    Configuration m_configuration;

    /// Time since the last objective update.
    double m_last_update;

    /// Calculation of the times of each horison step of the generator.
    std::vector<double> m_costs;

    /// Logger for 
    std::unique_ptr<CSV> m_logger;
};

} // namespace logger
