#include "simulation/frankaridgeback/dynamics.hpp"

namespace FrankaRidgeback {

std::unique_ptr<Dynamics> SimulatorDynamics::create(
    const Configuration &configuration,
    std::unique_ptr<DynamicsForecast::Handle> &&dynamics_forecast_handle,
    Simulator *simulator
) {
    std::unique_ptr<Dynamics> dynamics = nullptr;

    if (configuration.type == Configuration::Type::RAISIM) {
        if (!configuration.raisim) {
            std::cerr << "no raisim dynamics configuration provided" << std::endl;
            return nullptr;
        }
        dynamics = RaisimDynamics::create(
            *configuration.raisim,
            std::move(dynamics_forecast_handle),
            simulator
        );
    }
    else if (configuration.type == Configuration::Type::PINOCCHIO) {
        if (!configuration.pinocchio) {
            std::cerr << "no pinocchio dynamics configuration provided" << std::endl;
            return nullptr;
        }
        dynamics = PinocchioDynamics::create(
            *configuration.pinocchio,
            std::move(dynamics_forecast_handle)
        );
    }
    else {
        std::cerr << "unrecognised dynamics type " << configuration.type << std::endl;
        return nullptr;
    }

    return dynamics;
}

} // namespace FrankaRidgeback
