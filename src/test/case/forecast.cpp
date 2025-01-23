#include "test/case/forecast.hpp"

#include <vector>

#include "controller/kalman.hpp"
#include "controller/forecast.hpp"
#include "controller/gaussian.hpp"
#include "logging/csv.hpp"

ForecastTest::ForecastTest(std::filesystem::path folder)
    : m_folder(folder)
{}

bool ForecastTest::run()
{
    return (
        test_locf_forecast() &&
        test_average_forecast() &&
        test_kalman_linear_forecast()
    );
}

bool ForecastTest::test_locf_forecast()
{
    LOCFForecast::Configuration configuration {
        .observation = Vector3d(1.0, 2.0, 3.0)
    };

    auto forecast = LOCFForecast::create(configuration);
    if (!forecast) {
        std::cerr << "failed to create locf forecast" << std::endl;
        return false;
    }

    // The true positions at each time step.
    std::vector<std::tuple<double, Vector3d>> trajectory {
        {0, Vector3d::Random()},
        {0, Vector3d::Random()},
        {0, Vector3d::Random()},
        {0, Vector3d::Random()},
        {0, Vector3d::Random()},
    };

    // Add variance with the gaussian sampler.
    for (auto [time, position] : trajectory) {
        forecast->update(position, time);

        auto f1 = forecast->forecast(time);
        auto f2 = forecast->forecast(time + 1.0);
        auto f3 = forecast->forecast(time + 2.0);

        if (!position.isApprox(f1) || !f1.isApprox(f2) || !f2.isApprox(f3)) {
            std::cerr << "locf position " << position.transpose()
                      << " was not carried forward." << std::endl;
            return false;
        }
    }

    return true;
}

bool ForecastTest::test_average_forecast()
{
    AverageForecast::Configuration configuration {
        .states = 3,
        .window = 1.0
    };

    auto forecast = AverageForecast::create(configuration);
    if (!forecast) {
        std::cerr << "failed to create average forecast" << std::endl;
    }

    if (!forecast->forecast(0.0).isApprox(Vector3d(0, 0, 0)))
        return false;

    forecast->update(Vector3d(0, 1.0, 0), 1.01);
    if (!forecast->forecast(5.0).isApprox(Vector3d(0, 1.0, 0)))
        return false;

    forecast->update(Vector3d(0, 1.5, 0), 1.5);
    if (!forecast->forecast(10.0).isApprox(Vector3d(0, 1.25, 0)))
        return false;

    forecast->update(Vector3d(1.0, 1.0, 1.0), 3.0);
    if (!forecast->forecast(3.0).isApprox(Vector3d(1.0, 1.0, 1.0)))
        return false;

    for (int i = 0; i < 10; i++)
        forecast->update(Vector3d(i, i, i), 4.5 + i * 0.05);

    if (!forecast->forecast(3.5).isApprox(Vector3d(4.5, 4.5, 4.5)))
        return false;

    forecast->update(10.0);

    if (!forecast->forecast(10.0).isApprox(Vector3d(9.0, 9.0, 9.0)))
        return false;

    return true;
}

bool ForecastTest::test_kalman_linear_forecast()
{
    auto logger = logger::CSV::create(logger::CSV::Configuration{
        .path = m_folder / "linear.csv",
        .header = logger::CSV::make_header(
            "time", // Time.
            "x", "y", // True state.
            "sx", "sy", // Sampled state.
            "fx1", "fy1", "fx2", "fy2", "fx3", "fy3" // Horison forecast.
        )
    });

    if (!logger)
        return false;

    // auto transition_variance = Vector4d(0.1, 0.1, 0.1, 0.1);
    // auto observation_variance = Vector4d(0.1, 0.1, 0.1, 0.1);

    KalmanForecast::Configuration configuration {
        .observed_states = 2,
        .time_step = 1,
        .horison = 3,
        .order = 2,
        .initial_state = Vector2d(0, 0)
    };

    auto forecast = KalmanForecast::create(configuration);
    if (!forecast) {
        std::cerr << "failed to create kalman forecaster" << std::endl;
        return false;
    }

    // The true positions at each time step.
    std::vector<std::tuple<double, Vector2d>> trajectory {
        {0, Vector2d(0, 0)},
        {0, Vector2d(0, 1)},
        {0, Vector2d(0, 2)},
        {0, Vector2d(0, 3)},
        {0, Vector2d(0, 4)},
    };

    // Add variance with the gaussian sampler.
    auto sampler = Gaussian(Vector2d(0, 0), Vector2d(0.5, 0.5).asDiagonal());
    for (auto [time, position] : trajectory) {
        sampler.set_mean(position);
        VectorXd sampled = position + sampler();

        forecast->update(sampled, time);

        auto f1 = forecast->forecast(time);
        auto f2 = forecast->forecast(time + 1.0);
        auto f3 = forecast->forecast(time + 2.0);

        logger->write(time, position, sampled, f1, f2, f3);
    }

    return true;
}

bool ForecastTest::test_kalman_quadratic_forecast()
{
    return false;
}
