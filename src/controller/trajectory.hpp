#pragma once

#include <cmath>
#include <tuple>
#include <iostream>

#include "controller/eigen.hpp"
#include "controller/json.hpp"

/**
 * @brief Base class for implementing a positional trajectory.
 */
class PositionTrajectory
{
public:

    // Defined after all position trajectories are defined.
    struct Configuration;

    /// Virtual destructor for derived classes.
    virtual ~PositionTrajectory() {};

    /**
     * @brief Create a positional trajectory.
     * 
     * @param configuration The configuration of the positional trajectory.
     * @returns The position trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<PositionTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time The time of the position.
     * @returns The position. 
     */
    virtual Vector3d get_position(double time) = 0;

protected:

    PositionTrajectory() = default;
};

/**
 * @brief Base class for implementing an orientation trajectory.
 */
class OrientationTrajectory
{
public:

    // Defined after all orientation trajectories are defined.
    struct Configuration;

    /// Virtual destructor for derived classes.
    virtual ~OrientationTrajectory() {};

    /**
     * @brief Create an orientation trajectory.
     * 
     * @param configuration The configuration of the trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<OrientationTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the orientation at a given time.
     * 
     * @param time The time of the orientation.
     * @returns The orientation in world space of the trajectory.
     */
    virtual Quaterniond get_orientation(double time) = 0;

protected:

    OrientationTrajectory() = default;
};

/**
 * @brief A trajectory that stays in the same position forever.
 */
class PointTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// The point of the trajectory.
        Vector3d point;

        /// JSON conversion for point trajectory configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Configuration, point)
    };

    /**
     * @brief Default configuration of the point trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .point = Vector3d(1.0, 1.0, 1.0)
    };

    /**
     * @brief Create a new point trajectory.
     * 
     * @param configuration The configuration of the point trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<PointTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time Unused.
     * @returns The position. 
     */
    Vector3d get_position(double time) override;

private:

    /**
     * @brief Initialise a point trajectory.
     * @param configuration The configuration of the point trajectory.
     */
    PointTrajectory(const Configuration &configuration);

    /// The point of the trajectory.
    Vector3d m_point;
};

/**
 * @brief A point trajectory that rotates in a circle.
 */
class CircularTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// The origin of the circle to track.
        Vector3d origin;

        /// The axis of rotation about the origin.
        Vector3d axis;

        /// The radius of the circle.
        double radius;

        /// The angular velocity of the point to track.
        double angular_velocity;

        // JSON conversion for rotating point configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, axis, radius, angular_velocity
        )
    };

    /**
     * @brief Default configuration of the circular trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .origin = Vector3d(1.0, 1.0, 1.0),
        .axis = Vector3d(0.0, 0.0, 1.0).normalized(),
        .radius = 0.25,
        .angular_velocity = 1
    };

    /**
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<CircularTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the trajectory at a given time.
     * 
     * @param time The time of the position.
     * @returns The position around the circle. 
     */
    inline Vector3d get_position(double time) override;

private:

    CircularTrajectory(const Configuration &configuration);

    /// The point to rotate about the axis.
    Vector3d m_point;

    /// The origin of the circle to track, to, add to the point.
    Vector3d m_origin;

    /// The axis of rotation about the origin.
    Vector3d m_axis;

    /// The angular velocity of the point to track.
    double m_angular_velocity;
};

/**
 * @brief A trajectory that moves in a rectangle on a plane in space.
 */
class RectangularTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// The origin of the rectangle to track.
        Vector3d origin;

        /// The axis defining the orientation of the plane 
        Vector3d axis;

        // The angle to rotate the rectangle on the plane.
        double angle;

        /// Width of the rectangle along the x axis of the plane.
        double width;

        /// Height of the rectangle along the y axis of the plane.
        double height;

        /// The linear velocity of the trajectory, in m/s.
        double velocity;

        /// JSON conversion for rectangular trajectory configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, axis, angle, width, height, velocity
        );
    };

    /**
     * @brief Default configuration of the rectangular trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .origin = Vector3d(1.0, 1.0, 1.0),
        .axis = Vector3d(0.0, 0.0, 1.0).normalized(),
        .angle = 0.0,
        .width = 1.0,
        .height = 0.0,
        .velocity = 0.5
    };

    /**
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<RectangularTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position around the rectangle.
     * 
     * @todo Make this nicer.
     * 
     * @param time The time of the position.
     * @return The position. 
     */
    inline Vector3d get_position(double time) override;

private:

    RectangularTrajectory(const Configuration &configuration);

    /// The configuration of the rectangular trajectory.
    Configuration m_configuration;

    /// The total circumference of the rectangle on the 2D plane.
    double m_circumference;

    /// Mapping of frequency to linear velocity around the rectangular.
    double m_velocity;

    /// Transformation from the 2D plane to the 3D plane.
    Eigen::Transform<double, 3, Eigen::Affine>  m_transform;
};

/**
 * @brief A point-wise trajectory following a 3D lissajous curve.
 * 
 * A lissajous curve is parameterised by:
 * 
 *      x = a * sin(t)
 *      y = b * sin(nu * t + phi)
 *      z = c * sin(gamma * t + psi)
 * 
 * For the configuration:
 * 
 *      x = x_amplitude * sin(x_frequency * t)
 *      y = y_amplitude * sin(y_frequency * t + y_phase)
 *      z = z_amplitude * sin(z_frequencz * t + z_phase)
 */
class LissajousTrajectory : public PositionTrajectory
{
public:

    struct Configuration {

        /// Origin of the lissajous trajectory in space.
        Vector3d origin;

        /// Amplitude of the sine curve in the x world axis.
        double x_amplitude;

        /// Amplitude of the sine curve in the y world axis.
        double y_amplitude;

        /// Amplitude of the sine curve in the z world axis.
        double z_amplitude;

        /// Frequency of the sine curve in the x world axis.
        double x_frequency;

        /// Frequency of the sine curve in the y world axis.
        double y_frequency;

        /// Frequency of the sine curve in the z world axis.
        double z_frequency;

        /// Phase of the sine curve in the y world axis.
        double y_phase;

        /// Phase of the sine curve in the z world axis.
        double z_phase;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, x_amplitude, y_amplitude, z_amplitude, x_frequency,
            y_frequency, z_frequency, y_phase, z_phase
        )
    };

    /**
     * @brief Default configuration of the lissajous trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .origin = Vector3d(1.0, 1.0, 1.0),
        .x_amplitude = 0.4,
        .y_amplitude = 0.2,
        .z_amplitude = 0.0,
        .x_frequency = 0.5,
        .y_frequency = 1.5,
        .z_frequency = 0.0,
        .y_phase = 3.141592653589793 / 2,
        .z_phase = 0.0
    };

    /**
     * @brief Create a new lissajous trajectory.
     * 
     * @param configuration The configuration of the lissajous trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<LissajousTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the position of the lissajous curve at a given time.
     * 
     * @param time The time of the position.
     * @returns The position. 
     */
    inline Vector3d get_position(double time) override;

protected:

    /**
     * @brief Initialise a new lissajous trajectory.
     * @param configuration The lissajous trajectory configuration.
     */
    inline LissajousTrajectory(const Configuration &configuration);

    /// The configuration of the lissajous trajectory.
    Configuration m_configuration;
};

/**
 * @brief A figure 8 position trajectory along the x axis.
 */
class FigureEightTrajectory : public LissajousTrajectory
{
public:

    struct Configuration {

        /// Origin of the figure eight in space.
        Vector3d origin;

        /// Amplitude of the curve in the x world axis.
        double x_amplitude;

        /// Amplitude of the curve in the y world axis.
        double y_amplitude;

        /// Frequency of the figure eight curve.
        double frequency;

        /// JSON conversion for figure eight configuration.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            origin, x_amplitude, y_amplitude, frequency
        )
    };

    /**
     * @brief Default configuration of the figure eight trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .origin = Vector3d(1.0, 1.0, 1.0),
        .x_amplitude = 0.5,
        .y_amplitude = 0.25,
        .frequency = 1
    };

    /**
     * @brief Create a new circular trajectory.
     * 
     * @param configuration The configuration of the circular trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<FigureEightTrajectory> create(
        const Configuration &configuration
    );

private:

    // Inherit lissajous constructor.
    using LissajousTrajectory::LissajousTrajectory;
};

/**
 * @brief A static single orientation returned regardless of time.
 */
class AxisAngleTrajectory : public OrientationTrajectory
{
public:

    struct Configuration {

        /// The axis the z axis is orientated to.
        Vector3d axis;

        /// The rotation about the z axis with respect to the x direction.
        double angle;

        /// JSON conversion for axis angle trajectory.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            axis, angle
        );
    };

    /**
     * @brief Default configuration of the axis angle trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .axis = Vector3d(1, 1, 1).normalized(),
        .angle = 0.0
    };

    /**
     * @brief Create a new static axis angle trajectory.
     * 
     * @param configuration The configuration of the axis angle trajectory.
     * @returns A pointer to the axis angle trajectory on success or nullptr on
     * failure.
     */
    static std::unique_ptr<AxisAngleTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the orientation.
     * 
     * @param time The time of the orientation, unused.
     * @returns The orientation.
     */
    Quaterniond get_orientation(double time) override;

private:

    /**
     * @brief Initialise the axis angle trajectory.
     */
    AxisAngleTrajectory(const Configuration &configuration);

    /// The calculated orientation.
    Quaterniond m_orientation;
};

/**
 * @brief Oscillation between two orientations as a function of time.
 * 
 * Returns slerp(orientation1, orientation2, t) where the parameterisation t is
 * given by the normalised sine wave t = (sin(time) + 1) / 2.
 */
class SlerpTrajectory : public OrientationTrajectory
{
public:

    struct Configuration {

        /// The axis the z axis is orientated to of the first orientation.
        Vector3d first_axis;

        /// The rotation about the z axis with respect to the x direction of the
        /// first orientation.
        double first_angle;

        /// The axis the z axis is orientated to of the second orientation
        Vector3d second_axis;

        /// The rotation about the z axis with respect to the x direction of the
        /// second orientation.
        double second_angle;

        /// The frequency of the orientation interpolation.
        double frequency;

        /// JSON conversion for SlerpTrajectory.
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Configuration,
            first_axis, first_angle, second_axis, second_angle, frequency
        )
    };

    /**
     * @brief Default configuration of the slerp trajectory.
     * @note Inlined since static initialisation order is undefined.
     */
    static inline const Configuration DEFAULT_CONFIGURATION {
        .first_axis = Vector3d(0.0, 0.0, 1.0).normalized(),
        .first_angle = 0.0,
        .second_axis = Vector3d(0.0, 1.0, 0.0).normalized(),
        .second_angle = 0.0
    };

    /**
     * @brief Create a new interpolated orientation trajectory.
     * 
     * @param configuration The configuration of the interpolated orientation
     * trajectory.
     * @returns A pointer to the trajectory on success or nullptr on failure.
     */
    static std::unique_ptr<SlerpTrajectory> create(
        const Configuration &configuration
    );

    /**
     * @brief Get the orientation interpolated between the two provided.
     * 
     * @param time The time of the orientation.
     * @returns The interpolated orientation.
     */
    Quaterniond get_orientation(double time) override;

private:

    /**
     * @brief Initialise an orientation spherically interpolated trajectory.
     * 
     * @param configuration The configuration of the trajectory.
     */
    SlerpTrajectory(const Configuration &configuration);

    /// The first orientation.
    Quaterniond m_first;

    /// The second orientation.
    Quaterniond m_second;

    /// Frequency of oscillation between the orientations in radians.
    double m_frequency;
};

struct PositionTrajectory::Configuration {

    enum Type {
        POINT,
        CIRCLE,
        RECTANGLE,
        LISSAJOUS,
        FIGURE_EIGHT
    };

    /// The configured position trajectory.
    Type type;

    /// Configuration for the point
    std::optional<PointTrajectory::Configuration> point;

    /// Configuration for the circle trajectory.
    std::optional<CircularTrajectory::Configuration> circle;

    /// Configuration for the rectangle trajectory.
    std::optional<RectangularTrajectory::Configuration> rectangle;

    /// Configuration for the lissajous trajectory.
    std::optional<LissajousTrajectory::Configuration> lissajous;

    /// Configuration for the figure eight trajectory.
    std::optional<FigureEightTrajectory::Configuration> figure_eight;

    /// JSON conversion for positional trajectory configuration.
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        PositionTrajectory::Configuration,
        type, point, circle, rectangle, lissajous, figure_eight
    );
};

struct OrientationTrajectory::Configuration {

    enum Type {
        AXIS_ANGLE,
        SLERP
    };

    /// The type of the orientation trajectory.
    Type type;

    /// Configuration for the axis angle trajectory.
    std::optional<AxisAngleTrajectory::Configuration> axis_angle;

    /// Configuration for the orientation spherically interpolated quaternion
    /// trajectory.
    std::optional<SlerpTrajectory::Configuration> slerp;

    /// JSON conversion for orientation trajectory configuration.
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        OrientationTrajectory::Configuration,
        type, axis_angle, slerp
    )
};
