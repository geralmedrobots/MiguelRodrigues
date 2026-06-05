#ifndef DIFFERENTIAL_DRIVE_KINEMATICS_HPP
#define DIFFERENTIAL_DRIVE_KINEMATICS_HPP

#include <cmath> // For std::sin, std::cos, std::abs, std::fmod, std::atan2

// Define PI if not already defined (common in C++ for math functions)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A small epsilon for floating point comparisons
const double EPSILON = 1e-9;

/**
 * @brief Represents the pose (position and orientation) of the robot.
 */
struct RobotPose {
    double x;     // X-coordinate in meters
    double y;     // Y-coordinate in meters
    double theta; // Orientation in radians
};

/**
 * @brief Represents the linear and angular velocities of the robot.
 */
struct RobotDisplacement {
    double linear_x;  // Linear velocity along the robot's forward direction (m)
    double delta_theta; // Angular velocity around the robot's Z-axis (rad)
};


/**
 * @brief Handles the forward kinematics for a differential drive robot.
 * Calculates robot velocities from wheel speeds and updates pose discretely.
 */
class DifferentialDriveKinematics {
public:
    /**
     * @brief Constructor for DifferentialDriveKinematics.
     */
    DifferentialDriveKinematics(double wheel_radius, double track_width, int encoder_resolution_ticks_per_rev);      


    /**
     * @brief       
     * 
     * @param wheel_radius 
     * @param track_width 
     * @param encoder_resolution_ticks_per_rev 
     */

    void initParam(double wheel_radius, double track_width, int encoder_resolution_ticks_per_rev);

 
    /**
     * @brief Calculates the robot's linear and angular velocities from wheel angular velocities.
     * @param omega_L Angular velocity of the left wheel in rad/s.
     * @param omega_R Angular velocity of the right wheel in rad/s.
     * @return RobotTwist containing the calculated linear_x and angular_z velocities.
     */
    RobotDisplacement calculateForwardKinematics(double omega_L, double omega_R) const;

    /**
     * @brief Updates the robot's pose based on its current pose, velocities, and discrete time step.
     * Uses the Instantaneous Center of Curvature (ICC) method for accurate updates.
     * @param current_pose The robot's current pose.
     * @param twist The robot's linear and angular velocities.
     * @param dt The discrete time step in seconds.
     * @return The updated robot pose.
     */
    RobotPose updateRobotPose(const RobotPose& current_pose, const RobotDisplacement& twist) const;

private:
    double wheel_radius_ {0.0}; // Radius of the wheels in meters
    double track_width_ {0.0};  // Distance between the wheels in meters
    int encoder_resolution_ticks_per_rev_ {0}; // Encoder resolution in ticks per revolution
};

#endif // DIFFERENTIAL_DRIVE_KINEMATICS_HPP