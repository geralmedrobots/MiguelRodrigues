#include "differential_drive_kinematics.hpp" // Include our own header
#include <iostream>                          // For std::cerr, std::cout (for debugging/warnings)


// Implementation of DifferentialDriveKinematics
//DifferentialDriveKinematics::DifferentialDriveKinematics()  = default;
    

DifferentialDriveKinematics::DifferentialDriveKinematics(double wheel_radius, double track_width, int encoder_resolution)
    : wheel_radius_(wheel_radius), track_width_(track_width), encoder_resolution_ticks_per_rev_(encoder_resolution) {
    if (wheel_radius_ <= 0 || track_width_ <= 0 || encoder_resolution_ticks_per_rev_ <= 0) {
        std::cerr << "Error: Wheel radius and track width must be positive for DifferentialDriveKinematics." << std::endl;
    }
}



void DifferentialDriveKinematics::initParam(double wheel_radius, double track_width, int encoder_resolution_ticks_per_rev) {
    
    
    wheel_radius_ = wheel_radius; 
    track_width_  = track_width;
    encoder_resolution_ticks_per_rev_ = encoder_resolution_ticks_per_rev;
    //,  encoder_resolution_ticks_per_rev_(0) 
    if (wheel_radius_ <= 0 || track_width_ <= 0 || encoder_resolution_ticks_per_rev_ <= 0) {
        std::cerr << "Error: Wheel radius and track width must be positive for DifferentialDriveKinematics." << std::endl;
    }
}
    
RobotDisplacement DifferentialDriveKinematics::calculateForwardKinematics(double ticks_L, double ticks_R) const {
    RobotDisplacement twist;

    //s# Convert pulses to distance
    double circ = 2 * M_PI * wheel_radius_;
    double d_L = (circ / encoder_resolution_ticks_per_rev_) * ticks_L;
    double d_R = (circ / encoder_resolution_ticks_per_rev_) * ticks_R;


    twist.linear_x = (d_R + d_L) / 2.0;
    twist.delta_theta = (d_R - d_L) / track_width_;

    return twist;
}




RobotPose DifferentialDriveKinematics::updateRobotPose(const RobotPose& current_pose, const RobotDisplacement& twist) const {
    RobotPose new_pose = current_pose;

    if (std::abs(twist.delta_theta) < EPSILON) {
        // Straight line motion
        new_pose.x = current_pose.x + twist.linear_x * std::cos(current_pose.theta + twist.delta_theta/2);
        new_pose.y = current_pose.y + twist.linear_x * std::sin(current_pose.theta + twist.delta_theta/2);
        new_pose.theta = current_pose.theta + twist.delta_theta;
    } else {
        // Rotational motion around an Instantaneous Center of Curvature (ICC)
        double R = twist.linear_x / twist.delta_theta;

        double icc_x = current_pose.x - R * std::sin(current_pose.theta);
        double icc_y = current_pose.y + R * std::cos(current_pose.theta);

        new_pose.x = icc_x + R * std::sin(current_pose.theta + twist.delta_theta);
        new_pose.y = icc_y - R * std::cos(current_pose.theta + twist.delta_theta);
        new_pose.theta = current_pose.theta + twist.delta_theta;
    }

    new_pose.theta = std::atan2(std::sin(new_pose.theta), std::cos(new_pose.theta));

    return new_pose;
}