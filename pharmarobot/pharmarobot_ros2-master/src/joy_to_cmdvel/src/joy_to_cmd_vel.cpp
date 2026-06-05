#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_test_logger/robot_test_logger.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

constexpr double MAX_CMD_LINEAR_VEL = 0.4;
constexpr double MAX_CMD_ANG_VEL = 0.3;

// Joystick axis mapping.
// axes[1] = forward/backward
// axes[3] = rotation
constexpr int LINEAR_AXIS_INDEX = 1;
constexpr int ANGULAR_AXIS_INDEX = 3;

// Sign convention.
// ROS convention: positive angular.z = turn left / counter-clockwise.
// If the robot turns the wrong way, change ANGULAR_AXIS_SIGN from +1.0 to -1.0.
constexpr double LINEAR_AXIS_SIGN = 1.0;
constexpr double ANGULAR_AXIS_SIGN = 1.0;

// Small deadzone to avoid drift when joystick is centered.
constexpr double JOYSTICK_DEADZONE = 0.08;

double clamp_value(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

double apply_deadzone_and_scale(double raw_value)
{
    const double value = clamp_value(raw_value, -1.0, 1.0);

    if (std::abs(value) < JOYSTICK_DEADZONE) {
        return 0.0;
    }

    const double sign = value >= 0.0 ? 1.0 : -1.0;
    const double magnitude = (std::abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE);

    return sign * clamp_value(magnitude, 0.0, 1.0);
}

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmdvel")
    {
        logger_ = std::make_unique<RobotTestLogger>(this);

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            1,
            std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        RCLCPP_INFO(this->get_logger(), "RUNNING CPP JoyToCmdVel node");
        RCLCPP_INFO(this->get_logger(), "Linear axis: %d, angular axis: %d", LINEAR_AXIS_INDEX, ANGULAR_AXIS_INDEX);
        RCLCPP_INFO(this->get_logger(), "Linear sign: %.1f, angular sign: %.1f", LINEAR_AXIS_SIGN, ANGULAR_AXIS_SIGN);
        RCLCPP_INFO(this->get_logger(), "JoyToCmdVel node has been started.");

        if (logger_) {
            logger_->logJoystickConnected(false, "JoyToCmdVel started, waiting for /joy messages");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::unique_ptr<RobotTestLogger> logger_;

    bool joystick_connected_ = false;
    std::string speed_mode_ = "MANUAL";

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= static_cast<size_t>(std::max(LINEAR_AXIS_INDEX, ANGULAR_AXIS_INDEX))) {
            RCLCPP_WARN(this->get_logger(), "Joy message does not contain required axes");

            if (logger_) {
                logger_->logCommandRejected(0.0, 0.0, "Joy message does not contain required axes");
            }

            return;
        }

        if (!joystick_connected_) {
            joystick_connected_ = true;

            if (logger_) {
                logger_->logJoystickConnected(true, "First /joy message received");
            }
        }

        const double raw_linear_axis = msg->axes[LINEAR_AXIS_INDEX];
        const double raw_angular_axis = msg->axes[ANGULAR_AXIS_INDEX];

        const double normalized_linear = apply_deadzone_and_scale(raw_linear_axis);
        const double normalized_angular = apply_deadzone_and_scale(raw_angular_axis);

        const double linear_x = LINEAR_AXIS_SIGN * normalized_linear * MAX_CMD_LINEAR_VEL;
        const double angular_z = ANGULAR_AXIS_SIGN * normalized_angular * MAX_CMD_ANG_VEL;

        auto twist_msg = geometry_msgs::msg::Twist();

        twist_msg.linear.x = linear_x;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_z;

        publisher_->publish(twist_msg);

        if (logger_) {
            logger_->logVelocityCommand(
                twist_msg.linear.x,
                twist_msg.angular.z,
                true,
                true,
                speed_mode_,
                "cmd_vel published from joystick"
            );
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
