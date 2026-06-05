#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_test_logger/robot_test_logger.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

constexpr double MAX_CMD_LINEAR_VEL = 0.4;
constexpr double MAX_CMD_ANG_VEL = 0.3;

double clip_max_cmd_linear_vel(double x)
{
    return std::max(std::min(x, MAX_CMD_LINEAR_VEL), -MAX_CMD_LINEAR_VEL);
}

double clip_max_cmd_ang_vel(double x)
{
    return std::max(std::min(x, MAX_CMD_ANG_VEL), -MAX_CMD_ANG_VEL);
}

double rescale_function(double value)
{
    double scale = 0.1;

    if (std::abs(value) < 0.95) {
        value = value * scale;
    }

    return value;
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
        RCLCPP_INFO(this->get_logger(), "Using rescale function: rescale_function");
        RCLCPP_INFO(this->get_logger(), "JoyToCmdVel node has been started.");

        logger_->logJoystickConnected(false, "JoyToCmdVel started, waiting for /joy messages");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    std::unique_ptr<RobotTestLogger> logger_;

    bool joystick_connected_ = false;
    bool previous_deadman_active_ = false;
    std::string speed_mode_ = "SLOW";

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!joystick_connected_) {
            joystick_connected_ = true;
            logger_->logJoystickConnected(true, "First /joy message received");
        }

        if (msg->axes.size() <= 3) {
            logger_->logCommandRejected(
                0.0,
                0.0,
                "Joystick message rejected: expected at least 4 axes"
            );
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();

        const double raw_linear_x = rescale_function(msg->axes[1]);
        const double raw_angular_z = -rescale_function(msg->axes[3]);

        twist_msg.linear.x = clip_max_cmd_linear_vel(raw_linear_x);
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = clip_max_cmd_ang_vel(raw_angular_z);

        const bool command_was_clamped =
            twist_msg.linear.x != raw_linear_x ||
            twist_msg.angular.z != raw_angular_z;

        if (command_was_clamped) {
            logger_->logCommandClamped(
                raw_linear_x,
                raw_angular_z,
                twist_msg.linear.x,
                twist_msg.angular.z,
                "Joystick command exceeded configured velocity limits"
            );
        }

        publisher_->publish(twist_msg);

        logger_->logVelocityCommand(
            twist_msg.linear.x,
            twist_msg.angular.z,
            joystick_connected_,
            previous_deadman_active_,
            speed_mode_,
            "cmd_vel published"
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}