#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_test_logger/robot_test_logger.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace
{
// Joystick axis mapping
constexpr int LINEAR_AXIS_INDEX = 1;   // forward/backward
constexpr int ANGULAR_AXIS_INDEX = 3;  // left/right turn

// R2 detection.
// Common controller mappings:
// - R2 as button: buttons[7]
// - R2 as analog axis: axes[5], usually 0.0 or 1.0 released and -1.0 fully pressed
constexpr int R2_BUTTON_INDEX = 7;
constexpr int R2_AXIS_INDEX = 5;
constexpr double R2_AXIS_PRESSED_THRESHOLD = -0.5;

// Direction signs.
// Keep these as they are because your /cmd_vel angular sign is now correct:
// right turn = angular.z < 0
// left turn  = angular.z > 0
constexpr double LINEAR_AXIS_SIGN = 1.0;
constexpr double ANGULAR_AXIS_SIGN = 1.0;

// Normal speed limits
constexpr double NORMAL_MAX_LINEAR_VEL = 0.6;   // m/s
constexpr double NORMAL_MAX_ANGULAR_VEL = 0.5;  // rad/s

// Turbo speed limits
constexpr double TURBO_MAX_LINEAR_VEL = 1.0;    // m/s
constexpr double TURBO_MAX_ANGULAR_VEL = 0.9;   // rad/s

// Joystick conditioning
constexpr double JOYSTICK_DEADZONE = 0.08;

// Output ramp limits.
// Increase these if turbo feels too slow to reach max.
// Decrease them if acceleration is too aggressive.
constexpr double LINEAR_ACCEL_LIMIT = 1.2;      // m/s²
constexpr double ANGULAR_ACCEL_LIMIT = 1.8;     // rad/s²

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
    const double magnitude =
        (std::abs(value) - JOYSTICK_DEADZONE) / (1.0 - JOYSTICK_DEADZONE);

    return sign * clamp_value(magnitude, 0.0, 1.0);
}

double ramp_towards(double current, double target, double max_delta)
{
    if (target > current + max_delta) {
        return current + max_delta;
    }

    if (target < current - max_delta) {
        return current - max_delta;
    }

    return target;
}

bool is_r2_pressed(const sensor_msgs::msg::Joy::SharedPtr & msg)
{
    bool r2_button_pressed = false;
    bool r2_axis_pressed = false;

    if (msg->buttons.size() > static_cast<size_t>(R2_BUTTON_INDEX)) {
        r2_button_pressed = msg->buttons[R2_BUTTON_INDEX] != 0;
    }

    if (msg->axes.size() > static_cast<size_t>(R2_AXIS_INDEX)) {
        // Turbo only when trigger is clearly pressed.
        // This avoids treating a neutral 0.0 trigger value as pressed.
        // Common mapping: released ≈ 0.0 or 1.0, pressed goes toward -1.0.
        r2_axis_pressed = msg->axes[R2_AXIS_INDEX] < R2_AXIS_PRESSED_THRESHOLD;
    }

    return r2_button_pressed || r2_axis_pressed;
}

}  // namespace

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
        RCLCPP_INFO(this->get_logger(), "R2 button index: %d, R2 axis index: %d", R2_BUTTON_INDEX, R2_AXIS_INDEX);
        RCLCPP_INFO(this->get_logger(), "Normal limits: linear %.2f m/s, angular %.2f rad/s", NORMAL_MAX_LINEAR_VEL, NORMAL_MAX_ANGULAR_VEL);
        RCLCPP_INFO(this->get_logger(), "Turbo limits: linear %.2f m/s, angular %.2f rad/s", TURBO_MAX_LINEAR_VEL, TURBO_MAX_ANGULAR_VEL);
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
    bool has_last_output_time_ = false;

    rclcpp::Time last_output_time_;
    double last_linear_x_ = 0.0;
    double last_angular_z_ = 0.0;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const size_t required_axis_count =
            static_cast<size_t>(std::max(LINEAR_AXIS_INDEX, ANGULAR_AXIS_INDEX) + 1);

        if (msg->axes.size() < required_axis_count) {
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

        const bool turbo_active = is_r2_pressed(msg);

        const double max_linear_vel =
            turbo_active ? TURBO_MAX_LINEAR_VEL : NORMAL_MAX_LINEAR_VEL;

        const double max_angular_vel =
            turbo_active ? TURBO_MAX_ANGULAR_VEL : NORMAL_MAX_ANGULAR_VEL;

        const double raw_linear_axis = msg->axes[LINEAR_AXIS_INDEX];
        const double raw_angular_axis = msg->axes[ANGULAR_AXIS_INDEX];

        const double normalized_linear = apply_deadzone_and_scale(raw_linear_axis);
        const double normalized_angular = apply_deadzone_and_scale(raw_angular_axis);

        const double target_linear_x =
            LINEAR_AXIS_SIGN * normalized_linear * max_linear_vel;

        const double target_angular_z =
            ANGULAR_AXIS_SIGN * normalized_angular * max_angular_vel;

        const auto now = this->now();

        double dt = 0.02;
        if (has_last_output_time_) {
            dt = (now - last_output_time_).seconds();
            if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
                dt = 0.02;
            }
        } else {
            has_last_output_time_ = true;
        }

        last_output_time_ = now;

        const double max_linear_delta = LINEAR_ACCEL_LIMIT * dt;
        const double max_angular_delta = ANGULAR_ACCEL_LIMIT * dt;

        const double linear_x =
            ramp_towards(last_linear_x_, target_linear_x, max_linear_delta);

        const double angular_z =
            ramp_towards(last_angular_z_, target_angular_z, max_angular_delta);

        last_linear_x_ = linear_x;
        last_angular_z_ = angular_z;

        auto twist_msg = geometry_msgs::msg::Twist();

        twist_msg.linear.x = linear_x;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_z;

        publisher_->publish(twist_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "JOY_TO_CMDVEL | mode=%s | target_linear=%.3f target_angular=%.3f | linear=%.3f angular=%.3f",
            turbo_active ? "TURBO" : "NORMAL",
            target_linear_x,
            target_angular_z,
            twist_msg.linear.x,
            twist_msg.angular.z
        );

        if (logger_) {
            logger_->logVelocityCommand(
                twist_msg.linear.x,
                twist_msg.angular.z,
                true,
                true,
                turbo_active ? "TURBO" : "NORMAL",
                turbo_active ? "cmd_vel published from joystick with R2 turbo" :
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
