
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>  // For wheel displacements
#include <geometry_msgs/msg/pose2_d.hpp>  // For lidar correction
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "roboteq_ros2_driver/msg/wheel_ticks.hpp"


class KalmanOdometryNode : public rclcpp::Node {
public:
    KalmanOdometryNode() : Node("kalman_odometry_node") {
        this->declare_parameter("initial_r", 0.05);
        this->declare_parameter("initial_b", 0.3);
        this->declare_parameter("Q_diag", std::vector<double>{0.001, 0.001, 0.001, 0.001, 0.001});
        this->declare_parameter("R_diag", std::vector<double>{0.5, 0.5, 0.05});
        
        this->get_parameter("ticks_per_rev",ticks_per_rev_);

        x_[0] = this->get_parameter("initial_x").as_double();
        x_[1] = this->get_parameter("initial_y").as_double();
        x_[2] = this->get_parameter("initial_theta").as_double();
        x_[3] = this->get_parameter("initial_r").as_double();
        x_[4] = this->get_parameter("initial_b").as_double();

        auto q = this->get_parameter("Q_diag").as_double_array();
        auto r = this->get_parameter("R_diag").as_double_array();

        for (size_t i = 0; i < 5; ++i) Q_[i][i] = q[i];
        for (size_t i = 0; i < 3; ++i) R_[i][i] = r[i];

        for (int i = 0; i < 5; ++i) {
            x_[i] = 0.0;
            P_[i][i] = 1.0;
        }

        sub_wheels_ = create_subscription<roboteq_ros2_driver::msg::WheelTicks>(
            "/wheel_tick", 10,
            std::bind(&KalmanOdometryNode::wheel_callback, this, std::placeholders::_1));

        sub_lidar_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/lidar/pose2d", 10,
            std::bind(&KalmanOdometryNode::lidar_callback, this, std::placeholders::_1));

        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/kalman/odom", 10);
    }

private:
    double x_[5];          // [x, y, theta, r, b]
    double P_[5][5] = {{0}}, Q_[5][5] = {{0}}, R_[3][3] = {{0}};
    int ticks_per_rev_ = 1024; // Default value, can be set via parameter



    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_lidar_;
    rclcpp::Subscription<roboteq_ros2_driver::msg::WheelTicks>::SharedPtr sub_wheels_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;


    void wheel_callback(const roboteq_ros2_driver::msg::WheelTicks::SharedPtr msg) {
        
        double wheel_radius_ = x_[3];
        double b = x_[4];
        
        double circ = 2 * M_PI * wheel_radius_;

        int ticks_L = msg->left_ticks;  // Left wheel ticks
        int ticks_R = msg->right_ticks;  // Right wheel ticks

        double d_L = (circ / ticks_per_rev_) * ticks_L;
        double d_R = (circ / ticks_per_rev_) * ticks_R;
       

        //if (std::abs(dl) < 1e-6 && std::abs(dr) < 1e-6) {
        //    RCLCPP_WARN(this->get_logger(), "Received zero wheel displacement, skipping update.");
        //    return;
        //}

        double d_left = d_L;
        double d_right = d_R;

        double d = (d_left + d_right) / 2.0;
        double dtheta = (d_right - d_left) / b;

        x_[0] += d * std::cos(x_[2] + dtheta / 2.0);
        x_[1] += d * std::sin(x_[2] + dtheta / 2.0);
        x_[2] += dtheta;

        while (x_[2] > M_PI) x_[2] -= 2.0 * M_PI;
        while (x_[2] < -M_PI) x_[2] += 2.0 * M_PI;

        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j)
                P_[i][j] += Q_[i][j];

        publish_odom();
    }

    void lidar_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        double z[3] = {msg->x, msg->y, msg->theta};
        double y[3] = {z[0] - x_[0], z[1] - x_[1], z[2] - x_[2]};

        double K[3];
        for (int i = 0; i < 3; ++i)
            K[i] = P_[i][i] / (P_[i][i] + R_[i][i]);

        for (int i = 0; i < 3; ++i) {
            x_[i] += K[i] * y[i];
            P_[i][i] *= (1 - K[i]);
        }
    }

    void publish_odom() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "odom_kalman";

        msg.pose.pose.position.x = x_[0];
        msg.pose.pose.position.y = x_[1];

        tf2::Quaternion q;
        q.setRPY(0, 0, x_[2]);
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        pub_odom_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanOdometryNode>());
    rclcpp::shutdown();
    return 0;
}