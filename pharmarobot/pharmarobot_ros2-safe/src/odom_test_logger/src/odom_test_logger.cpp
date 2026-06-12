#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "roboteq_ros2_driver/msg/wheel_ticks.hpp"

#include <cmath>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

class OdomTestLogger : public rclcpp::Node
{
public:
  OdomTestLogger()
  : Node("odom_test_logger")
  {
    log_dir_ = this->declare_parameter<std::string>("log_dir", "/root/odom_test_logs");
    test_name_ = this->declare_parameter<std::string>("test_name", "odom_test");

    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel/safe");
    wheel_ticks_topic_ = this->declare_parameter<std::string>("wheel_ticks_topic", "/wheel_ticks");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");

    wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", 0.085);
    wheelbase_m_ = this->declare_parameter<double>("wheelbase_m", 0.453);
    encoder_cpr_ = this->declare_parameter<double>("encoder_cpr", 4096.0);

    log_zero_ticks_ = this->declare_parameter<bool>("log_zero_ticks", false);

    meters_per_tick_ = (2.0 * M_PI * wheel_radius_m_) / std::abs(encoder_cpr_);

    open_log_file();

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&OdomTestLogger::cmd_callback, this, std::placeholders::_1)
    );

    ticks_sub_ = this->create_subscription<roboteq_ros2_driver::msg::WheelTicks>(
      wheel_ticks_topic_,
      100,
      std::bind(&OdomTestLogger::ticks_callback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      std::bind(&OdomTestLogger::odom_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Odom test logger started.");
    RCLCPP_INFO(this->get_logger(), "Logging to: %s", log_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "wheel_ticks topic: %s", wheel_ticks_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "meters_per_tick: %.12f", meters_per_tick_);

    write_metadata_row("START", "Odom test logger started");
  }

private:
  std::string log_dir_;
  std::string test_name_;
  std::string log_path_;

  std::string cmd_vel_topic_;
  std::string wheel_ticks_topic_;
  std::string odom_topic_;

  double wheel_radius_m_ = 0.085;
  double wheelbase_m_ = 0.453;
  double encoder_cpr_ = 4096.0;
  double meters_per_tick_ = 0.0;

  bool log_zero_ticks_ = false;

  double latest_cmd_linear_x_ = 0.0;
  double latest_cmd_angular_z_ = 0.0;

  bool odom_received_ = false;
  double latest_odom_x_ = 0.0;
  double latest_odom_y_ = 0.0;
  double latest_odom_yaw_ = 0.0;

  int64_t left_total_ticks_ = 0;
  int64_t right_total_ticks_ = 0;

  double left_total_m_ = 0.0;
  double right_total_m_ = 0.0;
  double center_total_m_ = 0.0;
  double yaw_total_rad_ = 0.0;

  // Integrated 2D pose reconstructed directly from wheel ticks.
  // center_total_m_ is the signed path length along the travelled curve.
  // x_m_/y_m_ represent start-to-current displacement in the odometry frame.
  double x_m_ = 0.0;
  double y_m_ = 0.0;

  std::ofstream file_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<roboteq_ros2_driver::msg::WheelTicks>::SharedPtr ticks_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  static std::string utc_timestamp_for_filename()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    gmtime_r(&now_c, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string utc_timestamp_now()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    gmtime_r(&now_c, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double straight_line_distance_m() const
  {
    return std::sqrt((x_m_ * x_m_) + (y_m_ * y_m_));
  }

  void open_log_file()
  {
    fs::create_directories(log_dir_);

    std::ostringstream path;
    path << log_dir_
         << "/"
         << test_name_
         << "_"
         << utc_timestamp_for_filename()
         << ".csv";

    log_path_ = path.str();

    file_.open(log_path_, std::ios::out | std::ios::trunc);

    if (!file_.is_open()) {
      throw std::runtime_error("Failed to open odometry log file: " + log_path_);
    }

    file_
      << "timestamp_ros_s,"
      << "timestamp_utc,"
      << "event,"
      << "cmd_linear_x_mps,"
      << "cmd_angular_z_radps,"
      << "left_delta_ticks,"
      << "right_delta_ticks,"
      << "left_total_ticks,"
      << "right_total_ticks,"
      << "left_delta_m,"
      << "right_delta_m,"
      << "left_total_m,"
      << "right_total_m,"
      << "center_delta_m,"
      << "center_total_m,"
      << "delta_yaw_rad,"
      << "yaw_total_rad,"
      << "path_length_m,"
      << "x_m,"
      << "y_m,"
      << "yaw_rad,"
      << "straight_line_distance_m,"
      << "odom_available,"
      << "odom_x_m,"
      << "odom_y_m,"
      << "odom_yaw_rad,"
      << "message"
      << "\n";

    file_.flush();
  }

  void write_metadata_row(const std::string & event, const std::string & message)
  {
    const double t = this->now().seconds();

    file_
      << std::fixed << std::setprecision(9)
      << t << ","
      << utc_timestamp_now() << ","
      << event << ","
      << latest_cmd_linear_x_ << ","
      << latest_cmd_angular_z_ << ","
      << 0 << ","
      << 0 << ","
      << left_total_ticks_ << ","
      << right_total_ticks_ << ","
      << 0.0 << ","
      << 0.0 << ","
      << left_total_m_ << ","
      << right_total_m_ << ","
      << 0.0 << ","
      << center_total_m_ << ","
      << 0.0 << ","
      << yaw_total_rad_ << ","
      << center_total_m_ << ","
      << x_m_ << ","
      << y_m_ << ","
      << yaw_total_rad_ << ","
      << straight_line_distance_m() << ","
      << (odom_received_ ? "true" : "false") << ","
      << latest_odom_x_ << ","
      << latest_odom_y_ << ","
      << latest_odom_yaw_ << ","
      << message
      << "\n";

    file_.flush();
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_linear_x_ = msg->linear.x;
    latest_cmd_angular_z_ = msg->angular.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_received_ = true;
    latest_odom_x_ = msg->pose.pose.position.x;
    latest_odom_y_ = msg->pose.pose.position.y;
    latest_odom_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
  }

  void ticks_callback(const roboteq_ros2_driver::msg::WheelTicks::SharedPtr msg)
  {
    const int64_t left_delta_ticks = static_cast<int64_t>(msg->left_ticks);
    const int64_t right_delta_ticks = static_cast<int64_t>(msg->right_ticks);

    if (!log_zero_ticks_ && left_delta_ticks == 0 && right_delta_ticks == 0) {
      return;
    }

    left_total_ticks_ += left_delta_ticks;
    right_total_ticks_ += right_delta_ticks;

    const double left_delta_m = static_cast<double>(left_delta_ticks) * meters_per_tick_;
    const double right_delta_m = static_cast<double>(right_delta_ticks) * meters_per_tick_;

    left_total_m_ += left_delta_m;
    right_total_m_ += right_delta_m;

    const double center_delta_m = 0.5 * (left_delta_m + right_delta_m);
    center_total_m_ += center_delta_m;

    const double delta_yaw_rad = (right_delta_m - left_delta_m) / wheelbase_m_;

    // Midpoint integration for differential-drive odometry.
    const double theta_mid_rad = yaw_total_rad_ + (0.5 * delta_yaw_rad);
    x_m_ += center_delta_m * std::cos(theta_mid_rad);
    y_m_ += center_delta_m * std::sin(theta_mid_rad);
    yaw_total_rad_ += delta_yaw_rad;

    const double path_length_m = center_total_m_;
    const double straight_line_distance = straight_line_distance_m();

    const double t = this->now().seconds();

    file_
      << std::fixed << std::setprecision(9)
      << t << ","
      << utc_timestamp_now() << ","
      << "TICKS,"
      << latest_cmd_linear_x_ << ","
      << latest_cmd_angular_z_ << ","
      << left_delta_ticks << ","
      << right_delta_ticks << ","
      << left_total_ticks_ << ","
      << right_total_ticks_ << ","
      << left_delta_m << ","
      << right_delta_m << ","
      << left_total_m_ << ","
      << right_total_m_ << ","
      << center_delta_m << ","
      << center_total_m_ << ","
      << delta_yaw_rad << ","
      << yaw_total_rad_ << ","
      << path_length_m << ","
      << x_m_ << ","
      << y_m_ << ","
      << yaw_total_rad_ << ","
      << straight_line_distance << ","
      << (odom_received_ ? "true" : "false") << ","
      << latest_odom_x_ << ","
      << latest_odom_y_ << ","
      << latest_odom_yaw_ << ","
      << "wheel tick update"
      << "\n";

    file_.flush();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTestLogger>());
  rclcpp::shutdown();
  return 0;
}
