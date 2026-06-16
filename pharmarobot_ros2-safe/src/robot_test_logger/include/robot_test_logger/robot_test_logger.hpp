#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

class RobotTestLogger
{
public:
  explicit RobotTestLogger(
    rclcpp::Node * node,
    const std::string & log_dir = "",
    bool flush_each_row = true)
  : node_(node),
    flush_each_row_(flush_each_row)
  {
    std::string final_log_dir = log_dir;

    if (final_log_dir.empty()) {
      const char * home = std::getenv("HOME");
      if (home != nullptr) {
        final_log_dir = std::string(home) + "/robot_logs";
      } else {
        final_log_dir = "/tmp/robot_logs";
      }
    }

    std::filesystem::create_directories(final_log_dir);

    file_path_ = final_log_dir + "/robot_test_log_" + timestampForFilename() + ".csv";

    file_.open(file_path_, std::ios::out | std::ios::app);

    if (!file_.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open robot test log file: %s", file_path_.c_str());
      return;
    }

    writeHeader();

    RCLCPP_INFO(node_->get_logger(), "Robot test log file created: %s", file_path_.c_str());
  }

  ~RobotTestLogger()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (file_.is_open()) {
      file_.flush();
      file_.close();
    }
  }

  std::string filePath() const
  {
    return file_path_;
  }

  void logJoystickConnected(bool connected, const std::string & message = "")
  {
    LogRow row;
    row.event = connected ? "JOYSTICK_CONNECTED" : "JOYSTICK_DISCONNECTED";
    row.joystick_connected = boolToStr(connected);
    row.message = message;
    write(row);
  }

  void logDeadman(bool active, const std::string & speed_mode, const std::string & message = "")
  {
    LogRow row;
    row.event = active ? "DEADMAN_ACTIVE" : "DEADMAN_INACTIVE";
    row.deadman_active = boolToStr(active);
    row.speed_mode = speed_mode;
    row.message = message;
    write(row);
  }

  void logSpeedMode(const std::string & speed_mode, const std::string & message = "")
  {
    LogRow row;
    row.event = "SPEED_MODE_CHANGED";
    row.speed_mode = speed_mode;
    row.message = message;
    write(row);
  }

  void logVelocityCommand(
    double linear_x,
    double angular_z,
    bool joystick_connected,
    bool deadman_active,
    const std::string & speed_mode,
    const std::string & message = "")
  {
    LogRow row;
    row.event = "VELOCITY_COMMAND";
    row.joystick_connected = boolToStr(joystick_connected);
    row.deadman_active = boolToStr(deadman_active);
    row.speed_mode = speed_mode;
    row.linear_x = toStr(linear_x);
    row.angular_z = toStr(angular_z);
    row.message = message;
    write(row);
  }

  void logCommandRejected(
    double linear_x,
    double angular_z,
    const std::string & reason)
  {
    LogRow row;
    row.event = "COMMAND_REJECTED";
    row.command_rejected = "true";
    row.linear_x = toStr(linear_x);
    row.angular_z = toStr(angular_z);
    row.message = reason;
    write(row);
  }

  void logCommandClamped(
    double input_linear_x,
    double input_angular_z,
    double output_linear_x,
    double output_angular_z,
    const std::string & reason)
  {
    LogRow row;
    row.event = "COMMAND_CLAMPED";
    row.command_clamped = "true";
    row.linear_x = toStr(output_linear_x);
    row.angular_z = toStr(output_angular_z);

    std::ostringstream msg;
    msg << reason
        << " | input_linear_x=" << input_linear_x
        << " input_angular_z=" << input_angular_z
        << " output_linear_x=" << output_linear_x
        << " output_angular_z=" << output_angular_z;

    row.message = msg.str();
    write(row);
  }

  void logCommandTimeout(const std::string & message = "No fresh velocity command received")
  {
    LogRow row;
    row.event = "COMMAND_TIMEOUT";
    row.command_timeout = "true";
    row.message = message;
    write(row);
  }

  void logSerialConnected(bool connected, const std::string & port, const std::string & message = "")
  {
    LogRow row;
    row.event = connected ? "SERIAL_CONNECTED" : "SERIAL_DISCONNECTED";
    row.serial_connected = boolToStr(connected);

    if (message.empty()) {
      row.message = port;
    } else {
      row.message = port + " | " + message;
    }

    write(row);
  }

  void logRoboteqResponseInvalid(const std::string & raw_response)
  {
    LogRow row;
    row.event = "ROBOTEQ_RESPONSE_INVALID";
    row.roboteq_response_invalid = "true";
    row.message = raw_response;
    write(row);
  }

  void logMotorCommandSent(
    double linear_x,
    double angular_z,
    double left_motor_cmd,
    double right_motor_cmd,
    const std::string & message = "")
  {
    LogRow row;
    row.event = "MOTOR_COMMAND_SENT";
    row.motor_command_sent = "true";
    row.linear_x = toStr(linear_x);
    row.angular_z = toStr(angular_z);
    row.left_motor_cmd = toStr(left_motor_cmd);
    row.right_motor_cmd = toStr(right_motor_cmd);
    row.message = message;
    write(row);
  }

  void logFaultStateEntered(const std::string & fault_description)
  {
    LogRow row;
    row.event = "FAULT_STATE_ENTERED";
    row.fault_state = "true";
    row.message = fault_description;
    write(row);
  }

  void logEncoderReading(
    int64_t left_encoder_ticks,
    int64_t right_encoder_ticks,
    const std::string & message = "")
  {
    LogRow row;
    row.event = "ENCODER_READING";
    row.left_encoder_ticks = std::to_string(left_encoder_ticks);
    row.right_encoder_ticks = std::to_string(right_encoder_ticks);
    row.message = message;
    write(row);
  }

  void logOdomReading(
    int64_t left_encoder_ticks,
    int64_t right_encoder_ticks,
    double odom_x,
    double odom_y,
    double odom_yaw,
    const std::string & message = "")
  {
    LogRow row;
    row.event = "ODOM_READING";
    row.left_encoder_ticks = std::to_string(left_encoder_ticks);
    row.right_encoder_ticks = std::to_string(right_encoder_ticks);
    row.odom_x = toStr(odom_x);
    row.odom_y = toStr(odom_y);
    row.odom_yaw = toStr(odom_yaw);
    row.message = message;
    write(row);
  }



private:
  struct LogRow
  {
    std::string event;

    std::string joystick_connected;
    std::string deadman_active;
    std::string speed_mode;

    std::string command_rejected = "false";
    std::string command_clamped = "false";
    std::string command_timeout = "false";

    std::string serial_connected;
    std::string roboteq_response_invalid = "false";
    std::string motor_command_sent = "false";
    std::string fault_state = "false";

    std::string linear_x;
    std::string angular_z;
    std::string left_motor_cmd;
    std::string right_motor_cmd;

    std::string left_encoder_ticks;
    std::string right_encoder_ticks;

    std::string odom_x;
    std::string odom_y;
    std::string odom_yaw;

    std::string message;
  };

  rclcpp::Node * node_;
  std::ofstream file_;
  std::string file_path_;
  bool flush_each_row_;
  std::mutex mutex_;

  void writeHeader()
  {
    file_
      << "timestamp_utc,"
      << "event,"
      << "joystick_connected,"
      << "deadman_active,"
      << "speed_mode,"
      << "command_rejected,"
      << "command_clamped,"
      << "command_timeout,"
      << "serial_connected,"
      << "roboteq_response_invalid,"
      << "motor_command_sent,"
      << "fault_state,"
      << "linear_x,"
      << "angular_z,"
      << "left_motor_cmd,"
      << "right_motor_cmd,"
      << "left_encoder_ticks,"
      << "right_encoder_ticks,"
      << "odom_x,"
      << "odom_y,"
      << "odom_yaw,"
      << "message\n";

    if (flush_each_row_) {
      file_.flush();
    }
  }

  void write(const LogRow & row)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!file_.is_open()) {
      return;
    }

    file_
      << csvEscape(timestampUtc()) << ","
      << csvEscape(row.event) << ","
      << csvEscape(row.joystick_connected) << ","
      << csvEscape(row.deadman_active) << ","
      << csvEscape(row.speed_mode) << ","
      << csvEscape(row.command_rejected) << ","
      << csvEscape(row.command_clamped) << ","
      << csvEscape(row.command_timeout) << ","
      << csvEscape(row.serial_connected) << ","
      << csvEscape(row.roboteq_response_invalid) << ","
      << csvEscape(row.motor_command_sent) << ","
      << csvEscape(row.fault_state) << ","
      << csvEscape(row.linear_x) << ","
      << csvEscape(row.angular_z) << ","
      << csvEscape(row.left_motor_cmd) << ","
      << csvEscape(row.right_motor_cmd) << ","
      << csvEscape(row.left_encoder_ticks) << ","
      << csvEscape(row.right_encoder_ticks) << ","
      << csvEscape(row.odom_x) << ","
      << csvEscape(row.odom_y) << ","
      << csvEscape(row.odom_yaw) << ","
      << csvEscape(row.message)
      << "\n";

    if (flush_each_row_) {
      file_.flush();
    }
  }

  static std::string timestampUtc()
  {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto now_time_t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm utc_tm {};
    gmtime_r(&now_time_t, &utc_tm);

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");
    oss << "." << std::setw(3) << std::setfill('0') << ms.count() << "Z";

    return oss.str();
  }

  static std::string timestampForFilename()
  {
    using namespace std::chrono;

    const auto now = system_clock::now();
    const auto now_time_t = system_clock::to_time_t(now);

    std::tm utc_tm {};
    gmtime_r(&now_time_t, &utc_tm);

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y%m%d_%H%M%S");

    return oss.str();
  }

  static std::string boolToStr(bool value)
  {
    return value ? "true" : "false";
  }

  static std::string toStr(double value)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
  }

  static std::string csvEscape(const std::string & value)
  {
    bool needs_quotes = false;

    for (const char c : value) {
      if (c == ',' || c == '"' || c == '\n' || c == '\r') {
        needs_quotes = true;
        break;
      }
    }

    if (!needs_quotes) {
      return value;
    }

    std::string escaped = "\"";

    for (const char c : value) {
      if (c == '"') {
        escaped += "\"\"";
      } else {
        escaped += c;
      }
    }

    escaped += "\"";

    return escaped;
  }
};