#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <serial/serial.h>

namespace roboteq_ros2_driver
{

struct SerialTransportConfig
{
  std::string port{"/dev/roboteq"};
  int baud{115200};
  std::chrono::milliseconds read_timeout{50};
  std::chrono::milliseconds write_timeout{50};
  std::chrono::milliseconds transaction_timeout{100};
  std::size_t max_response_bytes{256};
};

class IRoboteqSerialTransport
{
public:
  virtual ~IRoboteqSerialTransport() = default;

  virtual bool open(std::string & error) = 0;
  virtual void close() noexcept = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual bool sendCommands(const std::vector<std::string> & commands, std::string & error) = 0;
  virtual bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) = 0;
};

class RoboteqSerialTransport : public IRoboteqSerialTransport
{
public:
  explicit RoboteqSerialTransport(SerialTransportConfig config);

  bool open(std::string & error) override;
  void close() noexcept override;
  bool isOpen() const noexcept override;
  bool sendCommands(const std::vector<std::string> & commands, std::string & error) override;
  bool query(
    const std::string & command,
    const std::string & expected_prefix,
    std::string & response,
    std::string & error) override;

private:
  bool readLine(
    const std::chrono::steady_clock::time_point & deadline,
    std::string & line,
    std::string & error);

  SerialTransportConfig config_;
  serial::Serial serial_;
};

std::string strip_roboteq_line_endings(const std::string & text);

}  // namespace roboteq_ros2_driver

#endif  // ROBOTEQ_ROS2_DRIVER__ROBOTEQ_SERIAL_TRANSPORT_HPP_
