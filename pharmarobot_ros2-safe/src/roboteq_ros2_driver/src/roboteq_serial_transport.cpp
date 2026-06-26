// Copyright 2026 Medrobots
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace roboteq_ros2_driver
{
namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

std::string strip_roboteq_line_endings(const std::string & text)
{
  std::string stripped = text;
  while (!stripped.empty() && (stripped.back() == '\r' || stripped.back() == '\n')) {
    stripped.pop_back();
  }
  return stripped;
}

RoboteqSerialTransport::RoboteqSerialTransport(SerialTransportConfig config)
: config_(std::move(config))
{
  serial_.setPort(config_.port);
  serial_.setBaudrate(static_cast<uint32_t>(config_.baud));
  const auto timeout_ms = static_cast<uint32_t>(
    std::max(config_.read_timeout, config_.write_timeout).count());
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
  serial_.setTimeout(timeout);
}

bool RoboteqSerialTransport::open(std::string & error)
{
  try {
    if (!serial_.isOpen()) {
      serial_.open();
    }
    if (!serial_.isOpen()) {
      error = "serial port did not open";
      return false;
    }
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

void RoboteqSerialTransport::close() noexcept
{
  try {
    if (serial_.isOpen()) {
      serial_.close();
    }
  } catch (...) {
  }
}

bool RoboteqSerialTransport::isOpen() const noexcept
{
  try {
    return serial_.isOpen();
  } catch (...) {
    return false;
  }
}

bool RoboteqSerialTransport::sendCommands(
  const std::vector<std::string> & commands, std::string & error)
{
  if (!isOpen()) {
    error = "serial port is not open";
    return false;
  }

  try {
    for (const auto & command : commands) {
      const std::size_t written = serial_.write(command);
      if (written != command.size()) {
        std::ostringstream stream;
        stream << "partial serial write: " << written << " of " << command.size() << " bytes";
        error = stream.str();
        return false;
      }
    }
    serial_.flush();
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

bool RoboteqSerialTransport::query(
  const std::string & command,
  const std::string & expected_prefix,
  std::string & response,
  std::string & error)
{
  if (!isOpen()) {
    error = "serial port is not open";
    return false;
  }

  try {
    const std::size_t written = serial_.write(command);
    if (written != command.size()) {
      std::ostringstream stream;
      stream << "partial serial query write: " << written << " of " << command.size() << " bytes";
      error = stream.str();
      return false;
    }
    serial_.flush();
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + config_.transaction_timeout;
  const std::string stripped_command = strip_roboteq_line_endings(command);
  std::size_t total_bytes = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    std::string line;
    if (!readLine(deadline, line, error)) {
      if (!response.empty()) {
        const std::string partial = line.empty() ? "" : "; subsequent_partial=" + line;
        error = "unexpected response prefix: expected " + expected_prefix + " received " +
          response + partial + "; subsequent_error=" + error;
      } else if (!line.empty()) {
        response = line;
      }
      return false;
    }
    if (line.empty()) {
      continue;
    }

    total_bytes += line.size();
    if (total_bytes > config_.max_response_bytes) {
      error = "serial query response exceeded maximum size";
      return false;
    }

    if (line == stripped_command || line == "+") {
      continue;
    }
    if (line == "-") {
      response = line;
      error = "Roboteq rejected query";
      return false;
    }
    if (starts_with(line, expected_prefix)) {
      response = line;
      return true;
    }
    response = line;
  }

  if (response.empty()) {
    error = "serial query timed out waiting for " + expected_prefix;
  } else {
    error = "unexpected response prefix: expected " + expected_prefix + " received " + response;
  }
  return false;
}

bool RoboteqSerialTransport::readLine(
  const std::chrono::steady_clock::time_point & deadline,
  std::string & line,
  std::string & error)
{
  line.clear();
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (serial_.available() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      char ch = 0;
      if (serial_.read(reinterpret_cast<uint8_t *>(&ch), 1) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (ch == '\r' || ch == '\n') {
        if (!line.empty()) {
          return true;
        }
        continue;
      }
      line.push_back(ch);
      if (line.size() > config_.max_response_bytes) {
        error = "serial query line exceeded maximum size";
        return false;
      }
    } catch (const std::exception & ex) {
      error = ex.what();
      return false;
    }
  }

  error = "serial query timed out before line delimiter";
  return false;
}

}  // namespace roboteq_ros2_driver
