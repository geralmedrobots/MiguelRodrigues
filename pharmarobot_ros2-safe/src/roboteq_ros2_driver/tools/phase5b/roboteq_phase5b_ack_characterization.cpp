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

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "roboteq_ros2_driver/phase5b_ack_characterization_logic.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

constexpr auto kWriteTimeout = 50ms;
constexpr int kDefaultCaptureDeadlineMs = 100;
constexpr int kDefaultQuietPeriodMs = 20;
constexpr int kDefaultMaxBytes = 4096;
constexpr int kDefaultQueryDeadlineMs = 100;
constexpr int kPrewriteCaptureDeadlineMs = 100;
constexpr int kPrewriteQuietPeriodMs = 20;
constexpr int kMaxAttempts = 100;

std::string value_after(int & index, int argc, char ** argv)
{
  if (++index >= argc) {
    throw std::runtime_error("missing option value");
  }
  return argv[index];
}

driver::AckCharacterizationMode parseMode(const std::string & value)
{
  if (value == "h1") {return driver::AckCharacterizationMode::h1_single;}
  if (value == "h2") {return driver::AckCharacterizationMode::h2_batch;}
  if (value == "h3-fid") {return driver::AckCharacterizationMode::h3_stop_then_fid;}
  if (value == "h3-ff") {return driver::AckCharacterizationMode::h3_stop_then_ff;}
  throw std::runtime_error("unsupported mode");
}

driver::ZeroCommandKind parseCommand(const std::string & value)
{
  if (value == "G1") {return driver::ZeroCommandKind::g1_zero;}
  if (value == "G2") {return driver::ZeroCommandKind::g2_zero;}
  if (value == "S1") {return driver::ZeroCommandKind::s1_zero;}
  if (value == "S2") {return driver::ZeroCommandKind::s2_zero;}
  throw std::runtime_error("unsupported zero command selector");
}

driver::AckCharacterizationArguments parseArguments(int argc, char ** argv)
{
  driver::AckCharacterizationArguments args;
  args.capture_deadline_ms = kDefaultCaptureDeadlineMs;
  args.quiet_period_ms = kDefaultQuietPeriodMs;
  args.max_bytes = kDefaultMaxBytes;
  args.query_deadline_ms = kDefaultQueryDeadlineMs;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--port") {
      args.port = value_after(index, argc, argv);
    } else if (option == "--baud") {
      args.baud = std::stoi(value_after(index, argc, argv));
    } else if (option == "--output") {
      args.output = value_after(index, argc, argv);
    } else if (option == "--mode") {
      args.mode = parseMode(value_after(index, argc, argv));
    } else if (option == "--command") {
      args.command = parseCommand(value_after(index, argc, argv));
    } else if (option == "--attempts") {
      args.attempts = std::stoi(value_after(index, argc, argv));
    } else if (option == "--capture-deadline-ms") {
      args.capture_deadline_ms = std::stoi(value_after(index, argc, argv));
    } else if (option == "--quiet-period-ms") {
      args.quiet_period_ms = std::stoi(value_after(index, argc, argv));
    } else if (option == "--max-bytes") {
      args.max_bytes = std::stoi(value_after(index, argc, argv));
    } else if (option == "--query-deadline-ms") {
      args.query_deadline_ms = std::stoi(value_after(index, argc, argv));
    } else {
      throw std::runtime_error("unsupported argument: " + option);
    }
  }
  if (args.port != "/dev/roboteq" || args.baud != 115200 || args.output.empty() ||
    args.attempts <= 0 || args.attempts > kMaxAttempts || args.capture_deadline_ms < 20 ||
    args.capture_deadline_ms > 500 || args.quiet_period_ms < 5 || args.quiet_period_ms > 100 ||
    args.max_bytes <= 0 || args.max_bytes > 4096 || args.query_deadline_ms < 20 ||
    args.query_deadline_ms > 500)
  {
    throw std::runtime_error("arguments are outside the fixed Phase 5B ACK characterization policy");
  }
  if (args.mode == driver::AckCharacterizationMode::h1_single && !args.command.has_value()) {
    throw std::runtime_error("H1 requires --command G1|G2|S1|S2");
  }
  if (args.mode != driver::AckCharacterizationMode::h1_single && args.command.has_value()) {
    throw std::runtime_error("--command is valid only for H1");
  }
  return args;
}

class EvidenceFile
{
public:
  explicit EvidenceFile(const std::string & path)
  : descriptor_(::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_APPEND, 0600))
  {
    if (descriptor_ < 0) {
      throw std::runtime_error("could not create evidence file with O_EXCL");
    }
  }

  ~EvidenceFile()
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
  }

  void append(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string record = line + "\n";
    std::size_t offset = 0;
    while (offset < record.size()) {
      const auto written = ::write(descriptor_, record.data() + offset, record.size() - offset);
      if (written <= 0) {
        throw std::runtime_error("evidence write failed");
      }
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor_) != 0) {
      throw std::runtime_error("evidence fsync failed");
    }
  }

private:
  int descriptor_{-1};
  std::mutex mutex_;
};

class RawSerialCapture
{
public:
  struct CaptureResult
  {
    std::vector<driver::ByteObservation> observations;
    bool ended_by_quiet{false};
    bool ended_by_deadline{false};
  };

  explicit RawSerialCapture(const driver::AckCharacterizationArguments & args)
  : args_(args) {}

  void open()
  {
    descriptor_ = ::open(args_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor_ < 0) {
      throw std::runtime_error("serial open failed");
    }
    if (::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error("serial lock failed");
    }
    if (ioctl(descriptor_, TIOCEXCL) != 0) {
      throw std::runtime_error("TIOCEXCL failed");
    }
    termios attrs{};
    if (tcgetattr(descriptor_, &attrs) != 0) {
      throw std::runtime_error("tcgetattr failed");
    }
    cfmakeraw(&attrs);
    if (cfsetispeed(&attrs, B115200) != 0 || cfsetospeed(&attrs, B115200) != 0) {
      throw std::runtime_error("baud configuration failed");
    }
    attrs.c_cflag |= CLOCAL | CREAD;
    attrs.c_cc[VMIN] = 0;
    attrs.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor_, TCSANOW, &attrs) != 0) {
      throw std::runtime_error("tcsetattr failed");
    }
    ++connection_generation_;
  }

  void close() noexcept
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

  int connectionGeneration() const
  {
    return connection_generation_;
  }

  std::pair<std::chrono::steady_clock::time_point, std::chrono::steady_clock::time_point> writeAll(
    const std::string & bytes)
  {
    const auto started_at = std::chrono::steady_clock::now();
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto deadline = std::chrono::steady_clock::now() + kWriteTimeout;
      while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
          throw std::runtime_error("bounded serial write timed out");
        }
        fd_set writable;
        FD_ZERO(&writable);
        FD_SET(descriptor_, &writable);
        const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
          deadline - now);
        timeval timeout{};
        timeout.tv_sec = static_cast<long>(remaining.count() / 1000000);
        timeout.tv_usec = static_cast<long>(remaining.count() % 1000000);
        const auto ready = ::select(descriptor_ + 1, nullptr, &writable, nullptr, &timeout);
        if (ready < 0) {
          throw std::runtime_error("select for write failed");
        }
        if (ready == 0) {
          continue;
        }
        const auto written = ::write(descriptor_, bytes.data() + offset, bytes.size() - offset);
        if (written < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
          }
          throw std::runtime_error("serial write failed");
        }
        offset += static_cast<std::size_t>(written);
        break;
      }
    }
    return {started_at, std::chrono::steady_clock::now()};
  }

  CaptureResult captureUntilQuiet(
    std::chrono::milliseconds absolute_deadline,
    std::chrono::milliseconds quiet_period,
    std::size_t max_bytes,
    bool after_query_write)
  {
    const auto started_at = std::chrono::steady_clock::now();
    auto quiet_deadline = started_at + quiet_period;
    const auto hard_deadline = started_at + absolute_deadline;
    CaptureResult result;
    result.observations.reserve(max_bytes);
    while (std::chrono::steady_clock::now() <= hard_deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= quiet_deadline) {
        result.ended_by_quiet = true;
        break;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        std::min(quiet_deadline, hard_deadline) - now);
      timeval timeout{};
      timeout.tv_sec = static_cast<long>(remaining.count() / 1000000);
      timeout.tv_usec = static_cast<long>(remaining.count() % 1000000);
      fd_set readable;
      FD_ZERO(&readable);
      FD_SET(descriptor_, &readable);
      const auto ready = ::select(descriptor_ + 1, &readable, nullptr, nullptr, &timeout);
      if (ready < 0) {
        throw std::runtime_error("select for read failed");
      }
      if (ready == 0) {
        continue;
      }
      std::array<char, 1> byte{};
      const auto read_now = ::read(descriptor_, byte.data(), byte.size());
      if (read_now < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        throw std::runtime_error("serial read failed");
      }
      if (read_now == 0) {
        continue;
      }
      result.observations.push_back(
        driver::ByteObservation{byte[0], std::chrono::steady_clock::now(), after_query_write});
      if (result.observations.size() > max_bytes) {
        throw std::runtime_error("capture exceeded maximum evidence size");
      }
      quiet_deadline = result.observations.back().timestamp + quiet_period;
    }
    if (!result.ended_by_quiet) {
      result.ended_by_deadline = true;
    }
    return result;
  }

private:
  driver::AckCharacterizationArguments args_;
  int descriptor_{-1};
  int connection_generation_{0};
};

std::string jsonStringArray(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << "\"" << driver::jsonEscapeExactBytes(values[index]) << "\"";
  }
  stream << "]";
  return stream.str();
}

std::string jsonIntArray(const std::vector<int64_t> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}

std::string jsonLinesArray(const std::vector<driver::CapturedLine> & lines)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < lines.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << "{"
           << "\"raw\":\"" << driver::jsonEscapeExactBytes(lines[index].raw_bytes) << "\","
           << "\"hex\":\"" << driver::bytesToHex(lines[index].raw_bytes) << "\","
           << "\"visible\":\"" << driver::visibleBytes(lines[index].raw_bytes) << "\","
           << "\"completed_ns\":" << driver::timePointNsOrNull(lines[index].completed_at) << ","
           << "\"after_query_write\":" << (lines[index].after_query_write ? "true" : "false")
           << "}";
  }
  stream << "]";
  return stream.str();
}

std::string flattenBytes(const std::vector<driver::ByteObservation> & observations)
{
  std::string bytes;
  bytes.reserve(observations.size());
  for (const auto & observation : observations) {
    bytes.push_back(observation.value);
  }
  return bytes;
}

std::chrono::steady_clock::time_point firstTimestamp(
  const std::vector<driver::ByteObservation> & observations)
{
  if (observations.empty()) {
    return {};
  }
  return observations.front().timestamp;
}

std::chrono::steady_clock::time_point lastTimestamp(
  const std::vector<driver::ByteObservation> & observations)
{
  if (observations.empty()) {
    return {};
  }
  return observations.back().timestamp;
}

void appendSessionStart(const driver::AckCharacterizationArguments & args, EvidenceFile & evidence)
{
  std::ostringstream line;
  line << "{\"type\":\"session_start\""
       << ",\"mode\":\"" << driver::ackCharacterizationModeName(args.mode) << "\""
       << ",\"port\":\"" << args.port << "\""
       << ",\"baud\":" << args.baud
       << ",\"attempts\":" << args.attempts
       << ",\"capture_deadline_ms\":" << args.capture_deadline_ms
       << ",\"quiet_period_ms\":" << args.quiet_period_ms
       << ",\"max_bytes\":" << args.max_bytes
       << ",\"query_deadline_ms\":" << args.query_deadline_ms;
  if (args.command.has_value()) {
    line << ",\"command\":\"" << driver::zeroCommandSelector(*args.command) << "\"";
  }
  line << "}";
  evidence.append(line.str());
}

void appendAttemptRecord(
  EvidenceFile & evidence,
  int connection_generation,
  int attempt,
  int capture_deadline_ms,
  int quiet_period_ms,
  const std::string & transmitted_bytes,
  const std::chrono::steady_clock::time_point & write_started_at,
  const std::chrono::steady_clock::time_point & write_accepted_at,
  const RawSerialCapture::CaptureResult & capture_result,
  const std::optional<std::string> & expected_query_prefix,
  const std::optional<int> & query_capture_deadline_ms = std::nullopt,
  const std::optional<std::chrono::steady_clock::time_point> & query_write_started_at = std::nullopt,
  const std::optional<std::chrono::steady_clock::time_point> & query_write_accepted_at = std::nullopt,
  const std::optional<std::string> & query_bytes = std::nullopt)
{
  const auto & observations = capture_result.observations;
  const auto received_bytes = flattenBytes(observations);
  const auto analysis = driver::analyzeCapture(observations, expected_query_prefix);
  std::vector<std::string> stage_labels;
  stage_labels.reserve(observations.size());
  for (const auto & observation : observations) {
    stage_labels.push_back(
      observation.after_query_write ? "after_query_write" : "before_query_write");
  }
  std::ostringstream line;
  line << "{\"type\":\"attempt_result\""
       << ",\"attempt\":" << attempt
       << ",\"connection_generation\":" << connection_generation
       << ",\"transmitted_raw\":\"" << driver::jsonEscapeExactBytes(transmitted_bytes) << "\""
       << ",\"transmitted_hex\":\"" << driver::bytesToHex(transmitted_bytes) << "\""
       << ",\"capture_deadline_ms\":" << capture_deadline_ms
       << ",\"quiet_period_ms\":" << quiet_period_ms
       << ",\"query_capture_deadline_ms\":"
       << (query_capture_deadline_ms.has_value() ? std::to_string(*query_capture_deadline_ms) :
  "null")
       << ",\"write_started_ns\":" << driver::timePointNsOrNull(write_started_at)
       << ",\"write_accepted_ns\":" << driver::timePointNsOrNull(write_accepted_at)
       << ",\"query_write_started_ns\":"
       << (query_write_started_at.has_value() ? driver::timePointNsOrNull(*query_write_started_at) :
  "null")
       << ",\"query_write_accepted_ns\":"
       << (query_write_accepted_at.has_value() ? driver::timePointNsOrNull(*query_write_accepted_at)
  :
  "null")
       << ",\"query_raw\":\""
       << driver::jsonEscapeExactBytes(query_bytes.value_or(std::string{})) << "\""
       << ",\"query_hex\":\"" << driver::bytesToHex(query_bytes.value_or(std::string{})) << "\""
       << ",\"received_raw\":\"" << driver::jsonEscapeExactBytes(received_bytes) << "\""
       << ",\"received_hex\":\"" << driver::bytesToHex(received_bytes) << "\""
       << ",\"received_visible\":\"" << driver::visibleBytes(received_bytes) << "\""
       << ",\"received_byte_count\":" << received_bytes.size()
       << ",\"capture_ended_by_quiet\":" << (capture_result.ended_by_quiet ? "true" : "false")
       << ",\"capture_ended_by_deadline\":" << (capture_result.ended_by_deadline ? "true" : "false")
       << ",\"delimiter_observed\":" << (analysis.delimiter_observed ? "true" : "false")
       << ",\"first_byte_ns\":" << driver::timePointNsOrNull(firstTimestamp(observations))
       << ",\"last_byte_ns\":" << driver::timePointNsOrNull(lastTimestamp(observations))
       << ",\"complete_lines\":" << jsonLinesArray(analysis.lines)
       << ",\"inter_line_gap_ns\":" << jsonIntArray(analysis.inter_line_gap_ns)
       << ",\"trailing_partial_raw\":\"" <<
    driver::jsonEscapeExactBytes(analysis.trailing_partial) << "\""
       << ",\"trailing_partial_hex\":\"" << driver::bytesToHex(analysis.trailing_partial) << "\""
       << ",\"ack_count\":" << analysis.ack_count
       << ",\"ack_count_after_query_write\":" << analysis.ack_count_after_query_write
       << ",\"expected_query_reply_count\":" << analysis.expected_query_reply_count
       << ",\"unexpected_line_count\":" << analysis.unexpected_line_count
       << ",\"byte_stage_labels\":" << jsonStringArray(stage_labels)
       << "}";
  evidence.append(line.str());
}

void appendAbortRecord(EvidenceFile & evidence, const std::string & reason)
{
  std::ostringstream line;
  line << "{\"type\":\"abort\",\"reason\":\"" << driver::jsonEscapeExactBytes(reason) << "\"}";
  evidence.append(line.str());
}

volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
  g_stop_requested = 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parseArguments(argc, argv);
    EvidenceFile evidence(args.output);
    appendSessionStart(args, evidence);
    const auto old_sigint = std::signal(SIGINT, signalHandler);
    const auto old_sigterm = std::signal(SIGTERM, signalHandler);
    RawSerialCapture capture(args);
    try {
      capture.open();
      for (int attempt = 1; attempt <= args.attempts; ++attempt) {
        if (g_stop_requested != 0) {
          appendAbortRecord(evidence, "signal_requested");
          break;
        }
        const auto preexisting = capture.captureUntilQuiet(
          std::chrono::milliseconds(kPrewriteCaptureDeadlineMs),
          std::chrono::milliseconds(kPrewriteQuietPeriodMs),
          static_cast<std::size_t>(args.max_bytes),
          false);
        if (!preexisting.observations.empty()) {
          const auto raw = flattenBytes(preexisting.observations);
          std::ostringstream line;
          line << "{\"type\":\"preexisting_bytes\""
               << ",\"attempt\":" << attempt
               << ",\"connection_generation\":" << capture.connectionGeneration()
               << ",\"received_raw\":\"" << driver::jsonEscapeExactBytes(raw) << "\""
               << ",\"received_hex\":\"" << driver::bytesToHex(raw) << "\""
               << ",\"received_visible\":\"" << driver::visibleBytes(raw) << "\""
               << ",\"received_byte_count\":" << raw.size()
               << ",\"capture_ended_by_quiet\":" << (preexisting.ended_by_quiet ? "true" : "false")
               << ",\"capture_ended_by_deadline\":" <<
            (preexisting.ended_by_deadline ? "true" : "false")
               << "}";
          evidence.append(line.str());
          throw std::runtime_error("unexpected preexisting serial bytes before attempt");
        }

        if (args.mode == driver::AckCharacterizationMode::h1_single) {
          const auto transmitted = driver::zeroCommandBytes(*args.command);
          const auto [write_started, write_accepted] = capture.writeAll(transmitted);
          const auto observations = capture.captureUntilQuiet(
            std::chrono::milliseconds(args.capture_deadline_ms),
            std::chrono::milliseconds(args.quiet_period_ms),
            static_cast<std::size_t>(args.max_bytes),
            false);
          appendAttemptRecord(
            evidence, capture.connectionGeneration(), attempt, args.capture_deadline_ms,
            args.quiet_period_ms, transmitted, write_started, write_accepted, observations,
            std::nullopt);
        } else if (args.mode == driver::AckCharacterizationMode::h2_batch) {
          const auto transmitted = driver::stopBatchBytes();
          const auto [write_started, write_accepted] = capture.writeAll(transmitted);
          const auto observations = capture.captureUntilQuiet(
            std::chrono::milliseconds(args.capture_deadline_ms),
            std::chrono::milliseconds(args.quiet_period_ms),
            static_cast<std::size_t>(args.max_bytes),
            false);
          appendAttemptRecord(
            evidence, capture.connectionGeneration(), attempt, args.capture_deadline_ms,
            args.quiet_period_ms, transmitted, write_started, write_accepted, observations,
            std::nullopt);
        } else {
          const auto transmitted = driver::stopBatchBytes();
          const auto [write_started, write_accepted] = capture.writeAll(transmitted);
          auto observations = capture.captureUntilQuiet(
            std::chrono::milliseconds(args.capture_deadline_ms),
            std::chrono::milliseconds(args.quiet_period_ms),
            static_cast<std::size_t>(args.max_bytes),
            false);
          const auto query_bytes = driver::queryBytesForMode(args.mode);
          const auto [query_write_started, query_write_accepted] = capture.writeAll(query_bytes);
          auto post_query = capture.captureUntilQuiet(
            std::chrono::milliseconds(args.query_deadline_ms),
            std::chrono::milliseconds(args.quiet_period_ms),
            static_cast<std::size_t>(args.max_bytes),
            true);
          observations.observations.insert(
            observations.observations.end(),
            post_query.observations.begin(),
            post_query.observations.end());
          observations.ended_by_quiet = post_query.ended_by_quiet;
          observations.ended_by_deadline = post_query.ended_by_deadline;
          appendAttemptRecord(
            evidence, capture.connectionGeneration(), attempt, args.capture_deadline_ms,
            args.quiet_period_ms, transmitted, write_started, write_accepted, observations,
            driver::queryExpectedPrefixForMode(args.mode),
            args.query_deadline_ms,
            query_write_started, query_write_accepted, query_bytes);
        }
      }
      evidence.append("{\"type\":\"session_end\",\"exit_code\":0}");
    } catch (const std::exception & ex) {
      appendAbortRecord(evidence, ex.what());
      evidence.append("{\"type\":\"session_end\",\"exit_code\":1}");
      capture.close();
      std::signal(SIGINT, old_sigint);
      std::signal(SIGTERM, old_sigterm);
      return 1;
    }
    capture.close();
    std::signal(SIGINT, old_sigint);
    std::signal(SIGTERM, old_sigterm);
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
}
