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
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "roboteq_ros2_driver/phase5b_harness_logic.hpp"
#include "roboteq_ros2_driver/roboteq_serial_transport.hpp"
#include "roboteq_ros2_driver/roboteq_serial_worker.hpp"

namespace driver = roboteq_ros2_driver;
using namespace std::chrono_literals;

namespace
{

constexpr auto kHarnessWatchdog = 10s;
constexpr auto kDeferredEncoderPoll = std::chrono::hours(24);
const std::vector<std::string> kExactStop{
  "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};

struct Arguments
{
  std::string port;
  int baud{0};
  std::string operator_name;
  std::string output;
  std::string scenario;
  std::string query;
  int deadline_ms{0};
  int attempts{0};
  std::string phase_plan;
};

std::string value_after(int & index, int argc, char ** argv)
{
  if (++index >= argc) {
    throw std::runtime_error("missing option value");
  }
  return argv[index];
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--port") {
      args.port = value_after(index, argc, argv);
    } else if (option == "--baud") {
      args.baud = std::stoi(value_after(index, argc, argv));
    } else if (option == "--operator") {
      args.operator_name = value_after(index, argc, argv);
    } else if (option == "--output") {
      args.output = value_after(index, argc, argv);
    } else if (option == "--query") {
      args.query = value_after(index, argc, argv);
    } else if (option == "--deadline-ms") {
      args.deadline_ms = std::stoi(value_after(index, argc, argv));
    } else if (option == "--attempts") {
      args.attempts = std::stoi(value_after(index, argc, argv));
    } else if (option == "--phase-plan") {
      args.phase_plan = value_after(index, argc, argv);
    } else if (!option.empty() && option.front() != '-' && args.scenario.empty()) {
      args.scenario = option;
    } else {
      throw std::runtime_error("unsupported argument: " + option);
    }
  }
  const bool scenario_ok =
    args.scenario == "preselection" || args.scenario == "normal" ||
    args.scenario == "timeout" || args.scenario == "bounded-resync" ||
    args.scenario == "fallback-injected";
  const bool query_ok = args.query == "FF" || args.query == "FS";
  const bool deadline_ok = args.deadline_ms == 3 || args.deadline_ms == 100;
  const bool plan_ok =
    args.phase_plan == "before-selection" || args.phase_plan == "after-write" ||
    args.phase_plan == "waiting-first-byte" || args.phase_plan == "after-first-byte" ||
    args.phase_plan == "after-reply" || args.phase_plan == "timeout-unresolved" ||
    args.phase_plan == "drain-started" || args.phase_plan == "drain-completed" ||
    args.phase_plan == "before-synchronization" ||
    args.phase_plan == "waiting-synchronization" ||
    args.phase_plan == "after-synchronization" ||
    args.phase_plan == "before-fallback-close" ||
    args.phase_plan == "reconnect-complete";
  if (args.port != "/dev/roboteq" || args.baud != 115200 || args.operator_name.empty() ||
    args.output.empty() || !scenario_ok || !query_ok || !deadline_ok ||
    args.attempts <= 0 || args.attempts > 100 || !plan_ok)
  {
    throw std::runtime_error("arguments are outside the fixed Phase 5B validation policy");
  }
  return args;
}

std::string time_point_ns_or_null(const std::chrono::steady_clock::time_point & value)
{
  if (value == std::chrono::steady_clock::time_point{}) {
    return "null";
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    value.time_since_epoch()).count();
  return std::to_string(ns);
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

class RestrictedTransport : public driver::IRoboteqSerialTransport
{
public:
  using CommandObserver = std::function<void (
        const std::vector<std::string> &, const driver::CommandTransactionResult &)>;
  using StartupDrainObserver = std::function<void (const driver::StartupDrainResult &)>;

  RestrictedTransport(
    std::unique_ptr<driver::IRoboteqSerialTransport> transport,
    bool inject,
    CommandObserver command_observer,
    StartupDrainObserver startup_drain_observer)
  : transport_(std::move(transport)),
    inject_fallback_(inject),
    command_observer_(std::move(command_observer)),
    startup_drain_observer_(std::move(startup_drain_observer))
  {
  }

  bool open(std::string & error) override {return transport_->open(error);}
  void close() noexcept override {transport_->close();}
  bool isOpen() const noexcept override {return transport_->isOpen();}
  driver::StartupDrainResult drainStartupInput(
    const driver::StartupDrainBounds & bounds) override
  {
    auto result = transport_->drainStartupInput(bounds);
    observeStartupDrain(result);
    return result;
  }

  driver::CommandTransactionResult commandTransaction(
    const std::vector<std::string> & commands,
    const driver::CommandTransactionBounds & bounds) override
  {
    if (commands != kExactStop) {
      unexpected_.store(true);
      driver::CommandTransactionResult result;
      result.reason = "harness rejected a write outside the exact four-command zero batch";
      observeCommand(commands, result);
      return result;
    }
    auto result = transport_->commandTransaction(commands, bounds);
    observeCommand(commands, result);
    return result;
  }

  bool query(
    const std::string & command, const std::string & prefix,
    std::string & response, std::string & error) override
  {
    if (command != "?FID\r" || prefix != "FID=") {
      unexpected_.store(true);
      error = "harness rejected a query outside startup FID validation";
      return false;
    }
    return transport_->query(command, prefix, response, error);
  }

  driver::DiagnosticTransactionResult diagnosticQuery(
    const driver::DiagnosticTransaction & transaction) override
  {
    if (!allowedDiagnostic(transaction)) {
      unexpected_.store(true);
      throw std::runtime_error("harness rejected diagnostic outside FF/FS allowlist");
    }
    return transport_->diagnosticQuery(transaction);
  }

  driver::DiagnosticRecoveryResult boundedDiagnosticRecovery(
    const driver::DiagnosticTransaction & timed_out,
    std::chrono::steady_clock::time_point started,
    const driver::DiagnosticTransaction & synchronization,
    const driver::DiagnosticRecoveryBounds & bounds,
    const std::function<bool(std::string &)> & checkpoint) override
  {
    if (!allowedDiagnostic(timed_out) || !allowedDiagnostic(synchronization)) {
      unexpected_.store(true);
      throw std::runtime_error("harness rejected recovery outside FF/FS allowlist");
    }
    auto result = transport_->boundedDiagnosticRecovery(
      timed_out, started, synchronization, bounds, checkpoint);
    if (inject_fallback_) {
      result.synchronized = false;
      result.reason = "synthetic reconnect-fallback path validation";
    }
    return result;
  }

  bool unexpected() const {return unexpected_.load();}

private:
  static bool allowedDiagnostic(const driver::DiagnosticTransaction & transaction)
  {
    return (transaction.command == "?FF\r" && transaction.expected_prefix == "FF=") ||
           (transaction.command == "?FS\r" && transaction.expected_prefix == "FS=");
  }

  void observeCommand(
    const std::vector<std::string> & commands,
    const driver::CommandTransactionResult & result) noexcept
  {
    if (!command_observer_) {
      return;
    }
    try {
      command_observer_(commands, result);
    } catch (...) {
      unexpected_.store(true);
    }
  }

  void observeStartupDrain(const driver::StartupDrainResult & result) noexcept
  {
    if (!startup_drain_observer_) {
      return;
    }
    try {
      startup_drain_observer_(result);
    } catch (...) {
      unexpected_.store(true);
    }
  }

  std::unique_ptr<driver::IRoboteqSerialTransport> transport_;
  bool inject_fallback_{false};
  CommandObserver command_observer_;
  StartupDrainObserver startup_drain_observer_;
  std::atomic<bool> unexpected_{false};
};

std::optional<driver::DiagnosticPhase> planned_phase(const std::string & plan)
{
  if (plan == "after-write") {return driver::DiagnosticPhase::write_accepted;}
  if (plan == "waiting-first-byte") {return driver::DiagnosticPhase::waiting_for_first_byte;}
  if (plan == "after-first-byte") {return driver::DiagnosticPhase::first_byte_received;}
  if (plan == "after-reply") {return driver::DiagnosticPhase::response_complete;}
  if (plan == "timeout-unresolved") {return driver::DiagnosticPhase::timeout_or_unresolved;}
  if (plan == "drain-started") {return driver::DiagnosticPhase::drain_started;}
  if (plan == "drain-completed") {return driver::DiagnosticPhase::drain_completed;}
  if (plan == "before-synchronization") {return driver::DiagnosticPhase::before_synchronization;}
  if (plan == "waiting-synchronization") {
    return driver::DiagnosticPhase::waiting_for_synchronization;
  }
  if (plan == "after-synchronization") {
    return driver::DiagnosticPhase::synchronization_complete;
  }
  if (plan == "before-fallback-close") {return driver::DiagnosticPhase::before_fallback_close;}
  if (plan == "reconnect-complete") {return driver::DiagnosticPhase::reconnect_complete;}
  return std::nullopt;
}

const char * phase_name(driver::DiagnosticPhase phase)
{
  switch (phase) {
    case driver::DiagnosticPhase::selected: return "selected";
    case driver::DiagnosticPhase::write_started: return "write_started";
    case driver::DiagnosticPhase::write_accepted: return "write_accepted";
    case driver::DiagnosticPhase::waiting_for_first_byte: return "waiting_for_first_byte";
    case driver::DiagnosticPhase::first_byte_received: return "first_byte_received";
    case driver::DiagnosticPhase::response_complete: return "response_complete";
    case driver::DiagnosticPhase::transaction_complete: return "transaction_complete";
    case driver::DiagnosticPhase::timeout_or_unresolved: return "timeout_or_unresolved";
    case driver::DiagnosticPhase::drain_started: return "drain_started";
    case driver::DiagnosticPhase::drain_completed: return "drain_completed";
    case driver::DiagnosticPhase::before_synchronization: return "before_synchronization";
    case driver::DiagnosticPhase::waiting_for_synchronization: return "waiting_synchronization";
    case driver::DiagnosticPhase::synchronization_complete: return "synchronization_complete";
    case driver::DiagnosticPhase::before_fallback_close: return "before_fallback_close";
    case driver::DiagnosticPhase::reconnect_complete: return "reconnect_complete";
  }
  return "unknown";
}

const char * stop_phase_name(driver::StopRequestPhase phase)
{
  switch (phase) {
    case driver::StopRequestPhase::requested: return "requested";
    case driver::StopRequestPhase::coalesced: return "coalesced";
    case driver::StopRequestPhase::write_started: return "write_started";
    case driver::StopRequestPhase::write_accepted: return "write_accepted";
    case driver::StopRequestPhase::write_failed: return "write_failed";
  }
  return "unknown";
}

const char * connection_state_name(driver::SerialConnectionState state)
{
  switch (state) {
    case driver::SerialConnectionState::disconnected: return "disconnected";
    case driver::SerialConnectionState::connecting: return "connecting";
    case driver::SerialConnectionState::configuring: return "configuring";
    case driver::SerialConnectionState::waiting_for_fresh_command:
      return "waiting_for_fresh_command";
    case driver::SerialConnectionState::ready: return "ready";
    case driver::SerialConnectionState::unhealthy: return "unhealthy";
    case driver::SerialConnectionState::reconnecting: return "reconnecting";
  }
  return "unknown";
}

const char * framing_state_name(driver::SerialFramingState state)
{
  switch (state) {
    case driver::SerialFramingState::synchronized: return "synchronized";
    case driver::SerialFramingState::unresolved: return "unresolved";
  }
  return "unknown";
}

std::string worker_status_json_line(
  const char * label,
  const driver::SerialWorkerStatus & status)
{
  std::ostringstream line;
  line << "{\"type\":\"worker_status\""
       << ",\"label\":\"" << label << "\""
       << ",\"connection_state\":\"" << connection_state_name(status.connection_state) << "\""
       << ",\"framing_state\":\"" << framing_state_name(status.framing_state) << "\""
       << ",\"transport_open\":" << (status.transport_open ? "true" : "false")
       << ",\"ready_for_motion\":" << (status.ready_for_motion ? "true" : "false")
       << ",\"generation\":" << status.connection_generation
       << ",\"diagnostic_recovery_pending\":"
       << (status.diagnostic_recovery_pending ? "true" : "false")
       << ",\"update_sequence\":" << status.update_sequence
       << "}";
  return line.str();
}

bool allowed_transition(
  const std::optional<driver::DiagnosticPhase> & previous,
  driver::DiagnosticPhase next)
{
  if (!previous.has_value()) {return next == driver::DiagnosticPhase::selected;}
  using Phase = driver::DiagnosticPhase;
  switch (next) {
    case Phase::write_started:
      return *previous == Phase::selected || *previous == Phase::waiting_for_synchronization;
    case Phase::write_accepted: return *previous == Phase::write_started;
    case Phase::waiting_for_first_byte: return *previous == Phase::write_accepted;
    case Phase::first_byte_received: return *previous == Phase::waiting_for_first_byte;
    case Phase::response_complete: return *previous == Phase::first_byte_received;
    case Phase::transaction_complete: return *previous == Phase::response_complete;
    case Phase::timeout_or_unresolved:
      return *previous == Phase::waiting_for_first_byte ||
             *previous == Phase::first_byte_received || *previous == Phase::response_complete ||
             *previous == Phase::write_started;
    case Phase::drain_started: return *previous == Phase::timeout_or_unresolved;
    case Phase::drain_completed: return *previous == Phase::drain_started;
    case Phase::before_synchronization: return *previous == Phase::drain_completed;
    case Phase::waiting_for_synchronization: return *previous == Phase::before_synchronization;
    case Phase::synchronization_complete: return *previous == Phase::transaction_complete;
    case Phase::before_fallback_close:
      return *previous == Phase::timeout_or_unresolved || *previous == Phase::drain_started ||
             *previous == Phase::drain_completed || *previous == Phase::before_synchronization ||
             *previous == Phase::waiting_for_synchronization ||
             *previous == Phase::synchronization_complete;
    case Phase::reconnect_complete: return *previous == Phase::before_fallback_close;
    case Phase::selected: return false;
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = parse_arguments(argc, argv);
    EvidenceFile evidence(args.output);
    std::mutex mutex;
    std::condition_variable cv;
    bool evidence_failure = false;
    driver::SerialTransportConfig transport_config;
    transport_config.port = args.port;
    transport_config.baud = args.baud;
    transport_config.read_timeout = 50ms;
    transport_config.write_timeout = 50ms;
    transport_config.transaction_timeout = 100ms;
    transport_config.query_observer = [&](const driver::QueryTraceEvent & event) {
        try {
          evidence.append(driver::phase5bQueryTraceJsonLine(event));
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      };

    int accepted_stops = 0;
    int completed_attempts = 0;
    bool phase_fired = false;
    bool invalid_transition = false;
    uint64_t phase_correlation = 0;
    std::optional<driver::DiagnosticPhase> previous_phase;
    std::optional<driver::Phase5bAttemptDecision> attempt_decision;
    driver::SerialIoWorker * worker_view = nullptr;
    const auto target_phase = planned_phase(args.phase_plan);

    auto restricted = std::make_unique<RestrictedTransport>(
      std::make_unique<driver::RoboteqSerialTransport>(transport_config),
      args.scenario == "fallback-injected",
      [&](const std::vector<std::string> & commands,
      const driver::CommandTransactionResult & result) {
        try {
          evidence.append(driver::phase5bCommandTransactionJsonLine(commands, result));
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      },
      [&](const driver::StartupDrainResult & result) {
        try {
          evidence.append(driver::phase5bStartupDrainJsonLine(result));
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      });
    auto * restricted_view = restricted.get();

    driver::SerialWorkerConfig worker_config;
    worker_config.encoder_poll_period = kDeferredEncoderPoll;
    worker_config.command_timeout = kDeferredEncoderPoll;
    worker_config.reconnect_interval = 100ms;
    worker_config.diagnostic_query_timeout = std::chrono::milliseconds(args.deadline_ms);
    worker_config.required_settings.clear();
    worker_config.log_callback = [&](const std::string & message) {
        try {
          std::ostringstream line;
          line << "{\"type\":\"worker_log\",\"message\":\"" <<
            driver::jsonEscapeExactBytes(message) << "\"}";
          evidence.append(line.str());
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      };
    worker_config.stop_request_observer = [&](const driver::StopRequestEvent & event) {
        try {
          if (event.phase == driver::StopRequestPhase::write_accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            ++accepted_stops;
            cv.notify_all();
          }
          std::ostringstream line;
          line << "{\"type\":\"stop\",\"phase\":\"" << stop_phase_name(event.phase) <<
            "\",\"correlation\":" << event.correlation_id <<
            ",\"generation\":" << event.connection_generation <<
            ",\"monotonic_ns\":" << time_point_ns_or_null(event.timestamp) <<
            ",\"accepted_bytes\":" << event.byte_count << "}";
          evidence.append(line.str());
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      };
    worker_config.diagnostic_phase_observer = [&](const driver::DiagnosticPhaseEvent & event) {
        try {
          bool fire_stop = false;
          if (event.phase == driver::DiagnosticPhase::selected) {
            phase_correlation = event.correlation_id;
            previous_phase.reset();
          }
          if (event.correlation_id != phase_correlation ||
            !allowed_transition(previous_phase, event.phase))
          {
            std::lock_guard<std::mutex> lock(mutex);
            invalid_transition = true;
            cv.notify_all();
            return;
          }
          previous_phase = event.phase;
          std::ostringstream line;
          line << "{\"type\":\"diagnostic_phase\",\"phase\":\"" << phase_name(event.phase) <<
            "\",\"correlation\":" << event.correlation_id <<
            ",\"generation\":" << event.connection_generation <<
            ",\"monotonic_ns\":" << time_point_ns_or_null(event.timestamp) <<
            ",\"command\":\"" << driver::jsonEscapeExactBytes(event.command) <<
            "\",\"byte_count\":" << event.byte_count << "}";
          evidence.append(line.str());
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (target_phase.has_value() && event.phase == *target_phase && !phase_fired) {
              phase_fired = true;
              fire_stop = true;
            }
          }
          const auto decision = driver::phase5bAttemptDecisionForPhase(
            args.scenario, event.phase);
          if (decision.has_value()) {
            std::lock_guard<std::mutex> lock(mutex);
            attempt_decision = *decision;
            ++completed_attempts;
            cv.notify_all();
          }
          if (fire_stop) {
            worker_view->requestStop();
          }
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      };
    worker_config.diagnostic_result_observer = [&](const driver::DiagnosticResultEvent & event) {
        try {
          evidence.append(driver::phase5bDiagnosticResultJsonLine(event));
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          evidence_failure = true;
          cv.notify_all();
        }
      };

    driver::SerialIoWorker worker(std::move(restricted), worker_config);
    worker_view = &worker;
    worker.start();
    const auto ready_deadline = std::chrono::steady_clock::now() + kHarnessWatchdog;
    while (worker.status().connection_state !=
      driver::SerialConnectionState::waiting_for_fresh_command &&
      std::chrono::steady_clock::now() < ready_deadline)
    {
      std::this_thread::sleep_for(2ms);
    }
    if (worker.status().connection_state !=
      driver::SerialConnectionState::waiting_for_fresh_command)
    {
      try {
        evidence.append(worker_status_json_line("startup_watchdog", worker.status()));
      } catch (...) {
        throw std::runtime_error("evidence write or fsync failure");
      }
      throw std::runtime_error("harness watchdog expired during startup");
    }
    evidence.append(worker_status_json_line("startup_ready", worker.status()));

    for (int attempt = 0; attempt < args.attempts; ++attempt) {
      int expected_stops = 0;
      {
        std::lock_guard<std::mutex> lock(mutex);
        phase_fired = false;
        expected_stops = accepted_stops + 1;
        attempt_decision.reset();
      }
      if (args.phase_plan == "before-selection") {
        worker.requestStop();
      }
      const auto query = args.query == "FF" ?
        driver::DiagnosticQueryKind::fault_flags : driver::DiagnosticQueryKind::status_flags;
      if (!worker.queueDiagnosticQuery(query)) {
        throw std::runtime_error("diagnostic queue rejected fixed harness query");
      }
      std::unique_lock<std::mutex> lock(mutex);
      if (!cv.wait_for(
          lock, kHarnessWatchdog, [&]() {
            return invalid_transition || evidence_failure ||
            (accepted_stops >= expected_stops && completed_attempts >= attempt + 1);
          }))
      {
        throw std::runtime_error("harness watchdog expired waiting for requested stop");
      }
      if (invalid_transition) {
        throw std::runtime_error("unexpected diagnostic phase transition");
      }
      if (evidence_failure) {
        throw std::runtime_error("evidence write or fsync failure");
      }

      const auto status = worker.status();
      const auto telemetry = worker.latestDiagnosticTelemetry();
      std::optional<driver::Phase5bDiagnosticEvidence> diagnostic_evidence;
      if (telemetry.has_value()) {
        diagnostic_evidence = driver::Phase5bDiagnosticEvidence{
          telemetry->correlation_id,
          telemetry->connection_generation,
          time_point_ns_or_null(telemetry->started_at),
          time_point_ns_or_null(telemetry->write_accepted_at),
          time_point_ns_or_null(telemetry->first_byte_at),
          time_point_ns_or_null(telemetry->last_byte_at),
          time_point_ns_or_null(telemetry->timeout_at),
          telemetry->delimiter_observed,
          std::to_string(telemetry->raw_value.size()),
          telemetry->raw_value,
          driver::bytesToHex(telemetry->raw_value),
          telemetry->failure_reason};
      }
      evidence.append(
        driver::phase5bAttemptResultJsonLine(
          attempt + 1,
          args.scenario,
          attempt_decision.has_value() ? attempt_decision->outcome : "watchdog",
          attempt_decision.has_value() && attempt_decision->success,
          connection_state_name(status.connection_state),
          framing_state_name(status.framing_state),
          diagnostic_evidence));

      if (attempt_decision.has_value() && !attempt_decision->success) {
        worker.stop();
        throw std::runtime_error(
                std::string("attempt ended without a normal diagnostic completion: ") +
                attempt_decision->outcome);
      }
    }
    worker.stop();
    if (restricted_view->unexpected()) {
      throw std::runtime_error("unexpected write or query was attempted");
    }
    if (evidence_failure) {
      throw std::runtime_error("evidence write or fsync failure");
    }
    std::ostringstream summary;
    summary << "{\"type\":\"summary\",\"operator\":\"" <<
      driver::jsonEscapeExactBytes(args.operator_name) << "\",\"scenario\":\"" << args.scenario <<
      "\",\"attempts\":" << args.attempts <<
      ",\"synthetic_fault_injection\":" <<
      (args.scenario == "fallback-injected" ? "true" : "false") <<
      ",\"evidence_label\":\"" <<
      (args.scenario == "fallback-injected" ?
    "synthetic reconnect-fallback path validation" : "physical observation") << "\"}";
    evidence.append(summary.str());
    return EXIT_SUCCESS;
  } catch (const std::exception & ex) {
    std::cerr << "Phase 5B harness aborted: " << ex.what() << '\n';
    return EXIT_FAILURE;
  }
}
