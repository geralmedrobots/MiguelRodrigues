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

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "roboteq_ros2_driver/phase5b_harness_logic.hpp"

namespace driver = roboteq_ros2_driver;

TEST(Phase5bHarnessLogic, PreselectionTreatsTimeoutAsPartialTerminal)
{
  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "preselection", driver::DiagnosticPhase::timeout_or_unresolved);
  ASSERT_TRUE(decision.has_value());
  EXPECT_FALSE(decision->success);
  EXPECT_STREQ(decision->outcome, "partial_unresolved");
}

TEST(Phase5bHarnessLogic, PreselectionTreatsRecoveredTimeoutAsPartialTerminal)
{
  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "preselection", driver::DiagnosticPhase::synchronization_complete);
  ASSERT_TRUE(decision.has_value());
  EXPECT_FALSE(decision->success);
  EXPECT_STREQ(decision->outcome, "partial_recovered_after_unresolved");
}

TEST(Phase5bHarnessLogic, PreselectionTreatsRecoveryFailureAsTerminalFailure)
{
  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "preselection", driver::DiagnosticPhase::before_fallback_close);
  ASSERT_TRUE(decision.has_value());
  EXPECT_FALSE(decision->success);
  EXPECT_STREQ(decision->outcome, "failed_reconnect_after_unresolved");
}

TEST(Phase5bHarnessLogic, TimeoutScenarioRequiresRecoveryCompletionForSuccess)
{
  EXPECT_FALSE(
    driver::phase5bAttemptDecisionForPhase(
      "timeout", driver::DiagnosticPhase::timeout_or_unresolved).has_value());

  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "timeout", driver::DiagnosticPhase::synchronization_complete);
  ASSERT_TRUE(decision.has_value());
  EXPECT_TRUE(decision->success);
  EXPECT_STREQ(decision->outcome, "success");
}

TEST(Phase5bHarnessLogic, TimeoutScenarioTreatsReconnectAsFailureTerminal)
{
  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "timeout", driver::DiagnosticPhase::reconnect_complete);
  ASSERT_TRUE(decision.has_value());
  EXPECT_FALSE(decision->success);
  EXPECT_STREQ(decision->outcome, "failed_reconnect_after_unresolved");
}

TEST(Phase5bHarnessLogic, FallbackInjectedStillWaitsForReconnectCompletion)
{
  EXPECT_FALSE(
    driver::phase5bAttemptDecisionForPhase(
      "fallback-injected", driver::DiagnosticPhase::before_fallback_close).has_value());

  const auto decision = driver::phase5bAttemptDecisionForPhase(
    "fallback-injected", driver::DiagnosticPhase::reconnect_complete);
  ASSERT_TRUE(decision.has_value());
  EXPECT_TRUE(decision->success);
  EXPECT_STREQ(decision->outcome, "success");
}

TEST(Phase5bHarnessLogic, ExactRawBytesAreHexAndJsonEscapedWithoutLoss)
{
  const std::string bytes{std::string("\x01", 1) + "F\r"};
  EXPECT_EQ(driver::bytesToHex(bytes), "01460d");
  EXPECT_EQ(driver::jsonEscapeExactBytes(bytes), "\\u0001F\\r");
}

TEST(Phase5bHarnessLogic, DiagnosticResultJsonPreservesExactPartialBytes)
{
  const auto stamp = std::chrono::steady_clock::now();
  driver::DiagnosticResultEvent event;
  event.query = driver::DiagnosticQueryKind::fault_flags;
  event.status = driver::DiagnosticTransportStatus::timeout;
  event.framing_state = driver::SerialFramingState::unresolved;
  event.correlation_id = 9;
  event.connection_generation = 3;
  event.started_at = stamp;
  event.write_accepted_at = stamp;
  event.first_byte_at = stamp;
  event.last_byte_at = stamp;
  event.timeout_at = stamp;
  event.completed_at = stamp;
  event.delimiter_observed = false;
  event.raw_bytes = "FF";
  event.response.clear();
  event.reason = "diagnostic query timed out with a partial response";

  const auto json = driver::phase5bDiagnosticResultJsonLine(event);
  EXPECT_NE(json.find("\"type\":\"diagnostic_result\""), std::string::npos);
  EXPECT_NE(json.find("\"query\":\"FF\""), std::string::npos);
  EXPECT_NE(json.find("\"status\":\"timeout\""), std::string::npos);
  EXPECT_NE(json.find("\"framing_state\":\"unresolved\""), std::string::npos);
  EXPECT_NE(json.find("\"byte_count\":2"), std::string::npos);
  EXPECT_NE(json.find("\"raw\":\"FF\""), std::string::npos);
  EXPECT_NE(json.find("\"hex\":\"4646\""), std::string::npos);
  EXPECT_NE(json.find("\"delimiter_observed\":false"), std::string::npos);
}

TEST(Phase5bHarnessLogic, CommandTransactionJsonPreservesExactAckBytes)
{
  driver::CommandTransactionResult result;
  result.status = driver::CommandTransportStatus::success;
  result.raw_bytes = "+\r+\r+\r+\r";
  result.reason.clear();
  result.expected_acknowledgements = 4;
  result.received_acknowledgements = 4;
  result.write_fully_accepted = true;
  result.partial_line = false;
  result.started_at = std::chrono::steady_clock::now();
  result.write_accepted_at = result.started_at;
  result.completed_at = result.started_at;

  const std::vector<std::string> commands{
    "!G 1 0\r", "!G 2 0\r", "!S 1 0\r", "!S 2 0\r"};
  const auto json = driver::phase5bCommandTransactionJsonLine(commands, result);

  EXPECT_NE(json.find("\"type\":\"command_transaction\""), std::string::npos);
  EXPECT_NE(json.find("\"status\":\"success\""), std::string::npos);
  EXPECT_NE(json.find("\"expected_ack_count\":4"), std::string::npos);
  EXPECT_NE(json.find("\"received_ack_count\":4"), std::string::npos);
  EXPECT_NE(json.find("\"write_fully_accepted\":true"), std::string::npos);
  EXPECT_NE(json.find("\"quiet_verified\":true"), std::string::npos);
  EXPECT_NE(json.find("\"transmitted_byte_count\":28"), std::string::npos);
  EXPECT_NE(
    json.find("\"transmitted_hex\":\"2147203120300d2147203220300d2153203120300d2153203220300d\""),
    std::string::npos);
  EXPECT_NE(json.find("\"ack_byte_count\":8"), std::string::npos);
  EXPECT_NE(json.find("\"ack_raw\":\"+\\r+\\r+\\r+\\r\""), std::string::npos);
  EXPECT_NE(json.find("\"ack_hex\":\"2b0d2b0d2b0d2b0d\""), std::string::npos);
}

TEST(Phase5bHarnessLogic, StartupDrainJsonPreservesRawBytesAndHex)
{
  const auto stamp = std::chrono::steady_clock::now();
  driver::StartupDrainResult result;
  result.synchronized = true;
  result.raw_bytes = std::string("\0Starting ...\r", 14);
  result.delimiter_observed = true;
  result.started_at = stamp;
  result.last_byte_at = stamp;
  result.completed_at = stamp;

  const auto json = driver::phase5bStartupDrainJsonLine(result);

  EXPECT_NE(json.find("\"type\":\"startup_drain\""), std::string::npos);
  EXPECT_NE(json.find("\"synchronized\":true"), std::string::npos);
  EXPECT_NE(json.find("\"raw\":\"\\u0000Starting ...\\r\""), std::string::npos);
  EXPECT_NE(json.find("\"hex\":\"005374617274696e67202e2e2e0d\""), std::string::npos);
  EXPECT_NE(json.find("\"delimiter_observed\":true"), std::string::npos);
}

TEST(Phase5bHarnessLogic, QueryTraceJsonPreservesStartupFidBytesAndClassification)
{
  const auto stamp = std::chrono::steady_clock::now();
  driver::QueryTraceEvent event;
  event.command = "?FID\r";
  event.expected_prefix = "FID=";
  event.success = true;
  event.classification = driver::QueryLineClassification::expected_reply;
  event.raw_bytes = "FID=Roboteq v1.8d SBL2XXX 1/8/2018\r";
  event.response = "FID=Roboteq v1.8d SBL2XXX 1/8/2018";
  event.delimiter_observed = true;
  event.started_at = stamp;
  event.write_started_at = stamp;
  event.write_accepted_at = stamp;
  event.first_byte_at = stamp;
  event.last_byte_at = stamp;
  event.completed_at = stamp;

  const auto json = driver::phase5bQueryTraceJsonLine(event);

  EXPECT_NE(json.find("\"type\":\"query_trace\""), std::string::npos);
  EXPECT_NE(json.find("\"command\":\"?FID\\r\""), std::string::npos);
  EXPECT_NE(json.find("\"command_hex\":\"3f4649440d\""), std::string::npos);
  EXPECT_NE(json.find("\"success\":true"), std::string::npos);
  EXPECT_NE(json.find("\"classification\":\"expected_reply\""), std::string::npos);
  EXPECT_NE(
    json.find("\"raw\":\"FID=Roboteq v1.8d SBL2XXX 1/8/2018\\r\""),
    std::string::npos);
  EXPECT_NE(
    json.find("\"hex\":\"4649443d526f626f7465712076312e38642053424c3258585820312f382f323031380d\""),
    std::string::npos);
}

TEST(Phase5bHarnessLogic, AttemptResultJsonPreservesIncompleteDiagnosticBytes)
{
  driver::Phase5bDiagnosticEvidence diagnostic;
  diagnostic.diagnostic_correlation_id = 42;
  diagnostic.diagnostic_connection_generation = 7;
  diagnostic.diagnostic_started_ns = "100";
  diagnostic.diagnostic_write_accepted_ns = "200";
  diagnostic.diagnostic_first_byte_ns = "300";
  diagnostic.diagnostic_last_byte_ns = "400";
  diagnostic.diagnostic_timeout_ns = "500";
  diagnostic.diagnostic_delimiter_observed = false;
  diagnostic.diagnostic_byte_count = "2";
  diagnostic.diagnostic_raw = "FF";
  diagnostic.diagnostic_raw_hex = "4646";
  diagnostic.diagnostic_failure_reason = "diagnostic query timed out with a partial response";

  const auto line = driver::phase5bAttemptResultJsonLine(
    1, "preselection", "partial_unresolved", false,
    "waiting_for_fresh_command", "unresolved", diagnostic);

  EXPECT_NE(line.find("\"diagnostic_raw\":\"FF\""), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_raw_hex\":\"4646\""), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_byte_count\":2"), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_delimiter_observed\":false"), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_first_byte_ns\":300"), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_last_byte_ns\":400"), std::string::npos);
  EXPECT_NE(line.find("\"diagnostic_timeout_ns\":500"), std::string::npos);
  EXPECT_NE(
    line.find(
      "\"diagnostic_failure_reason\":\"diagnostic query timed out with a partial response\""),
    std::string::npos);
}
