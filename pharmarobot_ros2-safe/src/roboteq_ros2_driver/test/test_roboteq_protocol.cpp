#include "roboteq_ros2_driver/roboteq_protocol.hpp"

#include <climits>
#include <string>

#include <gtest/gtest.h>

namespace protocol = roboteq_ros2_driver::protocol;

TEST(RoboteqProtocol, ParsesFirmwareId)
{
  const auto parsed = protocol::parse_firmware_id("FID=Roboteq v2.0");

  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(*parsed, "Roboteq v2.0");
}

TEST(RoboteqProtocol, ParsesVoltageFields)
{
  const auto parsed = protocol::parse_voltage_fields("V=120:50:5000");

  ASSERT_TRUE(parsed.has_value());
  ASSERT_EQ(parsed->size(), 3u);
  EXPECT_EQ((*parsed)[0], 120);
  EXPECT_EQ((*parsed)[1], 50);
  EXPECT_EQ((*parsed)[2], 5000);
}

TEST(RoboteqProtocol, ParsesEncoderCounts)
{
  const auto parsed = protocol::parse_encoder_counts("CR=1234:-5678");

  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, 1234);
  EXPECT_EQ(parsed->second, -5678);

  const auto negative_zero = protocol::parse_encoder_counts("CR=-1:0");
  ASSERT_TRUE(negative_zero.has_value());
  EXPECT_EQ(negative_zero->first, -1);
  EXPECT_EQ(negative_zero->second, 0);
}

TEST(RoboteqProtocol, ParsesConfigReadback)
{
  const auto motor_mode = protocol::parse_config_readback("MMOD=1", "MMOD");

  ASSERT_TRUE(motor_mode.has_value());
  EXPECT_EQ(*motor_mode, 1);

  const auto encoder_ppr = protocol::parse_config_readback("EPPR=-1024\r\n", "EPPR");
  ASSERT_TRUE(encoder_ppr.has_value());
  EXPECT_EQ(*encoder_ppr, -1024);
}

TEST(RoboteqProtocol, AcceptsLineTerminatedResponses)
{
  EXPECT_EQ(protocol::parse_firmware_id("FID=firmware\r").value(), "firmware");
  EXPECT_TRUE(protocol::parse_voltage_fields("V=1:2:3\n").has_value());
  EXPECT_TRUE(protocol::parse_encoder_counts("CR=4:5\r\n").has_value());
  EXPECT_TRUE(protocol::parse_config_readback("MXRPM=100\r", "MXRPM").has_value());
}

TEST(RoboteqProtocol, RejectsMalformedResponses)
{
  EXPECT_FALSE(protocol::parse_firmware_id("").has_value());
  EXPECT_FALSE(protocol::parse_firmware_id("FID=").has_value());
  EXPECT_FALSE(protocol::parse_firmware_id("ID=firmware").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=123").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=123:").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=:456").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("C=123:456").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=1", "").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MXRPM=100", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("~MMOD 1", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=1:1", "MMOD").has_value());
}

TEST(RoboteqProtocol, RejectsInvalidNumericFields)
{
  EXPECT_FALSE(protocol::parse_voltage_fields("V=abc:2:3").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=1:2x:3").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=1::3").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=1:2").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=1:2:3:4").has_value());
  EXPECT_FALSE(protocol::parse_voltage_fields("V=999999999999999999999:2:3").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=left:right").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=12x:34").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=12:34y").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=1.2:3").has_value());
  EXPECT_FALSE(protocol::parse_encoder_counts("CR=999999999999999999999:34").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=open", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=1x", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=1.0", "MMOD").has_value());
  EXPECT_FALSE(protocol::parse_config_readback("MMOD=999999999999999999999", "MMOD").has_value());
}

TEST(RoboteqProtocol, ParsesIntegerLimits)
{
  const auto parsed = protocol::parse_encoder_counts(
    "CR=" + std::to_string(INT_MAX) + ":" + std::to_string(INT_MIN));

  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, INT_MAX);
  EXPECT_EQ(parsed->second, INT_MIN);
}
