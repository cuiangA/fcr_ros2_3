#include "robot_platform_pkg/protocol/dji_rs2_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace dji_rs2 = robot_platform_pkg::dji_rs2;

TEST(DjiRs2Protocol, ParsesRs2QueryResponsePayloadAfterStatusBytes)
{
  std::vector<uint8_t> frame(26, 0x00);
  frame[0] = dji_rs2::SOF;
  frame[3] = dji_rs2::RSP_TYPE;
  frame[12] = dji_rs2::CMD_SET;
  frame[13] = dji_rs2::CMD_ID_QUERY;
  frame[14] = 0x00;  // success
  frame[15] = 0x01;  // attitude angle
  frame[16] = 0xCB;  // yaw = 203 tenth-degrees
  frame[17] = 0x00;
  frame[18] = 0xF6;  // roll = -10 tenth-degrees
  frame[19] = 0xFF;
  frame[20] = 0x19;  // pitch = 25 tenth-degrees
  frame[21] = 0x00;

  int16_t yaw = 0;
  int16_t roll = 0;
  int16_t pitch = 0;
  ASSERT_TRUE(dji_rs2::parse_position_response(frame, yaw, roll, pitch));
  EXPECT_EQ(yaw, 203);
  EXPECT_EQ(roll, -10);
  EXPECT_EQ(pitch, 25);
}

TEST(DjiRs2Protocol, RejectsFailedQueryResponse)
{
  std::vector<uint8_t> frame(26, 0x00);
  frame[3] = dji_rs2::RSP_TYPE;
  frame[12] = dji_rs2::CMD_SET;
  frame[13] = dji_rs2::CMD_ID_QUERY;
  frame[14] = 0x01;  // failure
  frame[15] = 0x01;

  int16_t yaw = 0;
  int16_t roll = 0;
  int16_t pitch = 0;
  EXPECT_FALSE(dji_rs2::parse_position_response(frame, yaw, roll, pitch));
}
