#include "robot_platform_pkg/hardware_interfaces/feetech_sts3215_bus.hpp"

#include <gtest/gtest.h>

using robot_platform_pkg::FeetechSts3215Bus;

TEST(FeetechSts3215Bus, SignedMagnitudeRoundTrip) {
  for (const int value : {-3000, -1, 0, 1, 3000, 32767}) {
    const auto encoded = FeetechSts3215Bus::encodeSignedMagnitude(
      static_cast<int16_t>(value));
    EXPECT_EQ(FeetechSts3215Bus::decodeSignedMagnitude(encoded), value);
  }
}

TEST(FeetechSts3215Bus, SignedMagnitudeUsesHighDirectionBit) {
  EXPECT_EQ(FeetechSts3215Bus::encodeSignedMagnitude(123), 123U);
  EXPECT_EQ(FeetechSts3215Bus::encodeSignedMagnitude(-123), 0x807BU);
  EXPECT_EQ(FeetechSts3215Bus::decodeSignedMagnitude(0x807B), -123);
}
