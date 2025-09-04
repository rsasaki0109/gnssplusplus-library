#include <gtest/gtest.h>
#include "../include/libgnss++/io/rtcm.hpp"

// Test fixture for RTCMProcessor
class RTCMProcessorTest : public ::testing::Test {
protected:
    void TearDown() override {
        processor.clear();
    }
    libgnss::io::RTCMProcessor processor;
};

// Test case for a valid RTCM 1005 message
TEST_F(RTCMProcessorTest, DecodeValidRTCM1005) {
    // Sample RTCM 1005 message (from a real stream, simplified)
    // This message sets the reference position to a known value.

    // A complete, valid RTCM 1005 message including header and CRC
    // Coords: X=-2709506.7000, Y=-4292295.6000, Z=3854247.3000
    // Valid RTCM 1005 message for X=-2709506.7000, Y=-4292295.6000, Z=3854247.3000
    const uint8_t valid_rtcm_1005[] = {
        0xD3, 0x00, 0x0F, 0x3E, 0xD4, 0x00, 0xFB, 0x9D, 0xC7, 0x9E, 0x82, 0x0F, 0x06, 0x84, 0xF8, 0xE0, 0xC3, 0x72, 0x4F, 0x0F, 0x4D
    };

    auto decoded_messages = processor.decode(valid_rtcm_1005, sizeof(valid_rtcm_1005));
    ASSERT_EQ(decoded_messages.size(), 1);

    libgnss::Vector3d position = processor.getReferencePosition();
    EXPECT_NEAR(position.x(), -2709506.7000, 1e-4);
    EXPECT_NEAR(position.y(), -4292295.6000, 1e-4);
    EXPECT_NEAR(position.z(), 3854247.3000, 1e-4);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
