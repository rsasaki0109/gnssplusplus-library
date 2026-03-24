#include <gtest/gtest.h>
#include <libgnss++/io/rtcm.hpp>

using namespace libgnss;

class RTCMProcessorTest : public ::testing::Test {
protected:
    void TearDown() override {
        processor.clear();
    }

    io::RTCMProcessor processor;
};

TEST_F(RTCMProcessorTest, RejectsMessageWithInvalidCRC) {
    const uint8_t invalid_rtcm_1005[] = {
        0xD3, 0x00, 0x0F, 0x3E, 0xD4, 0x00, 0xFB, 0x9D, 0xC7, 0x9E,
        0x82, 0x0F, 0x06, 0x84, 0xF8, 0xE0, 0xC3, 0x72, 0x4F, 0x0F,
        0x4D
    };

    const auto decoded_messages =
        processor.decode(invalid_rtcm_1005, sizeof(invalid_rtcm_1005));

    EXPECT_TRUE(decoded_messages.empty());
    EXPECT_FALSE(processor.hasReferencePosition());
}

TEST(RTCMUtilsTest, ClassifiesMessageFamilies) {
    EXPECT_TRUE(io::rtcm_utils::isStationMessage(io::RTCMMessageType::RTCM_1005));
    EXPECT_TRUE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1074));
    EXPECT_TRUE(io::rtcm_utils::isEphemerisMessage(io::RTCMMessageType::RTCM_1019));
    EXPECT_FALSE(io::rtcm_utils::isObservationMessage(io::RTCMMessageType::RTCM_1005));
}
