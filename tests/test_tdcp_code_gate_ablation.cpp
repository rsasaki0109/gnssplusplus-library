#include <gtest/gtest.h>
#include <libgnss++/algorithms/tdcp_contract.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>

namespace {
std::vector<libgnss::ObservationData> pair(double phase_jump_m) {
    std::vector<libgnss::ObservationData> epochs(2);
    for (int i = 0; i < 2; ++i) {
        epochs[i].time = {2200, 100.0 + i};
        libgnss::Observation row;
        row.signal = libgnss::SignalType::GPS_L1CA;
        row.has_pseudorange = row.has_carrier_phase = row.has_doppler = true;
        row.pseudorange = 20000000.0 + i * 30.0;
        row.doppler = 0;
        row.carrier_phase = i * phase_jump_m / libgnss::signalWavelengthMeters(row);
        epochs[i].observations.push_back(row);
    }
    return epochs;
}
}

TEST(TdcpCodeGateAblation, NoisyCodeSmoothPhaseRetainsLdSupport) {
    const auto epochs = pair(0);
    std::vector<libgnss::observable_upstream::EpochMask> masks;
    std::size_t p = 0, l = 0;
    libgnss::observable_upstream::applyAdjacentMasks(epochs, "pixel5", masks, p, l);
    EXPECT_EQ(l, 0U);
    using namespace libgnss::tdcp_contract;
    EXPECT_EQ(evaluateAdjacentPair(1,false,false,0,30,1.5,true,true,10).reason,
              PairRejectReason::CodePhaseJump);
    EXPECT_TRUE(evaluateAdjacentPair(1,false,false,0,30,1.5,true,false,10).accepted());
}

TEST(TdcpCodeGateAblation, TruePhaseJumpStillRemovedByUpstreamLd) {
    const auto epochs = pair(5);
    std::vector<libgnss::observable_upstream::EpochMask> masks;
    std::size_t p = 0, l = 0;
    libgnss::observable_upstream::applyAdjacentMasks(epochs, "pixel5", masks, p, l);
    EXPECT_EQ(l, 2U);
    EXPECT_EQ(masks[0].carrier.size(), 1U);
    EXPECT_EQ(masks[1].carrier.size(), 1U);
}

TEST(TdcpCodeGateAblation, OtherTemporalRejectionsRemainEnabled) {
    using namespace libgnss::tdcp_contract;
    EXPECT_EQ(evaluateAdjacentPair(2,false,false,0,30,1.5,true,false,10).reason, PairRejectReason::Gap);
    EXPECT_EQ(evaluateAdjacentPair(1,true,false,0,30,1.5,true,false,10).reason, PairRejectReason::LossOfLock);
    EXPECT_EQ(evaluateAdjacentPair(1,false,false,0,30,1.5,true,false,10,false,true).reason,
              PairRejectReason::ClockDiscontinuity);
    EXPECT_EQ(evaluateAdjacentPair(1,false,false,NAN,30,1.5,true,false,10).reason,
              PairRejectReason::NonFiniteMeasurement);
}
