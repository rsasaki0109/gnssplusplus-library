#include <gtest/gtest.h>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/tdcp_frequency_pairing.hpp>

namespace {
using Factor=libgnss::FGOProcessor::TimeDifferencedCarrierFactor;
using libgnss::tdcp_frequency::pairAdmittedFactors;
Factor row(libgnss::SignalType signal, unsigned prn=1) {
    Factor f;
    f.previous_epoch_index=0; f.current_epoch_index=1; f.dt_s=1.;
    f.signal=signal; f.satellite={libgnss::GNSSSystem::GPS,static_cast<uint8_t>(prn)};
    return f;
}
TEST(TdcpFrequencyPairing, ExactPairAndUnmatchedPreservation) {
    const std::vector<Factor> rows{row(libgnss::SignalType::GPS_L5),
        row(libgnss::SignalType::GPS_L1CA,2),row(libgnss::SignalType::GPS_L1CA)};
    const auto result=pairAdmittedFactors(rows,{false,false});
    ASSERT_EQ(result.size(),1); EXPECT_EQ(result[0].l1_index,2);
    EXPECT_EQ(result[0].l5_index,0); EXPECT_EQ(rows.size(),3);
}
TEST(TdcpFrequencyPairing, NeverCrossesSatelliteOrInterval) {
    std::vector<Factor> rows{row(libgnss::SignalType::GPS_L1CA),row(libgnss::SignalType::GPS_L5,2)};
    EXPECT_TRUE(pairAdmittedFactors(rows,{false,false}).empty());
    rows[1].satellite=rows[0].satellite;
    rows[1].previous_epoch_index=1; rows[1].current_epoch_index=2;
    EXPECT_TRUE(pairAdmittedFactors(rows,{false,false,false}).empty());
}
TEST(TdcpFrequencyPairing, DuplicateAndDurationMismatchFail) {
    std::vector<Factor> rows{row(libgnss::SignalType::GPS_L1CA),row(libgnss::SignalType::GPS_L5)};
    rows[1].dt_s=.9;
    EXPECT_THROW(pairAdmittedFactors(rows,{false,false}),std::invalid_argument);
    rows[1]=rows[0];
    EXPECT_THROW(pairAdmittedFactors(rows,{false,false}),std::invalid_argument);
}
TEST(TdcpFrequencyPairing, ClockResetAndBridgedEpochRemainUnpaired) {
    std::vector<Factor> rows{row(libgnss::SignalType::GPS_L1CA),row(libgnss::SignalType::GPS_L5)};
    EXPECT_TRUE(pairAdmittedFactors(rows,{false,true}).empty());
    EXPECT_TRUE(pairAdmittedFactors(rows,{true,false}).empty());
    for (auto& f:rows) f.current_epoch_index=2;
    EXPECT_TRUE(pairAdmittedFactors(rows,{false,false,false}).empty());
}
TEST(TdcpFrequencyPairing, InvalidIndexFails) {
    std::vector<Factor> rows{row(libgnss::SignalType::GPS_L1CA)};
    EXPECT_THROW(pairAdmittedFactors(rows,{false}),std::invalid_argument);
}
TEST(TdcpFrequencyPairing, ChecksRealEpochDurationNotOnlyAgreementBetweenBands) {
    struct Epoch { double time; };
    const std::vector<Factor> rows{row(libgnss::SignalType::GPS_L1CA),row(libgnss::SignalType::GPS_L5)};
    using libgnss::tdcp_frequency::pairAdmittedFactorsAtEpochs;
    EXPECT_EQ(pairAdmittedFactorsAtEpochs(rows,{false,false},std::vector<Epoch>{{10.},{11.}}).size(),1);
    EXPECT_THROW(pairAdmittedFactorsAtEpochs(rows,{false,false},std::vector<Epoch>{{10.},{11.01}}),std::invalid_argument);
    EXPECT_THROW(pairAdmittedFactorsAtEpochs(rows,{false,false},std::vector<Epoch>{{10.},{9.}}),std::invalid_argument);
    EXPECT_THROW(pairAdmittedFactorsAtEpochs(rows,{false,false},std::vector<Epoch>{{10.},{NAN}}),std::invalid_argument);
    EXPECT_THROW(pairAdmittedFactorsAtEpochs(rows,{false,false},std::vector<Epoch>{{10.}}),std::invalid_argument);
}
}  // namespace
