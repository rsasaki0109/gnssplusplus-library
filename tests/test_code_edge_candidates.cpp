#include <gtest/gtest.h>
#include <libgnss++/algorithms/code_edge_candidates.hpp>

namespace {
std::vector<libgnss::ObservationData> impulse() {
    std::vector<libgnss::ObservationData> epochs(5);
    for (std::size_t i=0; i<5; ++i) {
        epochs[i].time.week=2200; epochs[i].time.tow=100.+i;
        libgnss::Observation row;
        row.satellite={libgnss::GNSSSystem::GPS,1};
        row.signal=libgnss::SignalType::GPS_L1CA;
        row.has_pseudorange=row.has_doppler=true;
        row.pseudorange=20000000.+(i==2 ? 100. : 0.); row.doppler=0.;
        epochs[i].observations.push_back(row);
    }
    return epochs;
}
TEST(CodeEdgeCandidates, ExactNeighborsAndMissingWitness) {
    auto epochs=impulse();
    auto result=libgnss::codeEdgeCandidates(epochs, {0,0,0,0,0});
    ASSERT_EQ(result.size(),2U);
    EXPECT_EQ(std::get<0>(*result.begin()),1U);
    EXPECT_EQ(std::get<0>(*result.rbegin()),3U);
    epochs[0].observations.clear();
    result=libgnss::codeEdgeCandidates(epochs, {0,0,0,0,0});
    ASSERT_EQ(result.size(),1U);
    EXPECT_EQ(std::get<0>(*result.begin()),3U);
}
TEST(CodeEdgeCandidates, ClockBreakAndDuplicateFailClosed) {
    auto epochs=impulse();
    EXPECT_THROW(libgnss::codeEdgeCandidates(epochs, {}),std::invalid_argument);
    EXPECT_TRUE(libgnss::codeEdgeCandidates(epochs,{0,0,1,1,1}).empty());
    epochs[2].observations.push_back(epochs[2].observations.front());
    EXPECT_TRUE(libgnss::codeEdgeCandidates(epochs,{0,0,0,0,0}).empty());
}
TEST(CodeEdgeCandidates, PersistentOffsetCannotBeCertifiedByTemporalSupport) {
    const auto clean_neighbors=impulse();
    auto biased_neighbors=clean_neighbors;
    for (auto& epoch:biased_neighbors)
        epoch.observations.front().pseudorange+=75.;
    // Identical edge differences despite biased neighbors: diagnostic
    // membership is not a clean-code certificate or permission to readmit.
    EXPECT_EQ(libgnss::codeEdgeCandidates(clean_neighbors,{0,0,0,0,0}),
              libgnss::codeEdgeCandidates(biased_neighbors,{0,0,0,0,0}));
}
TEST(CodeEdgeCandidates, OutsideClockBreakAndTimeGapRemoveOnlyUnsupportedSide) {
    auto epochs=impulse();
    auto result=libgnss::codeEdgeCandidates(epochs,{1,0,0,0,0});
    ASSERT_EQ(result.size(),1U);
    EXPECT_EQ(std::get<0>(*result.begin()),3U);
    epochs[0].time.tow-=1.;
    result=libgnss::codeEdgeCandidates(epochs,{0,0,0,0,0});
    ASSERT_EQ(result.size(),1U);
    EXPECT_EQ(std::get<0>(*result.begin()),3U);
}
TEST(CodeEdgeCandidates, DiagnosticLabelSurvivesEpochFilteringWithoutChangingMasks) {
    auto epochs=impulse();
    namespace up=libgnss::observable_upstream;
    std::vector<up::EpochMask> before, after;
    std::size_t p_before=0, l_before=0, p_after=0, l_after=0;
    up::applyAdjacentMasks(epochs,"pixel5",before,p_before,l_before);
    const auto candidates=libgnss::codeEdgeCandidates(epochs,{0,0,0,0,0});
    for (std::size_t i=0; i<epochs.size(); ++i) {
        epochs[i].raw_source_index=i;
        epochs[i].raw_utc_time_millis=100000+1000*i;
        for (auto& row:epochs[i].observations) {
            ASSERT_FALSE(row.native_code_edge_diagnostic_candidate);
            row.native_code_edge_diagnostic_candidate=
                candidates.count({i,row.satellite,row.signal}) != 0;
        }
    }
    up::applyAdjacentMasks(epochs,"pixel5",after,p_after,l_after);
    EXPECT_EQ(p_before,p_after); EXPECT_EQ(l_before,l_after);
    ASSERT_EQ(before.size(),after.size());
    for (std::size_t i=0; i<before.size(); ++i)
        EXPECT_EQ(before[i].pseudorange,after[i].pseudorange);
    const auto original_code=epochs[3].observations.front().pseudorange;
    std::vector<libgnss::ObservationData> retained;
    retained.push_back(epochs[3]);
    retained.push_back(std::move(epochs[1]));
    EXPECT_EQ(retained[0].raw_source_index,3U);
    EXPECT_EQ(retained[0].raw_utc_time_millis,103000);
    EXPECT_EQ(retained[1].raw_source_index,1U);
    EXPECT_TRUE(retained[0].observations.front().native_code_edge_diagnostic_candidate);
    EXPECT_TRUE(retained[1].observations.front().native_code_edge_diagnostic_candidate);
    EXPECT_DOUBLE_EQ(retained[0].observations.front().pseudorange,original_code);
}
}
