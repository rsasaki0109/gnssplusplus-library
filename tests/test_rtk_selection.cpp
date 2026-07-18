#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_selection.hpp>

using namespace libgnss;

namespace {

rtk_selection::SatelliteSelectionData makeSatellite(GNSSSystem system,
                                                    int prn,
                                                    double elevation,
                                                    bool has_l1,
                                                    bool has_l2,
                                                    double l1_wavelength = 0.19,
                                                    double l2_wavelength = 0.24,
                                                    bool n1_active = true,
                                                    bool n2_active = true,
                                                    int lock_l1 = 10,
                                                    int lock_l2 = 10) {
    rtk_selection::SatelliteSelectionData data;
    data.satellite = SatelliteId(system, prn);
    data.elevation = elevation;
    data.has_l1 = has_l1;
    data.has_l2 = has_l2;
    data.l1_wavelength = l1_wavelength;
    data.l2_wavelength = l2_wavelength;
    data.n1_active = n1_active;
    data.n2_active = n2_active;
    data.lock_count_l1 = lock_l1;
    data.lock_count_l2 = lock_l2;
    return data;
}

}  // namespace

TEST(RTKSelectionTest, PrefersDualFrequencyReferenceSatellite) {
    const std::vector<rtk_selection::SatelliteSelectionData> satellites = {
        makeSatellite(GNSSSystem::GPS, 1, 0.7, true, false),
        makeSatellite(GNSSSystem::GPS, 2, 0.6, true, true),
        makeSatellite(GNSSSystem::GPS, 3, 0.5, true, true),
    };

    SatelliteId ref_sat;
    ASSERT_TRUE(rtk_selection::selectSystemReferenceSatellite(
        satellites, GNSSSystem::GPS, 5, ref_sat));
    EXPECT_EQ(ref_sat.system, GNSSSystem::GPS);
    EXPECT_EQ(ref_sat.prn, 2);
}

TEST(RTKSelectionTest, FallsBackToL1ReferenceWhenNoDualFrequencyExists) {
    const std::vector<rtk_selection::SatelliteSelectionData> satellites = {
        makeSatellite(GNSSSystem::Galileo, 11, 0.4, true, false),
        makeSatellite(GNSSSystem::Galileo, 12, 0.8, true, false),
    };

    SatelliteId ref_sat;
    ASSERT_TRUE(rtk_selection::selectSystemReferenceSatellite(
        satellites, GNSSSystem::Galileo, 0, ref_sat));
    EXPECT_EQ(ref_sat.system, GNSSSystem::Galileo);
    EXPECT_EQ(ref_sat.prn, 12);
}

TEST(RTKSelectionTest, EnforcesMatchedWavelengthConstraintWhenRequested) {
    const std::vector<rtk_selection::SatelliteSelectionData> satellites = {
        makeSatellite(GNSSSystem::GPS, 1, 0.9, true, true, 0.19, 0.24),
        makeSatellite(GNSSSystem::GPS, 2, 0.8, true, true, 0.19, 0.24),
        makeSatellite(GNSSSystem::GPS, 3, 0.7, true, true, 0.22, 0.28),
    };

    const auto matched_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, true);
    ASSERT_EQ(matched_pairs.size(), 2U);
    EXPECT_EQ(matched_pairs[0].ref_sat.prn, 1);
    EXPECT_EQ(matched_pairs[0].sat.prn, 2);
    EXPECT_EQ(matched_pairs[0].freq, 0);
    EXPECT_EQ(matched_pairs[1].sat.prn, 2);
    EXPECT_EQ(matched_pairs[1].freq, 1);

    const auto unmatched_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, false);
    EXPECT_EQ(unmatched_pairs.size(), 4U);
}

// Phase 2a: forced_ref_sat overload lets a caller (RTKProcessor's CMC-aware
// hysteresis selector) override the plain max-elevation reference pick.
TEST(RTKSelectionTest, ForcedRefSatOverridesPlainElevationPick) {
    const std::vector<rtk_selection::SatelliteSelectionData> satellites = {
        makeSatellite(GNSSSystem::GPS, 1, 0.7, true, true),   // highest elevation
        makeSatellite(GNSSSystem::GPS, 2, 0.5, true, true),
        makeSatellite(GNSSSystem::GPS, 3, 0.3, true, true),
    };

    const SatelliteId forced_ref(GNSSSystem::GPS, 2);
    const auto pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, false, &forced_ref);
    ASSERT_FALSE(pairs.empty());
    for (const auto& pair : pairs) {
        EXPECT_EQ(pair.ref_sat.prn, 2);
        EXPECT_NE(pair.sat.prn, 2);
    }

    // 4-argument overload (no forced ref) is unaffected -- still picks the
    // highest-elevation satellite, matching pre-Phase-2a behavior exactly.
    const auto default_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, false);
    ASSERT_FALSE(default_pairs.empty());
    EXPECT_EQ(default_pairs[0].ref_sat.prn, 1);
}

TEST(RTKSelectionTest, ForcedRefSatFallsBackWhenIneligible) {
    const std::vector<rtk_selection::SatelliteSelectionData> satellites = {
        makeSatellite(GNSSSystem::GPS, 1, 0.7, true, true),
        makeSatellite(GNSSSystem::GPS, 2, 0.5, true, true),
    };

    // Forced satellite absent from the candidate list entirely (e.g. it
    // dropped out this epoch) -- must fall back to the plain selector
    // rather than returning an empty pair list.
    const SatelliteId missing_ref(GNSSSystem::GPS, 9);
    const auto pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, false, &missing_ref);
    ASSERT_FALSE(pairs.empty());
    EXPECT_EQ(pairs[0].ref_sat.prn, 1);

    // Forced satellite present but in the wrong system -- also falls back.
    const SatelliteId wrong_system_ref(GNSSSystem::Galileo, 1);
    const auto pairs2 = rtk_selection::buildDoubleDifferencePairsForSystem(
        satellites, GNSSSystem::GPS, 5, false, &wrong_system_ref);
    ASSERT_FALSE(pairs2.empty());
    EXPECT_EQ(pairs2[0].ref_sat.prn, 1);
}
