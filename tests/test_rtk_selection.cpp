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
