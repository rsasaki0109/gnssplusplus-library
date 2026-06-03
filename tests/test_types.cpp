#include <gtest/gtest.h>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <filesystem>
#include <fstream>

using namespace libgnss;

class TypesTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TypesTest, GNSSTimeBasicOperations) {
    GNSSTime time1(2000, 345600.0);  // GPS week 2000, TOW 345600
    GNSSTime time2(2000, 345700.0);  // 100 seconds later
    
    EXPECT_EQ(time1.week, 2000);
    EXPECT_DOUBLE_EQ(time1.tow, 345600.0);
    
    // Test time difference
    double diff = time2 - time1;
    EXPECT_DOUBLE_EQ(diff, 100.0);
    
    // Test comparison operators
    EXPECT_TRUE(time1 < time2);
    EXPECT_FALSE(time1 == time2);
}

TEST_F(TypesTest, SatelliteIdOperations) {
    SatelliteId gps_sat(GNSSSystem::GPS, 1);
    SatelliteId glo_sat(GNSSSystem::GLONASS, 1);
    SatelliteId gps_sat2(GNSSSystem::GPS, 2);
    
    EXPECT_EQ(gps_sat.system, GNSSSystem::GPS);
    EXPECT_EQ(gps_sat.prn, 1);
    
    // Test comparison operators
    EXPECT_TRUE(gps_sat < glo_sat);  // GPS < GLONASS
    EXPECT_TRUE(gps_sat < gps_sat2); // PRN 1 < PRN 2
    EXPECT_FALSE(gps_sat == glo_sat);
    
    // Test string conversion
    std::string sat_str = gps_sat.toString();
    EXPECT_FALSE(sat_str.empty());
}

TEST_F(TypesTest, CoordinateConversions) {
    // Test ECEF coordinates
    ECEFCoord ecef(4000000.0, 3000000.0, 2000000.0);
    Vector3d vec = ecef.toVector();
    
    EXPECT_DOUBLE_EQ(vec(0), 4000000.0);
    EXPECT_DOUBLE_EQ(vec(1), 3000000.0);
    EXPECT_DOUBLE_EQ(vec(2), 2000000.0);
    
    ECEFCoord ecef2;
    ecef2.fromVector(vec);
    EXPECT_DOUBLE_EQ(ecef2.x, ecef.x);
    EXPECT_DOUBLE_EQ(ecef2.y, ecef.y);
    EXPECT_DOUBLE_EQ(ecef2.z, ecef.z);
}

TEST_F(TypesTest, GeodeticCoordinates) {
    GeodeticCoord geo(0.7854, 1.5708, 100.0);  // 45°, 90°, 100m
    
    EXPECT_DOUBLE_EQ(geo.latitude, 0.7854);
    EXPECT_DOUBLE_EQ(geo.longitude, 1.5708);
    EXPECT_DOUBLE_EQ(geo.height, 100.0);
}

TEST_F(TypesTest, Constants) {
    EXPECT_DOUBLE_EQ(constants::SPEED_OF_LIGHT, 299792458.0);
    EXPECT_GT(constants::GPS_L1_FREQ, 1.5e9);
    EXPECT_GT(constants::WGS84_A, 6.3e6);
    EXPECT_GT(constants::WGS84_F, 0.0);
    EXPECT_LT(constants::WGS84_F, 1.0);
}

TEST_F(TypesTest, SolutionFileRoundTripsFgoQualityColumns) {
    Solution solution;
    PositionSolution sol;
    sol.time = GNSSTime(2300, 12345.0);
    sol.status = SolutionStatus::FIXED;
    sol.position_ecef = Vector3d(1.0, 2.0, 3.0);
    sol.position_geodetic = GeodeticCoord(0.1, 0.2, 3.5);
    sol.num_satellites = 12;
    sol.pdop = 1.25;
    sol.ratio = 4.5;
    sol.num_fixed_ambiguities = 9;
    sol.iterations = 7;
    solution.addSolution(sol);

    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_roundtrip_test.pos";
    std::filesystem::remove(path);

    ASSERT_TRUE(solution.writeToFile(path.string()));

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 1u);
    const PositionSolution& loaded_sol = loaded.solutions.front();
    EXPECT_DOUBLE_EQ(loaded_sol.ratio, sol.ratio);
    EXPECT_EQ(loaded_sol.num_fixed_ambiguities, sol.num_fixed_ambiguities);
    EXPECT_EQ(loaded_sol.iterations, sol.iterations);
}

TEST_F(TypesTest, SolutionLoaderMapsNamedOptionalColumns) {
    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_named_optional_columns_test.pos";
    std::filesystem::remove(path);

    {
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open());
        file << "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
                "Status NumSat PDOP Ratio Baseline(m) RTKIter RTKObs RTKPhaseObs "
                "RTKCodeObs RTKOutliers RTKPrefitRMS(m) RTKPrefitMax(m) "
                "RTKPostSuppressRMS(m) RTKPostSuppressMax(m) RTKUpdateNIS "
                "RTKUpdateNISPerObs RTKUpdateNISRejected\n";
        file << "2300 12346.000 1 2 3 5 6 7 4 10 1.2 8.5 12.25 4 "
                "20 11 9 3 0.5 2.0 0.25 1.5 7.0 0.35 1\n";
    }

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 1u);
    const PositionSolution& sol = loaded.solutions.front();
    EXPECT_DOUBLE_EQ(sol.ratio, 8.5);
    EXPECT_DOUBLE_EQ(sol.baseline_length, 12.25);
    EXPECT_EQ(sol.iterations, 4);
    EXPECT_EQ(sol.num_fixed_ambiguities, 0);
    EXPECT_EQ(sol.rtk_update_observations, 20);
    EXPECT_EQ(sol.rtk_update_phase_observations, 11);
    EXPECT_EQ(sol.rtk_update_code_observations, 9);
    EXPECT_EQ(sol.rtk_update_suppressed_outliers, 3);
    EXPECT_DOUBLE_EQ(sol.rtk_update_prefit_residual_rms_m, 0.5);
    EXPECT_DOUBLE_EQ(sol.rtk_update_prefit_residual_max_m, 2.0);
    EXPECT_DOUBLE_EQ(sol.rtk_update_post_suppression_residual_rms_m, 0.25);
    EXPECT_DOUBLE_EQ(sol.rtk_update_post_suppression_residual_max_m, 1.5);
    EXPECT_DOUBLE_EQ(sol.rtk_update_normalized_innovation_squared, 7.0);
    EXPECT_DOUBLE_EQ(sol.rtk_update_normalized_innovation_squared_per_observation, 0.35);
    EXPECT_EQ(sol.rtk_update_rejected_by_innovation_gate, 1);
}
