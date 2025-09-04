#include <gtest/gtest.h>
#include <libgnss++/core/types.hpp>

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
