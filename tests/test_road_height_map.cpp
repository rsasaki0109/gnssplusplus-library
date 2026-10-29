#include <gtest/gtest.h>

#include <libgnss++/algorithms/road_height_map.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace libgnss {
namespace road_height {
namespace {

std::string writeTempCsv(const std::string& contents, const std::string& suffix) {
    const auto path =
        std::filesystem::temp_directory_path() / ("road_height_test_" + suffix + ".csv");
    std::ofstream out(path);
    out << contents;
    return path.string();
}

// Point `east_m` east of (35.66 deg N, 139.79 deg E) at ellipsoidal height `h`.
Eigen::Vector3d pointEast(double east_m, double h) {
    const double lat = 35.66 * M_PI / 180.0;
    const double lon = 139.79 * M_PI / 180.0;
    return geodetic2ecef(lat, lon, h) + enu2ecef(Eigen::Vector3d(east_m, 0.0, 0.0), lat, lon);
}

TEST(RoadHeightMapTest, LookupReturnsTheNearestPointWithinTheRadius) {
    const double dlon = 1.0 / (111320.0 * std::cos(35.66 * M_PI / 180.0));  // ~1 m east
    const std::string path = writeTempCsv(
        "lat_deg,lon_deg,height_m,extra\n"
        "35.66,139.79,38.0,x\n" +
            std::string("35.66,") + std::to_string(139.79 + 2.0 * dlon) + ",38.5,y\n",
        "nearest");
    const RoadHeightMap map = loadRoadHeightCsv(path, 1.5);
    ASSERT_EQ(map.points.size(), 2u);

    // The query height does not matter, only the horizontal position.
    const auto near_first = lookupRoadHeight(map, pointEast(0.6, 45.0), 1.5);
    ASSERT_TRUE(near_first.has_value());
    EXPECT_NEAR(*near_first, 38.0, 1e-9);
    const auto near_second = lookupRoadHeight(map, pointEast(1.6, 41.0), 1.5);
    ASSERT_TRUE(near_second.has_value());
    EXPECT_NEAR(*near_second, 38.5, 1e-9);
    EXPECT_FALSE(lookupRoadHeight(map, pointEast(4.0, 40.0), 1.5).has_value());
    EXPECT_FALSE(lookupRoadHeight(map, pointEast(0.6, 40.0), 0.5).has_value());
}

TEST(RoadHeightMapTest, LoaderRejectsMissingColumnsAndEmptyFiles) {
    EXPECT_THROW(loadRoadHeightCsv(writeTempCsv("lat_deg,lon_deg\n35.66,139.79\n", "cols")),
                 std::runtime_error);
    EXPECT_THROW(loadRoadHeightCsv(writeTempCsv("lat_deg,lon_deg,height_m\n", "empty")),
                 std::runtime_error);
    EXPECT_THROW(loadRoadHeightCsv("/nonexistent/road_height.csv"), std::runtime_error);
    EXPECT_FALSE(lookupRoadHeight(RoadHeightMap{}, pointEast(0.0, 40.0), 1.5).has_value());
}

}  // namespace
}  // namespace road_height
}  // namespace libgnss
