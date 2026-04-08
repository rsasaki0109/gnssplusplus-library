#include <gtest/gtest.h>

#include <libgnss++/core/solution.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace libgnss;

namespace {

std::filesystem::path uniqueTempPos(const std::string& stem) {
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() / (stem + "_" + std::to_string(tick) + ".pos");
}

}  // namespace

TEST(SolutionIo, WriteReadRoundTripPreservesEcefTimeStatus) {
    Solution original;
    PositionSolution s;
    s.time = GNSSTime(2068, 234016.0);
    s.position_ecef << -3957237.5, 3310369.5, 3737531.5;
    s.position_geodetic.latitude = 36.1 * M_PI / 180.0;
    s.position_geodetic.longitude = 140.08 * M_PI / 180.0;
    s.position_geodetic.height = 72.8;
    s.status = SolutionStatus::PPP_FLOAT;
    s.num_satellites = 17;
    s.pdop = 1.8;
    original.addSolution(s);

    const auto path = uniqueTempPos("gnsspp_sol_roundtrip");
    ASSERT_TRUE(original.writeToFile(path.string(), "pos"));

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    ASSERT_EQ(loaded.solutions.size(), 1u);

    const auto& round = loaded.solutions[0];
    EXPECT_EQ(round.time.week, 2068);
    EXPECT_DOUBLE_EQ(round.time.tow, 234016.0);
    EXPECT_NEAR(round.position_ecef(0), -3957237.5, 1e-6);
    EXPECT_NEAR(round.position_ecef(1), 3310369.5, 1e-6);
    EXPECT_NEAR(round.position_ecef(2), 3737531.5, 1e-6);
    EXPECT_EQ(round.status, SolutionStatus::PPP_FLOAT);
    EXPECT_EQ(round.num_satellites, 17);
    EXPECT_DOUBLE_EQ(round.pdop, 1.8);

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

TEST(SolutionIo, LoadFromFileIgnoresCommentLines) {
    const auto path = uniqueTempPos("gnsspp_sol_comments");
    {
        std::ofstream f(path);
        ASSERT_TRUE(f.is_open());
        f << "% LibGNSS++ Position Solution\n";
        f << "% comment\n";
        f << "2068 0.000000 1.0 2.0 3.0 0.0 0.0 0.0 5 4 9.9\n";
    }

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    ASSERT_EQ(loaded.solutions.size(), 1u);
    EXPECT_EQ(loaded.solutions[0].time.week, 2068);
    EXPECT_DOUBLE_EQ(loaded.solutions[0].position_ecef(0), 1.0);

    std::error_code ec;
    std::filesystem::remove(path, ec);
}
