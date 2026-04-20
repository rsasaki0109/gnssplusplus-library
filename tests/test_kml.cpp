#include <gtest/gtest.h>
#include <libgnss++/kml/kml.hpp>
#include <cmath>
#include <fstream>
#include <cstdio>

using namespace libgnss::kml;

// ============================================================
// Utils tests
// ============================================================

TEST(UtilsTest, Col2Hex) {
    // Red with full alpha -> FF0000FF
    auto hex = col2hex(Color(1, 0, 0), 1.0);
    EXPECT_EQ(hex, "FF0000FF");

    // Blue with full alpha -> FFFF0000
    hex = col2hex(Color(0, 0, 1), 1.0);
    EXPECT_EQ(hex, "FFFF0000");

    // White with half alpha -> 80FFFFFF
    hex = col2hex(Color(1, 1, 1), 0.5);
    EXPECT_EQ(hex, "80FFFFFF");
}

TEST(UtilsTest, WrapTo720) {
    EXPECT_DOUBLE_EQ(wrapTo720(0.0), 0.0);
    EXPECT_DOUBLE_EQ(wrapTo720(360.0), -360.0);  // mod(720,720)-360 = -360
    EXPECT_DOUBLE_EQ(wrapTo720(-360.0), -360.0);
    EXPECT_DOUBLE_EQ(wrapTo720(720.0), 0.0);
    EXPECT_DOUBLE_EQ(wrapTo720(180.0), 180.0);
}

TEST(UtilsTest, Unwrap360) {
    std::vector<double> input = {350, 355, 0, 5, 10};
    auto result = unwrap360(input);

    // After unwrapping, the sequence should be monotonically increasing
    for (size_t i = 1; i < result.size(); ++i) {
        EXPECT_GT(result[i], result[i - 1]);
    }
    // 0 should become 360, 5->365, 10->370
    EXPECT_NEAR(result[2], 360.0, 0.01);
}

TEST(UtilsTest, Enu2Llh) {
    // Zero offset should return reference point
    auto loc = enu2llh(0, 0, 0, 35.0, 139.0, 50.0);
    EXPECT_NEAR(loc.lat, 35.0, 1e-8);
    EXPECT_NEAR(loc.lon, 139.0, 1e-8);
    EXPECT_NEAR(loc.alt, 50.0, 1e-3);

    // North offset should increase latitude
    auto loc_n = enu2llh(0, 100, 0, 35.0, 139.0, 0.0);
    EXPECT_GT(loc_n.lat, 35.0);
    EXPECT_NEAR(loc_n.lon, 139.0, 1e-8);

    // East offset should increase longitude
    auto loc_e = enu2llh(100, 0, 0, 35.0, 139.0, 0.0);
    EXPECT_NEAR(loc_e.lat, 35.0, 1e-8);
    EXPECT_GT(loc_e.lon, 139.0);
}

TEST(UtilsTest, CalcCubeVertices) {
    Location center(35.0, 139.0, 10.0);
    auto cv = calcCubeVertices(center, 1.0);

    // All vertices should be near center
    for (const auto& v : cv.vertices) {
        EXPECT_NEAR(v.lat, 35.0, 0.001);
        EXPECT_NEAR(v.lon, 139.0, 0.001);
    }

    // 5 faces
    EXPECT_EQ(cv.faces.size(), 5);
}

// ============================================================
// Elements tests
// ============================================================

TEST(ElementsTest, CreateLine) {
    std::vector<Location> locs = {{35.0, 139.0, 0}, {35.001, 139.001, 0}};
    auto kml = createLine("TestLine", locs, 3.0, Color(1, 0, 0));

    EXPECT_NE(kml.find("<Placemark>"), std::string::npos);
    EXPECT_NE(kml.find("TestLine"), std::string::npos);
    EXPECT_NE(kml.find("<LineString>"), std::string::npos);
}

TEST(ElementsTest, CreateModel) {
    auto kml = createModel("Vehicle", "car.dae",
                           Location(35.0, 139.0, 0),
                           Orientation(90, 0, 0),
                           1.0, AltitudeMode::Absolute);

    EXPECT_NE(kml.find("Vehicle"), std::string::npos);
    EXPECT_NE(kml.find("car.dae"), std::string::npos);
    EXPECT_NE(kml.find("VehicleLoc"), std::string::npos);
    EXPECT_NE(kml.find("VehicleOri"), std::string::npos);
}

TEST(ElementsTest, CreatePoints) {
    std::vector<Location> locs = {{35.0, 139.0, 0}, {35.001, 139.001, 0}};
    auto kml = createPoints({"P1", "P2"}, locs, 1.0,
                            {Color(1, 0, 0), Color(0, 0, 1)});

    EXPECT_NE(kml.find("PSP1"), std::string::npos);
    EXPECT_NE(kml.find("PSP2"), std::string::npos);
    EXPECT_NE(kml.find("Points"), std::string::npos);  // folder
}

TEST(ElementsTest, CreateCubes) {
    std::vector<Location> locs = {{35.0, 139.0, 1.0}};
    auto kml = createCubes({"C1"}, locs, 0.6,
                           {Color(1, 0, 0)}, 0.8,
                           AltitudeMode::RelativeToGround);

    EXPECT_NE(kml.find("CSC1"), std::string::npos);
    EXPECT_NE(kml.find("CPM1-C1"), std::string::npos);
    EXPECT_NE(kml.find("<Polygon>"), std::string::npos);
}

TEST(ElementsTest, CreateScreenOverlay) {
    auto kml = createScreenOverlay("legend", "legend.png");
    EXPECT_NE(kml.find("SOlegend"), std::string::npos);
    EXPECT_NE(kml.find("legend.png"), std::string::npos);
}

// ============================================================
// Camera tests
// ============================================================

TEST(CameraTest, LookAt) {
    auto kml = createLookAt(Location(35.0, 139.0, 0),
                            LookAtParams(0, 50, 90));

    EXPECT_NE(kml.find("<LookAt>"), std::string::npos);
    EXPECT_NE(kml.find("<range>"), std::string::npos);
}

TEST(CameraTest, Camera) {
    auto kml = createCamera(Location(35.0, 139.0, 100),
                            CameraParams(0, 35, 0),
                            AltitudeMode::Absolute);

    EXPECT_NE(kml.find("<Camera>"), std::string::npos);
    EXPECT_NE(kml.find("<roll>"), std::string::npos);
}

// ============================================================
// Tour/Wrapper tests
// ============================================================

TEST(TourTest, WrapTour) {
    auto kml = wrapTour("<gx:Wait/>", "TestTour");
    EXPECT_NE(kml.find("<gx:Tour>"), std::string::npos);
    EXPECT_NE(kml.find("TestTour"), std::string::npos);
    EXPECT_NE(kml.find("<gx:Playlist>"), std::string::npos);
}

TEST(TourTest, WrapFlyTo) {
    auto kml = wrapFlyTo("<LookAt/>", 1.0, FlyToMode::Smooth);
    EXPECT_NE(kml.find("<gx:FlyTo>"), std::string::npos);
    EXPECT_NE(kml.find("smooth"), std::string::npos);
}

TEST(TourTest, WrapFolder) {
    auto kml = wrapFolder("<Placemark/>", "TestFolder");
    EXPECT_NE(kml.find("<Folder>"), std::string::npos);
    EXPECT_NE(kml.find("TestFolder"), std::string::npos);
}

TEST(TourTest, Wait) {
    auto kml = createWait(2.5);
    EXPECT_NE(kml.find("<gx:Wait>"), std::string::npos);
    EXPECT_NE(kml.find("2.500"), std::string::npos);
}

// ============================================================
// Animation tests
// ============================================================

TEST(AnimationTest, AnimatedUpdateModel) {
    auto kml = animatedUpdateModel("Vehicle", 0.2,
                                   Location(35.001, 139.001, 0),
                                   Orientation(30, 0, 0));

    EXPECT_NE(kml.find("<gx:AnimatedUpdate>"), std::string::npos);
    EXPECT_NE(kml.find("VehicleLoc"), std::string::npos);
    EXPECT_NE(kml.find("VehicleOri"), std::string::npos);
}

TEST(AnimationTest, AnimatedUpdateModelNoOrientation) {
    auto kml = animatedUpdateModel("Vehicle", 0.2,
                                   Location(35.001, 139.001, 0));

    EXPECT_NE(kml.find("VehicleLoc"), std::string::npos);
    EXPECT_EQ(kml.find("VehicleOri"), std::string::npos);  // no orientation
}

TEST(AnimationTest, AnimatedUpdatePoints) {
    auto kml = animatedUpdatePoints({"P1", "P2"}, 0.0,
                                    {Location(35.001, 139.0, 0), Location(35.001, 139.001, 0)},
                                    0.5,
                                    {Color(0, 1, 0), Color(0, 1, 1)},
                                    0.8);

    EXPECT_NE(kml.find("PP1"), std::string::npos);
    EXPECT_NE(kml.find("PISP2"), std::string::npos);
}

TEST(AnimationTest, AnimatedUpdateCubes) {
    auto kml = animatedUpdateCubes({"C1"}, 0.0,
                                   {Location(35.001, 139.0, 2.0)},
                                   0.6,
                                   {Color(0, 1, 0)}, 0.8, 0.5);

    EXPECT_NE(kml.find("CPM1-C1"), std::string::npos);
    EXPECT_NE(kml.find("CPL1-C1"), std::string::npos);
}

// ============================================================
// Writer tests
// ============================================================

TEST(WriterTest, WriteKml) {
    std::string content = createLookAt(Location(35.0, 139.0, 0),
                                       LookAtParams(0, 50, 90));
    bool ok = writeKml("/tmp/test_ge_drive_vis.kml", content, "TestDoc");
    EXPECT_TRUE(ok);

    // Verify file content
    std::ifstream ifs("/tmp/test_ge_drive_vis.kml");
    std::string file_content((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
    EXPECT_NE(file_content.find("<?xml version"), std::string::npos);
    EXPECT_NE(file_content.find("TestDoc"), std::string::npos);
    EXPECT_NE(file_content.find("<LookAt>"), std::string::npos);

    std::remove("/tmp/test_ge_drive_vis.kml");
}
