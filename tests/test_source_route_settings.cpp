#include <gtest/gtest.h>
#include <libgnss++/algorithms/source_route_settings.hpp>
using libgnss::smartphone_temporal::sourceRouteSettings;

TEST(SourceRouteSettings, ExplicitMetadataOverridesGeographicHeuristic) {
    const auto highway = sourceRouteSettings("Highway", "pixel5", 1);
    ASSERT_TRUE(highway.valid);
    EXPECT_DOUBLE_EQ(highway.pseudorange_huber, .2);
    EXPECT_DOUBLE_EQ(highway.doppler_huber, .8);
    EXPECT_DOUBLE_EQ(highway.tdcp_huber, .5);
    EXPECT_DOUBLE_EQ(highway.elevation_deg, 5);
    EXPECT_DOUBLE_EQ(highway.velocity_motion_sigma_m, .01);
    const auto street = sourceRouteSettings("Street", "sm-a205u", 0);
    EXPECT_DOUBLE_EQ(street.pseudorange_huber, .1);
    EXPECT_DOUBLE_EQ(street.doppler_huber, .4);
    EXPECT_DOUBLE_EQ(street.tdcp_huber, .2);
    EXPECT_DOUBLE_EQ(street.elevation_deg, 10);
    EXPECT_DOUBLE_EQ(street.velocity_motion_sigma_m, .05);
}

TEST(SourceRouteSettings, MixHasUrbanHuberButHighwayMotion) {
    const auto mixed = sourceRouteSettings("Mix", "pixel5", 1);
    ASSERT_TRUE(mixed.valid);
    EXPECT_DOUBLE_EQ(mixed.pseudorange_huber, .1);
    EXPECT_DOUBLE_EQ(mixed.doppler_huber, .4);
    EXPECT_DOUBLE_EQ(mixed.velocity_motion_sigma_m, .01);
}

TEST(SourceRouteSettings, SourcePhoneExceptionsAreIndependentOfType) {
    for (const auto type : {"Highway", "Street", "Mix"}) {
        EXPECT_DOUBLE_EQ(sourceRouteSettings(type, "pixel4xl", 1).doppler_huber, .2);
        EXPECT_DOUBLE_EQ(sourceRouteSettings(type, "mi8", 1).velocity_motion_sigma_m, .1);
        EXPECT_DOUBLE_EQ(sourceRouteSettings(type, "xiaomimi8", 1).velocity_motion_sigma_m, .1);
    }
}

TEST(SourceRouteSettings, InvalidOrIncompleteMetadataFailsClosed) {
    EXPECT_FALSE(sourceRouteSettings("", "pixel5", 1).valid);
    EXPECT_FALSE(sourceRouteSettings("street", "pixel5", 1).valid);
    EXPECT_FALSE(sourceRouteSettings("Highway", "", 1).valid);
    EXPECT_FALSE(sourceRouteSettings("Highway", "pixel5", -1).valid);
    EXPECT_FALSE(sourceRouteSettings("Highway", "pixel5", 2).valid);
}
