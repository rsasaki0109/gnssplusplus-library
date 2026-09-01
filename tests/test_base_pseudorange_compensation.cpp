#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <limits>

using namespace libgnss;
using namespace libgnss::base_pseudorange_compensation;

TEST(BasePseudorangeCompensationTest, CenteredMovingMeanShrinksAtEdges) {
    const auto result = centeredMovingMean({1.0, 2.0, 3.0, 4.0, 5.0}, 3U);
    ASSERT_EQ(result.size(), 5U);
    EXPECT_DOUBLE_EQ(result[0], 1.5);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
    EXPECT_DOUBLE_EQ(result[3], 4.0);
    EXPECT_DOUBLE_EQ(result[4], 4.5);
}

TEST(BasePseudorangeCompensationTest, MovingMeanOmitsNonfiniteSamples) {
    const auto result = centeredMovingMean(
        {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0}, 3U);
    ASSERT_EQ(result.size(), 3U);
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(BasePseudorangeCompensationTest, SourceSignSubtractsPositiveCorrection) {
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, 12.5), 1'999'987.5);
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, -12.5), 2'000'012.5);
}

TEST(BasePseudorangeCompensationTest, InvalidBuildFailsClosedWithoutPayloadIo) {
    Model model;
    ObservationSeries empty;
    NavigationData nav;
    Config config;
    config.base_position_ecef = Vector3d(1.0, 2.0, 3.0);
    config.expected_interval_s = 1.0;
    config.moving_mean_samples = 151U;
    EXPECT_FALSE(model.build(empty, nav, config));
    EXPECT_FALSE(model.diagnostics().built);
    EXPECT_FALSE(model.diagnostics().failure.empty());
}

TEST(BasePseudorangeCompensationTest, AdditionalBandsAreExplicitlyOptIn) {
    io::RINEXReader reader;
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(true);
    EXPECT_TRUE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(false);
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
}
