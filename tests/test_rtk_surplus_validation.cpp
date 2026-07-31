#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_surplus_validation.hpp>

#include <limits>

namespace libgnss::rtk_surplus_validation {
namespace {

TEST(RtkSurplusValidation, DisabledIsNoOp) {
    Config config;
    const auto outcome =
        evaluate({{GNSSSystem::GPS, 0.01}, {GNSSSystem::Galileo, 0.02}},
                 1.5, config);
    EXPECT_FALSE(outcome.evaluated);
    EXPECT_FALSE(outcome.passed);
}

TEST(RtkSurplusValidation, EitherIndependentEvidenceRouteCanAuthorizePromotion) {
    EXPECT_FALSE(promotionEvidencePassed(false, false));
    EXPECT_TRUE(promotionEvidencePassed(true, false));
    EXPECT_TRUE(promotionEvidencePassed(false, true));
    EXPECT_TRUE(promotionEvidencePassed(true, true));
}

TEST(RtkSurplusValidation, RequireAllPassesAndFailsClosed) {
    Config config;
    config.enabled = true;
    config.minimum_surplus_pairs = 2;

    auto outcome =
        evaluate({{GNSSSystem::GPS, 0.05}, {GNSSSystem::Galileo, 0.19}},
                 1.5, config);
    ASSERT_TRUE(outcome.evaluated);
    EXPECT_TRUE(outcome.passed);
    EXPECT_EQ(outcome.surplus_used, 2);
    EXPECT_DOUBLE_EQ(outcome.aperture_cycles, 0.20);

    outcome =
        evaluate({{GNSSSystem::GPS, 0.05}, {GNSSSystem::Galileo, 0.21}},
                 1.5, config);
    ASSERT_TRUE(outcome.evaluated);
    EXPECT_FALSE(outcome.passed);
}

TEST(RtkSurplusValidation, UsesConstellationFallbackWhenFullPoolIsTooSmall) {
    Config config;
    config.enabled = true;
    config.minimum_surplus_pairs = 2;
    const auto outcome =
        evaluate({{GNSSSystem::GPS, 0.02},
                  {GNSSSystem::QZSS, 0.03},
                  {GNSSSystem::GLONASS, 0.45}},
                 0.8, config);
    ASSERT_TRUE(outcome.evaluated);
    EXPECT_TRUE(outcome.passed);
    EXPECT_EQ(outcome.fallback_level, 1);
    EXPECT_EQ(outcome.surplus_used, 2);
}

TEST(RtkSurplusValidation, MajorityAggregationIsExplicit) {
    Config config;
    config.enabled = true;
    config.require_all = false;
    config.majority_fraction = 2.0 / 3.0;
    config.minimum_surplus_pairs = 3;
    const auto outcome =
        evaluate({{GNSSSystem::GPS, 0.02},
                  {GNSSSystem::QZSS, 0.03},
                  {GNSSSystem::Galileo, 0.40}},
                 1.2, config);
    ASSERT_TRUE(outcome.evaluated);
    EXPECT_TRUE(outcome.passed);
    EXPECT_EQ(outcome.passing_pairs, 2);
}

TEST(RtkSurplusValidation, WeakGeometryAndInsufficientPoolHaveNoVerdict) {
    Config config;
    config.enabled = true;
    config.minimum_surplus_pairs = 2;
    EXPECT_FALSE(evaluate({{GNSSSystem::GPS, 0.01}},
                          1.0, config).evaluated);
    EXPECT_FALSE(evaluate({{GNSSSystem::GPS, 0.01},
                           {GNSSSystem::QZSS, 0.01}},
                          std::numeric_limits<double>::infinity(),
                          config).evaluated);
}

}  // namespace
}  // namespace libgnss::rtk_surplus_validation
