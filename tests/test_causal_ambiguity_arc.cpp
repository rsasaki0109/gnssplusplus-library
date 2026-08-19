#include <gtest/gtest.h>

#include <libgnss++/algorithms/causal_ambiguity_arc.hpp>

namespace {

using libgnss::GNSSSystem;
using libgnss::SatelliteId;
using libgnss::causal_ambiguity_arc::Bank;
using libgnss::causal_ambiguity_arc::Config;

TEST(CausalAmbiguityArc, UsesOnlyPastAndCurrentSamples) {
    Config config;
    config.minimum_samples = 3;
    Bank bank(config);
    const SatelliteId satellite(GNSSSystem::GPS, 2);
    const SatelliteId reference(GNSSSystem::GPS, 1);
    EXPECT_DOUBLE_EQ(
        bank.update(satellite, reference, 15, 1.0, 10.2, false)
            .smoothed_value,
        10.2);
    EXPECT_DOUBLE_EQ(
        bank.update(satellite, reference, 15, 2.0, 9.8, false)
            .smoothed_value,
        10.0);
    const auto third =
        bank.update(satellite, reference, 15, 3.0, 10.1, false);
    EXPECT_TRUE(third.ready);
    EXPECT_NEAR(third.smoothed_value, 10.033333333333333, 1e-12);
    EXPECT_TRUE(std::isfinite(third.smoothed_value_variance));
    EXPECT_GT(third.smoothed_value_variance, 0.0);
}

TEST(CausalAmbiguityArc, ReportsVarianceOfTheCausalSmoothedMean) {
    Config config;
    config.minimum_samples = 3;
    Bank bank(config);
    const SatelliteId satellite(GNSSSystem::GPS, 2);
    bank.updateSignal(satellite, 15, 1.0, 9.8, false);
    bank.updateSignal(satellite, 15, 2.0, 10.0, false);
    const auto result =
        bank.updateSignal(satellite, 15, 3.0, 10.2, false);
    EXPECT_TRUE(result.ready);
    EXPECT_NEAR(result.smoothed_value, 10.0, 1e-12);
    EXPECT_NEAR(result.smoothed_value_variance, 2.0 / 225.0, 1e-12);
}

TEST(CausalAmbiguityArc, SlipResetsOnlyTheAddressedArc) {
    Config config;
    config.minimum_samples = 2;
    Bank bank(config);
    const SatelliteId reference(GNSSSystem::GPS, 1);
    const SatelliteId first(GNSSSystem::GPS, 2);
    const SatelliteId second(GNSSSystem::GPS, 3);
    bank.update(first, reference, 15, 1.0, 10.0, false);
    bank.update(second, reference, 15, 1.0, 20.2, false);
    const auto reset =
        bank.update(first, reference, 15, 2.0, 30.0, true);
    const auto unaffected =
        bank.update(second, reference, 15, 2.0, 19.8, false);
    EXPECT_TRUE(reset.reset);
    EXPECT_FALSE(reset.ready);
    EXPECT_DOUBLE_EQ(reset.smoothed_value, 30.0);
    EXPECT_TRUE(unaffected.ready);
    EXPECT_DOUBLE_EQ(unaffected.smoothed_value, 20.0);
}

TEST(CausalAmbiguityArc, ReferenceReturnCreatesNewGeneration) {
    Config config;
    config.minimum_samples = 2;
    Bank bank(config);
    const SatelliteId satellite(GNSSSystem::GPS, 3);
    const SatelliteId first_reference(GNSSSystem::GPS, 1);
    const SatelliteId second_reference(GNSSSystem::GPS, 2);
    const auto first = bank.update(
        satellite, first_reference, 15, 1.0, 10.0, false);
    const auto changed = bank.update(
        satellite, second_reference, 15, 2.0, 20.0, false);
    const auto returned = bank.update(
        satellite, first_reference, 15, 3.0, 30.0, false);
    EXPECT_LT(first.reference_generation, changed.reference_generation);
    EXPECT_LT(changed.reference_generation, returned.reference_generation);
    EXPECT_FALSE(returned.ready);
    EXPECT_DOUBLE_EQ(returned.smoothed_value, 30.0);
}

TEST(CausalAmbiguityArc, TracksReferenceGenerationPerSatellite) {
    Config config;
    config.minimum_samples = 2;
    Bank bank(config);
    const SatelliteId first(GNSSSystem::GPS, 3);
    const SatelliteId second(GNSSSystem::GPS, 4);
    const SatelliteId first_reference(GNSSSystem::GPS, 1);
    const SatelliteId second_reference(GNSSSystem::GPS, 2);
    bank.update(first, first_reference, 15, 1.0, 10.2, false);
    bank.update(second, second_reference, 15, 1.0, 20.2, false);
    const auto first_next =
        bank.update(first, first_reference, 15, 2.0, 9.8, false);
    const auto second_next =
        bank.update(second, second_reference, 15, 2.0, 19.8, false);
    EXPECT_TRUE(first_next.ready);
    EXPECT_TRUE(second_next.ready);
    EXPECT_EQ(
        first_next.reference_generation,
        second_next.reference_generation);
}

TEST(CausalAmbiguityArc, SingleDifferenceArcsSurviveReferenceChanges) {
    Config config;
    config.minimum_samples = 2;
    Bank bank(config);
    const SatelliteId satellite(GNSSSystem::GPS, 3);
    const SatelliteId first_reference(GNSSSystem::GPS, 1);
    const SatelliteId second_reference(GNSSSystem::GPS, 2);
    bank.updateSignal(satellite, 15, 1.0, 30.2, false);
    bank.updateSignal(first_reference, 15, 1.0, 10.2, false);
    bank.updateSignal(second_reference, 15, 1.0, 20.2, false);
    const auto satellite_next =
        bank.updateSignal(satellite, 15, 2.0, 29.8, false);
    const auto first_next =
        bank.updateSignal(first_reference, 15, 2.0, 9.8, false);
    const auto second_next =
        bank.updateSignal(second_reference, 15, 2.0, 19.8, false);
    EXPECT_TRUE(satellite_next.ready);
    EXPECT_TRUE(first_next.ready);
    EXPECT_TRUE(second_next.ready);
    EXPECT_DOUBLE_EQ(
        first_next.smoothed_value - satellite_next.smoothed_value,
        -20.0);
    EXPECT_DOUBLE_EQ(
        second_next.smoothed_value - satellite_next.smoothed_value,
        -10.0);
}

TEST(CausalAmbiguityArc, GapAndNonMonotonicTimeFailClosedByReset) {
    Config config;
    config.minimum_samples = 2;
    config.maximum_gap_s = 1.0;
    Bank bank(config);
    const SatelliteId satellite(GNSSSystem::GPS, 2);
    const SatelliteId reference(GNSSSystem::GPS, 1);
    bank.update(satellite, reference, 15, 2.0, 10.0, false);
    EXPECT_TRUE(
        bank.update(satellite, reference, 15, 4.0, 20.0, false)
            .reset);
    EXPECT_TRUE(
        bank.update(satellite, reference, 15, 3.0, 30.0, false)
            .reset);
}

}  // namespace
