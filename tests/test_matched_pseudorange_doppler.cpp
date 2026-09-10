#include <gtest/gtest.h>
#include <libgnss++/algorithms/matched_pseudorange_doppler.hpp>
#include <limits>

using namespace libgnss;

TEST(MatchedPseudorangeDoppler, IdentitySignAndMissingAreExplicit) {
    Observation row;
    row.satellite = SatelliteId(GNSSSystem::GPS, 1);
    row.signal = SignalType::GPS_L1CA;
    row.has_pseudorange = row.has_doppler = true;
    row.pseudorange = 20000000.;
    row.doppler = 0.;
    ObservationData previous, current;
    previous.observations.push_back(row);
    row.pseudorange -= 30.;
    current.observations.push_back(row);
    auto evaluate = [&](double dt = 1.) {
        return matchedPseudorangeDoppler(previous, current,
            row.satellite, row.signal, dt);
    };
    ASSERT_TRUE(evaluate());
    EXPECT_DOUBLE_EQ(*evaluate(), 30.);
    for (double dt : {0., -1., 2., std::numeric_limits<double>::quiet_NaN()})
        EXPECT_FALSE(evaluate(dt));
    current.observations.push_back(row);
    EXPECT_FALSE(evaluate()); // duplicate identity is ambiguous
    current.observations.pop_back();
    current.observations[0].signal = SignalType::GPS_L5;
    EXPECT_FALSE(evaluate()); // same satellite, wrong frequency
    current.observations[0] = row;
    current.observations[0].has_doppler = false;
    EXPECT_FALSE(evaluate());
    current.observations[0] = row;
    current.observations[0].pseudorange = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(evaluate());
    MatchedPseudorangeDopplerSummary summary;
    summary.observe(std::nullopt);
    summary.observe(0.);
    EXPECT_EQ(summary.missing, 1U);
    EXPECT_EQ(summary.values.zero, 1U);
}

TEST(MatchedPseudorangeDoppler, ConsistentRangeRateAndReverseJump) {
    Observation row;
    row.satellite = SatelliteId(GNSSSystem::GPS, 3);
    row.signal = SignalType::GPS_L1CA;
    row.has_pseudorange = row.has_doppler = true;
    row.pseudorange = 20000000.;
    row.doppler = -10. / signalWavelengthMeters(row);
    ObservationData a, b;
    a.observations.push_back(row);
    row.pseudorange += 10.;
    b.observations.push_back(row);
    auto result = matchedPseudorangeDoppler(a, b, row.satellite, row.signal, 1.);
    ASSERT_TRUE(result);
    EXPECT_NEAR(*result, 0., 1e-12);
    b.observations[0].pseudorange += 30.;
    result = matchedPseudorangeDoppler(a, b, row.satellite, row.signal, 1.);
    ASSERT_TRUE(result);
    EXPECT_NEAR(*result, -30., 1e-12);
    a.observations.push_back(a.observations.front());
    EXPECT_FALSE(matchedPseudorangeDoppler(a, b, row.satellite, row.signal, 1.));
}
