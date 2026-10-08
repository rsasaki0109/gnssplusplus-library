#include <gtest/gtest.h>

#include <libgnss++/algorithms/barometer_height.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

using namespace libgnss;
using namespace libgnss::barometer;

namespace {

BaroHeightConfig testConfig() {
    BaroHeightConfig config;
    config.enabled = true;
    config.baro_sigma_m = 0.5;
    config.bias_walk_m_per_sqrt_s = 0.02;
    config.height_walk_m_per_sqrt_s = 1.0;
    return config;
}

}  // namespace

TEST(BarometerHeightTest, StandardAtmosphereMatchesIcaoReferencePoints) {
    EXPECT_NEAR(pressureToStandardAtmosphereHeightM(1013.25), 0.0, 1e-9);
    // ICAO: 898.76 hPa <-> 1000 m, 795.0 hPa <-> 2000 m.
    EXPECT_NEAR(pressureToStandardAtmosphereHeightM(898.746), 1000.0, 0.5);
    EXPECT_NEAR(pressureToStandardAtmosphereHeightM(794.952), 2000.0, 0.5);
    // Near sea level 1 hPa is about 8.3 m.
    const double slope = pressureToStandardAtmosphereHeightM(1012.25) -
                         pressureToStandardAtmosphereHeightM(1013.25);
    EXPECT_NEAR(slope, 8.33, 0.05);
    // Lower pressure is higher.
    EXPECT_GT(pressureToStandardAtmosphereHeightM(1000.0),
              pressureToStandardAtmosphereHeightM(1010.0));
}

TEST(BarometerHeightTest, PressureHeightRoundTripsAndRejectsInvalidInput) {
    for (double h : {-100.0, 0.0, 12.3, 59.5, 500.0, 3000.0}) {
        const double p = standardAtmosphereHeightToPressureHpa(h);
        EXPECT_NEAR(pressureToStandardAtmosphereHeightM(p), h, 1e-6);
    }
    EXPECT_TRUE(std::isnan(pressureToStandardAtmosphereHeightM(0.0)));
    EXPECT_TRUE(std::isnan(pressureToStandardAtmosphereHeightM(-5.0)));
    EXPECT_TRUE(std::isnan(
        pressureToStandardAtmosphereHeightM(std::numeric_limits<double>::infinity())));
    EXPECT_TRUE(std::isnan(
        pressureToStandardAtmosphereHeightM(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(standardAtmosphereHeightToPressureHpa(1.0e6)));
}

TEST(BarometerHeightTest, SampleBufferIsCausalAndFresh) {
    BarometerSampleBuffer buffer;
    const double p0 = standardAtmosphereHeightToPressureHpa(10.0);
    const double p_future = standardAtmosphereHeightToPressureHpa(500.0);
    for (int k = 0; k <= 5; ++k) {
        buffer.add(GNSSTime(2300, 100.0 + k), p0);
    }
    buffer.add(GNSSTime(2300, 108.0), p_future);  // future relative to the query

    double h = 0.0;
    int count = 0;
    ASSERT_TRUE(buffer.heightAt(GNSSTime(2300, 105.5), 2.0, 3.0, h, &count));
    EXPECT_NEAR(h, 10.0, 1e-6);  // the future sample must not leak in
    EXPECT_EQ(count, 2);         // samples at 104 and 105 are inside (105.5-2, 105.5]
    // Window selects only recent samples.
    ASSERT_TRUE(buffer.heightAt(GNSSTime(2300, 105.0), 1.0, 3.0, h, &count));
    EXPECT_EQ(count, 1);
    // Stale: newest sample older than max age.
    EXPECT_FALSE(buffer.heightAt(GNSSTime(2300, 107.9), 2.0, 2.0, h, &count));
    // Before any sample.
    EXPECT_FALSE(buffer.heightAt(GNSSTime(2300, 50.0), 2.0, 3.0, h, &count));
    // Invalid pressure is dropped; out-of-order sample is dropped.
    const auto size = buffer.size();
    buffer.add(GNSSTime(2300, 109.0), -1.0);
    buffer.add(GNSSTime(2300, 90.0), p0);
    EXPECT_EQ(buffer.size(), size);
}

TEST(BarometerHeightTest, FilterEstimatesUnknownOffsetFromNoisyGnssHeight) {
    BaroHeightConfig config = testConfig();
    BarometerHeightFilter filter(config);
    std::mt19937 rng(7);
    std::normal_distribution<double> gnss_noise(0.0, 10.0);
    std::normal_distribution<double> baro_noise(0.0, 0.5);
    const double true_bias = 7.3;
    double t = 0.0;
    for (int k = 0; k < 600; ++k, t += 1.0) {
        // Ascend 10 m between 100 s and 140 s, then stay up.
        const double h = 60.0 + 10.0 * std::clamp((t - 100.0) / 40.0, 0.0, 1.0);
        const GNSSTime time(2300, 1000.0 + t);
        const double hb = h + true_bias + baro_noise(rng);
        const double hg = h + gnss_noise(rng);
        if (!filter.initialized()) {
            filter.initialize(time, hg, 10.0, hb);
            continue;
        }
        filter.predict(time);
        EXPECT_TRUE(filter.updateBaro(hb)) << "k=" << k;
        filter.updateGnss(hg, 10.0);
    }
    EXPECT_NEAR(filter.bias(), true_bias, 1.5);
    EXPECT_NEAR(filter.height(), 70.0, 1.5);
    EXPECT_LT(filter.heightSigma(), 2.0);
    // 10 m GNSS noise averaged by baro: well below the raw GNSS sigma.
    EXPECT_LT(filter.biasSigma(), 2.5);
}

TEST(BarometerHeightTest, BaroSpikeIsGatedAndPersistentStepRebases) {
    BaroHeightConfig config = testConfig();
    config.rebase_after_rejections = 5;
    BarometerHeightFilter filter(config);
    const double h = 50.0, bias = 3.0;
    double t = 0.0;
    filter.initialize(GNSSTime(2300, 0.0), h, 3.0, h + bias);
    for (int k = 1; k <= 120; ++k) {
        ++t;
        filter.predict(GNSSTime(2300, t));
        ASSERT_TRUE(filter.updateBaro(h + bias));
        filter.updateGnss(h, 3.0);
    }
    const double before = filter.height();
    // Single 30 m spike (door slam): rejected, state unchanged.
    ++t;
    filter.predict(GNSSTime(2300, t));
    EXPECT_FALSE(filter.updateBaro(h + bias + 30.0));
    EXPECT_NEAR(filter.height(), before, 1e-9);
    EXPECT_EQ(filter.consecutiveBaroRejections(), 1);
    ++t;
    filter.predict(GNSSTime(2300, t));
    EXPECT_TRUE(filter.updateBaro(h + bias));  // recovery resets the counter
    EXPECT_EQ(filter.consecutiveBaroRejections(), 0);

    // Persistent +12 m step in pressure height: after 5 rejections it re-bases
    // the bias (height is kept), it does not drag the height by 12 m.
    int rejected = 0;
    for (int k = 0; k < 5; ++k) {
        ++t;
        filter.predict(GNSSTime(2300, t));
        if (!filter.updateBaro(h + bias + 12.0)) {
            ++rejected;
        }
    }
    EXPECT_EQ(rejected, 4);
    EXPECT_EQ(filter.rebaseCount(), 1);
    EXPECT_NEAR(filter.height(), h, 1.0);
    EXPECT_NEAR(filter.bias(), bias + 12.0, 1.0);
    EXPECT_GT(filter.biasSigma(), config.rebase_bias_sigma_m * 0.9);
}

TEST(BarometerHeightTest, GnssOutlierIsGatedAndCovarianceStaysPsd) {
    BarometerHeightFilter filter(testConfig());
    filter.initialize(GNSSTime(2300, 0.0), 40.0, 4.0, 45.0);
    for (int k = 1; k <= 60; ++k) {
        filter.predict(GNSSTime(2300, k));
        filter.updateBaro(45.0);
        filter.updateGnss(40.0, 4.0);
    }
    filter.predict(GNSSTime(2300, 61.0));
    filter.updateBaro(45.0);
    EXPECT_FALSE(filter.updateGnss(140.0, 4.0));  // 100 m multipath outlier
    EXPECT_NEAR(filter.height(), 40.0, 1.0);
    const Eigen::Matrix2d p = filter.covariance();
    EXPECT_NEAR(p(0, 1), p(1, 0), 1e-12);
    EXPECT_GE(p(0, 0), 0.0);
    EXPECT_GE(p(1, 1), 0.0);
    EXPECT_GE(p(0, 0) * p(1, 1) - p(0, 1) * p(1, 0), -1e-9);
}

TEST(BarometerHeightTest, BiasRandomWalkTracksSlowDrift) {
    BaroHeightConfig config = testConfig();
    config.bias_walk_m_per_sqrt_s = 0.05;
    BarometerHeightFilter filter(config);
    const double h = 20.0;
    filter.initialize(GNSSTime(2300, 0.0), h, 2.0, h + 1.0);
    double last_bias = 0.0;
    for (int k = 1; k <= 900; ++k) {
        const double bias = 1.0 + 0.01 * k;  // 10 m over 15 min of weather/HVAC drift
        filter.predict(GNSSTime(2300, k));
        filter.updateBaro(h + bias);
        filter.updateGnss(h, 2.0);
        last_bias = bias;
    }
    EXPECT_NEAR(filter.bias(), last_bias, 1.0);
}

TEST(BarometerHeightTest, UninitialisedFilterIsInert) {
    BarometerHeightFilter filter(testConfig());
    EXPECT_FALSE(filter.initialized());
    filter.predict(GNSSTime(2300, 1.0));
    EXPECT_FALSE(filter.updateBaro(10.0));
    EXPECT_FALSE(filter.updateGnss(10.0, 3.0));
    EXPECT_FALSE(filter.initialized());
}
