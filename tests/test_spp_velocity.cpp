#include <gtest/gtest.h>

#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/constants.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace libgnss {
namespace {

// Six well-spread line-of-sight unit vectors (not all coplanar), scaled out
// to a GPS-like ~20,200 km orbital radius from an arbitrary receiver
// position, so the Doppler LS design matrix is well-conditioned (rank 4).
struct SyntheticSatellite {
    Eigen::Vector3d unit_los;
    Eigen::Vector3d velocity_ecef;
    double clock_drift;  // s/s
};

std::vector<SyntheticSatellite> syntheticConstellation() {
    return {
        {Eigen::Vector3d(0.6, 0.2, 0.7746794).normalized(), Eigen::Vector3d(400.0, -1200.0, 2500.0), 1.0e-11},
        {Eigen::Vector3d(-0.5, 0.4, 0.7681146).normalized(), Eigen::Vector3d(-900.0, 300.0, -1800.0), -2.0e-11},
        {Eigen::Vector3d(0.1, -0.6, 0.7937254).normalized(), Eigen::Vector3d(1500.0, 800.0, 400.0), 5.0e-12},
        {Eigen::Vector3d(-0.3, -0.5, 0.8124038).normalized(), Eigen::Vector3d(-600.0, -1400.0, 900.0), -3.0e-12},
        {Eigen::Vector3d(0.7, -0.1, 0.7).normalized(), Eigen::Vector3d(200.0, 1900.0, -700.0), 8.0e-12},
        {Eigen::Vector3d(-0.2, 0.75, 0.6294687).normalized(), Eigen::Vector3d(-1100.0, -300.0, 1600.0), -6.0e-12},
    };
}

constexpr double kOrbitRangeM = 2.02e7;
constexpr double kGpsL1FreqHz = 1575.42e6;

// Build the exact (noise-free) Doppler observation that a receiver moving at
// receiver_velocity_ecef with the given clock drift would see from a
// satellite at satellite_position/velocity_ecef, inverting the same
// formula spp_velocity::solveVelocity() uses internally: measured range
// rate = -doppler * c / freq, and range rate = dot(sat_vel - rx_vel, los) +
// c*rx_drift - c*sat_clock_drift.
spp_velocity::DopplerObservation makeExactObservation(const SyntheticSatellite& sat,
                                                       const Eigen::Vector3d& receiver_position_ecef,
                                                       const Eigen::Vector3d& receiver_velocity_ecef,
                                                       double receiver_clock_drift,
                                                       double elevation_rad) {
    const Eigen::Vector3d satellite_position_ecef =
        receiver_position_ecef + kOrbitRangeM * sat.unit_los;
    const double range_rate =
        (sat.velocity_ecef - receiver_velocity_ecef).dot(sat.unit_los) +
        constants::SPEED_OF_LIGHT * receiver_clock_drift -
        constants::SPEED_OF_LIGHT * sat.clock_drift;

    spp_velocity::DopplerObservation obs;
    obs.satellite_position_ecef = satellite_position_ecef;
    obs.satellite_velocity_ecef = sat.velocity_ecef;
    obs.satellite_clock_drift = sat.clock_drift;
    obs.doppler_hz = -range_rate * kGpsL1FreqHz / constants::SPEED_OF_LIGHT;
    obs.frequency_hz = kGpsL1FreqHz;
    obs.elevation_rad = elevation_rad;
    return obs;
}

}  // namespace

TEST(SppVelocityTest, RecoversKnownVelocityAndClockDriftFromExactGeometry) {
    const Eigen::Vector3d receiver_position_ecef(-3961905.0, 3348994.0, 3702789.0);
    const Eigen::Vector3d true_velocity_ecef(12.5, -4.2, 0.8);  // ~13.4 m/s, plausible vehicle speed
    const double true_clock_drift = 3.0e-8;                      // s/s

    const auto constellation = syntheticConstellation();
    std::vector<spp_velocity::DopplerObservation> observations;
    observations.reserve(constellation.size());
    double elevation_deg = 15.0;
    for (const auto& sat : constellation) {
        observations.push_back(makeExactObservation(sat, receiver_position_ecef, true_velocity_ecef,
                                                     true_clock_drift, elevation_deg * M_PI / 180.0));
        elevation_deg += 10.0;
    }

    const auto result = spp_velocity::solveVelocity(observations, receiver_position_ecef);

    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.num_satellites_used, static_cast<int>(constellation.size()));
    EXPECT_NEAR((result.velocity_ecef - true_velocity_ecef).norm(), 0.0, 1e-6);
    EXPECT_NEAR(result.receiver_clock_drift, true_clock_drift, 1e-15);
    EXPECT_NEAR(result.residual_rms_mps, 0.0, 1e-9);
    EXPECT_TRUE(result.velocity_covariance.allFinite());
    // Covariance should be positive (semi-)definite: diagonal terms >= 0.
    EXPECT_GE(result.velocity_covariance(0, 0), 0.0);
    EXPECT_GE(result.velocity_covariance(1, 1), 0.0);
    EXPECT_GE(result.velocity_covariance(2, 2), 0.0);
}

TEST(SppVelocityTest, RejectsFewerThanFourUsableSatellites) {
    const Eigen::Vector3d receiver_position_ecef(-3961905.0, 3348994.0, 3702789.0);
    const Eigen::Vector3d true_velocity_ecef(5.0, 1.0, 0.0);

    const auto constellation = syntheticConstellation();
    std::vector<spp_velocity::DopplerObservation> observations;
    for (int i = 0; i < 3; ++i) {
        observations.push_back(makeExactObservation(constellation[static_cast<size_t>(i)],
                                                     receiver_position_ecef, true_velocity_ecef, 0.0,
                                                     0.5));
    }

    const auto result = spp_velocity::solveVelocity(observations, receiver_position_ecef);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.num_satellites_used, 0);
    EXPECT_TRUE(result.velocity_ecef.isZero());
}

TEST(SppVelocityTest, IgnoresObservationsWithoutUsableDopplerOrFrequency) {
    const Eigen::Vector3d receiver_position_ecef(-3961905.0, 3348994.0, 3702789.0);
    const Eigen::Vector3d true_velocity_ecef(5.0, 1.0, 0.0);

    const auto constellation = syntheticConstellation();
    std::vector<spp_velocity::DopplerObservation> observations;
    for (size_t i = 0; i < constellation.size(); ++i) {
        observations.push_back(makeExactObservation(constellation[i], receiver_position_ecef,
                                                     true_velocity_ecef, 0.0, 0.5));
    }
    // Zero out Doppler/frequency on all but three observations, so only
    // three remain usable -- below the default min_satellites=4 floor.
    observations[0].doppler_hz = 0.0;             // filtered: doppler == 0
    observations[1].frequency_hz = 0.0;            // filtered: invalid frequency
    observations[2].frequency_hz = std::numeric_limits<double>::quiet_NaN();  // filtered: non-finite

    const auto result = spp_velocity::solveVelocity(observations, receiver_position_ecef);

    EXPECT_FALSE(result.ok);
}

TEST(SppVelocityTest, RejectsNonFiniteReceiverPosition) {
    const Eigen::Vector3d bad_position(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    const auto constellation = syntheticConstellation();
    std::vector<spp_velocity::DopplerObservation> observations;
    for (const auto& sat : constellation) {
        observations.push_back(
            makeExactObservation(sat, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.5));
    }

    const auto result = spp_velocity::solveVelocity(observations, bad_position);

    EXPECT_FALSE(result.ok);
}

}  // namespace libgnss
