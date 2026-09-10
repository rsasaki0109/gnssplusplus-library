#include <gtest/gtest.h>

#include <libgnss++/algorithms/raw_p_seed.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <iostream>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using libgnss::GNSSTime;
using libgnss::GNSSSystem;
using libgnss::NavigationData;
using libgnss::Observation;
using libgnss::ObservationData;
using libgnss::SignalType;
using libgnss::Vector3d;

libgnss::Ephemeris makeSyntheticEphemeris(GNSSSystem system, uint8_t prn) {
    libgnss::Ephemeris eph;
    eph.satellite = libgnss::SatelliteId(system, prn);
    eph.valid = true;
    eph.week = 2300;
    eph.toe = GNSSTime(eph.week, 100000.0);
    eph.toc = eph.toe;
    eph.tof = eph.toe;
    eph.toes = eph.toe.tow;
    eph.sqrt_a = std::sqrt(26560000.0);
    eph.e = 0.004 + 0.0002 * static_cast<double>(prn);
    eph.i0 = 0.94 + 0.01 * static_cast<double>(prn % 3);
    const double constellation_phase =
        system == GNSSSystem::Galileo ? 0.11 : 0.0;
    eph.omega0 = 0.35 * static_cast<double>(prn) + constellation_phase;
    eph.omega = 0.17 * static_cast<double>(prn) + constellation_phase * 0.5;
    eph.m0 = 0.61 * static_cast<double>(prn);
    eph.delta_n = 1e-9 * static_cast<double>(prn);
    eph.omega_dot = -8.0e-9;
    eph.health = 0;
    return eph;
}

libgnss::Ephemeris makeSyntheticGpsEphemeris(uint8_t prn) {
    return makeSyntheticEphemeris(GNSSSystem::GPS, prn);
}

NavigationData makeSyntheticNavigation() {
    NavigationData nav;
    for (uint8_t prn = 1; prn <= 4; ++prn) {
        nav.addEphemeris(makeSyntheticGpsEphemeris(prn));
    }
    return nav;
}

NavigationData makeSyntheticGpsNavigation(uint8_t last_prn) {
    NavigationData nav;
    for (uint8_t prn = 1; prn <= last_prn; ++prn) {
        nav.addEphemeris(makeSyntheticGpsEphemeris(prn));
    }
    return nav;
}

NavigationData makeSyntheticMultiConstellationNavigation() {
    NavigationData nav = makeSyntheticNavigation();
    for (uint8_t prn = 1; prn <= 4; ++prn) {
        nav.addEphemeris(makeSyntheticEphemeris(GNSSSystem::Galileo, prn));
    }
    return nav;
}

SignalType signalForSystem(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::Galileo: return SignalType::GAL_E1;
        case GNSSSystem::GPS: return SignalType::GPS_L1CA;
        default: return SignalType::GPS_L1CA;
    }
}

bool makePseudorangeObservation(const NavigationData& nav,
                                const libgnss::SatelliteId& satellite,
                                const GNSSTime& time,
                                const Vector3d& receiver,
                                double receiver_clock_bias_m,
                                bool include_doppler,
                                Observation& observation) {
    double pseudorange = 22'000'000.0;
    Vector3d satellite_position = Vector3d::Zero();
    Vector3d satellite_velocity = Vector3d::Zero();
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    for (int iteration = 0; iteration < 6; ++iteration) {
        GNSSTime transmit_time =
            time - pseudorange / libgnss::constants::SPEED_OF_LIGHT;
        if (!nav.calculateSatelliteState(satellite,
                                         transmit_time,
                                         satellite_position,
                                         satellite_velocity,
                                         satellite_clock_bias,
                                         satellite_clock_drift)) {
            return false;
        }
        transmit_time = transmit_time - satellite_clock_bias;
        if (!nav.calculateSatelliteState(satellite,
                                         transmit_time,
                                         satellite_position,
                                         satellite_velocity,
                                         satellite_clock_bias,
                                         satellite_clock_drift)) {
            return false;
        }
        const double travel_time =
            (satellite_position - receiver).norm() /
            libgnss::constants::SPEED_OF_LIGHT;
        const double angle = libgnss::constants::OMEGA_E * travel_time;
        const Eigen::Matrix3d earth_rotation =
            (Eigen::AngleAxisd(-angle, Eigen::Vector3d::UnitZ())).toRotationMatrix();
        const Vector3d corrected_satellite_position =
            earth_rotation * satellite_position;
        pseudorange = (corrected_satellite_position - receiver).norm() -
                      satellite_clock_bias * libgnss::constants::SPEED_OF_LIGHT +
                      receiver_clock_bias_m;
    }

    observation = Observation(satellite, signalForSystem(satellite.system));
    observation.pseudorange = pseudorange;
    observation.snr = 45.0;
    observation.has_pseudorange = true;
    observation.valid = true;
    if (include_doppler) {
        observation.doppler = 12345.0;
        observation.has_doppler = true;
        observation.pseudorange_rate_mps = -17.0;
        observation.has_pseudorange_rate_mps = true;
        observation.source_carrier_frequency_hz = 1.57542e9;
        observation.has_source_carrier_frequency_hz = true;
    }
    return true;
}

ObservationData makeEpoch(const NavigationData& nav,
                          const GNSSTime& time,
                          const Vector3d& receiver,
                          double receiver_clock_bias_m,
                          bool include_doppler = false,
                          std::size_t raw_source_index =
                              std::numeric_limits<std::size_t>::max()) {
    ObservationData epoch(time);
    epoch.receiver_position = receiver;
    epoch.raw_source_index = raw_source_index;
    epoch.raw_utc_time_millis = raw_source_index ==
                                        std::numeric_limits<std::size_t>::max()
                                    ? -1
                                    : 1610000000000LL +
                                          static_cast<std::int64_t>(raw_source_index) *
                                              1000LL;
    for (uint8_t prn = 1; prn <= 4; ++prn) {
        Observation observation;
        EXPECT_TRUE(makePseudorangeObservation(
            nav,
            libgnss::SatelliteId(GNSSSystem::GPS, prn),
            time,
            receiver,
            receiver_clock_bias_m,
            include_doppler,
            observation));
        epoch.addObservation(observation);
    }
    return epoch;
}

ObservationData makeGpsEpoch(const NavigationData& nav,
                             const GNSSTime& time,
                             const Vector3d& receiver,
                             double receiver_clock_bias_m,
                             uint8_t last_prn,
                             bool include_doppler = false) {
    ObservationData epoch(time);
    epoch.receiver_position = receiver;
    for (uint8_t prn = 1; prn <= last_prn; ++prn) {
        Observation observation;
        EXPECT_TRUE(makePseudorangeObservation(
            nav,
            libgnss::SatelliteId(GNSSSystem::GPS, prn),
            time,
            receiver,
            receiver_clock_bias_m,
            include_doppler,
            observation));
        epoch.addObservation(observation);
    }
    return epoch;
}

ObservationData makeMultiConstellationEpoch(const NavigationData& nav,
                                            const GNSSTime& time,
                                            const Vector3d& receiver,
                                            double receiver_clock_bias_m,
                                            double galileo_clock_bias_m,
                                            bool include_doppler = false) {
    ObservationData epoch(time);
    epoch.receiver_position = receiver;
    for (uint8_t prn = 1; prn <= 4; ++prn) {
        Observation gps_observation;
        EXPECT_TRUE(makePseudorangeObservation(
            nav,
            libgnss::SatelliteId(GNSSSystem::GPS, prn),
            time,
            receiver,
            receiver_clock_bias_m,
            include_doppler,
            gps_observation));
        epoch.addObservation(gps_observation);

        Observation galileo_observation;
        EXPECT_TRUE(makePseudorangeObservation(
            nav,
            libgnss::SatelliteId(GNSSSystem::Galileo, prn),
            time,
            receiver,
            receiver_clock_bias_m + galileo_clock_bias_m,
            include_doppler,
            galileo_observation));
        epoch.addObservation(galileo_observation);
    }
    return epoch;
}

std::vector<ObservationData> makeMultiConstellationTrajectory(
    const NavigationData& nav,
    std::size_t count,
    double galileo_clock_bias_m,
    bool include_doppler = false) {
    const Vector3d initial(1113194.0, -4841695.0, 3985350.0);
    const Vector3d velocity(5.0, -2.0, 1.0);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i);
        epochs.push_back(makeMultiConstellationEpoch(
            nav,
            GNSSTime(2300, 100100.0 + t),
            initial + velocity * t,
            30.0 + 0.75 * t,
            galileo_clock_bias_m,
            include_doppler));
    }
    return epochs;
}

libgnss::raw_p_seed::Config syntheticConfig() {
    libgnss::raw_p_seed::Config config;
    config.processor_config.elevation_mask = -90.0;
    config.processor_config.snr_mask = 0.0;
    config.processor_config.use_ionosphere_model = false;
    config.processor_config.use_troposphere_model = false;
    config.spp_config.apply_atmospheric_corrections = false;
    config.spp_config.use_precise_products = false;
    config.spp_config.use_ssr_corrections = false;
    config.spp_config.model_intersystem_bias = false;
    config.spp_config.enable_outlier_detection = false;
    config.spp_config.max_gdop = 0.0;
    config.spp_config.position_convergence_threshold = 1e-8;
    config.spp_config.max_iterations = 20;
    config.max_gap_s = 2.0;
    return config;
}

std::vector<ObservationData> makeTrajectory(const NavigationData& nav,
                                            std::size_t count,
                                            bool moving,
                                            bool include_doppler = false) {
    const Vector3d initial(1113194.0, -4841695.0, 3985350.0);
    const Vector3d velocity(5.0, -2.0, 1.0);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i);
        Vector3d receiver = initial;
        if (moving) {
            receiver += velocity * t;
        }
        epochs.push_back(makeEpoch(nav,
                                   GNSSTime(2300, 100100.0 + t),
                                   receiver,
                                   30.0 + 0.75 * t,
                                   include_doppler,
                                   i));
    }
    return epochs;
}

Vector3d rotateAroundZ(const Vector3d& vector, double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return Vector3d(c * vector.x() - s * vector.y(),
                    s * vector.x() + c * vector.y(),
                    vector.z());
}

NavigationData rotateNavigationAroundZ(const NavigationData& input,
                                       double angle_rad) {
    NavigationData rotated = input;
    for (auto& [satellite, ephemerides] : rotated.ephemeris_data) {
        for (auto& ephemeris : ephemerides) {
            ephemeris.omega0 += angle_rad;
        }
    }
    return rotated;
}

void expectPreprocessConservation(
    const libgnss::raw_p_seed::EpochSeed& epoch) {
    ASSERT_TRUE(epoch.preprocessing_diagnostics_available);
    EXPECT_EQ(epoch.preprocessing_input_rows,
              epoch.preprocessing_accepted_rows +
                  epoch.preprocessing_rejected_rows);
    EXPECT_EQ(epoch.preprocessing_rows.size(),
              epoch.preprocessing_input_rows);

    std::size_t reason_total = 0;
    for (const auto& entry : epoch.preprocessing_reason_counts) {
        reason_total += entry.count;
    }
    EXPECT_EQ(reason_total, epoch.preprocessing_input_rows);

    std::vector<bool> seen(epoch.preprocessing_input_rows, false);
    for (const auto& row : epoch.preprocessing_rows) {
        ASSERT_LT(row.input_row_index, epoch.preprocessing_input_rows);
        EXPECT_FALSE(seen[row.input_row_index]);
        seen[row.input_row_index] = true;
        EXPECT_FALSE(row.reason.empty());
    }
    for (const bool row_seen : seen) {
        EXPECT_TRUE(row_seen);
    }
}

}  // namespace

TEST(RawPSeedTest, StationaryPseudorangeAndClockBiasProduceFiniteSeeds) {
    const NavigationData nav = makeSyntheticNavigation();
    const auto result = libgnss::raw_p_seed::solve(
        makeTrajectory(nav, 3, false), nav, syntheticConfig());

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 3U);
    for (std::size_t i = 0; i < result.epochs.size(); ++i) {
        const auto& epoch = result.epochs[i];
        EXPECT_EQ(epoch.status, libgnss::raw_p_seed::EpochStatus::Accepted);
        EXPECT_TRUE(epoch.position_ecef.allFinite());
        EXPECT_TRUE(std::isfinite(epoch.receiver_clock_bias_m));
        EXPECT_TRUE(epoch.has_velocity);
        EXPECT_TRUE(epoch.velocity_ecef_mps.allFinite());
        EXPECT_NEAR(epoch.receiver_clock_bias_m,
                    30.0 + 0.75 * static_cast<double>(i),
                    1e-4);
        EXPECT_NEAR(epoch.velocity_ecef_mps.norm(), 0.0, 1e-5);
        EXPECT_EQ(epoch.geometry_rank.status,
                  libgnss::raw_p_seed::RankStatus::FullRank);
        EXPECT_EQ(epoch.geometry_rank.required_rank, 4);
        EXPECT_EQ(epoch.corrected_clock_groups, 1U);
        EXPECT_EQ(epoch.reference_clock_group, GNSSSystem::GPS);
        ASSERT_EQ(epoch.clock_group_biases.size(), 6U);
        const auto gps = std::find_if(
            epoch.clock_group_biases.begin(), epoch.clock_group_biases.end(),
            [](const auto& entry) { return entry.group == GNSSSystem::GPS; });
        ASSERT_NE(gps, epoch.clock_group_biases.end());
        EXPECT_TRUE(gps->observed);
        EXPECT_TRUE(gps->is_reference);
        EXPECT_TRUE(gps->estimate_available);
    }
}

TEST(RawPSeedTest, MovingPseudorangeAndClockBiasUseExplicitGradientEndpoints) {
    const NavigationData nav = makeSyntheticNavigation();
    const auto result = libgnss::raw_p_seed::solve(
        makeTrajectory(nav, 3, true), nav, syntheticConfig());

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 3U);
    const Vector3d expected_velocity(5.0, -2.0, 1.0);
    for (const auto& epoch : result.epochs) {
        EXPECT_TRUE(epoch.has_velocity);
        EXPECT_TRUE(epoch.velocity_ecef_mps.isApprox(expected_velocity, 1e-5));
    }
    EXPECT_NEAR(result.epochs[0].receiver_clock_bias_m, 30.0, 1e-4);
    EXPECT_NEAR(result.epochs[1].receiver_clock_bias_m, 30.75, 1e-4);
    EXPECT_NEAR(result.epochs[2].receiver_clock_bias_m, 31.5, 1e-4);
    EXPECT_STREQ(
        libgnss::raw_p_seed::velocityEndpointPolicyName(result.endpoint_policy),
        "one-sided-endpoints-centered-interior");
}

TEST(RawPSeedTest, DopplerIsClearedAndCannotChangePOnlySeeds) {
    const NavigationData nav = makeSyntheticNavigation();
    const auto without_doppler = libgnss::raw_p_seed::solve(
        makeTrajectory(nav, 3, true, false), nav, syntheticConfig());
    const auto with_doppler = libgnss::raw_p_seed::solve(
        makeTrajectory(nav, 3, true, true), nav, syntheticConfig());

    ASSERT_TRUE(without_doppler.ok) << without_doppler.failure_reason;
    ASSERT_TRUE(with_doppler.ok) << with_doppler.failure_reason;
    ASSERT_EQ(without_doppler.epochs.size(), with_doppler.epochs.size());
    for (std::size_t i = 0; i < without_doppler.epochs.size(); ++i) {
        EXPECT_TRUE(without_doppler.epochs[i].position_ecef.isApprox(
            with_doppler.epochs[i].position_ecef, 1e-9));
        EXPECT_DOUBLE_EQ(without_doppler.epochs[i].receiver_clock_bias_m,
                         with_doppler.epochs[i].receiver_clock_bias_m);
        EXPECT_TRUE(without_doppler.epochs[i].velocity_ecef_mps.isApprox(
            with_doppler.epochs[i].velocity_ecef_mps, 1e-9));
    }
}

TEST(RawPSeedTest, RejectsInsufficientPseudorangeSatellites) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 2, false);
    epochs[1].observations.pop_back();
    const auto result = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientPseudorange);
    EXPECT_EQ(result.epochs[1].raw_pseudorange_satellites, 3U);
    EXPECT_FALSE(result.epochs[1].position_ecef.allFinite());
}

TEST(RawPSeedTest, ReportsRankDeficientPositionClockGeometry) {
    const std::vector<Vector3d> collinear = {
        Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0),
        Vector3d(3.0, 0.0, 0.0), Vector3d(4.0, 0.0, 0.0)};
    const auto rank = libgnss::raw_p_seed::assessGeometryRank(
        collinear, Vector3d::Zero());

    EXPECT_EQ(rank.status, libgnss::raw_p_seed::RankStatus::RankDeficient);
    EXPECT_LT(rank.rank, rank.required_rank);
    EXPECT_EQ(rank.rows, 4U);

    const std::vector<GNSSSystem> groups(4, GNSSSystem::GPS);
    const std::vector<double> weights(4, 1.0);
    const auto weighted_rank = libgnss::raw_p_seed::assessGeometryRank(
        collinear,
        Vector3d::Zero(),
        groups,
        GNSSSystem::GPS,
        false,
        weights);
    EXPECT_EQ(weighted_rank.status,
              libgnss::raw_p_seed::RankStatus::RankDeficient);
    EXPECT_LT(weighted_rank.rank, weighted_rank.required_rank);
}

TEST(RawPSeedTest, WeightedRankMatchesNativeDesignScaling) {
    // A deterministic near-collinear design makes Eigen's default QR rank
    // threshold distinguish the legacy unweighted preflight from native
    // sqrt(weight)-scaled H.  This is synthetic geometry only; no position
    // payload or persisted seed is involved.
    constexpr double epsilon = 8.317637184061381e-8;
    std::vector<Vector3d> satellite_positions;
    std::vector<GNSSSystem> groups;
    std::vector<double> weights;
    satellite_positions.reserve(22);
    groups.reserve(22);
    weights.reserve(22);
    for (int i = 0; i < 22; ++i) {
        Vector3d direction(
            1.0 + epsilon * std::sin(static_cast<double>(i + 1) * 0.731),
            epsilon * std::cos(static_cast<double>(i + 2) * 0.826),
            epsilon * std::sin(static_cast<double>(i + 4) * 0.831));
        satellite_positions.push_back(direction.normalized());
        groups.push_back(
            i % 4 == 0 ? GNSSSystem::GPS
            : i % 4 == 1 ? GNSSSystem::GLONASS
            : i % 4 == 2 ? GNSSSystem::Galileo
                         : GNSSSystem::BeiDou);
        weights.push_back(i % 2 == 0 ? 1.0 : 1.0e-6);
    }

    const auto unweighted = libgnss::raw_p_seed::assessGeometryRank(
        satellite_positions,
        Vector3d::Zero(),
        groups,
        GNSSSystem::GPS,
        true);
    const auto weighted = libgnss::raw_p_seed::assessGeometryRank(
        satellite_positions,
        Vector3d::Zero(),
        groups,
        GNSSSystem::GPS,
        true,
        weights);

    EXPECT_EQ(unweighted.required_rank, 7);
    EXPECT_EQ(unweighted.status,
              libgnss::raw_p_seed::RankStatus::RankDeficient);
    EXPECT_EQ(unweighted.rank, 6);
    EXPECT_EQ(weighted.required_rank, 7);
    EXPECT_EQ(weighted.status, libgnss::raw_p_seed::RankStatus::FullRank);
    EXPECT_EQ(weighted.rank, 7);
}

TEST(RawPSeedTest, UsesExactNativeQcRowsForRankAndClockReports) {
    const NavigationData nav = makeSyntheticGpsNavigation(8);
    auto epochs = std::vector<ObservationData>{makeGpsEpoch(
        nav,
        GNSSTime(2300, 100100.0),
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        30.0,
        8)};
    // Native RAIM/FDE should remove this one gross P outlier from its final
    // solve.  The preprocessing view still contains the source row, so the
    // raw-P report must follow native exact row identity rather than merely
    // counting all corrected rows.
    epochs.front().observations.front().pseudorange += 1000.0;

    auto config = syntheticConfig();
    config.derive_velocity = false;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    EXPECT_EQ(epoch.status, libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_EQ(epoch.corrected_pseudorange_rows, 8U);
    EXPECT_LT(epoch.native_used_pseudorange_rows,
              epoch.corrected_pseudorange_rows);
    EXPECT_EQ(epoch.native_used_pseudorange_rows,
              epoch.geometry_rank.rows);
    EXPECT_EQ(epoch.corrected_clock_groups, 1U);
    EXPECT_EQ(epoch.reference_clock_group, GNSSSystem::GPS);
    EXPECT_TRUE(epoch.position_ecef.allFinite());
}

TEST(RawPSeedTest, ContaminatedCodeSeedMaskSensitivityControl) {
    // Mechanism control, not a phone-accuracy benchmark. Freeze one positive
    // code error; disable deletion to isolate the existing Huber initializer.
    const auto nav = makeSyntheticGpsNavigation(16);
    const GNSSTime time(2300, 100100.0);
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    const auto clean = makeGpsEpoch(nav, time, receiver, 30., 16);
    auto contaminated = clean;
    contaminated.observations.front().pseudorange += 300.;
    auto config = syntheticConfig();
    config.derive_velocity = false;
    config.spp_config.enable_raim_fde = false;
    config.spp_config.use_weighted_least_squares = false;
    config.spp_config.use_variance_model = false;
    config.spp_config.max_iterations = 100;
    config.spp_config.position_convergence_threshold = 1e-4;
    const auto reference = libgnss::raw_p_seed::solve({clean}, nav, config);
    ASSERT_TRUE(reference.ok) << reference.failure_reason;
    EXPECT_LT((reference.epochs.front().position_ecef - receiver).norm(), 0.1);
    std::array<std::size_t, 2> clean_rejections{};
    std::array<double, 2> position_errors{};
    for (int variant = 0; variant < 4; ++variant) {
        const bool robust = (variant % 2) != 0;
        config.spp_config.enable_raim_fde = variant >= 2;
        config.spp_config.enable_robust_weighting = robust;
        const auto result = libgnss::raw_p_seed::solve({contaminated}, nav, config);
        ASSERT_TRUE(result.ok) << result.failure_reason;
        const auto& seed = result.epochs.front();
        if (variant < 2) EXPECT_EQ(seed.native_used_pseudorange_rows, 16U);
        std::size_t rejected_clean = 0;
        double maximum = 0.;
        for (std::size_t i = 1; i < clean.observations.size(); ++i) {
            Observation predicted;
            ASSERT_TRUE(makePseudorangeObservation(nav, clean.observations[i].satellite,
                time, seed.position_ecef, seed.receiver_clock_bias_m, false, predicted));
            const double residual = clean.observations[i].pseudorange - predicted.pseudorange;
            maximum = std::max(maximum, std::abs(residual));
            rejected_clean += !libgnss::observable_upstream::acceptsCenteredPseudorangeResidual(
                residual, 0., 20.);
        }
        if (variant < 2) {
            clean_rejections[robust] = rejected_clean;
            position_errors[robust] = (seed.position_ecef-receiver).norm();
        } else {
            EXPECT_EQ(seed.native_used_pseudorange_rows, 15U);
            EXPECT_EQ(rejected_clean, 0U);
            EXPECT_LT((seed.position_ecef-receiver).norm(), 0.1);
        }
        std::cout << "[synthetic-seed-mask] robust=" << robust
                  << " raim=" << config.spp_config.enable_raim_fde
                  << " used_rows=" << seed.native_used_pseudorange_rows
                  << " position_error_m=" << (seed.position_ecef-receiver).norm()
                  << " clock_error_m=" << seed.receiver_clock_bias_m-30.
                  << " rejected_clean=" << rejected_clean
                  << " clean_count=15 max_clean_residual_m=" << maximum << '\n';
    }
    EXPECT_GT(clean_rejections[0], 0U);
    EXPECT_LT(clean_rejections[1], clean_rejections[0]);
    EXPECT_LT(position_errors[1], position_errors[0]);
}

TEST(RawPSeedTest, FrozenMultipleCodeContaminationMatrix) {
    // Predeclared cases, no threshold/geometry sweep based on results.
    const std::array<std::array<double, 4>, 5> errors{{
        {{0., 0., 0., 0.}}, {{300., 0., 0., 0.}},
        {{300., 300., 0., 0.}}, {{75., 75., 75., 75.}},
        {{150., -150., 0., 0.}}}};
    const auto nav = makeSyntheticGpsNavigation(16);
    const GNSSTime time(2300, 100100.0);
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    const auto clean = makeGpsEpoch(nav, time, receiver, 30., 16);
    for (bool native_quality : {false, true}) {
        for (std::size_t case_id = 0; case_id < errors.size(); ++case_id) {
            auto epoch = clean;
            for (std::size_t i = 0; i < 4; ++i)
                epoch.observations[i].pseudorange += errors[case_id][i];
            for (bool robust : {false, true}) {
                auto config = syntheticConfig();
                config.derive_velocity = false;
                config.spp_config.enable_raim_fde = true;
                config.spp_config.enable_outlier_detection = true;
                config.spp_config.enable_robust_weighting = robust;
                config.spp_config.use_weighted_least_squares = native_quality;
                config.spp_config.use_variance_model = native_quality;
                if (native_quality) {
                    // App seed quality defaults, except atmosphere is absent
                    // from this synthetic generator and remains disabled.
                    config.processor_config.elevation_mask = 0.;
                    config.spp_config.max_iterations = 10;
                    config.spp_config.position_convergence_threshold = 1e-4;
                    config.spp_config.max_gdop = 50.;
                } else {
                    config.spp_config.max_iterations = 100;
                    config.spp_config.position_convergence_threshold = 1e-4;
                }
                const auto result = libgnss::raw_p_seed::solve({epoch}, nav, config);
                std::cout << "[synthetic-seed-matrix] native_quality=" << native_quality
                          << " case=" << case_id << " robust=" << robust
                          << " accepted=" << result.ok;
                if (result.ok) {
                    ASSERT_EQ(result.epochs.size(), 1U);
                    const auto& seed = result.epochs.front();
                    const double error = (seed.position_ecef-receiver).norm();
                    EXPECT_TRUE(std::isfinite(error));
                    if (case_id == 0) EXPECT_LT(error, 0.1);
                    std::cout << " used_rows=" << seed.native_used_pseudorange_rows
                              << " position_error_m=" << error
                              << " clock_error_m=" << seed.receiver_clock_bias_m-30.;
                } else {
                    EXPECT_FALSE(result.failure_reason.empty());
                    std::cout << " failure=" << result.failure_reason;
                }
                std::cout << '\n';
            }
        }
    }
}

TEST(RawPSeedTest, AcceptsKnownMultipleClockGroupsAndExportsNativeBiases) {
    const NavigationData nav = makeSyntheticMultiConstellationNavigation();
    auto config = syntheticConfig();
    config.spp_config.model_intersystem_bias = true;
    const auto result = libgnss::raw_p_seed::solve(
        makeMultiConstellationTrajectory(nav, 3, 17.0), nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 3U);
    for (std::size_t i = 0; i < result.epochs.size(); ++i) {
        const auto& epoch = result.epochs[i];
        EXPECT_EQ(epoch.status, libgnss::raw_p_seed::EpochStatus::Accepted);
        EXPECT_EQ(epoch.raw_clock_groups, 2U);
        EXPECT_EQ(epoch.corrected_clock_groups, 2U);
        EXPECT_EQ(epoch.reference_clock_group, GNSSSystem::GPS);
        EXPECT_EQ(epoch.geometry_rank.required_rank, 5);
        EXPECT_EQ(epoch.geometry_rank.status,
                  libgnss::raw_p_seed::RankStatus::FullRank);
        EXPECT_EQ(epoch.geometry_rank.rank, 5);
        ASSERT_EQ(epoch.clock_group_biases.size(), 6U);

        const auto find_group = [&epoch](GNSSSystem group)
            -> const libgnss::raw_p_seed::ClockGroupBias& {
            for (const auto& entry : epoch.clock_group_biases) {
                if (entry.group == group) return entry;
            }
            ADD_FAILURE() << "missing clock group entry";
            return epoch.clock_group_biases.front();
        };
        const auto& gps = find_group(GNSSSystem::GPS);
        EXPECT_TRUE(gps.observed);
        EXPECT_TRUE(gps.is_reference);
        EXPECT_TRUE(gps.estimate_available);
        EXPECT_NEAR(gps.bias_m, 30.0 + 0.75 * static_cast<double>(i), 1e-4);

        const auto& galileo = find_group(GNSSSystem::Galileo);
        EXPECT_TRUE(galileo.observed);
        EXPECT_FALSE(galileo.is_reference);
        EXPECT_TRUE(galileo.estimate_available);
        EXPECT_NEAR(galileo.bias_m, 17.0, 1e-4);

        const auto& glonass = find_group(GNSSSystem::GLONASS);
        EXPECT_FALSE(glonass.observed);
        EXPECT_FALSE(glonass.estimate_available);
        EXPECT_TRUE(std::isnan(glonass.bias_m));
    }
}

TEST(RawPSeedTest, HeldOutClockCannotCertifyCorrelatedSupportOrMissingGroup) {
    const auto nav = makeSyntheticMultiConstellationNavigation();
    const Vector3d receiver(1113194., -4841695., 3985350.);
    const GNSSTime time(2300, 100100.);
    const libgnss::SatelliteId target(GNSSSystem::Galileo, 1);
    auto config = syntheticConfig();
    config.derive_velocity = false;
    config.spp_config.model_intersystem_bias = true;
    config.spp_config.enable_raim_fde = true;
    config.spp_config.enable_outlier_detection = true;
    config.spp_config.use_variance_model = false;
    config.spp_config.use_weighted_least_squares = false;
    config.spp_config.max_iterations = 100;
    config.spp_config.position_convergence_threshold = 1e-4;
    for (bool missing_group : {false, true}) {
        auto epoch = makeMultiConstellationEpoch(nav, time, receiver, 30., 17.);
        Observation held_out;
        for (const auto& row : epoch.observations)
            if (row.satellite == target) held_out = row;
        ASSERT_TRUE(held_out.has_pseudorange);
        epoch.observations.erase(std::remove_if(epoch.observations.begin(),
            epoch.observations.end(), [&](const auto& row) {
                return row.satellite == target ||
                    (missing_group && row.satellite.system == GNSSSystem::Galileo);
            }), epoch.observations.end());
        for (auto& row : epoch.observations)
            if (row.satellite.system == GNSSSystem::Galileo) row.pseudorange += 100.;
        const auto result = libgnss::raw_p_seed::solve({epoch}, nav, config);
        ASSERT_TRUE(result.ok) << result.failure_reason;
        const auto& seed = result.epochs.front();
        const auto group = std::find_if(seed.clock_group_biases.begin(),
            seed.clock_group_biases.end(), [](const auto& value) {
                return value.group == GNSSSystem::Galileo;
            });
        ASSERT_NE(group, seed.clock_group_biases.end());
        if (missing_group) {
            EXPECT_FALSE(group->observed);
            EXPECT_FALSE(group->estimate_available);
            EXPECT_FALSE(std::isfinite(group->bias_m));
            std::cout << "[held-out-clock-limit] missing_group=1 prediction_available=0\n";
            continue; // unavailable is not a zero clock correction
        }
        ASSERT_TRUE(group->estimate_available);
        EXPECT_NEAR(group->bias_m, 117., 0.01);
        Observation predicted;
        ASSERT_TRUE(makePseudorangeObservation(nav, target, time, seed.position_ecef,
            seed.receiver_clock_bias_m + group->bias_m, false, predicted));
        for (double target_error : {0., 100.}) {
            const double residual = held_out.pseudorange + target_error - predicted.pseudorange;
            EXPECT_NEAR(residual, target_error - 100., 0.01);
            const bool accepted = libgnss::observable_upstream::acceptsCenteredPseudorangeResidual(
                residual, 0., 20.);
            // Explicit negative control: contaminated support rejects clean
            // code and accepts similarly contaminated code. Not a safe gate.
            EXPECT_EQ(accepted, target_error == 100.);
            std::cout << "[held-out-clock-limit] support_error_m=100 target_error_m="
                      << target_error << " residual_m=" << residual
                      << " accepted=" << accepted << '\n';
        }
    }
}

TEST(RawPSeedTest, HeldOutSystemClockSeparatesCommonStepFromSingleCodeError) {
    const auto nav = makeSyntheticMultiConstellationNavigation();
    const Vector3d receiver(1113194., -4841695., 3985350.);
    auto config = syntheticConfig();
    config.derive_velocity = false;
    config.spp_config.model_intersystem_bias = true;
    config.spp_config.enable_raim_fde = true;
    config.spp_config.enable_outlier_detection = true;
    config.spp_config.use_variance_model = false;
    config.spp_config.use_weighted_least_squares = false;
    config.spp_config.max_iterations = 100;
    config.spp_config.position_convergence_threshold = 1e-4;
    for (double system_bias : {17., 57.}) {
        for (double code_error : {0., 100.}) {
            const GNSSTime time(2300, 100100.);
            auto epoch = makeMultiConstellationEpoch(nav, time, receiver, 30., system_bias);
            const libgnss::SatelliteId target(GNSSSystem::Galileo, 1);
            Observation held_out;
            for (const auto& row : epoch.observations)
                if (row.satellite == target) held_out = row;
            ASSERT_TRUE(held_out.has_pseudorange);
            held_out.pseudorange += code_error;
            epoch.observations.erase(std::remove_if(epoch.observations.begin(),
                epoch.observations.end(), [&](const auto& row) {
                    return row.satellite == target;
                }), epoch.observations.end());
            // Remove the target from the position AND clock solve, not just
            // from a post-fit bias average. Never include its second band.
            ASSERT_EQ(epoch.observations.size(), 7U);
            const auto result = libgnss::raw_p_seed::solve({epoch}, nav, config);
            ASSERT_TRUE(result.ok) << result.failure_reason;
            const auto& seed = result.epochs.front();
            const auto group = std::find_if(seed.clock_group_biases.begin(),
                seed.clock_group_biases.end(), [](const auto& value) {
                    return value.group == GNSSSystem::Galileo;
                });
            ASSERT_NE(group, seed.clock_group_biases.end());
            ASSERT_TRUE(group->estimate_available);
            EXPECT_GE(group->corrected_rows, 3U);
            Observation predicted;
            ASSERT_TRUE(makePseudorangeObservation(nav, target, time, seed.position_ecef,
                seed.receiver_clock_bias_m + group->bias_m, false, predicted));
            const double residual = held_out.pseudorange - predicted.pseudorange;
            EXPECT_NEAR(residual, code_error, 0.01);
            const bool accepted = libgnss::observable_upstream::acceptsCenteredPseudorangeResidual(
                residual, 0., 20.);
            EXPECT_EQ(accepted, code_error == 0.);
            std::cout << "[held-out-clock-control] common_bias_m=" << system_bias
                      << " injected_code_error_m=" << code_error
                      << " held_out_residual_m=" << residual
                      << " accepted=" << accepted << '\n';
        }
    }
}

TEST(RawPSeedTest, ExplicitlyReportsFilteredClockGroupAsAbsent) {
    const NavigationData nav = makeSyntheticMultiConstellationNavigation();
    auto epochs = makeMultiConstellationTrajectory(nav, 1, 17.0);
    for (auto& observation : epochs[0].observations) {
        if (observation.satellite.system == GNSSSystem::Galileo) {
            observation.snr = -1.0;
        }
    }
    auto config = syntheticConfig();
    config.spp_config.model_intersystem_bias = true;
    config.derive_velocity = false;
    const auto result = libgnss::raw_p_seed::solve(
        epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    EXPECT_EQ(epoch.raw_clock_groups, 2U);
    EXPECT_EQ(epoch.corrected_clock_groups, 1U);
    EXPECT_EQ(epoch.reference_clock_group, GNSSSystem::GPS);
    EXPECT_EQ(epoch.geometry_rank.required_rank, 4);
    EXPECT_EQ(epoch.corrected_pseudorange_rows, 4U);
    const auto galileo = std::find_if(
        epoch.clock_group_biases.begin(), epoch.clock_group_biases.end(),
        [](const auto& entry) { return entry.group == GNSSSystem::Galileo; });
    ASSERT_NE(galileo, epoch.clock_group_biases.end());
    EXPECT_FALSE(galileo->observed);
    EXPECT_FALSE(galileo->estimate_available);
    EXPECT_TRUE(std::isnan(galileo->bias_m));
}

TEST(RawPSeedTest, NativeQcIdentityTracksPerObservationOutlier) {
    // A single Galileo observation is a gross outlier while the remaining
    // Galileo observations retain a legitimate inter-system bias.  Native
    // residual QC should remove only the bad row; the raw-P report must use
    // exact source identity rather than satellite/group counts.
    const NavigationData nav = makeSyntheticMultiConstellationNavigation();
    auto epochs = makeMultiConstellationTrajectory(nav, 1, 17.0);
    for (auto& observation : epochs.front().observations) {
        if (observation.satellite.system == GNSSSystem::Galileo) {
            observation.pseudorange += 1000.0;
            break;
        }
    }
    auto config = syntheticConfig();
    config.spp_config.enable_outlier_detection = true;
    config.spp_config.enable_raim_fde = true;
    config.spp_config.model_intersystem_bias = true;
    config.derive_velocity = false;
    const auto result = libgnss::raw_p_seed::solve(
        epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    EXPECT_EQ(epoch.status, libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_EQ(epoch.raw_clock_groups, 2U);
    EXPECT_EQ(epoch.corrected_clock_groups, 2U);
    EXPECT_EQ(epoch.reference_clock_group, GNSSSystem::GPS);
    EXPECT_EQ(epoch.native_used_pseudorange_rows,
              epoch.geometry_rank.rows);
    EXPECT_LT(epoch.native_used_pseudorange_rows,
              epoch.corrected_pseudorange_rows);
    EXPECT_EQ(epoch.geometry_rank.required_rank, 5);

    const auto galileo = std::find_if(
        epoch.clock_group_biases.begin(), epoch.clock_group_biases.end(),
        [](const auto& entry) { return entry.group == GNSSSystem::Galileo; });
    ASSERT_NE(galileo, epoch.clock_group_biases.end());
    EXPECT_TRUE(galileo->observed);
    EXPECT_TRUE(galileo->estimate_available);
}

TEST(RawPSeedTest, RejectsUnknownClockGroupBeforeNativeSolve) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 2, false);
    epochs[0].observations[0].satellite.system = GNSSSystem::UNKNOWN;
    const auto result = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::UnsupportedClockGroups);
    EXPECT_EQ(result.failure_reason, "unknown-or-unsupported-raw-clock-group");
    EXPECT_EQ(result.epochs[0].raw_clock_groups, 2U);
}

TEST(RawPSeedTest, RejectsKnownButUnsupportedNativeClockGroup) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 2, false);
    epochs[0].observations[0].satellite.system = GNSSSystem::NavIC;
    const auto result = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::UnsupportedClockGroups);
    EXPECT_EQ(result.failure_reason, "unknown-or-unsupported-raw-clock-group");
    EXPECT_EQ(result.epochs[0].raw_clock_groups, 2U);
}

TEST(RawPSeedTest, MultiClockRankIncludesObservedBiasColumn) {
    const std::vector<Vector3d> collinear = {
        Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0),
        Vector3d(3.0, 0.0, 0.0), Vector3d(4.0, 0.0, 0.0),
        Vector3d(5.0, 0.0, 0.0)};
    const std::vector<GNSSSystem> groups = {
        GNSSSystem::GPS, GNSSSystem::GPS, GNSSSystem::GPS,
        GNSSSystem::GPS, GNSSSystem::Galileo};
    const auto rank = libgnss::raw_p_seed::assessGeometryRank(
        collinear, Vector3d::Zero(), groups, GNSSSystem::GPS, true);

    EXPECT_EQ(rank.required_rank, 5);
    EXPECT_EQ(rank.status, libgnss::raw_p_seed::RankStatus::RankDeficient);
    EXPECT_LT(rank.rank, rank.required_rank);
    EXPECT_EQ(rank.rows, 5U);
}

TEST(RawPSeedTest, RejectsNonfiniteGeometryInsteadOfFillingIt) {
    const std::vector<Vector3d> positions = {
        Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0),
        Vector3d(0.0, 0.0, 1.0),
        Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)};
    const auto rank = libgnss::raw_p_seed::assessGeometryRank(
        positions, Vector3d::Zero());

    EXPECT_EQ(rank.status,
              libgnss::raw_p_seed::RankStatus::NumericallyInvalid);
}

TEST(RawPSeedTest, RejectsDuplicateNonmonotonicAndOverGapTimesBeforeSolving) {
    const NavigationData nav = makeSyntheticNavigation();
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);

    auto duplicate = makeTrajectory(nav, 2, false);
    duplicate[1].time = duplicate[0].time;
    auto duplicate_result = libgnss::raw_p_seed::solve(
        duplicate, nav, syntheticConfig());
    EXPECT_FALSE(duplicate_result.ok);
    EXPECT_EQ(duplicate_result.failure_status,
              libgnss::raw_p_seed::EpochStatus::DuplicateTime);

    auto nonmonotonic = makeTrajectory(nav, 2, false);
    nonmonotonic[1].time = GNSSTime(2300, 100099.0);
    auto nonmonotonic_result = libgnss::raw_p_seed::solve(
        nonmonotonic, nav, syntheticConfig());
    EXPECT_FALSE(nonmonotonic_result.ok);
    EXPECT_EQ(nonmonotonic_result.failure_status,
              libgnss::raw_p_seed::EpochStatus::NonmonotonicTime);

    auto gap = makeTrajectory(nav, 2, false);
    gap[1].time = GNSSTime(2300, 100103.0);
    auto gap_result = libgnss::raw_p_seed::solve(gap, nav, syntheticConfig());
    EXPECT_FALSE(gap_result.ok);
    EXPECT_EQ(gap_result.failure_status,
              libgnss::raw_p_seed::EpochStatus::TimeGap);

    auto nonfinite = makeTrajectory(nav, 2, false);
    nonfinite[1].time.tow = std::numeric_limits<double>::quiet_NaN();
    auto nonfinite_result = libgnss::raw_p_seed::solve(
        nonfinite, nav, syntheticConfig());
    EXPECT_FALSE(nonfinite_result.ok);
    EXPECT_EQ(nonfinite_result.failure_status,
              libgnss::raw_p_seed::EpochStatus::NonfiniteTime);

    // Keep the helper's receiver construction visible to this test: its
    // observations are generated from a real same-run synthetic P model, not
    // from a persisted coordinate/trajectory file.
    EXPECT_TRUE(receiver.allFinite());
}

TEST(RawPSeedTest, AccountsForInvalidRowsAlongsideAcceptedRows) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 1, false);
    Observation invalid_observation(
        libgnss::SatelliteId(GNSSSystem::GPS, 5), SignalType::GPS_L1CA);
    invalid_observation.pseudorange = 22'000'000.0;
    invalid_observation.has_pseudorange = true;
    invalid_observation.valid = false;
    invalid_observation.snr = 45.0;
    epochs.front().addObservation(invalid_observation);

    auto config = syntheticConfig();
    config.derive_velocity = false;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    expectPreprocessConservation(epoch);
    EXPECT_EQ(epoch.preprocessing_input_rows, 5U);
    EXPECT_EQ(epoch.preprocessing_accepted_rows, 4U);
    EXPECT_EQ(epoch.preprocessing_rejected_rows, 1U);
    const auto invalid = std::find_if(
        epoch.preprocessing_rows.begin(), epoch.preprocessing_rows.end(),
        [](const auto& row) { return row.input_row_index == 4U; });
    ASSERT_NE(invalid, epoch.preprocessing_rows.end());
    EXPECT_FALSE(invalid->accepted);
    EXPECT_EQ(invalid->reason, "invalid-pseudorange");
}

TEST(RawPSeedTest, ReportsMissingEphemerisForEveryValidatedRow) {
    const NavigationData nav;
    const NavigationData source_nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(source_nav, 1, false);
    auto config = syntheticConfig();
    config.derive_velocity = false;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientGeometry);
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    expectPreprocessConservation(epoch);
    EXPECT_EQ(epoch.preprocessing_input_rows, 4U);
    EXPECT_EQ(epoch.preprocessing_accepted_rows, 0U);
    EXPECT_EQ(epoch.preprocessing_rejected_rows, 4U);
    ASSERT_EQ(epoch.preprocessing_reason_counts.size(), 1U);
    EXPECT_EQ(epoch.preprocessing_reason_counts.front().reason,
              "missing-ephemeris");
    EXPECT_EQ(epoch.preprocessing_reason_counts.front().count, 4U);
}

TEST(RawPSeedTest, ReportsCorrectionElevationRejectionWithoutRelaxingMask) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 1, false);
    auto config = syntheticConfig();
    config.derive_velocity = false;
    config.processor_config.elevation_mask = 90.0;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientGeometry);
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& epoch = result.epochs.front();
    expectPreprocessConservation(epoch);
    EXPECT_EQ(epoch.preprocessing_input_rows, 4U);
    EXPECT_EQ(epoch.preprocessing_accepted_rows, 0U);
    EXPECT_EQ(epoch.preprocessing_rejected_rows, 4U);
    ASSERT_EQ(epoch.preprocessing_reason_counts.size(), 1U);
    EXPECT_EQ(epoch.preprocessing_reason_counts.front().reason,
              "below-elevation-mask");
    EXPECT_EQ(epoch.preprocessing_reason_counts.front().count, 4U);
}

TEST(RawPSeedTest, BootstrapRecoversUnknownOriginBeforeNormalElevationMask) {
    const NavigationData nav = makeSyntheticGpsNavigation(8);
    const Vector3d receiver(-2761814.335408735, 1594534.25,
                            -5523628.670817467);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < 2; ++i) {
        epochs.push_back(makeGpsEpoch(
            nav,
            GNSSTime(2300, 100100.0 + static_cast<double>(i)),
            receiver,
            30.0,
            8));
        epochs.back().receiver_position = Vector3d::Zero();
    }

    auto config = syntheticConfig();
    config.processor_config.elevation_mask = 0.0;
    config.bootstrap_position_before_elevation = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 2U);
    for (const auto& epoch : result.epochs) {
        EXPECT_EQ(epoch.status, libgnss::raw_p_seed::EpochStatus::Accepted);
        EXPECT_TRUE(epoch.position_ecef.allFinite());
        EXPECT_TRUE(std::isfinite(epoch.receiver_clock_bias_m));
        EXPECT_GE(epoch.preprocessing_accepted_rows, 4U);
        EXPECT_EQ(epoch.corrected_pseudorange_rows,
                  epoch.preprocessing_accepted_rows);
        expectPreprocessConservation(epoch);
    }
}

TEST(RawPSeedTest, BootstrapIsRotationInvariantAndKeepsClockGroups) {
    constexpr double kRotationRad = 1.1;
    const NavigationData source_nav = makeSyntheticMultiConstellationNavigation();
    const NavigationData nav = rotateNavigationAroundZ(source_nav, kRotationRad);
    // The inverse-rotated point maps to lat=50, lon=-50 after the test's
    // navigation/receiver rotation; all eight synthetic rows are then
    // physically above the ordinary zero-degree elevation mask.
    const Vector3d source_receiver(-1603584.622898353,
                                   -3773164.902512220,
                                   4885936.406301549);
    const Vector3d receiver = rotateAroundZ(source_receiver, kRotationRad);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < 2; ++i) {
        epochs.push_back(makeMultiConstellationEpoch(
            nav,
            GNSSTime(2300, 100100.0 + static_cast<double>(i)),
            receiver,
            30.0 + 0.75 * static_cast<double>(i),
            17.0,
            false));
        // Exercise the same raw-P geometry under a rotated ECEF orientation,
        // with no location imported as the initial seed.
        epochs.back().receiver_position = Vector3d::Zero();
    }

    auto config = syntheticConfig();
    config.processor_config.elevation_mask = 0.0;
    config.spp_config.model_intersystem_bias = true;
    config.bootstrap_position_before_elevation = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_TRUE(result.ok) << result.failure_reason;
    ASSERT_EQ(result.epochs.size(), 2U);
    for (const auto& epoch : result.epochs) {
        EXPECT_EQ(epoch.corrected_clock_groups, 2U);
        EXPECT_EQ(epoch.geometry_rank.required_rank, 5);
        EXPECT_EQ(epoch.geometry_rank.status,
                  libgnss::raw_p_seed::RankStatus::FullRank);
        EXPECT_GE(epoch.preprocessing_accepted_rows, 5U);
        expectPreprocessConservation(epoch);
    }
}

TEST(RawPSeedTest, BootstrapRemainsDefaultOff) {
    const NavigationData nav = makeSyntheticGpsNavigation(8);
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < 2; ++i) {
        epochs.push_back(makeGpsEpoch(
            nav,
            GNSSTime(2300, 100100.0 + static_cast<double>(i)),
            receiver,
            30.0,
            8));
        epochs.back().receiver_position = Vector3d::Zero();
    }

    auto config = syntheticConfig();
    config.processor_config.elevation_mask = 0.0;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientGeometry);
}

TEST(RawPSeedTest, BootstrapPreservesNormalMaskAndRejectsInsufficientVisibility) {
    const NavigationData nav = makeSyntheticGpsNavigation(8);
    // This declared receiver has valid raw geometry, but fewer than four
    // rows above the configured horizon.  Bootstrap must not turn its
    // private no-mask pass into a normal-pass mask bypass.
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    std::vector<ObservationData> epochs;
    for (std::size_t i = 0; i < 2; ++i) {
        epochs.push_back(makeGpsEpoch(
            nav,
            GNSSTime(2300, 100100.0 + static_cast<double>(i)),
            receiver,
            30.0,
            8));
        epochs.back().receiver_position = Vector3d::Zero();
    }

    auto config = syntheticConfig();
    config.processor_config.elevation_mask = 0.0;
    config.bootstrap_position_before_elevation = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientGeometry);
    ASSERT_FALSE(result.epochs.empty());
    const auto& epoch = result.epochs.front();
    expectPreprocessConservation(epoch);
    EXPECT_LT(epoch.preprocessing_accepted_rows, 4U);
    EXPECT_GT(epoch.preprocessing_rejected_rows, 0U);
    const auto below_mask = std::find_if(
        epoch.preprocessing_reason_counts.begin(),
        epoch.preprocessing_reason_counts.end(),
        [](const auto& entry) { return entry.reason == "below-elevation-mask"; });
    ASSERT_NE(below_mask, epoch.preprocessing_reason_counts.end());
    EXPECT_GT(below_mask->count, 0U);
}

TEST(RawPSeedTest, CollectAllEvaluatesValidInvalidValidWithoutFillingSeeds) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 3, false);
    epochs[1].observations.pop_back();

    auto config = syntheticConfig();
    config.collect_all_epochs_for_diagnostics = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::InsufficientPseudorange);
    EXPECT_EQ(result.input_epoch_count, 3U);
    EXPECT_EQ(result.evaluated_epoch_count, 3U);
    EXPECT_EQ(result.accepted_epoch_count, 2U);
    EXPECT_EQ(result.rejected_epoch_count, 1U);
    EXPECT_TRUE(result.collect_all_epochs_for_diagnostics);
    EXPECT_TRUE(result.velocity_diagnostics_disabled);
    ASSERT_EQ(result.epochs.size(), 3U);

    EXPECT_EQ(result.epochs[0].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_EQ(result.epochs[2].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_TRUE(result.epochs[0].position_ecef.allFinite());
    EXPECT_TRUE(result.epochs[2].position_ecef.allFinite());
    EXPECT_TRUE(result.epochs[0].native_spp_status_available);
    EXPECT_TRUE(result.epochs[2].native_spp_status_available);
    EXPECT_EQ(result.epochs[0].native_spp_status, libgnss::SolutionStatus::SPP);
    EXPECT_EQ(result.epochs[2].native_spp_status, libgnss::SolutionStatus::SPP);

    const auto& rejected = result.epochs[1];
    EXPECT_EQ(rejected.status,
              libgnss::raw_p_seed::EpochStatus::InsufficientPseudorange);
    EXPECT_EQ(rejected.reason, "insufficient-pseudorange-satellites");
    EXPECT_FALSE(rejected.position_ecef.allFinite());
    EXPECT_FALSE(rejected.native_spp_status_available);
    for (const auto& epoch : result.epochs) {
        EXPECT_FALSE(epoch.has_velocity);
        EXPECT_FALSE(epoch.velocity_ecef_mps.allFinite());
    }
}

TEST(RawPSeedTest, CollectAllKeepsClockGroupsAndContinuesAfterGroupFailure) {
    const NavigationData nav = makeSyntheticMultiConstellationNavigation();
    auto epochs = makeMultiConstellationTrajectory(nav, 3, 17.0);
    Observation unsupported = epochs[1].observations.front();
    unsupported.satellite.system = GNSSSystem::SBAS;
    epochs[1].addObservation(unsupported);

    auto config = syntheticConfig();
    config.spp_config.model_intersystem_bias = true;
    config.collect_all_epochs_for_diagnostics = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::UnsupportedClockGroups);
    EXPECT_EQ(result.input_epoch_count, 3U);
    EXPECT_EQ(result.evaluated_epoch_count, 3U);
    EXPECT_EQ(result.accepted_epoch_count, 2U);
    EXPECT_EQ(result.rejected_epoch_count, 1U);
    ASSERT_EQ(result.epochs.size(), 3U);
    EXPECT_EQ(result.epochs[1].status,
              libgnss::raw_p_seed::EpochStatus::UnsupportedClockGroups);
    EXPECT_EQ(result.epochs[1].reason,
              "unknown-or-unsupported-raw-clock-group");
    EXPECT_EQ(result.epochs[0].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_EQ(result.epochs[2].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    for (const std::size_t index : {0U, 2U}) {
        EXPECT_EQ(result.epochs[index].corrected_clock_groups, 2U);
        EXPECT_EQ(result.epochs[index].reference_clock_group, GNSSSystem::GPS);
        EXPECT_EQ(result.epochs[index].geometry_rank.required_rank, 5);
        EXPECT_EQ(result.epochs[index].geometry_rank.status,
                  libgnss::raw_p_seed::RankStatus::FullRank);
        EXPECT_TRUE(result.epochs[index].position_ecef.allFinite());
    }
    EXPECT_FALSE(result.epochs[1].position_ecef.allFinite());
}

TEST(RawPSeedTest, CollectAllEvaluatesAfterTimestampFailure) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 3, false);
    epochs[1].time = epochs[0].time;

    auto config = syntheticConfig();
    config.collect_all_epochs_for_diagnostics = true;
    const auto result = libgnss::raw_p_seed::solve(epochs, nav, config);

    ASSERT_FALSE(result.ok);
    EXPECT_EQ(result.failure_status,
              libgnss::raw_p_seed::EpochStatus::DuplicateTime);
    EXPECT_EQ(result.input_epoch_count, 3U);
    EXPECT_EQ(result.evaluated_epoch_count, 3U);
    EXPECT_EQ(result.accepted_epoch_count, 2U);
    EXPECT_EQ(result.rejected_epoch_count, 1U);
    EXPECT_EQ(result.epochs[0].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_EQ(result.epochs[1].status,
              libgnss::raw_p_seed::EpochStatus::DuplicateTime);
    EXPECT_EQ(result.epochs[2].status,
              libgnss::raw_p_seed::EpochStatus::Accepted);
    EXPECT_FALSE(result.epochs[1].native_spp_status_available);
    EXPECT_TRUE(result.epochs[2].position_ecef.allFinite());
}

TEST(RawPSeedTest, CollectAllMatchesOrdinaryAcceptedSolvesWhenAllValid) {
    const NavigationData nav = makeSyntheticNavigation();
    const auto epochs = makeTrajectory(nav, 3, true);
    const auto ordinary = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());
    auto collect_config = syntheticConfig();
    collect_config.collect_all_epochs_for_diagnostics = true;
    const auto collected = libgnss::raw_p_seed::solve(
        epochs, nav, collect_config);

    ASSERT_TRUE(ordinary.ok) << ordinary.failure_reason;
    ASSERT_TRUE(collected.ok) << collected.failure_reason;
    EXPECT_EQ(collected.input_epoch_count, 3U);
    EXPECT_EQ(collected.evaluated_epoch_count, 3U);
    EXPECT_EQ(collected.accepted_epoch_count, 3U);
    EXPECT_EQ(collected.rejected_epoch_count, 0U);
    ASSERT_EQ(ordinary.epochs.size(), collected.epochs.size());
    for (std::size_t i = 0; i < ordinary.epochs.size(); ++i) {
        EXPECT_EQ(collected.epochs[i].status,
                  libgnss::raw_p_seed::EpochStatus::Accepted);
        EXPECT_TRUE(collected.epochs[i].position_ecef.isApprox(
            ordinary.epochs[i].position_ecef, 1e-9));
        EXPECT_NEAR(collected.epochs[i].receiver_clock_bias_m,
                    ordinary.epochs[i].receiver_clock_bias_m,
                    1e-6);
        EXPECT_FALSE(collected.epochs[i].has_velocity);
    }
}

TEST(RawPSeedAdapterTest, AcceptsSameRunFiniteClockRateAndCertifiesGpsC0) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 3, true);
    for (std::size_t i = 0; i < epochs.size(); ++i) {
        epochs[i].receiver_clock_drift_mps = 0.25 + 0.1 * i;
    }
    const auto raw_result = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());
    ASSERT_TRUE(raw_result.ok) << raw_result.failure_reason;

    const auto adapted =
        libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(epochs, raw_result);
    ASSERT_TRUE(adapted.ok) << adapted.failure_reason;
    EXPECT_TRUE(adapted.graph_compatible);
    EXPECT_EQ(adapted.status,
              libgnss::raw_p_seed::SeedAdapterStatus::Accepted);
    ASSERT_EQ(adapted.seeds.size(), epochs.size());
    for (std::size_t i = 0; i < adapted.seeds.size(); ++i) {
        const auto& seed = adapted.seeds[i];
        EXPECT_EQ(seed.status,
                  libgnss::raw_p_seed::SeedAdapterStatus::Accepted);
        EXPECT_TRUE(seed.has_position);
        EXPECT_TRUE(seed.has_velocity);
        EXPECT_TRUE(seed.has_clock);
        EXPECT_TRUE(seed.has_clock_rate);
        EXPECT_DOUBLE_EQ(seed.clock_rate_mps,
                         epochs[i].receiver_clock_drift_mps);
        EXPECT_EQ(seed.reference_clock_group, GNSSSystem::GPS);
        EXPECT_TRUE(seed.c7_clock_mapping_supported);
        EXPECT_TRUE(seed.clock_bias_component_available[0]);
        EXPECT_TRUE(std::isfinite(seed.clock_bias_components_m[0]));
        for (std::size_t component = 1; component < 7; ++component) {
            EXPECT_FALSE(seed.clock_bias_component_available[component]);
            EXPECT_TRUE(std::isnan(seed.clock_bias_components_m[component]));
        }
    }
}

TEST(RawPSeedAdapterTest, RejectsMissingNonfiniteAndMisalignedRawClockRate) {
    const NavigationData nav = makeSyntheticNavigation();

    auto missing_epochs = makeTrajectory(nav, 2, true);
    const auto missing_raw = libgnss::raw_p_seed::solve(
        missing_epochs, nav, syntheticConfig());
    ASSERT_TRUE(missing_raw.ok) << missing_raw.failure_reason;
    const auto missing = libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(
        missing_epochs, missing_raw);
    EXPECT_FALSE(missing.ok);
    EXPECT_EQ(missing.status,
              libgnss::raw_p_seed::SeedAdapterStatus::MissingRawClockDrift);
    EXPECT_EQ(missing.seeds[0].status,
              libgnss::raw_p_seed::SeedAdapterStatus::MissingRawClockDrift);

    auto nonfinite_epochs = makeTrajectory(nav, 2, true);
    nonfinite_epochs[0].receiver_clock_drift_mps =
        std::numeric_limits<double>::infinity();
    nonfinite_epochs[1].receiver_clock_drift_mps = 0.0;
    const auto nonfinite_raw = libgnss::raw_p_seed::solve(
        nonfinite_epochs, nav, syntheticConfig());
    ASSERT_TRUE(nonfinite_raw.ok) << nonfinite_raw.failure_reason;
    const auto nonfinite = libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(
        nonfinite_epochs, nonfinite_raw);
    EXPECT_FALSE(nonfinite.ok);
    EXPECT_EQ(nonfinite.status,
              libgnss::raw_p_seed::SeedAdapterStatus::NonfiniteRawClockDrift);

    auto misaligned_epochs = makeTrajectory(nav, 2, true);
    misaligned_epochs[0].receiver_clock_drift_mps = 0.0;
    misaligned_epochs[1].receiver_clock_drift_mps = 0.0;
    auto misaligned_raw = libgnss::raw_p_seed::solve(
        misaligned_epochs, nav, syntheticConfig());
    ASSERT_TRUE(misaligned_raw.ok) << misaligned_raw.failure_reason;
    misaligned_raw.epochs[1].time.tow += 1.0;
    const auto misaligned =
        libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(
            misaligned_epochs, misaligned_raw);
    EXPECT_FALSE(misaligned.ok);
    EXPECT_EQ(misaligned.status,
              libgnss::raw_p_seed::SeedAdapterStatus::EpochIdentityMismatch);
}

TEST(RawPSeedAdapterTest, KeepsGpsC0WithSupportedMixedClockRows) {
    const NavigationData nav = makeSyntheticMultiConstellationNavigation();
    auto epochs = makeMultiConstellationTrajectory(nav, 2, 17.0);
    for (auto& epoch : epochs) epoch.receiver_clock_drift_mps = 0.0;
    auto config = syntheticConfig();
    config.spp_config.model_intersystem_bias = true;
    const auto raw_result = libgnss::raw_p_seed::solve(epochs, nav, config);
    ASSERT_TRUE(raw_result.ok) << raw_result.failure_reason;

    const auto adapted =
        libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(epochs, raw_result);
    ASSERT_TRUE(adapted.ok) << adapted.failure_reason;
    EXPECT_TRUE(adapted.graph_compatible);
    EXPECT_TRUE(adapted.graph_disabled_reason.empty());
    for (const auto& seed : adapted.seeds) {
        EXPECT_TRUE(seed.c7_clock_mapping_supported);
        EXPECT_TRUE(seed.clock_bias_component_available[0]);
        EXPECT_TRUE(std::isfinite(seed.clock_bias_components_m[0]));
        EXPECT_GT(seed.c7_supported_pseudorange_rows, 0U);
        EXPECT_EQ(seed.c7_unsupported_pseudorange_rows, 0U);
        EXPECT_TRUE(seed.has_position);
        EXPECT_TRUE(seed.has_velocity);
        EXPECT_TRUE(seed.has_clock_rate);
    }
}

TEST(RawPSeedAdapterTest, AccountsUnsupportedC7SignalWithoutFoldingIntoC0) {
    const NavigationData nav = makeSyntheticNavigation();
    auto epochs = makeTrajectory(nav, 2, true);
    for (auto& epoch : epochs) epoch.receiver_clock_drift_mps = 0.0;
    const auto raw_result = libgnss::raw_p_seed::solve(
        epochs, nav, syntheticConfig());
    ASSERT_TRUE(raw_result.ok) << raw_result.failure_reason;

    epochs[0].observations.front().signal = SignalType::GPS_L2C;
    const auto adapted =
        libgnss::raw_p_seed::adaptSameRunNoDopplerSeeds(epochs, raw_result);
    ASSERT_TRUE(adapted.ok) << adapted.failure_reason;
    EXPECT_TRUE(adapted.graph_compatible);
    EXPECT_TRUE(adapted.graph_disabled_reason.empty());
    EXPECT_TRUE(adapted.seeds[0].c7_clock_mapping_supported);
    EXPECT_EQ(adapted.seeds[0].c7_unsupported_pseudorange_rows, 1U);
    EXPECT_TRUE(adapted.seeds[0].clock_bias_component_available[0]);
    EXPECT_TRUE(std::isfinite(adapted.seeds[0].clock_bias_components_m[0]));
}
