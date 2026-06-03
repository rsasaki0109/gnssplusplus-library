#include <gtest/gtest.h>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>

#include <array>
#include <cmath>
#include <map>
#include <vector>

using namespace libgnss;

namespace {

std::vector<Vector3d> makeSatelliteGeometry() {
    return {
        Vector3d(15600000.0, 7540000.0, 20140000.0),
        Vector3d(-18760000.0, 2750000.0, 18610000.0),
        Vector3d(17610000.0, -14630000.0, 13480000.0),
        Vector3d(19170000.0, 610000.0, -18390000.0),
        Vector3d(-13480000.0, -15600000.0, 17760000.0),
        Vector3d(21700000.0, 13000000.0, 9000000.0),
    };
}

double trueAmbiguityMeters(std::size_t satellite_index) {
    return constants::GPS_L1_WAVELENGTH *
           static_cast<double>(100 + static_cast<int>(satellite_index));
}

int trueAmbiguityCycles(std::size_t satellite_index) {
    return 100 + static_cast<int>(satellite_index);
}

template <typename Factor>
double doubleDifferenceGeometry(const Vector3d& position,
                                const Factor& factor) {
    const double rover_satellite_range =
        (factor.rover_satellite_position_ecef - position).norm();
    const double rover_reference_range =
        (factor.rover_reference_position_ecef - position).norm();
    const double base_satellite_range =
        (factor.base_satellite_position_ecef -
         factor.base_position_ecef)
            .norm();
    const double base_reference_range =
        (factor.base_reference_position_ecef -
         factor.base_position_ecef)
            .norm();
    return (rover_satellite_range - base_satellite_range) -
           (rover_reference_range - base_reference_range);
}

template <typename Factor>
Vector3d doubleDifferencePositionJacobian(const Vector3d& position,
                                          const Factor& factor) {
    const Vector3d rover_satellite_delta =
        factor.rover_satellite_position_ecef - position;
    const Vector3d rover_reference_delta =
        factor.rover_reference_position_ecef - position;
    const Vector3d rover_satellite_los =
        rover_satellite_delta / rover_satellite_delta.norm();
    const Vector3d rover_reference_los =
        rover_reference_delta / rover_reference_delta.norm();
    return -rover_satellite_los + rover_reference_los;
}

FGOProcessor::FGOProblem makeSyntheticProblem(bool include_tdcp = false,
                                              bool include_carrier_phase = false) {
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const std::array<double, 2> true_clock_bias_m = {42.0, 43.5};

    FGOProcessor::FGOProblem problem;
    const auto satellites = makeSatelliteGeometry();
    std::vector<std::size_t> ambiguity_indices(satellites.size(), 0);

    if (include_carrier_phase) {
        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            FGOProcessor::AmbiguityState ambiguity;
            ambiguity.satellite =
                SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            ambiguity.signal = SignalType::GPS_L1CA;
            ambiguity.segment_index = 0;
            ambiguity.wavelength_m = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;
            ambiguity.initial_ambiguity_m = trueAmbiguityMeters(sat) - 4.0;
            ambiguity_indices[sat] = problem.ambiguity_states.size();
            problem.ambiguity_states.push_back(ambiguity);
        }
    }

    for (std::size_t epoch = 0; epoch < true_positions.size(); ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + static_cast<double>(epoch));
        seed.position_ecef = true_positions[epoch] + Vector3d(35.0, -18.0, 12.0);
        seed.receiver_clock_bias_m = true_clock_bias_m[epoch] - 25.0;
        problem.epochs.push_back(seed);

        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            FGOProcessor::PseudorangeFactor factor;
            factor.epoch_index = epoch;
            factor.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            factor.satellite_position_ecef = satellites[sat];
            factor.corrected_pseudorange_m =
                (satellites[sat] - true_positions[epoch]).norm() +
                true_clock_bias_m[epoch];
            factor.sigma_m = 1.0;
            factor.elevation_rad = 0.7;
            problem.pseudorange_factors.push_back(factor);

            if (include_carrier_phase) {
                FGOProcessor::CarrierPhaseFactor carrier_factor;
                carrier_factor.epoch_index = epoch;
                carrier_factor.ambiguity_index = ambiguity_indices[sat];
                carrier_factor.satellite = factor.satellite;
                carrier_factor.signal = SignalType::GPS_L1CA;
                carrier_factor.satellite_position_ecef = satellites[sat];
                carrier_factor.corrected_carrier_m =
                    (satellites[sat] - true_positions[epoch]).norm() +
                    true_clock_bias_m[epoch] +
                    trueAmbiguityMeters(sat);
                carrier_factor.sigma_m = 0.01;
                carrier_factor.elevation_rad = 0.7;
                problem.carrier_phase_factors.push_back(carrier_factor);
            }
        }
    }

    if (include_tdcp) {
        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            FGOProcessor::TimeDifferencedCarrierFactor factor;
            factor.previous_epoch_index = 0;
            factor.current_epoch_index = 1;
            factor.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            factor.signal = SignalType::GPS_L1CA;
            factor.previous_satellite_position_ecef = satellites[sat];
            factor.current_satellite_position_ecef = satellites[sat];
            factor.delta_carrier_m =
                ((satellites[sat] - true_positions[1]).norm() + true_clock_bias_m[1]) -
                ((satellites[sat] - true_positions[0]).norm() + true_clock_bias_m[0]);
            factor.sigma_m = 0.02;
            factor.dt_s = 1.0;
            problem.tdcp_factors.push_back(factor);
        }
        problem.diagnostics.tdcp_candidate_pairs = satellites.size();
    }

    problem.diagnostics.input_epochs = true_positions.size();
    problem.diagnostics.seeded_epochs = true_positions.size();
    return problem;
}

FGOProcessor::FGOProblem makeSyntheticDoubleDifferenceProblem(
    bool include_pseudorange_dd = false) {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem(false, true);
    const auto satellites = makeSatelliteGeometry();
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const Vector3d base_position =
        true_positions[0] + Vector3d(-320.0, 180.0, 45.0);

    for (std::size_t epoch = 0; epoch < true_positions.size(); ++epoch) {
        for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
            if (include_pseudorange_dd) {
                FGOProcessor::DoubleDifferencePseudorangeFactor code_factor;
                code_factor.epoch_index = epoch;
                code_factor.satellite =
                    SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
                code_factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
                code_factor.signal = SignalType::GPS_L1CA;
                code_factor.rover_satellite_position_ecef = satellites[sat];
                code_factor.rover_reference_position_ecef = satellites[0];
                code_factor.base_satellite_position_ecef = satellites[sat];
                code_factor.base_reference_position_ecef = satellites[0];
                code_factor.base_position_ecef = base_position;
                code_factor.observed_dd_pseudorange_m =
                    ((satellites[sat] - true_positions[epoch]).norm() -
                     (satellites[sat] - base_position).norm()) -
                    ((satellites[0] - true_positions[epoch]).norm() -
                     (satellites[0] - base_position).norm());
                code_factor.sigma_m = 0.5;
                code_factor.elevation_rad = 0.7;
                problem.double_difference_pseudorange_factors.push_back(
                    code_factor);
            }

            FGOProcessor::DoubleDifferenceCarrierFactor factor;
            factor.epoch_index = epoch;
            factor.ambiguity_index = sat;
            factor.reference_ambiguity_index = 0;
            factor.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
            factor.signal = SignalType::GPS_L1CA;
            factor.rover_satellite_position_ecef = satellites[sat];
            factor.rover_reference_position_ecef = satellites[0];
            factor.base_satellite_position_ecef = satellites[sat];
            factor.base_reference_position_ecef = satellites[0];
            factor.base_position_ecef = base_position;
            factor.observed_dd_carrier_m =
                ((satellites[sat] - true_positions[epoch]).norm() -
                 (satellites[sat] - base_position).norm()) -
                ((satellites[0] - true_positions[epoch]).norm() -
                 (satellites[0] - base_position).norm()) +
                trueAmbiguityMeters(sat) - trueAmbiguityMeters(0);
            factor.sigma_m = 0.01;
            factor.elevation_rad = 0.7;
            problem.double_difference_carrier_factors.push_back(factor);
        }
    }

    problem.carrier_phase_factors.clear();
    problem.diagnostics.double_difference_matched_base_epochs = true_positions.size();
    problem.diagnostics.double_difference_candidate_pairs =
        problem.double_difference_carrier_factors.size();
    return problem;
}

FGOProcessor::FGOProblem makeSyntheticSingleDifferenceProblem() {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem();
    const auto satellites = makeSatelliteGeometry();
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const double dt = problem.epochs[1].time - problem.epochs[0].time;
    const Vector3d true_velocity =
        (true_positions[1] - true_positions[0]) / dt;

    auto sd_los = [&](std::size_t epoch, std::size_t sat) -> Vector3d {
        const Vector3d sat_delta =
            satellites[sat] - problem.epochs[epoch].position_ecef;
        const Vector3d ref_delta =
            satellites[0] - problem.epochs[epoch].position_ecef;
        return -sat_delta / sat_delta.norm() + ref_delta / ref_delta.norm();
    };

    for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
        const Vector3d previous_los = sd_los(0, sat);
        const Vector3d current_los = sd_los(1, sat);

        FGOProcessor::SingleDifferenceDopplerFactor doppler_factor;
        doppler_factor.epoch_index = 1;
        doppler_factor.satellite =
            SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
        doppler_factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
        doppler_factor.signal = SignalType::GPS_L1CA;
        doppler_factor.los = current_los;
        doppler_factor.residual_mps = current_los.dot(true_velocity);
        doppler_factor.sigma_mps = 0.2;
        doppler_factor.elevation_rad = 0.7;
        problem.single_difference_doppler_factors.push_back(doppler_factor);

        FGOProcessor::SingleDifferenceTdcpFactor tdcp_factor;
        tdcp_factor.previous_epoch_index = 0;
        tdcp_factor.current_epoch_index = 1;
        tdcp_factor.satellite = doppler_factor.satellite;
        tdcp_factor.reference_satellite = doppler_factor.reference_satellite;
        tdcp_factor.signal = SignalType::GPS_L1CA;
        tdcp_factor.previous_los = previous_los;
        tdcp_factor.los = current_los;
        tdcp_factor.delta_carrier_m =
            current_los.dot(true_positions[1] -
                            problem.epochs[1].position_ecef) -
            previous_los.dot(true_positions[0] -
                             problem.epochs[0].position_ecef);
        tdcp_factor.sigma_m = 0.003;
        tdcp_factor.elevation_rad = 0.7;
        problem.single_difference_tdcp_factors.push_back(tdcp_factor);
    }

    return problem;
}

Ephemeris makeSyntheticGpsEphemeris(uint8_t prn) {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
    eph.valid = true;
    eph.week = 2300;
    eph.toe = GNSSTime(eph.week, 100000.0);
    eph.toc = eph.toe;
    eph.tof = eph.toe;
    eph.toes = eph.toe.tow;
    eph.sqrt_a = std::sqrt(26560000.0);
    eph.e = 0.004 + 0.0002 * static_cast<double>(prn);
    eph.i0 = 0.94 + 0.01 * static_cast<double>(prn % 3);
    eph.omega0 = 0.35 * static_cast<double>(prn);
    eph.omega = 0.17 * static_cast<double>(prn);
    eph.m0 = 0.61 * static_cast<double>(prn);
    eph.delta_n = 1e-9 * static_cast<double>(prn);
    eph.omega_dot = -8.0e-9;
    eph.health = 0;
    return eph;
}

NavigationData makeSyntheticGpsNavigation(std::size_t satellite_count) {
    NavigationData nav;
    for (std::size_t sat = 1; sat <= satellite_count; ++sat) {
        nav.addEphemeris(
            makeSyntheticGpsEphemeris(static_cast<uint8_t>(sat)));
    }
    return nav;
}

bool makeSyntheticGpsL1Observation(const NavigationData& nav,
                                   const SatelliteId& satellite,
                                   const GNSSTime& time,
                                   const Vector3d& receiver_position,
                                   double carrier_bias_m,
                                   Observation& observation) {
    double pseudorange = 22'000'000.0;
    Vector3d satellite_position = Vector3d::Zero();
    Vector3d satellite_velocity = Vector3d::Zero();
    double satellite_clock_bias = 0.0;
    double satellite_clock_drift = 0.0;
    for (int iteration = 0; iteration < 3; ++iteration) {
        const GNSSTime transmit_time =
            time - pseudorange / constants::SPEED_OF_LIGHT;
        if (!nav.calculateSatelliteState(satellite,
                                         transmit_time,
                                         satellite_position,
                                         satellite_velocity,
                                         satellite_clock_bias,
                                         satellite_clock_drift)) {
            return false;
        }
        pseudorange =
            (satellite_position - receiver_position).norm() -
            satellite_clock_bias * constants::SPEED_OF_LIGHT;
    }

    observation = Observation(satellite, SignalType::GPS_L1CA);
    observation.pseudorange = pseudorange;
    observation.carrier_phase =
        (pseudorange + carrier_bias_m) / constants::GPS_L1_WAVELENGTH;
    observation.snr = 45.0;
    observation.has_pseudorange = true;
    observation.has_carrier_phase = true;
    observation.valid = true;
    return true;
}

std::vector<ObservationData> makeSyntheticDoubleDifferenceObservationEpochs(
    const NavigationData& nav,
    const std::array<Vector3d, 2>& receiver_positions,
    double carrier_epoch_slope_m) {
    std::vector<ObservationData> epochs;
    for (std::size_t epoch_index = 0; epoch_index < receiver_positions.size();
         ++epoch_index) {
        ObservationData epoch(GNSSTime(2300, 100100.0 + epoch_index));
        epoch.receiver_position = receiver_positions[epoch_index];
        for (uint8_t prn = 1; prn <= 4; ++prn) {
            Observation observation;
            const double carrier_bias_m =
                10.0 * static_cast<double>(prn) +
                carrier_epoch_slope_m * static_cast<double>(epoch_index) *
                    static_cast<double>(prn);
            if (makeSyntheticGpsL1Observation(
                    nav,
                    SatelliteId(GNSSSystem::GPS, prn),
                    epoch.time,
                    receiver_positions[epoch_index],
                    carrier_bias_m,
                    observation)) {
                epoch.addObservation(observation);
            }
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

FGOProcessor::FGOProblem makeSyntheticInterSystemBiasProblem() {
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const std::array<double, 2> true_clock_bias_m = {42.0, 43.5};
    constexpr double galileo_bias_m = 87.0;

    FGOProcessor::FGOProblem problem;
    const auto satellites = makeSatelliteGeometry();
    for (std::size_t epoch = 0; epoch < true_positions.size(); ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + static_cast<double>(epoch));
        seed.position_ecef =
            true_positions[epoch] + Vector3d(35.0, -18.0, 12.0);
        seed.receiver_clock_bias_m = true_clock_bias_m[epoch] - 25.0;
        problem.epochs.push_back(seed);

        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            const bool is_galileo = sat >= satellites.size() / 2;
            FGOProcessor::PseudorangeFactor factor;
            factor.epoch_index = epoch;
            factor.satellite =
                SatelliteId(is_galileo ? GNSSSystem::Galileo : GNSSSystem::GPS,
                            static_cast<uint8_t>(sat + 1));
            factor.clock_group =
                is_galileo ? GNSSSystem::Galileo : GNSSSystem::GPS;
            factor.satellite_position_ecef = satellites[sat];
            factor.corrected_pseudorange_m =
                (satellites[sat] - true_positions[epoch]).norm() +
                true_clock_bias_m[epoch] +
                (is_galileo ? galileo_bias_m : 0.0);
            factor.sigma_m = 1.0;
            factor.elevation_rad = 0.7;
            problem.pseudorange_factors.push_back(factor);
        }
    }

    problem.diagnostics.input_epochs = true_positions.size();
    problem.diagnostics.seeded_epochs = true_positions.size();
    return problem;
}

}  // namespace

TEST(FGOTest, EmptyProblemProducesNoSolutions) {
    FGOProcessor processor;
    const auto result = processor.optimizeProblem(FGOProcessor::FGOProblem{});

    EXPECT_TRUE(result.solution.isEmpty());
    EXPECT_EQ(result.diagnostics.epochs, 0u);
    EXPECT_EQ(result.diagnostics.pseudorange_factors, 0u);
}

TEST(FGOTest, BatchPseudorangeFactorsRecoverSyntheticTrajectory) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(makeSyntheticProblem());

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.pseudorange_factors, 12u);
    EXPECT_LT(result.diagnostics.residual_rms_m, 1e-5);

    const Vector3d expected_first(1113194.0, -4841695.0, 3985350.0);
    const Vector3d expected_second(1113196.5, -4841694.0, 3985349.4);

    ASSERT_TRUE(result.solution.solutions[0].isValid());
    ASSERT_TRUE(result.solution.solutions[1].isValid());
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::SPP);
    EXPECT_EQ(result.solution.solutions[1].status, SolutionStatus::SPP);
    const auto stats = result.solution.calculateStatistics();
    EXPECT_EQ(stats.float_solutions, 0u);
    EXPECT_EQ(stats.fixed_solutions, 0u);
    EXPECT_LT((result.solution.solutions[0].position_ecef - expected_first).norm(), 1e-3);
    EXPECT_LT((result.solution.solutions[1].position_ecef - expected_second).norm(), 1e-3);
    EXPECT_NEAR(result.solution.solutions[0].receiver_clock_bias,
                42.0 / constants::SPEED_OF_LIGHT,
                1e-12);
    EXPECT_NEAR(result.solution.solutions[1].receiver_clock_bias,
                43.5 / constants::SPEED_OF_LIGHT,
                1e-12);
}

TEST(FGOTest, InterSystemBiasFactorsRecoverMixedConstellationTrajectory) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_inter_system_biases = true;
    FGOProcessor processor(config);

    const auto result =
        processor.optimizeProblem(makeSyntheticInterSystemBiasProblem());

    ASSERT_EQ(result.solution.solutions.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_LT(result.diagnostics.residual_rms_m, 1e-5);
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    for (std::size_t i = 0; i < result.solution.solutions.size(); ++i) {
        EXPECT_LT((result.solution.solutions[i].position_ecef -
                   true_positions[i])
                      .norm(),
                  0.05);
    }
}

TEST(FGOTest, TimeDifferencedCarrierFactorsRecoverSyntheticTrajectory) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.tdcp_sigma_m = 0.02;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(makeSyntheticProblem(true));

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::SPP);
    EXPECT_EQ(result.solution.solutions[1].status, SolutionStatus::SPP);
    const auto stats = result.solution.calculateStatistics();
    EXPECT_EQ(stats.float_solutions, 0u);
    EXPECT_EQ(stats.fixed_solutions, 0u);
    EXPECT_EQ(result.diagnostics.pseudorange_factors, 12u);
    EXPECT_EQ(result.diagnostics.tdcp_factors, 6u);
    EXPECT_EQ(result.diagnostics.tdcp_candidate_pairs, 6u);
    EXPECT_LT(result.diagnostics.residual_rms_m, 1e-5);
    EXPECT_LT(result.diagnostics.tdcp_residual_rms_m, 1e-5);

    const Vector3d expected_second(1113196.5, -4841694.0, 3985349.4);
    EXPECT_LT((result.solution.solutions[1].position_ecef - expected_second).norm(), 1e-3);
}

TEST(FGOTest, SingleDifferenceDopplerAndTdcpFactorsConstrainMotion) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.use_robust_loss = false;

    FGOProcessor processor(config);
    const auto result =
        processor.optimizeProblem(makeSyntheticSingleDifferenceProblem());

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.single_difference_doppler_factors, 5u);
    EXPECT_EQ(result.diagnostics.single_difference_tdcp_factors, 5u);
    EXPECT_LT(result.diagnostics.single_difference_doppler_residual_rms_mps,
              1e-5);
    EXPECT_LT(result.diagnostics.single_difference_tdcp_residual_rms_m,
              1e-5);
}

TEST(FGOTest, BuiltSingleDifferenceTdcpFactorsUseCurrentEpochLos) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> rover_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113214.0, -4841680.0, 3985342.0),
    };
    const std::array<Vector3d, 2> base_positions = {
        rover_positions[0] + Vector3d(-320.0, 180.0, 45.0),
        rover_positions[1] + Vector3d(-320.0, 180.0, 45.0),
    };
    const auto rover_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, rover_positions, 0.04);
    const auto base_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, base_positions, 0.02);

    FGOProcessor::FGOConfig config;
    config.use_spp_seed = false;
    config.use_pseudorange_factors = false;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_double_difference_factors = true;
    config.use_single_difference_tdcp_factors = true;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.reject_tdcp_code_phase_jump = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_positions[0]);

    ASSERT_FALSE(problem.single_difference_tdcp_factors.empty());

    using ObservationKey = std::tuple<std::size_t, SatelliteId, SignalType>;
    std::map<ObservationKey, const FGOProcessor::CarrierPhaseFactor*>
        rover_observations;
    for (const auto& observation :
         problem.double_difference_pseudorange_observations) {
        rover_observations[{observation.epoch_index,
                            observation.satellite,
                            observation.signal}] = &observation;
    }

    double max_previous_current_los_delta = 0.0;
    for (const auto& factor : problem.single_difference_tdcp_factors) {
        const auto current_satellite_it =
            rover_observations.find({factor.current_epoch_index,
                                     factor.satellite,
                                     factor.signal});
        const auto current_reference_it =
            rover_observations.find({factor.current_epoch_index,
                                     factor.reference_satellite,
                                     factor.signal});
        const auto previous_satellite_it =
            rover_observations.find({factor.previous_epoch_index,
                                     factor.satellite,
                                     factor.signal});
        const auto previous_reference_it =
            rover_observations.find({factor.previous_epoch_index,
                                     factor.reference_satellite,
                                     factor.signal});
        ASSERT_NE(current_satellite_it, rover_observations.end());
        ASSERT_NE(current_reference_it, rover_observations.end());
        ASSERT_NE(previous_satellite_it, rover_observations.end());
        ASSERT_NE(previous_reference_it, rover_observations.end());

        const Vector3d current_sd_los =
            current_satellite_it->second->los - current_reference_it->second->los;
        const Vector3d previous_sd_los =
            previous_satellite_it->second->los - previous_reference_it->second->los;

        EXPECT_LT((factor.los - current_sd_los).norm(), 1e-12);
        EXPECT_LT((factor.previous_los - current_sd_los).norm(), 1e-12);
        max_previous_current_los_delta =
            std::max(max_previous_current_los_delta,
                     (previous_sd_los - current_sd_los).norm());
    }

    EXPECT_GT(max_previous_current_los_delta, 1e-8);
}

TEST(FGOTest, RobustLossDownweightsPseudorangeOutlier) {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem();
    for (auto& factor : problem.pseudorange_factors) {
        if (factor.epoch_index == 0 && factor.satellite.prn == 6) {
            factor.corrected_pseudorange_m += 5000.0;
        }
    }

    FGOProcessor::FGOConfig base_config;
    base_config.max_iterations = 20;
    base_config.convergence_threshold_m = 1e-8;
    base_config.use_motion_factors = false;
    base_config.pseudorange_huber_threshold_sigma = 2.0;

    FGOProcessor::FGOConfig plain_config = base_config;
    plain_config.use_robust_loss = false;
    FGOProcessor plain_processor(plain_config);
    const auto plain_result = plain_processor.optimizeProblem(problem);

    FGOProcessor robust_processor(base_config);
    const auto robust_result = robust_processor.optimizeProblem(problem);

    ASSERT_EQ(plain_result.solution.size(), 2u);
    ASSERT_EQ(robust_result.solution.size(), 2u);
    const Vector3d expected_first(1113194.0, -4841695.0, 3985350.0);
    const double plain_error =
        (plain_result.solution.solutions[0].position_ecef - expected_first).norm();
    const double robust_error =
        (robust_result.solution.solutions[0].position_ecef - expected_first).norm();

    EXPECT_GT(plain_error, 100.0);
    EXPECT_LT(robust_error, plain_error * 0.25);
    EXPECT_GT(robust_result.diagnostics.robust_pseudorange_factors, 0u);
    EXPECT_EQ(robust_result.diagnostics.robust_carrier_phase_factors, 0u);
    EXPECT_EQ(robust_result.diagnostics.robust_tdcp_factors, 0u);
}

TEST(FGOTest, CarrierPhaseAmbiguityStatesRecoverSyntheticTrajectory) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.carrier_phase_sigma_m = 0.01;
    config.ambiguity_prior_sigma_m = 1000.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::FLOAT);
    EXPECT_EQ(result.solution.solutions[1].status, SolutionStatus::FLOAT);
    const auto stats = result.solution.calculateStatistics();
    EXPECT_EQ(stats.float_solutions, 2u);
    EXPECT_EQ(stats.fixed_solutions, 0u);
    EXPECT_EQ(result.diagnostics.pseudorange_factors, 12u);
    EXPECT_EQ(result.diagnostics.carrier_phase_factors, 12u);
    EXPECT_EQ(result.diagnostics.ambiguity_states, 6u);
    EXPECT_EQ(result.ambiguity_estimates.size(), 6u);
    EXPECT_LT(result.diagnostics.residual_rms_m, 1e-5);
    EXPECT_LT(result.diagnostics.carrier_phase_residual_rms_m, 1e-5);

    const Vector3d expected_second(1113196.5, -4841694.0, 3985349.4);
    EXPECT_LT((result.solution.solutions[1].position_ecef - expected_second).norm(), 1e-3);

    for (std::size_t sat = 0; sat < result.ambiguity_estimates.size(); ++sat) {
        EXPECT_NEAR(result.ambiguity_estimates[sat].ambiguity_m,
                    trueAmbiguityMeters(sat),
                    1e-3);
    }
}

TEST(FGOTest, DoubleDifferenceCarrierFactorsConstrainAmbiguityDifferences) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.carrier_phase_sigma_m = 0.01;
    config.double_difference_carrier_sigma_m = 0.01;
    config.ambiguity_prior_sigma_m = 1000.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(makeSyntheticDoubleDifferenceProblem());

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.carrier_phase_factors, 0u);
    EXPECT_EQ(result.diagnostics.double_difference_pseudorange_factors, 0u);
    EXPECT_EQ(result.diagnostics.double_difference_carrier_factors, 10u);
    EXPECT_EQ(result.diagnostics.double_difference_matched_base_epochs, 2u);
    EXPECT_EQ(result.diagnostics.double_difference_candidate_pairs, 10u);
    EXPECT_LT(result.diagnostics.double_difference_carrier_residual_rms_m, 1e-5);
    EXPECT_EQ(result.diagnostics.robust_double_difference_carrier_factors, 0u);

    ASSERT_EQ(result.ambiguity_estimates.size(), 6u);
    for (std::size_t sat = 1; sat < result.ambiguity_estimates.size(); ++sat) {
        const double dd_ambiguity =
            result.ambiguity_estimates[sat].ambiguity_m -
            result.ambiguity_estimates[0].ambiguity_m;
        EXPECT_NEAR(dd_ambiguity,
                    trueAmbiguityMeters(sat) - trueAmbiguityMeters(0),
                    1e-3);
    }
}

TEST(FGOTest, DoubleDifferencePseudorangeFactorsRecoverSyntheticGeometry) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.double_difference_pseudorange_sigma_m = 0.5;
    config.double_difference_carrier_sigma_m = 0.01;
    config.ambiguity_prior_sigma_m = 1000.0;

    FGOProcessor processor(config);
    const auto result =
        processor.optimizeProblem(makeSyntheticDoubleDifferenceProblem(true));

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.carrier_phase_factors, 0u);
    EXPECT_EQ(result.diagnostics.double_difference_pseudorange_factors, 10u);
    EXPECT_EQ(result.diagnostics.double_difference_carrier_factors, 10u);
    EXPECT_LT(result.diagnostics.double_difference_pseudorange_residual_rms_m,
              1e-5);
    EXPECT_LT(result.diagnostics.double_difference_carrier_residual_rms_m,
              1e-5);
    EXPECT_EQ(result.diagnostics.robust_double_difference_pseudorange_factors,
              0u);
}

TEST(FGOTest, DoubleDifferenceOutputGateRequiresEnoughCarrierCandidates) {
    FGOProcessor::FGOProblem problem = makeSyntheticDoubleDifferenceProblem();
    problem.pseudorange_factors.clear();

    FGOProcessor::FGOConfig config;
    config.max_iterations = 1;
    config.use_motion_factors = false;
    config.min_output_double_difference_carrier_factors_per_epoch = 6;

    FGOProcessor gated_processor(config);
    const auto gated_result = gated_processor.optimizeProblem(problem);

    ASSERT_EQ(gated_result.solution.size(), 2u);
    EXPECT_EQ(gated_result.diagnostics.double_difference_carrier_factors, 10u);
    EXPECT_EQ(gated_result.solution.solutions[0].num_satellites, 6);
    EXPECT_EQ(gated_result.solution.solutions[0].status, SolutionStatus::NONE);
    EXPECT_EQ(gated_result.solution.solutions[1].status, SolutionStatus::NONE);

    config.min_output_double_difference_carrier_factors_per_epoch = 5;
    FGOProcessor accepted_processor(config);
    const auto accepted_result = accepted_processor.optimizeProblem(problem);

    ASSERT_EQ(accepted_result.solution.size(), 2u);
    EXPECT_EQ(accepted_result.solution.solutions[0].status,
              SolutionStatus::FLOAT);
    EXPECT_EQ(accepted_result.solution.solutions[1].status,
              SolutionStatus::FLOAT);
}

TEST(FGOTest, FloatSeedDivergenceGateRejectsOnlyFloatOutputs) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.max_float_seed_position_divergence_m = 1.0;

    FGOProcessor gated_processor(config);
    const auto gated_result =
        gated_processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(gated_result.solution.size(), 2u);
    EXPECT_EQ(gated_result.solution.solutions[0].status, SolutionStatus::NONE);
    EXPECT_EQ(gated_result.solution.solutions[1].status, SolutionStatus::NONE);
    EXPECT_EQ(gated_result.diagnostics.float_rejected_seed_position_divergence,
              2u);

    config.max_float_seed_position_divergence_m = 100.0;
    FGOProcessor accepted_processor(config);
    const auto accepted_result =
        accepted_processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(accepted_result.solution.size(), 2u);
    EXPECT_EQ(accepted_result.solution.solutions[0].status,
              SolutionStatus::FLOAT);
    EXPECT_EQ(accepted_result.solution.solutions[1].status,
              SolutionStatus::FLOAT);
    EXPECT_EQ(accepted_result.diagnostics.float_rejected_seed_position_divergence,
              0u);

    config.max_float_seed_position_divergence_m = 1.0;
    config.fix_ambiguities = true;
    config.fixed_ambiguity_sigma_m = 1e-4;
    config.lambda_ratio_threshold = 1.5;
    FGOProcessor fixed_processor(config);
    const auto fixed_result =
        fixed_processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(fixed_result.solution.size(), 2u);
    EXPECT_EQ(fixed_result.solution.solutions[0].status, SolutionStatus::FIXED);
    EXPECT_EQ(fixed_result.solution.solutions[1].status, SolutionStatus::FIXED);
    EXPECT_EQ(fixed_result.diagnostics.float_rejected_seed_position_divergence,
              0u);
}

TEST(FGOTest, FloatPositionJumpGateRejectsFloatAfterLargeJump) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.max_float_position_jump_m = 1.0;

    FGOProcessor processor(config);
    const auto result =
        processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::FLOAT);
    EXPECT_EQ(result.solution.solutions[1].status, SolutionStatus::NONE);
    EXPECT_EQ(result.diagnostics.float_rejected_position_jump, 1u);
    EXPECT_EQ(result.diagnostics.float_rejected_seed_position_divergence, 0u);
}

TEST(FGOTest, DoubleDifferenceInternalsMatchTarozLinearizedFactors) {
    const auto problem = makeSyntheticDoubleDifferenceProblem(true);
    ASSERT_FALSE(problem.double_difference_pseudorange_factors.empty());
    ASSERT_FALSE(problem.double_difference_carrier_factors.empty());

    const auto& code_factor =
        problem.double_difference_pseudorange_factors.front();
    const auto& carrier_factor =
        problem.double_difference_carrier_factors.front();
    ASSERT_EQ(code_factor.epoch_index, carrier_factor.epoch_index);
    ASSERT_EQ(code_factor.satellite, carrier_factor.satellite);
    ASSERT_EQ(code_factor.reference_satellite,
              carrier_factor.reference_satellite);

    const Vector3d seed_position =
        problem.epochs[code_factor.epoch_index].position_ecef;
    const double geometry_at_seed =
        doubleDifferenceGeometry(seed_position, code_factor);
    const double code_residual_at_seed =
        code_factor.observed_dd_pseudorange_m - geometry_at_seed;
    const double carrier_residual_at_seed =
        carrier_factor.observed_dd_carrier_m - geometry_at_seed;
    const double taroz_ambiguity_seed_m =
        carrier_residual_at_seed - code_residual_at_seed;

    EXPECT_NEAR(taroz_ambiguity_seed_m,
                trueAmbiguityMeters(1) - trueAmbiguityMeters(0),
                1e-6);

    const Vector3d dx(0.001, -0.002, 0.0015);
    const Vector3d taroz_los =
        doubleDifferencePositionJacobian(seed_position, code_factor);
    const Vector3d moved_position = seed_position + dx;
    const double linearized_geometry =
        geometry_at_seed + taroz_los.dot(dx);
    const double moved_geometry =
        doubleDifferenceGeometry(moved_position, code_factor);
    EXPECT_NEAR(linearized_geometry, moved_geometry, 1e-9);

    const double taroz_code_error =
        taroz_los.dot(dx) - code_residual_at_seed;
    const double nonlinear_code_error =
        -(code_factor.observed_dd_pseudorange_m - moved_geometry);
    EXPECT_NEAR(taroz_code_error, nonlinear_code_error, 1e-9);

    const double taroz_bias_cycles =
        taroz_ambiguity_seed_m / constants::GPS_L1_WAVELENGTH;
    const double taroz_carrier_error =
        taroz_los.dot(dx) -
        (carrier_residual_at_seed -
         constants::GPS_L1_WAVELENGTH * taroz_bias_cycles);
    const double nonlinear_carrier_error =
        -(carrier_factor.observed_dd_carrier_m -
          (doubleDifferenceGeometry(moved_position, carrier_factor) +
           taroz_ambiguity_seed_m));
    EXPECT_NEAR(taroz_carrier_error, nonlinear_carrier_error, 1e-9);
}

TEST(FGOTest, FixedAmbiguityPassSnapsIntegerCarrierPhaseStates) {
    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.fix_ambiguities = true;
    config.use_lambda_ambiguity_fix = true;
    config.carrier_phase_sigma_m = 0.01;
    config.ambiguity_prior_sigma_m = 1000.0;
    config.fixed_ambiguity_sigma_m = 1e-4;
    config.ambiguity_fix_max_fractional_cycles = 0.2;
    config.lambda_ratio_threshold = 1.5;
    config.min_fixed_ambiguities = 4;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(makeSyntheticProblem(false, true));

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics.fixed_solution);
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::FIXED);
    EXPECT_EQ(result.solution.solutions[1].status, SolutionStatus::FIXED);
    EXPECT_TRUE(result.solution.solutions[0].isFixed());
    EXPECT_EQ(result.solution.solutions[0].num_fixed_ambiguities, 6);
    EXPECT_GT(result.solution.solutions[0].ratio, config.lambda_ratio_threshold);
    const auto stats = result.solution.calculateStatistics();
    EXPECT_EQ(stats.float_solutions, 0u);
    EXPECT_EQ(stats.fixed_solutions, 2u);
    EXPECT_DOUBLE_EQ(stats.fix_rate, 1.0);
    EXPECT_TRUE(result.diagnostics.lambda_ambiguity_fix_solved);
    EXPECT_TRUE(result.diagnostics.lambda_ambiguity_fix_used);
    EXPECT_GT(result.diagnostics.lambda_ambiguity_ratio, config.lambda_ratio_threshold);
    EXPECT_EQ(result.diagnostics.ambiguity_fix_candidates, 6u);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_candidates, 6u);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_used_candidates, 6u);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_attempts, 1u);
    EXPECT_FALSE(result.diagnostics.partial_lambda_ambiguity_fix_used);
    EXPECT_EQ(result.diagnostics.fixed_ambiguities, 6u);
    EXPECT_LT(result.diagnostics.fixed_ambiguity_residual_rms_cycles, 1e-4);
    ASSERT_EQ(result.ambiguity_estimates.size(), 6u);

    for (std::size_t sat = 0; sat < result.ambiguity_estimates.size(); ++sat) {
        const auto& estimate = result.ambiguity_estimates[sat];
        EXPECT_TRUE(estimate.is_fixed);
        EXPECT_TRUE(estimate.fixed_by_lambda);
        EXPECT_EQ(estimate.fixed_cycles, trueAmbiguityCycles(sat));
        EXPECT_NEAR(estimate.ambiguity_cycles,
                    static_cast<double>(trueAmbiguityCycles(sat)),
                    1e-4);
        EXPECT_NEAR(estimate.fixed_ambiguity_m,
                    trueAmbiguityMeters(sat),
                    1e-10);
    }
}

TEST(FGOTest, LambdaAcceptsRatioQualifiedCandidatesWithoutFractionalGate) {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem(false, true);
    const SatelliteId degraded_satellite(GNSSSystem::GPS, 6);
    for (auto& factor : problem.carrier_phase_factors) {
        if (factor.satellite == degraded_satellite) {
            factor.corrected_carrier_m +=
                0.45 * constants::GPS_L1_WAVELENGTH;
            factor.sigma_m = 0.5;
        }
    }

    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.fix_ambiguities = true;
    config.use_lambda_ambiguity_fix = true;
    config.use_partial_lambda_ambiguity_fix = true;
    config.carrier_phase_sigma_m = 0.01;
    config.ambiguity_prior_sigma_m = 1000.0;
    config.fixed_ambiguity_sigma_m = 1e-4;
    config.ambiguity_fix_max_fractional_cycles = 0.2;
    config.lambda_ratio_threshold = 0.0;
    config.min_fixed_ambiguities = 4;
    config.max_lambda_ambiguities = 6;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics.fixed_solution);
    EXPECT_TRUE(result.diagnostics.lambda_ambiguity_fix_solved);
    EXPECT_TRUE(result.diagnostics.lambda_ambiguity_fix_used);
    EXPECT_FALSE(result.diagnostics.partial_lambda_ambiguity_fix_used);
    EXPECT_EQ(result.solution.solutions[0].status, SolutionStatus::FIXED);
    EXPECT_EQ(result.solution.solutions[0].num_fixed_ambiguities, 6);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_candidates, 6u);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_used_candidates, 6u);
    EXPECT_EQ(result.diagnostics.lambda_ambiguity_attempts, 1u);
    EXPECT_EQ(result.diagnostics.fixed_ambiguities, 6u);
    ASSERT_EQ(result.ambiguity_estimates.size(), 6u);

    for (const auto& estimate : result.ambiguity_estimates) {
        EXPECT_TRUE(estimate.is_fixed);
        EXPECT_TRUE(estimate.fixed_by_lambda);
    }
}

TEST(FGOTest, PropagatesTdcpQualityDiagnostics) {
    auto problem = makeSyntheticProblem();
    problem.diagnostics.tdcp_candidate_pairs = 9;
    problem.diagnostics.tdcp_rejected_gap = 2;
    problem.diagnostics.tdcp_rejected_missing_previous = 3;
    problem.diagnostics.tdcp_rejected_loss_of_lock = 4;
    problem.diagnostics.tdcp_rejected_code_phase_jump = 5;

    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_FALSE(result.solution.isEmpty());
    EXPECT_EQ(result.diagnostics.tdcp_candidate_pairs, 9u);
    EXPECT_EQ(result.diagnostics.tdcp_rejected_gap, 2u);
    EXPECT_EQ(result.diagnostics.tdcp_rejected_missing_previous, 3u);
    EXPECT_EQ(result.diagnostics.tdcp_rejected_loss_of_lock, 4u);
    EXPECT_EQ(result.diagnostics.tdcp_rejected_code_phase_jump, 5u);
}
