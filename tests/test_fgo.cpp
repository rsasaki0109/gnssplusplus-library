#include <gtest/gtest.h>
#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/android_sv_time_uncertainty.hpp>
#include <libgnss++/algorithms/cn0_doppler_calibration.hpp>
#include <libgnss++/algorithms/doppler_velocity_wls.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/pseudorange_remasking.hpp>
#include <libgnss++/algorithms/fgo_quality_anchor.hpp>
#include <libgnss++/algorithms/fgo_ddpr_gnc.hpp>
#include <libgnss++/algorithms/pdc_state_bridge.hpp>
#include <libgnss++/algorithms/residual_ionosphere_contract.hpp>
#include <libgnss++/algorithms/tdcp_contract.hpp>
#include <libgnss++/core/constants.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

TEST(NativeImuStopPhaseTest, PreservesLegacyAndSeparatesSourceVelocityFromPose) {
    using Phase = libgnss::NativeImuStopPhase;
    EXPECT_EQ(libgnss::FGOProcessor::FGOConfig{}.native_imu_stop_phase, Phase::Legacy);
    EXPECT_TRUE(libgnss::nativeImuStopVelocityPrior(Phase::Legacy));
    EXPECT_TRUE(libgnss::nativeImuStopPoseInterval(Phase::Legacy, 60.0));
    EXPECT_FALSE(libgnss::nativeImuStopVelocityPrior(Phase::Initialization));
    EXPECT_TRUE(libgnss::nativeImuStopVelocityPrior(Phase::Final));
    for (const auto phase : {Phase::Initialization, Phase::Final}) {
        EXPECT_TRUE(libgnss::nativeImuStopPoseUtcInterval(phase, 1700000000000LL, 1700000001499LL));
        EXPECT_FALSE(libgnss::nativeImuStopPoseUtcInterval(phase, 1700000000000LL, 1700000001500LL));
        EXPECT_FALSE(libgnss::nativeImuStopPoseUtcInterval(phase, 0, 1000));
        EXPECT_FALSE(libgnss::nativeImuStopPoseUtcInterval(phase, 1000, 1000));
        EXPECT_FALSE(libgnss::nativeImuStopPoseUtcInterval(phase, 2000, 1000));
        EXPECT_TRUE(libgnss::nativeImuStopPoseInterval(phase, 1.0));
        EXPECT_TRUE(libgnss::nativeImuStopPoseInterval(phase, std::nextafter(1.5, 0.0)));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, 1.5));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, 2.0));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, 0.0));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, -1.0));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, std::numeric_limits<double>::quiet_NaN()));
        EXPECT_FALSE(libgnss::nativeImuStopPoseInterval(phase, std::numeric_limits<double>::infinity()));
    }
    EXPECT_THROW(libgnss::nativeImuStopVelocityPrior(static_cast<Phase>(99)), std::invalid_argument);
    EXPECT_THROW(libgnss::nativeImuStopPoseInterval(static_cast<Phase>(99), 1.0), std::invalid_argument);
}

using namespace libgnss;

TEST(FGORemaskingSelectionTest, RecoversAndRemovesRowsWithoutMutatingInput) {
    FGOProcessor::FGOProblem p;
    p.epochs.resize(1);
    auto& epoch=p.epochs.front();
    epoch.time=GNSSTime(2300,100.0);
    epoch.position_ecef=Vector3d(6378137,0,0);
    epoch.receiver_clock_bias_m=0.0;
    epoch.receiver_clock_bias_is_meters=true;
    const std::array<Vector3d,4> directions{
        Vector3d(1,0,0),Vector3d(0,1,0),Vector3d(0,-1,0),Vector3d(-1,0,0)};
    for (std::size_t i=0;i<4;++i) {
        FGOProcessor::PseudorangeFactor row;
        row.satellite=SatelliteId(GNSSSystem::GPS,static_cast<uint8_t>(i+1));
        row.satellite_position_ecef=epoch.position_ecef+20000000.0*directions[i];
        row.corrected_pseudorange_m=20000000.0+(i==3 ? 25.0 : 0.0);
        p.native_pseudorange_remasking_pool.push_back(row);
        if (i!=0) p.pseudorange_factors.push_back(row);
    }
    const auto result=pseudorange_remasking::select(p);
    EXPECT_EQ(result.pool_indices,(std::vector<std::size_t>{0,1,2}));
    EXPECT_EQ(result.recovered,1u);
    EXPECT_EQ(result.removed,1u);
    EXPECT_EQ(result.unchanged,2u);
    EXPECT_EQ(p.pseudorange_factors.front().satellite,SatelliteId(GNSSSystem::GPS,2));
    EXPECT_DOUBLE_EQ(p.native_pseudorange_remasking_pool.back().corrected_pseudorange_m,20000025.0);
    p.native_pseudorange_remasking_pool.push_back(p.native_pseudorange_remasking_pool.front());
    EXPECT_THROW(pseudorange_remasking::select(p),std::invalid_argument);
}

TEST(FGORemaskingSelectionTest, SeparateBandsOddEvenMediansAndInvalidInputs) {
    FGOProcessor::FGOProblem p;
    p.epochs.resize(1);
    p.epochs[0].time=GNSSTime(2300,100);
    p.epochs[0].position_ecef=Vector3d(6378137,0,0);
    p.epochs[0].receiver_clock_bias_is_meters=true;
    p.epochs[0].receiver_clock_bias_m=0;
    // L1 even median=20: both boundary rows accepted. L5 odd median=100:
    // only first two rows accepted. Pool ordering remains deterministic.
    const std::array<double,5> residuals{0,40,100,100,116};
    for(std::size_t i=0;i<residuals.size();++i) {
        FGOProcessor::PseudorangeFactor row;
        row.satellite=SatelliteId(GNSSSystem::GPS,static_cast<uint8_t>(i+1));
        row.signal=i<2 ? SignalType::GPS_L1CA : SignalType::GPS_L5;
        row.satellite_position_ecef=p.epochs[0].position_ecef+Vector3d(20000000,0,0);
        row.corrected_pseudorange_m=20000000+residuals[i];
        p.native_pseudorange_remasking_pool.push_back(row);
    }
    EXPECT_EQ(pseudorange_remasking::select(p).pool_indices,
              (std::vector<std::size_t>{0,1,2,3}));
    for (int field=0;field<3;++field) {
        auto mutated=p;
        mutated.pseudorange_factors.push_back(p.native_pseudorange_remasking_pool[0]);
        auto& row=mutated.pseudorange_factors.front();
        if (field==0) row.corrected_pseudorange_m+=1;
        if (field==1) row.sigma_m+=1;
        if (field==2) row.satellite_position_ecef.x()+=1;
        EXPECT_THROW(pseudorange_remasking::select(mutated),std::invalid_argument);
    }
    auto bad=p;
    bad.epochs[0].receiver_clock_bias_is_meters=false;
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
    bad=p; bad.epochs[0].time.tow=604800;
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
    bad=p; bad.native_pseudorange_remasking_pool[0].epoch_index=1;
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
    bad=p; bad.native_pseudorange_remasking_pool[0].sigma_m=0;
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
    bad=p; bad.pseudorange_factors.push_back(p.native_pseudorange_remasking_pool[0]);
    bad.pseudorange_factors[0].satellite=SatelliteId(GNSSSystem::GPS,30);
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
    bad=p; bad.epochs.push_back(bad.epochs[0]);
    EXPECT_THROW(pseudorange_remasking::select(bad),std::invalid_argument);
}

TEST(FGORemaskingPoolTest, DefaultOffAndInvalidConfigurationRejected) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.retain_native_pseudorange_remasking_pool);
    FGOProcessor::FGOProblem problem;
    EXPECT_TRUE(problem.native_pseudorange_remasking_pool.empty());
    config.retain_native_pseudorange_remasking_pool = true;
    config.use_upstream_observable_quality = false;
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem({}, NavigationData{}),
                 std::invalid_argument);
    config.use_upstream_observable_quality = true;
    config.use_pseudorange_factors = false;
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem({}, NavigationData{}),
                 std::invalid_argument);
}

TEST(FGOQualityAnchorTest, RankingIsDeterministicAndSatelliteFirst) {
    using fgo_quality_anchor::Candidate;
    const Candidate fewer_satellites{2, 9, 1.01, 0.1};
    const Candidate more_satellites{7, 10, 9.0, 9.0};
    EXPECT_TRUE(fgo_quality_anchor::better(more_satellites, fewer_satellites));
    EXPECT_FALSE(fgo_quality_anchor::better(fewer_satellites, more_satellites));

    const Candidate lower_gdop{4, 10, 1.1, 2.0};
    const Candidate higher_gdop{3, 10, 1.2, 0.1};
    EXPECT_TRUE(fgo_quality_anchor::better(lower_gdop, higher_gdop));

    const Candidate lower_residual{5, 10, 1.1, 0.5};
    const Candidate higher_residual{6, 10, 1.1, 0.6};
    EXPECT_TRUE(fgo_quality_anchor::better(lower_residual, higher_residual));
    const Candidate earlier{1, 10, 1.1, 0.5};
    const Candidate later{8, 10, 1.1, 0.5};
    EXPECT_TRUE(fgo_quality_anchor::better(earlier, later));
}

TEST(FGOQualityAnchorTest, NoEligibleEpochFailsClosedToHistoricalPass) {
    EXPECT_FALSE(fgo_quality_anchor::eligible(
        false, true, 6.4e6, 20, 1.0, 1.0));
    EXPECT_FALSE(fgo_quality_anchor::eligible(
        true, true, 6.4e6, 3, 1.0, 1.0));
    EXPECT_FALSE(fgo_quality_anchor::eligible(
        true, false, 6.4e6, 20, 1.0, 1.0));
    const std::vector<fgo_quality_anchor::Candidate> no_candidates;
    EXPECT_EQ(fgo_quality_anchor::choose(no_candidates), nullptr);
}

TEST(FGOQualityAnchorTest, ReplayOrdersAreAnchorOutwardAndGraphChronological) {
    const auto forward = fgo_quality_anchor::outwardOrder(6, 2, true);
    const auto backward = fgo_quality_anchor::outwardOrder(6, 2, false);
    EXPECT_EQ(forward, (std::vector<std::size_t>{2, 3, 4, 5}));
    EXPECT_EQ(backward, (std::vector<std::size_t>{2, 1, 0}));

    // The replay order is allowed to decrease only while collecting seeds;
    // the builder merges by input index before constructing graph factors.
    std::vector<std::size_t> merged(6, 0);
    for (std::size_t index = 0; index < backward.size(); ++index) {
        merged[backward[index]] = backward[index];
    }
    for (std::size_t index = 0; index < forward.size(); ++index) {
        merged[forward[index]] = forward[index];
    }
    EXPECT_EQ(merged, (std::vector<std::size_t>{0, 1, 2, 3, 4, 5}));
    EXPECT_EQ(fgo_quality_anchor::graphOrder(6),
              (std::vector<std::size_t>{0, 1, 2, 3, 4, 5}));

    // The reverse SPP pass is seed collection only.  Graph construction is
    // required to use the chronological input order, so no negative-dt graph
    // factor can be introduced by the reverse traversal.
    EXPECT_TRUE(std::is_sorted(merged.begin(), merged.end()));
}

TEST(FGOQualityAnchorTest, ConfigIsOptInOnly) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_quality_anchor_initialization);
    EXPECT_FALSE(config.use_native_android_sv_time_uncertainty_sigma_floor);
    EXPECT_FALSE(config.use_native_phase143_official_main_lm_termination_budget);
}

TEST(FGOPhase143Test, SelectorRequiresGtsamAndFrozenConfigurationBoundary) {
    FGOProcessor::FGOConfig config;
    config.use_native_phase143_official_main_lm_termination_budget = true;
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);

    config.backend = FGOBackend::GTSAM;
    config.max_iterations = 8;
    // The boundary is checked before any graph work, including in a build
    // where the requested GTSAM backend is unavailable.
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);

    config.max_iterations = 12;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);
}

TEST(FGOPhase167Test, DedicatedNoDopplerBudgetIsOptInAndFailClosed) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(
        config.use_native_phase167_raw_p_no_doppler_lm_termination_budget);

    config.use_native_phase167_raw_p_no_doppler_lm_termination_budget = true;
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);

    config.backend = FGOBackend::GTSAM;
    config.use_native_raw_p_no_doppler_graph = true;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.max_iterations = 12;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);

    // The dedicated selector's contract is explicit: the caller must carry
    // the source-backed 1000-iteration configuration into the graph.
    config.max_iterations = 1000;
    EXPECT_EQ(config.max_iterations, 1000);
}

TEST(FGOTdcpRobustKTest, OfficialTypeMappingIsOptInAndFailClosed) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_official_tdcp_huber_k);
    EXPECT_TRUE(config.official_tdcp_setting_type.empty());

    double threshold = 0.0;
    EXPECT_TRUE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, config.tdcp_huber_threshold_sigma);

    config.use_official_tdcp_huber_k = true;
    config.official_tdcp_setting_type = "Street";
    EXPECT_TRUE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 0.2);
    config.official_tdcp_setting_type = "Mix";
    EXPECT_TRUE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 0.2);
    config.official_tdcp_setting_type = "Highway";
    EXPECT_TRUE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 0.5);

    config.official_tdcp_setting_type.clear();
    EXPECT_FALSE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
    config.official_tdcp_setting_type = "unrecognised";
    EXPECT_FALSE(fgo::resolveOfficialTdcpHuberThresholdSigma(config, threshold));
}

TEST(FGOTdcpRobustKTest, Phase184SourceMappingIsSeparateFromPhase118) {
    FGOProcessor::FGOConfig config;
    double threshold = 0.0;
    EXPECT_TRUE(fgo::resolveOrdinaryTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 4.0);

    config.use_native_phase184_source_tdcp_huber_k = true;
    config.native_phase184_tdcp_setting_type = "Street";
    EXPECT_TRUE(fgo::resolveOrdinaryTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 0.2);
    EXPECT_FALSE(config.use_official_tdcp_huber_k);

    config.native_phase184_tdcp_setting_type = "Highway";
    EXPECT_TRUE(fgo::resolveOrdinaryTdcpHuberThresholdSigma(config, threshold));
    EXPECT_DOUBLE_EQ(threshold, 0.5);

    config.native_phase184_tdcp_setting_type = "unknown";
    EXPECT_FALSE(fgo::resolveOrdinaryTdcpHuberThresholdSigma(config, threshold));

    config.native_phase184_tdcp_setting_type = "Street";
    config.use_official_tdcp_huber_k = true;
    config.official_tdcp_setting_type = "Street";
    EXPECT_FALSE(fgo::resolveOrdinaryTdcpHuberThresholdSigma(config, threshold));
}

TEST(FGOTdcpReslNormalizationTest, SelectorIsOptInAndExcludesAtmosphereOnlyWhenOn) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_official_tdcp_resl_atmosphere_cancellation);

    constexpr double raw_carrier_m = 123.4;
    constexpr double satellite_clock_m = -2.5;
    constexpr double ionosphere_m = 0.75;
    constexpr double troposphere_m = 2.25;

    const double legacy = tdcp_contract::ordinaryTdcpCarrierMeters(
        raw_carrier_m, satellite_clock_m, ionosphere_m, troposphere_m,
        config.use_official_tdcp_resl_atmosphere_cancellation);
    EXPECT_DOUBLE_EQ(legacy, raw_carrier_m + satellite_clock_m -
                               troposphere_m + ionosphere_m);

    config.use_official_tdcp_resl_atmosphere_cancellation = true;
    const double source_parity = tdcp_contract::ordinaryTdcpCarrierMeters(
        raw_carrier_m, satellite_clock_m, ionosphere_m, troposphere_m,
        config.use_official_tdcp_resl_atmosphere_cancellation);
    EXPECT_DOUBLE_EQ(source_parity, raw_carrier_m + satellite_clock_m);
    EXPECT_NE(source_parity, legacy);
}

TEST(FGOTdcpReslNormalizationTest, SourceParityIgnoresAtmosphereButRejectsBadCoreTerms) {
    const double source_parity = tdcp_contract::ordinaryTdcpCarrierMeters(
        10.0, 2.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), true);
    EXPECT_DOUBLE_EQ(source_parity, 12.0);

    const double bad_carrier = tdcp_contract::ordinaryTdcpCarrierMeters(
        std::numeric_limits<double>::quiet_NaN(), 2.0, 0.0, 0.0, true);
    EXPECT_TRUE(std::isnan(bad_carrier));
    const double bad_clock = tdcp_contract::ordinaryTdcpCarrierMeters(
        10.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, true);
    EXPECT_TRUE(std::isnan(bad_clock));
    const double overflow = tdcp_contract::ordinaryTdcpCarrierMeters(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(), 0.0, 0.0, true);
    EXPECT_TRUE(std::isnan(overflow));
}

TEST(FGOTdcpReslNormalizationTest, TemporalAtmosphereDifferenceHasSourceResLSign) {
    // Binary-exact synthetic terms isolate the measurement convention,
    // independently of positions, truth, robust loss and noise weighting.
    const auto delta = [](bool source, double ion2, double trop2) {
        return tdcp_contract::ordinaryTdcpCarrierMeters(
                   104.0, 2.5, ion2, trop2, source) -
               tdcp_contract::ordinaryTdcpCarrierMeters(
                   100.0, 2.0, 1.0, 3.0, source);
    };
    const double source = delta(true, 1.75, 3.25);
    const double legacy = delta(false, 1.75, 3.25);
    EXPECT_DOUBLE_EQ(source, 4.5);  // delta(raw carrier + satellite clock)
    EXPECT_DOUBLE_EQ(legacy, 5.0);
    EXPECT_DOUBLE_EQ(source - legacy, (3.25 - 3.0) - (1.75 - 1.0));
    EXPECT_DOUBLE_EQ(delta(true, 20.0, 50.0), source);
    EXPECT_DOUBLE_EQ(delta(false, 1.0, 3.0), source);
    // For fixed geometry/receiver clocks, residual = prediction - observation.
    constexpr double prediction = 6.0;
    EXPECT_DOUBLE_EQ((prediction - source) - (prediction - legacy), 0.5);
}

TEST(FGOTdcpReslNormalizationTest, DynamicSigmaCompositionFailsClosedAtOptimizerBoundary) {
    FGOProcessor::FGOConfig config;
    config.use_official_tdcp_resl_atmosphere_cancellation = true;
    config.use_official_tdcp_snr_type_sigma = true;
    FGOProcessor processor(config);
    FGOProcessor::FGOProblem problem;
    EXPECT_THROW(processor.optimizeProblem(problem), std::invalid_argument);
}

TEST(AndroidSvTimeUncertaintyTest, ConvertsNanosecondsWithoutScaleOrClip) {
    using namespace android_sv_time_uncertainty;
    EXPECT_NEAR(metersFromNanoseconds(10.0),
                10.0e-9 * constants::SPEED_OF_LIGHT, 1e-12);
    EXPECT_TRUE(std::isnan(metersFromNanoseconds(0.0)));
    EXPECT_TRUE(std::isnan(metersFromNanoseconds(-1.0)));
    EXPECT_TRUE(std::isnan(metersFromNanoseconds(
        std::numeric_limits<double>::quiet_NaN())));
    EXPECT_TRUE(std::isnan(metersFromNanoseconds(
        std::numeric_limits<double>::infinity())));
}

TEST(AndroidSvTimeUncertaintyTest, FloorsOnlyWhenEnabledAndNeverClips) {
    using namespace android_sv_time_uncertainty;
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, 1.0, true), 3.0);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, 10.0, true), 10.0);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, 10.0, false), 3.0);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, 0.0, true), 3.0);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, std::numeric_limits<double>::quiet_NaN(), true),
                     3.0);
    // There is no upper clip: an intentionally large source uncertainty is
    // retained exactly as the source-derived floor.
    EXPECT_DOUBLE_EQ(sigmaWithFloor(3.0, 1.0e9, true), 1.0e9);
}

TEST(Cn0DopplerCalibrationTest, UsesFrozenAlphaAndTwentyDbShape) {
    using namespace cn0_doppler_calibration;
    EXPECT_DOUBLE_EQ(kReferenceCn0DbHz, 40.0);
    EXPECT_DOUBLE_EQ(kAlphaMpsAtReference, 0.7586783350728457);
    EXPECT_DOUBLE_EQ(modelSigmaMps(40.0), kAlphaMpsAtReference);
    EXPECT_DOUBLE_EQ(modelSigmaMps(20.0),
                     kAlphaMpsAtReference * 10.0);
    EXPECT_DOUBLE_EQ(modelSigmaMps(60.0),
                     kAlphaMpsAtReference * 0.1);
}

TEST(Cn0DopplerCalibrationTest, FloorsOnlyWhenEnabledAndFallsBackExactly) {
    using namespace cn0_doppler_calibration;
    EXPECT_DOUBLE_EQ(sigmaWithFloor(0.2, 40.0, true),
                     kAlphaMpsAtReference);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(2.0, 40.0, true), 2.0);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(0.2, 40.0, false), 0.2);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(0.2, 0.0, true), 0.2);
    EXPECT_DOUBLE_EQ(sigmaWithFloor(0.2,
                                    std::numeric_limits<double>::quiet_NaN(),
                                    true),
                     0.2);
    // No upper clip: a very low finite C/N0 retains the source-shaped floor.
    EXPECT_DOUBLE_EQ(sigmaWithFloor(0.2, 1.0, true),
                     modelSigmaMps(1.0));
}

TEST(Cn0DopplerCalibrationTest, ConfigIsExplicitlyOptIn) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_cn0_doppler_calibration);
    FGOProcessor::UndifferencedDopplerFactor factor;
    EXPECT_DOUBLE_EQ(factor.cn0_doppler_model_sigma_mps, 0.0);
    EXPECT_FALSE(factor.cn0_doppler_model_sigma_available);
    EXPECT_FALSE(factor.cn0_doppler_sigma_floor_applied);
}

TEST(ResidualIonosphereContractTest, UsesFiniteThinShellAndFrequencyScale) {
    using namespace residual_ionosphere;
    constexpr double half_pi = 1.57079632679489661923;
    EXPECT_NEAR(mappingFactor(half_pi), 1.0, 1e-12);
    EXPECT_GT(mappingFactor(0.25), 1.0);
    const double l1 = signalCoefficient(half_pi, constants::GPS_L1_FREQ);
    const double l5 = signalCoefficient(half_pi, constants::GPS_L5_FREQ);
    EXPECT_NEAR(l1, 1.0, 1e-12);
    EXPECT_NEAR(l5, (constants::GPS_L1_FREQ / constants::GPS_L5_FREQ) *
                         (constants::GPS_L1_FREQ / constants::GPS_L5_FREQ),
                1e-12);
    EXPECT_TRUE(finiteCoefficient(l5));
    EXPECT_FALSE(finiteCoefficient(signalCoefficient(0.0, constants::GPS_L1_FREQ)));
    EXPECT_FALSE(finiteCoefficient(signalCoefficient(0.5, 0.0)));
    EXPECT_NEAR(randomWalkSigma(4.0, 0.5), 1.0, 1e-12);
    EXPECT_FALSE(std::isfinite(randomWalkSigma(0.0, 0.5)));
}

TEST(ResidualIonosphereContractTest, DefaultConfigDoesNotEnableState) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_residual_ionosphere_states);
    EXPECT_DOUBLE_EQ(config.residual_ionosphere_prior_sigma_m, 10.0);
    FGOProcessor::PseudorangeFactor factor;
    EXPECT_DOUBLE_EQ(factor.residual_ionosphere_coefficient, 0.0);
}

TEST(PdcStateBridgeTest, SolvesSyntheticNativePositionAndClock) {
    const Vector3d seed(6'370'000.0, 1'000.0, 2'000.0);
    const Vector3d target = seed + Vector3d(1.25, -2.0, 0.75);
    constexpr double clock_bias_m = 12.5;
    const Vector3d velocity(4.0, -2.0, 1.0);
    constexpr double clock_rate_mps = 0.3;
    const std::vector<Vector3d> satellites = {
        {20'000'000.0, 0.0, 0.0},
        {0.0, 20'000'000.0, 0.0},
        {0.0, 0.0, 20'000'000.0},
        {-20'000'000.0, -20'000'000.0, 20'000'000.0},
        {20'000'000.0, -20'000'000.0, -20'000'000.0},
    };
    const std::vector<Vector3d> lines = {
        {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {-0.5773502691896258, -0.5773502691896258, 0.5773502691896258},
        {0.5773502691896258, -0.5773502691896258, -0.5773502691896258}};
    std::vector<pdc_state_bridge::PseudorangeRow> pseudorange;
    std::vector<pdc_state_bridge::DopplerRow> doppler;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const Vector3d satellite = target + 20'000'000.0 * lines[i];
        pseudorange.push_back({0, SatelliteId{}, GNSSSystem::GPS, satellite,
                               (satellite - target).norm() + clock_bias_m, 0.5});
        doppler.push_back({0, lines[i], lines[i].dot(velocity) + clock_rate_mps,
                           0.05});
    }
    const std::vector<pdc_state_bridge::EpochInput> epochs = {
        {GNSSTime(2200, 100.0), seed, 0.0, false}};
    pdc_state_bridge::Options options;
    options.max_iterations = 100;
    const auto result = pdc_state_bridge::solve(epochs, pseudorange, doppler,
                                                options);
    ASSERT_TRUE(result.valid) << result.reason;
    ASSERT_EQ(result.epochs.size(), 1U);
    const auto& estimate = result.epochs.front();
    ASSERT_TRUE(estimate.valid) << estimate.reason;
    EXPECT_NEAR((estimate.state.position_ecef - target).norm(), 0.0, 1e-3);
    EXPECT_NEAR(estimate.state.clock_bias_m[0], clock_bias_m, 1e-3);
    EXPECT_NEAR((estimate.state.velocity_ecef_mps - velocity).norm(), 0.0,
                1e-3);
    EXPECT_NEAR(estimate.state.clock_rate_mps, clock_rate_mps, 1e-3);
    EXPECT_LT(result.final_cost, result.initial_cost);
}

TEST(PdcStateBridgeTest, UsesFiniteRawWlsVelocityAndClockRateSeed) {
    const Vector3d seed(6'370'000.0, 1'000.0, 2'000.0);
    const Vector3d target = seed + Vector3d(1.25, -2.0, 0.75);
    const Vector3d velocity(4.0, -2.0, 1.0);
    constexpr double clock_rate_mps = 0.3;
    const std::vector<Vector3d> lines = {
        {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {-0.5773502691896258, -0.5773502691896258, 0.5773502691896258},
        {0.5773502691896258, -0.5773502691896258, -0.5773502691896258}};
    std::vector<pdc_state_bridge::PseudorangeRow> pseudorange;
    std::vector<pdc_state_bridge::DopplerRow> doppler;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const Vector3d satellite = target + 20'000'000.0 * lines[i];
        pseudorange.push_back({0, SatelliteId{}, GNSSSystem::GPS, satellite,
                               (satellite - target).norm() + 12.5, 0.5});
        doppler.push_back({0, lines[i], lines[i].dot(velocity) + clock_rate_mps,
                           0.05});
    }
    const std::vector<pdc_state_bridge::EpochInput> unseeded_epochs = {
        {GNSSTime(2200, 100.0), seed, 0.0, false}};
    auto seeded_epochs = unseeded_epochs;
    seeded_epochs[0].seed_velocity_ecef_mps = velocity;
    seeded_epochs[0].seed_clock_rate_mps = clock_rate_mps;
    seeded_epochs[0].has_seed_velocity = true;
    seeded_epochs[0].has_seed_clock_rate = true;

    pdc_state_bridge::Options options;
    options.max_iterations = 100;
    const auto unseeded = pdc_state_bridge::solve(
        unseeded_epochs, pseudorange, doppler, options);
    const auto seeded = pdc_state_bridge::solve(
        seeded_epochs, pseudorange, doppler, options);
    ASSERT_TRUE(unseeded.valid) << unseeded.reason;
    ASSERT_TRUE(seeded.valid) << seeded.reason;
    EXPECT_LT(seeded.initial_cost, unseeded.initial_cost);
    EXPECT_NEAR((seeded.epochs.front().state.velocity_ecef_mps - velocity).norm(),
                0.0, 1e-3);
    EXPECT_NEAR(seeded.epochs.front().state.clock_rate_mps, clock_rate_mps,
                1e-3);

    seeded_epochs[0].seed_velocity_ecef_mps =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const auto invalid = pdc_state_bridge::solve(
        seeded_epochs, pseudorange, doppler, options);
    EXPECT_FALSE(invalid.valid);
    EXPECT_EQ(invalid.reason, "invalid-velocity-seed");
}

TEST(PdcStateBridgeTest, IntegratesWlsPositionSeedsAndResetsAtClockJump) {
    const Vector3d anchor(6'370'000.0, 1'000.0, 2'000.0);
    const Vector3d velocity0(4.0, -2.0, 1.0);
    const Vector3d velocity2(6.0, -1.0, 2.0);
    const std::vector<pdc_state_bridge::EpochInput> epochs = {
        {GNSSTime(2200, 100.0), anchor, 0.0, false},
        {GNSSTime(2200, 101.0), anchor + Vector3d(100.0, 0.0, 0.0), 0.0,
         false},
        {GNSSTime(2200, 102.0), anchor + Vector3d(200.0, 0.0, 0.0), 0.0,
         false}};
    const std::vector<pdc_state_bridge::PositionSeedVelocity> velocities = {
        {true, velocity0}, {false, Vector3d::Zero()}, {true, velocity2}};
    const auto integrated = pdc_state_bridge::integratePositionSeeds(
        epochs, velocities, {false, false, false});
    ASSERT_TRUE(integrated.valid);
    EXPECT_EQ(integrated.anchor_index, 0U);
    EXPECT_EQ(integrated.integrated_epochs, 2U);
    EXPECT_EQ(integrated.held_velocity_epochs, 1U);
    EXPECT_EQ(integrated.per_epoch_spp_fallback_epochs, 0U);
    EXPECT_EQ(integrated.reset_intervals, 0U);
    EXPECT_NEAR((integrated.positions[1] - (anchor + velocity0)).norm(), 0.0,
                1e-9);
    EXPECT_NEAR((integrated.positions[2] -
                 (anchor + velocity0 + 0.5 * (velocity0 + velocity2)))
                    .norm(),
                0.0, 1e-9);

    const auto reset = pdc_state_bridge::integratePositionSeeds(
        epochs, {{true, velocity0}, {true, velocity0}, {true, velocity2}},
        {false, true, false});
    ASSERT_TRUE(reset.valid);
    EXPECT_EQ(reset.integrated_epochs, 1U);
    EXPECT_EQ(reset.per_epoch_spp_fallback_epochs, 1U);
    EXPECT_EQ(reset.reset_intervals, 1U);
    EXPECT_NEAR((reset.positions[1] - epochs[1].seed_position_ecef).norm(),
                0.0, 1e-9);
}

TEST(PdcStateBridgeTest, RejectsInvalidDopplerGeometryBeforeSolving) {
    const Vector3d seed(6'370'000.0, 0.0, 0.0);
    std::vector<pdc_state_bridge::PseudorangeRow> pseudorange;
    const std::vector<Vector3d> lines = {
        {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {-0.5773502691896258, -0.5773502691896258, 0.5773502691896258}};
    for (const auto& line : lines) {
        const Vector3d satellite = seed + 20'000'000.0 * line;
        pseudorange.push_back({0, SatelliteId{}, GNSSSystem::GPS, satellite,
                               (satellite - seed).norm(), 1.0});
    }
    const std::vector<pdc_state_bridge::DopplerRow> invalid_doppler = {
        {0, Vector3d::Zero(), 0.0, 0.2}};
    const std::vector<pdc_state_bridge::EpochInput> epochs = {
        {GNSSTime(2200, 100.0), seed, 0.0, false}};
    const auto result = pdc_state_bridge::solve(epochs, pseudorange,
                                                invalid_doppler);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "invalid-doppler-row");
}

TEST(PdcStateBridgeTest, SolvesTemporalStatesAndHonorsClockJumpBoundary) {
    const Vector3d seed0(6'370'000.0, 1'000.0, 2'000.0);
    const Vector3d velocity(4.0, -2.0, 1.0);
    constexpr double clock0_m = 12.5;
    constexpr double clock_rate_mps = 0.3;
    const std::vector<Vector3d> lines = {
        {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
        {-0.5773502691896258, -0.5773502691896258, 0.5773502691896258},
        {0.5773502691896258, -0.5773502691896258, -0.5773502691896258}};
    const std::vector<pdc_state_bridge::EpochInput> epochs = {
        {GNSSTime(2200, 100.0), seed0, 0.0, false},
        {GNSSTime(2200, 101.0), seed0 + velocity, 0.0, true}};
    std::vector<pdc_state_bridge::PseudorangeRow> pseudorange;
    std::vector<pdc_state_bridge::DopplerRow> doppler;
    for (std::size_t epoch = 0; epoch < epochs.size(); ++epoch) {
        const Vector3d target = seed0 + static_cast<double>(epoch) * velocity;
        const double clock = clock0_m + static_cast<double>(epoch) * clock_rate_mps;
        for (const auto& line : lines) {
            const Vector3d satellite = target + 20'000'000.0 * line;
            pseudorange.push_back({epoch, SatelliteId{}, GNSSSystem::GPS,
                                   satellite, (satellite - target).norm() + clock,
                                   0.5});
            doppler.push_back({epoch, line, line.dot(velocity) + clock_rate_mps,
                               0.05});
        }
    }
    pdc_state_bridge::Options options;
    options.max_iterations = 100;
    const auto result = pdc_state_bridge::solve(epochs, pseudorange, doppler,
                                                options);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.motion_intervals, 1U);
    EXPECT_EQ(result.valid_epochs, 2U);
    ASSERT_EQ(result.epochs.size(), 2U);
    EXPECT_NEAR((result.epochs[1].state.position_ecef -
                 result.epochs[0].state.position_ecef - velocity).norm(),
                0.0, 1e-3);
    EXPECT_NEAR((result.epochs[1].state.velocity_ecef_mps - velocity).norm(),
                0.0, 1e-3);
    EXPECT_NEAR(result.epochs[1].state.clock_rate_mps, clock_rate_mps, 1e-3);
}

TEST(FgoDdprGncTest, GraduatesToRobustWeightsAndSuppressesGrossOutlier) {
    const std::vector<fgo_ddpr_gnc::Residual> residuals = {
        {0.25, 1.0}, {0.5, 1.0}, {1.0, 1.0}, {20.0, 1.0}};

    const auto result = fgo_ddpr_gnc::evaluate(residuals);

    ASSERT_TRUE(result.evaluated);
    ASSERT_EQ(result.weights.size(), residuals.size());
    EXPECT_GT(result.stages, 1);
    EXPECT_DOUBLE_EQ(result.final_mu, 1.0);
    EXPECT_GT(result.weights[0], result.weights[1]);
    EXPECT_GT(result.weights[1], result.weights[2]);
    EXPECT_GT(result.weights[2], result.weights[3]);
    EXPECT_LT(result.weights[3], 0.02);
    EXPECT_NEAR(result.weights.back(),
                4.0 / (4.0 + 400.0), 1e-12);
    EXPECT_EQ(result.downweighted_factors, 1u);
    EXPECT_LT(result.effective_factor_count,
              static_cast<double>(residuals.size()));
}

TEST(FgoDdprGncTest, FailsClosedForInvalidSigmaOrSchedule) {
    EXPECT_FALSE(fgo_ddpr_gnc::evaluate({{1.0, 0.0}}).evaluated);

    fgo_ddpr_gnc::Config invalid;
    invalid.graduation_divisor = 1.0;
    EXPECT_FALSE(fgo_ddpr_gnc::evaluate({{1.0, 1.0}}, invalid).evaluated);
}

TEST(FgoDdprGncTest, DefaultScheduleReachesFinalKernelForUrbanScaleResidual) {
    const auto result = fgo_ddpr_gnc::evaluate({{166.8, 1.0}});

    ASSERT_TRUE(result.evaluated);
    EXPECT_DOUBLE_EQ(result.final_mu, 1.0);
    EXPECT_LT(result.stages, 32);
}

namespace {

TEST(TdcpContractTest, AcceptsFiniteAdjacentPairAtConfiguredGapBoundary) {
    const auto decision = tdcp_contract::evaluateAdjacentPair(
        2.0, false, false, 1.25, 1.20, 2.0, true, true, 10.0);
    EXPECT_TRUE(decision.accepted());
    EXPECT_EQ(decision.reason, tdcp_contract::PairRejectReason::Accepted);
}

TEST(TdcpContractTest, RejectsGapLossLockNonfiniteAndCodePhaseJump) {
    const auto gap = tdcp_contract::evaluateAdjacentPair(
        2.000001, false, false, 1.0, 1.0, 2.0, true, true, 10.0);
    EXPECT_EQ(gap.reason, tdcp_contract::PairRejectReason::Gap);

    const auto loss = tdcp_contract::evaluateAdjacentPair(
        1.0, true, false, 1.0, 1.0, 2.0, true, true, 10.0);
    EXPECT_EQ(loss.reason, tdcp_contract::PairRejectReason::LossOfLock);

    const auto nonfinite = tdcp_contract::evaluateAdjacentPair(
        1.0, false, false, std::numeric_limits<double>::quiet_NaN(), 1.0,
        2.0, true, true, 10.0);
    EXPECT_EQ(nonfinite.reason,
              tdcp_contract::PairRejectReason::NonFiniteMeasurement);

    const auto jump = tdcp_contract::evaluateAdjacentPair(
        1.0, false, false, 11.000001, 1.0, 2.0, true, true, 10.0);
    EXPECT_EQ(jump.reason, tdcp_contract::PairRejectReason::CodePhaseJump);

    const auto clock = tdcp_contract::evaluateAdjacentPair(
        1.0, false, false, 1.0, 1.0, 2.0, true, true, 10.0, false, true);
    EXPECT_EQ(clock.reason,
              tdcp_contract::PairRejectReason::ClockDiscontinuity);
}

TEST(DopplerContractTest, AndroidRinexRoundTripAndApproachSigns) {
    const double frequency_hz = constants::GPS_L1_FREQ;
    const double wavelength_m = constants::SPEED_OF_LIGHT / frequency_hz;

    const double receding_rate_mps = 25.0;
    const double receding_doppler =
        doppler_contract::androidRateToRinexDoppler(
            receding_rate_mps, frequency_hz);
    EXPECT_NEAR(receding_doppler, -receding_rate_mps / wavelength_m, 1e-12);
    EXPECT_NEAR(doppler_contract::rinexDopplerToRangeRate(
                    receding_doppler, frequency_hz),
                receding_rate_mps, 1e-12);

    const double approaching_rate_mps = -12.5;
    const double approaching_doppler =
        doppler_contract::androidRateToRinexDoppler(
            approaching_rate_mps, frequency_hz);
    EXPECT_GT(approaching_doppler, 0.0);
    EXPECT_NEAR(doppler_contract::rinexDopplerToRangeRate(
                    approaching_doppler, frequency_hz),
                approaching_rate_mps, 1e-12);

    // A sign inversion must not be silently accepted as the same physical
    // observation.
    EXPECT_NEAR(doppler_contract::rinexDopplerToRangeRate(
                    -receding_doppler, frequency_hz),
                -receding_rate_mps, 1e-12);
}

TEST(DopplerContractTest, StationaryReceiverClockDriftIsExplicit) {
    const Vector3d receiver = Vector3d::Zero();
    const Vector3d satellite(20'000'000.0, 0.0, 0.0);
    const Vector3d satellite_velocity = Vector3d::Zero();
    Vector3d los = Vector3d::Zero();
    double known_range_rate = 0.0;
    ASSERT_TRUE(doppler_contract::knownSatelliteRangeRate(
        satellite, satellite_velocity, receiver, false, los,
        known_range_rate));
    EXPECT_NEAR(los.x(), 1.0, 1e-12);
    EXPECT_NEAR(known_range_rate, 0.0, 1e-12);

    constexpr double receiver_clock_drift_mps = 3.0;
    const double measured_range_rate = receiver_clock_drift_mps;
    const double residual = doppler_contract::receiverOnlyResidual(
        measured_range_rate, known_range_rate, 0.0);
    EXPECT_NEAR(residual, receiver_clock_drift_mps, 1e-12);
    EXPECT_NEAR(doppler_contract::receiverPrediction(
                    los, Vector3d::Zero(), receiver_clock_drift_mps),
                residual, 1e-12);
}

TEST(DopplerContractTest, ReceiverVelocityAndClockJacobianMatchesFiniteDifference) {
    const Vector3d los = Vector3d(0.6, 0.8, 0.0);
    const Vector3d velocity(3.0, -2.0, 0.5);
    constexpr double clock_drift_mps = 0.7;
    const double base = doppler_contract::receiverPrediction(
        los, velocity, clock_drift_mps);
    constexpr double epsilon = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        Vector3d perturbed = velocity;
        perturbed(axis) += epsilon;
        const double derivative =
            (doppler_contract::receiverPrediction(
                 los, perturbed, clock_drift_mps) - base) /
            epsilon;
        EXPECT_NEAR(derivative, -los(axis), 1e-9);
    }
    const double clock_derivative =
        (doppler_contract::receiverPrediction(
             los, velocity, clock_drift_mps + epsilon) - base) /
        epsilon;
    EXPECT_NEAR(clock_derivative, 1.0, 1e-9);
}

TEST(DopplerContractTest, EarthRotationRotatesPositionAndVelocityTogether) {
    const Vector3d receiver(4.2e6, 1.1e6, 4.7e6);
    const Vector3d satellite(1.56e7, 7.54e6, 2.014e7);
    const Vector3d velocity(1200.0, -2300.0, 900.0);
    Vector3d corrected_position = Vector3d::Zero();
    Vector3d corrected_velocity = Vector3d::Zero();
    ASSERT_TRUE(doppler_contract::earthRotationCorrectedSatelliteState(
        satellite, velocity, receiver, corrected_position,
        corrected_velocity));
    EXPECT_TRUE(corrected_position.allFinite());
    EXPECT_TRUE(corrected_velocity.allFinite());
    EXPECT_NEAR(corrected_position.norm(), satellite.norm(), 1e-6);
    EXPECT_NEAR(corrected_velocity.norm(), velocity.norm(), 1e-9);
    EXPECT_GT((corrected_position - satellite).norm(), 0.0);
}

TEST(DopplerVelocityWlsTest, ExactRowsRecoverVelocityAndClockRate) {
    const Vector3d velocity(12.0, -4.0, 2.5);
    constexpr double clock_rate = 1.7;
    const double root_three = std::sqrt(3.0);
    const std::vector<Vector3d> losses = {
        Vector3d(1.0, 0.0, 0.0),
        Vector3d(0.0, 1.0, 0.0),
        Vector3d(0.0, 0.0, 1.0),
        Vector3d(1.0 / root_three, 1.0 / root_three, 1.0 / root_three),
        Vector3d(-1.0 / root_three, 1.0 / root_three, 1.0 / root_three),
    };
    std::vector<doppler_velocity_wls::ObservationRow> rows;
    for (const auto& los : losses) {
        rows.push_back({los,
                        doppler_velocity_wls::predict(los, velocity, clock_rate),
                        0.2});
    }

    const auto result = doppler_velocity_wls::solve(rows);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_EQ(result.rank, 4);
    EXPECT_NEAR((result.velocity_ecef_mps - velocity).norm(), 0.0, 1e-9);
    EXPECT_NEAR(result.clock_rate_mps, clock_rate, 1e-9);
    EXPECT_TRUE(result.covariance.allFinite());
    EXPECT_LT(result.normalized_rms, 1e-9);
}

TEST(DopplerVelocityWlsTest, HuberDownweightsOneNoisyObservation) {
    const Vector3d velocity(8.0, -3.0, 1.5);
    constexpr double clock_rate = -0.8;
    const double root_three = std::sqrt(3.0);
    const std::vector<Vector3d> losses = {
        Vector3d(1.0, 0.0, 0.0),
        Vector3d(0.0, 1.0, 0.0),
        Vector3d(0.0, 0.0, 1.0),
        Vector3d(1.0 / root_three, 1.0 / root_three, 1.0 / root_three),
        Vector3d(-1.0 / root_three, 1.0 / root_three, 1.0 / root_three),
        Vector3d(1.0 / root_three, -1.0 / root_three, 1.0 / root_three),
        Vector3d(1.0 / root_three, 1.0 / root_three, -1.0 / root_three),
        Vector3d(-1.0 / root_three, 1.0 / root_three, -1.0 / root_three),
        Vector3d(1.0 / root_three, -1.0 / root_three, -1.0 / root_three),
        Vector3d(-1.0 / root_three, -1.0 / root_three, 1.0 / root_three),
        Vector3d(-1.0 / root_three, -1.0 / root_three, -1.0 / root_three),
        Vector3d(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), 0.0),
    };
    std::vector<doppler_velocity_wls::ObservationRow> rows;
    for (std::size_t i = 0; i < losses.size(); ++i) {
        double residual =
            doppler_velocity_wls::predict(losses[i], velocity, clock_rate);
        if (i == losses.size() - 1) {
            residual += 4.0;  // 20 sigma: robust but physically bounded.
        }
        rows.push_back({losses[i], residual, 0.2});
    }

    const auto result = doppler_velocity_wls::solve(rows);
    ASSERT_TRUE(result.valid) << result.reason;
    EXPECT_LT(result.inlier_rows, static_cast<int>(rows.size()));
    EXPECT_LT((result.velocity_ecef_mps - velocity).norm(), 0.5);
    EXPECT_NEAR(result.clock_rate_mps, clock_rate, 0.5);
    EXPECT_GT(result.max_abs_normalized_residual, 4.0);
}

TEST(DopplerVelocityWlsTest, RankDeficientRowsFailClosed) {
    std::vector<doppler_velocity_wls::ObservationRow> rows;
    for (int i = 0; i < 6; ++i) {
        rows.push_back({Vector3d(1.0, 0.0, 0.0), 2.0, 0.2});
    }
    const auto result = doppler_velocity_wls::solve(rows);
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.reason == "rank-deficient" ||
                result.reason == "rank-or-condition-gate");
}

TEST(DopplerVelocityWlsTest, SignAndDesignRowAreTheSameContract) {
    const Vector3d factor_los(0.6, -0.8, 0.0);
    const Vector3d velocity(5.0, 2.0, 0.0);
    constexpr double clock_rate = 0.25;
    const double predicted = doppler_velocity_wls::predict(
        factor_los, velocity, clock_rate);
    const auto row = doppler_velocity_wls::designRow(factor_los);
    Eigen::Vector4d state;
    state << velocity, clock_rate;
    EXPECT_NEAR((row * state)(0), predicted, 1e-12);
    EXPECT_NEAR(predicted,
                doppler_contract::receiverPrediction(
                    -factor_los, velocity, clock_rate),
                1e-12);
}

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

TEST(FGORefinementBuilderTest, KeepsFirstAndPostGapDopplerWithIndependentClockDrift) {
    const auto nav=makeSyntheticGpsNavigation(1);
    const Vector3d receiver(1113194.0,-4841695.0,3985350.0);
    const Vector3d velocity(40.0,-12.0,3.0);
    const std::array<double,3> times{100100.0,100101.0,100104.0};
    std::vector<ObservationData> raw;
    for (std::size_t i=0;i<times.size();++i) {
        ObservationData epoch(GNSSTime(2300,times[i]));
        epoch.receiver_position=receiver; epoch.receiver_clock_drift_mps=0;
        epoch.raw_source_index=i; epoch.raw_utc_time_millis=1700000000000LL+static_cast<std::int64_t>((times[i]-times[0])*1000);
        Observation obs;
        ASSERT_TRUE(makeSyntheticGpsL1Observation(nav,SatelliteId(GNSSSystem::GPS,1),epoch.time,receiver,0,obs));
        Vector3d position,sv_velocity,los; double clock=0,drift=0,rate=0;
        ASSERT_TRUE(nav.calculateSatelliteState(obs.satellite,epoch.time-obs.pseudorange/constants::SPEED_OF_LIGHT,
                                                position,sv_velocity,clock,drift));
        ASSERT_TRUE(doppler_contract::knownSatelliteRangeRate(position,sv_velocity,receiver,true,los,rate));
        const double measured=rate-los.dot(velocity)-drift*constants::SPEED_OF_LIGHT;
        obs.doppler=-measured/constants::GPS_L1_WAVELENGTH;
        obs.has_doppler=true; epoch.observations.push_back(obs); raw.push_back(epoch);
    }
    FGOProcessor::FGOConfig config;
    config.use_spp_seed=false; config.use_pseudorange_factors=false; config.use_tdcp_factors=false;
    config.use_upstream_observable_quality=true; config.upstream_min_elevation_deg=-90;
    config.use_native_phase171_raw_p_no_doppler_imu_main=true;
    config.use_native_source_clock_c0d_factor=true;
    config.use_undifferenced_doppler_factors=true;
    config.use_corrected_undifferenced_doppler_factors=true;
    config.retain_sparse_epochs_for_imu=true; config.min_satellites_per_epoch=0;
    config.max_tdcp_gap_s=1.5;
    const std::vector<Vector3d> velocities(raw.size(),velocity);
    const auto rebuilt=FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities);
    ASSERT_EQ(rebuilt.epochs.size(),3U);
    ASSERT_EQ(rebuilt.undifferenced_doppler_factors.size(),3U);
    for (std::size_t i=0;i<3;++i) {
        const auto& row=rebuilt.undifferenced_doppler_factors[i];
        EXPECT_EQ(row.epoch_index,i);
        EXPECT_TRUE(row.includes_receiver_clock_drift); EXPECT_TRUE(row.uses_rotated_satellite_state);
        EXPECT_DOUBLE_EQ(row.dt_s,0);
    }
    raw[1].observations[0].doppler+=50/constants::GPS_L1_WAVELENGTH;
    const auto masked=FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities);
    ASSERT_EQ(masked.undifferenced_doppler_factors.size(),2U);
    EXPECT_EQ(masked.undifferenced_doppler_factors[0].epoch_index,0U);
    EXPECT_EQ(masked.undifferenced_doppler_factors[1].epoch_index,2U);
}

TEST(FGORefinementBuilderTest, SourcePhasesCenterCachedRowsBeforeElevationAdmission) {
    const auto nav=makeSyntheticGpsNavigation(5);
    const Vector3d receiver(1113194.0,-4841695.0,3985350.0);
    ObservationData epoch(GNSSTime(2300,100100.0));
    epoch.receiver_position=receiver; epoch.receiver_clock_drift_mps=0;
    epoch.raw_source_index=0; epoch.raw_utc_time_millis=1700000000000LL;
    for (uint8_t prn=1;prn<=5;++prn) {
        Observation obs;
        ASSERT_TRUE(makeSyntheticGpsL1Observation(nav,SatelliteId(GNSSSystem::GPS,prn),epoch.time,receiver,0,obs));
        epoch.observations.push_back(obs);
    }
    FGOProcessor::FGOConfig config;
    config.use_spp_seed=false; config.use_tdcp_factors=false;
    config.use_ionosphere_model=false; config.use_troposphere_model=false;
    config.use_upstream_observable_quality=true; config.upstream_min_elevation_deg=-90;
    config.use_native_phase171_raw_p_no_doppler_imu_main=true;
    config.use_native_source_clock_c0d_factor=true;
    config.use_corrected_undifferenced_doppler_factors=true;
    config.retain_sparse_epochs_for_imu=true; config.min_satellites_per_epoch=0;
    const std::vector<Vector3d> velocities{Vector3d::Zero()};
    // The synthetic input does not include the builder's Earth-rotation
    // correction. Calibrate from the pre-residual pool, since that correction
    // alone can exceed the final residual limit on this artificial geometry.
    config.retain_native_pseudorange_remasking_pool=true;
    auto baseline=FGOProcessor(config).buildPseudorangeProblem({epoch},nav,velocities);
    ASSERT_EQ(baseline.native_pseudorange_remasking_pool.size(),5U);
    auto rows=baseline.native_pseudorange_remasking_pool;
    config.retain_native_pseudorange_remasking_pool=false;
    std::sort(rows.begin(),rows.end(),[](const auto& a,const auto& b){return a.elevation_rad<b.elevation_rad;});
    ASSERT_GT(rows[2].elevation_rad-rows[1].elevation_rad,1e-5);
    const std::array<double,5> residuals{-100,-90,0,19,29};
    for (std::size_t i=0;i<rows.size();++i) {
        auto& obs=epoch.observations.at(rows[i].satellite.prn-1);
        obs.pseudorange+=residuals[i]-rows[i].upstream_seed_residual_m;
    }
    config.upstream_min_elevation_deg=(rows[1].elevation_rad+rows[2].elevation_rad)*0.5*180.0/std::acos(-1.0);
    const auto legacy=FGOProcessor(config).buildPseudorangeProblem({epoch},nav,velocities);
    ASSERT_EQ(legacy.pseudorange_factors.size(),3U);
    EXPECT_TRUE(legacy.diagnostics.native_pseudorange_center_audit.empty());
    config.native_imu_observation_phase=NativeImuObservationPhase::Final;
    const auto final=FGOProcessor(config).buildPseudorangeProblem({epoch},nav,velocities);
    ASSERT_EQ(final.pseudorange_factors.size(),2U);
    ASSERT_EQ(final.diagnostics.native_pseudorange_center_audit.size(),1U);
    const auto& audit=final.diagnostics.native_pseudorange_center_audit.front();
    EXPECT_EQ(audit.cached_rows,5U); EXPECT_EQ(audit.admitted_rows,3U);
    EXPECT_NEAR(audit.cached_center_m,0,0.01); EXPECT_NEAR(audit.admitted_center_m,19,0.01);
    EXPECT_DOUBLE_EQ(audit.selected_threshold_m,20);
    config.native_imu_observation_phase=NativeImuObservationPhase::Initialization;
    const auto initial=FGOProcessor(config).buildPseudorangeProblem({epoch},nav,velocities);
    EXPECT_EQ(initial.pseudorange_factors.size(),3U);
    EXPECT_DOUBLE_EQ(initial.diagnostics.native_pseudorange_center_audit.front().selected_threshold_m,50);
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem({epoch},nav),std::invalid_argument);
    // The same source initialization limits also apply to the cold GNSS graph.
    config.use_native_phase171_raw_p_no_doppler_imu_main=false;
    config.use_native_raw_p_ecef_doppler_gnss_first=true;
    config.use_imu=false;
    const auto cold=FGOProcessor(config).buildPseudorangeProblem({epoch},nav,velocities);
    EXPECT_EQ(cold.pseudorange_factors.size(),3U);
    ASSERT_EQ(cold.diagnostics.native_pseudorange_center_audit.size(),1U);
    EXPECT_EQ(cold.diagnostics.native_pseudorange_center_audit.front().cached_rows,5U);
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

// --- CMC (Code-Minus-Carrier) screening test helper ---
//
// Builds a fixed-position, two-satellite (GPS PRN1/PRN2) epoch stream where
// the per-satellite, per-epoch carrier bias is fully controlled. Reusing
// makeSyntheticGpsL1Observation's construction (carrier_phase = (pseudorange
// + carrier_bias_m) / wavelength) means the receiver's own geometric
// pseudorange term cancels exactly out of the single-difference CMC, so for
// a satellite tracked at both receivers:
//   CMC = (PR_rover - PR_base) - (CP_rover - CP_base) * wavelength
//       = carrier_bias_base_m - carrier_bias_rover_m
// exactly (no residual geometry term), regardless of the rover/base
// baseline. The same bias sequence is applied identically to BOTH
// satellites so the test does not need to predict which one
// select_reference() will pick as the DD reference (elevation-driven, not
// controlled here); whichever one is NOT selected is the one the CMC
// screening actually gates for that (system, signal) group, and it is
// identified after the fact from the built problem.
std::vector<ObservationData> makeCmcObservationEpochs(
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const std::vector<std::array<double, 2>>& carrier_bias_m,
    double dt_s,
    const std::vector<bool>& loss_of_lock_epochs = {}) {
    std::vector<ObservationData> epochs;
    for (std::size_t epoch_index = 0; epoch_index < carrier_bias_m.size();
         ++epoch_index) {
        ObservationData epoch(
            GNSSTime(2300, 100100.0 + dt_s * static_cast<double>(epoch_index)));
        epoch.receiver_position = receiver_position;
        for (std::size_t sat_index = 0; sat_index < 2; ++sat_index) {
            Observation observation;
            const uint8_t prn = static_cast<uint8_t>(sat_index + 1);
            if (makeSyntheticGpsL1Observation(nav,
                                              SatelliteId(GNSSSystem::GPS, prn),
                                              epoch.time,
                                              receiver_position,
                                              carrier_bias_m[epoch_index][sat_index],
                                              observation)) {
                if (epoch_index < loss_of_lock_epochs.size() &&
                    loss_of_lock_epochs[epoch_index]) {
                    observation.loss_of_lock = true;
                }
                epoch.addObservation(observation);
            }
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

std::vector<ObservationData> makeGeometryFreeObservationEpochs(
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const std::vector<double>& l1_bias_m,
    const std::vector<double>& l2_bias_m) {
    std::vector<ObservationData> epochs;
    for (std::size_t epoch_index = 0; epoch_index < l1_bias_m.size();
         ++epoch_index) {
        ObservationData epoch(
            GNSSTime(2300, 100100.0 + static_cast<double>(epoch_index)));
        epoch.receiver_position = receiver_position;
        for (uint8_t prn = 1; prn <= 2; ++prn) {
            Observation l1;
            if (!makeSyntheticGpsL1Observation(
                    nav, SatelliteId(GNSSSystem::GPS, prn), epoch.time,
                    receiver_position, l1_bias_m[epoch_index], l1)) {
                continue;
            }
            l1.has_doppler = true;
            l1.doppler = 0.0;
            epoch.addObservation(l1);
            Observation l2 = l1;
            l2.signal = SignalType::GPS_L2C;
            l2.carrier_phase =
                (l2.pseudorange + l2_bias_m[epoch_index]) /
                constants::GPS_L2_WAVELENGTH;
            epoch.addObservation(l2);
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

// Common config for CMC integration tests: minimal DD-only problem, all
// satellites pass (no elevation/SNR gating gets in the way of a controlled
// synthetic scenario), matching the style of
// BuiltSingleDifferenceTdcpFactorsUseCurrentEpochLos above.
FGOProcessor::FGOConfig makeCmcTestConfig() {
    FGOProcessor::FGOConfig config;
    config.use_spp_seed = false;
    config.use_pseudorange_factors = false;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_double_difference_factors = true;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;
    config.max_tdcp_gap_s = 10.0;
    config.use_code_minus_carrier_screening = true;
    config.code_minus_carrier_jump_threshold_m = 1.0;
    config.code_minus_carrier_level_threshold_m = 0.0;  // off unless a test opts in
    return config;
}

// Identifies, from a built DD problem, the (satellite, signal) that the CMC
// screening actually gates -- i.e. whichever of PRN1/PRN2 was NOT picked as
// the per-epoch DD reference -- and returns its ambiguity_index (or
// SIZE_MAX if absent) plus whether a DD carrier/pseudorange factor exists,
// for every requested epoch index.
struct TrackedSatelliteRow {
    bool has_carrier_factor = false;
    bool has_pseudorange_factor = false;
    std::size_t ambiguity_index = std::numeric_limits<std::size_t>::max();
};

std::map<std::size_t, TrackedSatelliteRow> trackedSatelliteRowsByEpoch(
    const FGOProcessor::FGOProblem& problem,
    SatelliteId* out_tracked_satellite = nullptr) {
    SatelliteId tracked;
    bool found_tracked = false;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        tracked = factor.satellite;
        found_tracked = true;
        break;
    }
    std::map<std::size_t, TrackedSatelliteRow> rows;
    if (!found_tracked) {
        return rows;
    }
    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.satellite == tracked) {
            rows[factor.epoch_index].has_carrier_factor = true;
            rows[factor.epoch_index].ambiguity_index = factor.ambiguity_index;
        }
    }
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        if (factor.satellite == tracked) {
            rows[factor.epoch_index].has_pseudorange_factor = true;
        }
    }
    if (out_tracked_satellite != nullptr) {
        *out_tracked_satellite = tracked;
    }
    return rows;
}

}  // namespace

TEST(FGOTest, EmptyProblemProducesNoSolutions) {
    FGOProcessor processor;
    const auto result = processor.optimizeProblem(FGOProcessor::FGOProblem{});

    EXPECT_TRUE(result.solution.isEmpty());
    EXPECT_EQ(result.diagnostics.epochs, 0u);
    EXPECT_EQ(result.diagnostics.pseudorange_factors, 0u);
}

TEST(FGOTest, SparseEpochRecoveryIsOptInAndRetainsBelowFloorEpochs) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const Vector3d receiver_position(1113194.0, -4841695.0, 3985350.0);
    std::vector<ObservationData> epochs;
    for (std::size_t epoch_index = 0; epoch_index < 2; ++epoch_index) {
        ObservationData epoch(GNSSTime(2300, 100100.0 + epoch_index));
        epoch.receiver_position = receiver_position;
        const std::size_t satellite_count = epoch_index == 0 ? 4 : 3;
        for (std::size_t prn = 1; prn <= satellite_count; ++prn) {
            Observation observation;
            ASSERT_TRUE(makeSyntheticGpsL1Observation(
                nav, SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn)),
                epoch.time, receiver_position, 0.0, observation));
            epoch.addObservation(observation);
        }
        epochs.push_back(std::move(epoch));
    }

    FGOProcessor::FGOConfig default_config;
    default_config.use_multi_constellation = false;
    default_config.use_motion_factors = false;
    default_config.use_tdcp_factors = false;
    default_config.use_carrier_phase_factors = false;
    default_config.use_ionosphere_model = false;
    default_config.use_troposphere_model = false;
    default_config.min_elevation_deg = -90.0;
    default_config.min_snr_dbhz = 0.0;
    default_config.min_satellites_per_epoch = 4;

    const auto baseline = FGOProcessor(default_config).buildPseudorangeProblem(
        epochs, nav);
    EXPECT_EQ(baseline.epochs.size(), 1u);
    EXPECT_EQ(baseline.diagnostics.sparse_epochs_retained, 0u);

    auto recovery_config = default_config;
    recovery_config.retain_sparse_epochs_for_imu = true;
    const auto recovery = FGOProcessor(recovery_config).buildPseudorangeProblem(
        epochs, nav);
    EXPECT_EQ(recovery.epochs.size(), 2u);
    EXPECT_EQ(recovery.diagnostics.sparse_epochs_retained, 1u);
    EXPECT_EQ(recovery.diagnostics.sparse_empty_epochs_retained, 0u);
    ASSERT_EQ(recovery.pseudorange_factors.size(), 7u);
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
    EXPECT_EQ(result.diagnostics.tdcp_factors_inserted, 6u);
    EXPECT_EQ(result.diagnostics.tdcp_candidate_pairs, 6u);
    EXPECT_LT(result.diagnostics.residual_rms_m, 1e-5);
    EXPECT_LT(result.diagnostics.tdcp_residual_rms_m, 1e-5);

    const Vector3d expected_second(1113196.5, -4841694.0, 3985349.4);
    EXPECT_LT((result.solution.solutions[1].position_ecef - expected_second).norm(), 1e-3);
}

TEST(FGOSourceRoverStateTest, BuildsSharedStatesAndKeepsNavigationMissLocal) {
    auto nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> positions = {
        Vector3d(1113194.0,-4841695.0,3985350.0),
        Vector3d(1113196.0,-4841694.0,3985351.0)};
    auto observations = makeSyntheticDoubleDifferenceObservationEpochs(nav, positions, 0.0);
    for (auto& epoch : observations) {
        for (auto& row : epoch.observations) row.pseudorange_observation_type = "C1C";
    }
    FGOProcessor::FGOConfig c;
    EXPECT_FALSE(c.use_source_rover_epoch_states);
    c.use_source_rover_epoch_states = true;
    c.use_spp_seed = false;
    c.use_ionosphere_model = c.use_troposphere_model = false;
    c.min_elevation_deg = -90;
    c.min_satellites_per_epoch = 1;
    const auto complete = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    EXPECT_EQ(complete.diagnostics.source_rover_epoch_states_built, 8U);
    EXPECT_EQ(complete.diagnostics.source_rover_missing_ephemeris_satellite_epochs, 0U);
    ASSERT_EQ(complete.pseudorange_factors.size(), 8U);
    auto dual = observations;
    auto l5 = dual.front().observations.front();
    l5.signal = SignalType::GPS_L5;
    l5.pseudorange_observation_type = "C5I";
    l5.pseudorange += 300.0;
    l5.has_carrier_phase = false;
    dual.front().observations.push_back(l5);
    c.use_multi_constellation = c.use_multi_frequency_double_difference = true;
    const auto shared = FGOProcessor(c).buildPseudorangeProblem(dual, nav);
    EXPECT_EQ(shared.diagnostics.source_rover_epoch_states_built, 8U);
    ASSERT_EQ(shared.pseudorange_factors.size(), 9U);
    const FGOProcessor::PseudorangeFactor* first = nullptr;
    const FGOProcessor::PseudorangeFactor* second = nullptr;
    for (const auto& factor : shared.pseudorange_factors) {
        if (factor.epoch_index != 0 || !(factor.satellite == l5.satellite)) continue;
        if (factor.signal == SignalType::GPS_L1CA) first = &factor;
        if (factor.signal == SignalType::GPS_L5) second = &factor;
    }
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    EXPECT_TRUE(first->source_satellite_position_ecef.isApprox(
        second->source_satellite_position_ecef, 0.0));
    c.use_multi_constellation = c.use_multi_frequency_double_difference = false;
    nav.ephemeris_data.erase(SatelliteId(GNSSSystem::GPS, 4));
    const auto partial = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    EXPECT_EQ(partial.diagnostics.source_rover_epoch_states_built, 6U);
    EXPECT_EQ(partial.diagnostics.source_rover_missing_ephemeris_satellite_epochs, 2U);
    EXPECT_EQ(partial.pseudorange_factors.size(), 6U);
    // A malformed code is not a missing-nav event and must not silently drop.
    observations.front().observations.front().pseudorange_observation_type.clear();
    EXPECT_THROW(FGOProcessor(c).buildPseudorangeProblem(observations, nav),
                 std::invalid_argument);
}

TEST(FGORemaskingPoolTest, BuilderRetainsPreMaskRowsWithoutChangingAcceptedFactors) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> positions = {
        Vector3d(1113194.0,-4841695.0,3985350.0),
        Vector3d(1113196.0,-4841694.0,3985351.0)};
    const auto observations = makeSyntheticDoubleDifferenceObservationEpochs(nav, positions, 0.0);
    FGOProcessor::FGOConfig c;
    c.use_spp_seed = false;
    c.use_ionosphere_model = false;
    c.use_troposphere_model = false;
    c.min_elevation_deg = -90.0;
    c.min_satellites_per_epoch = 1;
    c.use_upstream_observable_quality = true;
    const auto baseline = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    c.retain_native_pseudorange_remasking_pool = true;
    const auto retained = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    EXPECT_TRUE(baseline.native_pseudorange_remasking_pool.empty());
    ASSERT_FALSE(retained.native_pseudorange_remasking_pool.empty());
    EXPECT_EQ(retained.native_pseudorange_remasking_pool.size(),
              retained.diagnostics.upstream_pseudorange_candidates);
    EXPECT_EQ(retained.native_pseudorange_remasking_pool.size(),
              retained.pseudorange_factors.size() +
              retained.diagnostics.upstream_pseudorange_residual_rejections);
    ASSERT_EQ(baseline.pseudorange_factors.size(),retained.pseudorange_factors.size());
    for (std::size_t i=0;i<baseline.pseudorange_factors.size();++i) {
        const auto& a=baseline.pseudorange_factors[i];
        const auto& b=retained.pseudorange_factors[i];
        EXPECT_EQ(a.epoch_index,b.epoch_index);
        EXPECT_EQ(a.satellite,b.satellite);
        EXPECT_EQ(a.signal,b.signal);
        EXPECT_DOUBLE_EQ(a.corrected_pseudorange_m,b.corrected_pseudorange_m);
        EXPECT_DOUBLE_EQ(a.sigma_m,b.sigma_m);
        EXPECT_DOUBLE_EQ(a.upstream_seed_residual_m,b.upstream_seed_residual_m);
        EXPECT_TRUE(a.satellite_position_ecef==b.satellite_position_ecef);
    }
}

TEST(FGOTest, OfficialTdcpWeightingPreservesAcceptedPairStructure) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> receiver_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.0, -4841694.0, 3985351.0)};
    const auto observations = makeSyntheticDoubleDifferenceObservationEpochs(
        nav, receiver_positions, 0.0);

    FGOProcessor::FGOConfig legacy_config;
    legacy_config.use_spp_seed = false;
    legacy_config.use_motion_factors = false;
    legacy_config.use_position_motion_factors = false;
    legacy_config.use_clock_motion_factors = false;
    legacy_config.use_ionosphere_model = false;
    legacy_config.use_troposphere_model = false;
    legacy_config.min_elevation_deg = -90.0;
    legacy_config.min_snr_dbhz = 0.0;
    legacy_config.min_satellites_per_epoch = 4;
    legacy_config.use_carrier_tdcp_incidence_diagnostic = true;
    legacy_config.tdcp_sigma_m = 0.03;

    FGOProcessor::FGOConfig official_config = legacy_config;
    official_config.use_official_tdcp_snr_type_sigma = true;

    const auto legacy = FGOProcessor(legacy_config).buildPseudorangeProblem(
        observations, nav);
    const auto official = FGOProcessor(official_config).buildPseudorangeProblem(
        observations, nav);
    ASSERT_EQ(legacy.tdcp_factors.size(), 4U);
    ASSERT_EQ(official.tdcp_factors.size(), legacy.tdcp_factors.size());
    EXPECT_EQ(official.diagnostics.tdcp_candidate_pairs,
              legacy.diagnostics.tdcp_candidate_pairs);
    EXPECT_EQ(official.diagnostics.tdcp_rejected_invalid_weight, 0U);

    const double expected_sigma_m =
        0.8 / 400.0 * constants::GPS_L1_WAVELENGTH;
    for (std::size_t i = 0; i < legacy.tdcp_factors.size(); ++i) {
        const auto& before = legacy.tdcp_factors[i];
        const auto& after = official.tdcp_factors[i];
        EXPECT_EQ(after.previous_epoch_index, before.previous_epoch_index);
        EXPECT_EQ(after.current_epoch_index, before.current_epoch_index);
        EXPECT_EQ(after.satellite, before.satellite);
        EXPECT_EQ(after.signal, before.signal);
        EXPECT_DOUBLE_EQ(after.delta_carrier_m, before.delta_carrier_m);
        EXPECT_DOUBLE_EQ(before.sigma_m, 0.03);
        EXPECT_NEAR(after.sigma_m, expected_sigma_m, 1e-12);
    }

    // A missing raw SNR is a weighting-contract failure, not a reason to
    // silently restore the legacy 0.03 m value for that pair.
    auto missing_snr_observations = observations;
    missing_snr_observations.front().observations.front().snr = 0.0;
    const auto missing_snr =
        FGOProcessor(official_config).buildPseudorangeProblem(
            missing_snr_observations, nav);
    EXPECT_EQ(missing_snr.diagnostics.tdcp_rejected_invalid_weight, 1U);
    EXPECT_EQ(missing_snr.tdcp_factors.size(), official.tdcp_factors.size() - 1U);
}

TEST(FGOTest, SourceMeterTdcpWeightingPreservesAcceptedPairStructure) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> receiver_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.0, -4841694.0, 3985351.0)};
    const auto observations = makeSyntheticDoubleDifferenceObservationEpochs(
        nav, receiver_positions, 0.0);

    FGOProcessor::FGOConfig legacy_config;
    legacy_config.use_spp_seed = false;
    legacy_config.use_motion_factors = false;
    legacy_config.use_position_motion_factors = false;
    legacy_config.use_clock_motion_factors = false;
    legacy_config.use_ionosphere_model = false;
    legacy_config.use_troposphere_model = false;
    legacy_config.min_elevation_deg = -90.0;
    legacy_config.min_snr_dbhz = 0.0;
    legacy_config.min_satellites_per_epoch = 4;
    legacy_config.use_carrier_tdcp_incidence_diagnostic = true;
    legacy_config.tdcp_sigma_m = 0.03;

    FGOProcessor::FGOConfig official_config = legacy_config;
    official_config.use_source_tdcp_meter_sigma = true;

    auto unequal_snr = observations;
    for (auto& o : unequal_snr.front().observations) o.snr = 40.0;
    for (auto& o : unequal_snr.back().observations) o.snr = 60.0;
    const auto previous_endpoint = FGOProcessor(official_config).buildPseudorangeProblem(
        unequal_snr, nav);
    ASSERT_EQ(previous_endpoint.tdcp_factors.size(), 4U);
    // Band p85 is 60 dB-Hz; previous endpoint 40 gives 10 * 0.002 m.
    // Using the current endpoint would incorrectly give 0.002 m.
    for (const auto& factor : previous_endpoint.tdcp_factors) {
        EXPECT_NEAR(factor.sigma_m, 0.02, 1e-12);
    }

    const auto legacy = FGOProcessor(legacy_config).buildPseudorangeProblem(
        observations, nav);
    const auto official = FGOProcessor(official_config).buildPseudorangeProblem(
        observations, nav);
    ASSERT_EQ(legacy.tdcp_factors.size(), 4U);
    ASSERT_EQ(official.tdcp_factors.size(), legacy.tdcp_factors.size());
    EXPECT_EQ(official.diagnostics.tdcp_candidate_pairs,
              legacy.diagnostics.tdcp_candidate_pairs);
    EXPECT_EQ(official.diagnostics.tdcp_rejected_invalid_weight, 0U);

    const double expected_sigma_m =
        0.8 / 400.0;
    for (std::size_t i = 0; i < legacy.tdcp_factors.size(); ++i) {
        const auto& before = legacy.tdcp_factors[i];
        const auto& after = official.tdcp_factors[i];
        EXPECT_EQ(after.previous_epoch_index, before.previous_epoch_index);
        EXPECT_EQ(after.current_epoch_index, before.current_epoch_index);
        EXPECT_EQ(after.satellite, before.satellite);
        EXPECT_EQ(after.signal, before.signal);
        EXPECT_DOUBLE_EQ(after.delta_carrier_m, before.delta_carrier_m);
        EXPECT_DOUBLE_EQ(before.sigma_m, 0.03);
        EXPECT_NEAR(after.sigma_m, expected_sigma_m, 1e-12);
    }

    // A missing raw SNR is a weighting-contract failure, not a reason to
    // silently restore the legacy 0.03 m value for that pair.
    auto missing_snr_observations = observations;
    missing_snr_observations.front().observations.front().snr = 0.0;
    const auto missing_snr =
        FGOProcessor(official_config).buildPseudorangeProblem(
            missing_snr_observations, nav);
    EXPECT_EQ(missing_snr.diagnostics.tdcp_rejected_invalid_weight, 1U);
    EXPECT_EQ(missing_snr.tdcp_factors.size(), official.tdcp_factors.size() - 1U);
}

TEST(FGOTest, SourceReslObservablePreservesPairsAndDynamicSigma) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    // Equatorial surface points keep the atmosphere model's height gate
    // active; the historical fixture ECEF points are not surface-validated.
    const std::array<Vector3d, 2> positions = {
        Vector3d(6378237.0, 0.0, 0.0),
        Vector3d(6378239.0, 1.0, 1.0)};
    const auto observations = makeSyntheticDoubleDifferenceObservationEpochs(nav, positions, 0.0);
    FGOProcessor::FGOConfig c;
    EXPECT_FALSE(c.use_source_tdcp_resl_observable);
    c.use_spp_seed = false;
    c.use_motion_factors = false;
    c.use_position_motion_factors = false;
    c.use_clock_motion_factors = false;
    c.use_ionosphere_model = false;
    c.use_troposphere_model = true;
    c.min_elevation_deg = -90.0;
    c.min_snr_dbhz = 0.0;
    c.min_satellites_per_epoch = 4;
    c.use_carrier_tdcp_incidence_diagnostic = true;
    c.use_source_tdcp_meter_sigma = true;
    const auto before = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    c.use_source_tdcp_resl_observable = true;
    const auto after = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    // Independently exercise the existing observable switch as a builder
    // reference, without its incompatible dynamic-sigma configuration.
    c.use_source_tdcp_resl_observable = false;
    c.use_source_tdcp_meter_sigma = false;
    c.use_official_tdcp_resl_atmosphere_cancellation = true;
    const auto reference = FGOProcessor(c).buildPseudorangeProblem(observations, nav);
    ASSERT_EQ(before.tdcp_factors.size(), 4U);
    ASSERT_EQ(after.tdcp_factors.size(), before.tdcp_factors.size());
    ASSERT_EQ(reference.tdcp_factors.size(), before.tdcp_factors.size());
    bool measurement_changed = false;
    for (std::size_t i = 0; i < before.tdcp_factors.size(); ++i) {
        const auto& a = after.tdcp_factors[i];
        const auto& b = before.tdcp_factors[i];
        EXPECT_EQ(a.previous_epoch_index, b.previous_epoch_index);
        EXPECT_EQ(a.current_epoch_index, b.current_epoch_index);
        EXPECT_EQ(a.satellite, b.satellite);
        EXPECT_EQ(a.signal, b.signal);
        EXPECT_DOUBLE_EQ(a.sigma_m, b.sigma_m);
        EXPECT_DOUBLE_EQ(a.delta_carrier_m, reference.tdcp_factors[i].delta_carrier_m);
        measurement_changed |= a.delta_carrier_m != b.delta_carrier_m;
    }
    EXPECT_TRUE(measurement_changed);
    EXPECT_EQ(after.diagnostics.tdcp_candidate_pairs, before.diagnostics.tdcp_candidate_pairs);
    c.use_source_tdcp_resl_observable = true;
    EXPECT_THROW(FGOProcessor(c).buildPseudorangeProblem({}, nav), std::invalid_argument);
    EXPECT_THROW(FGOProcessor(c).optimizeProblem(FGOProcessor::FGOProblem{}), std::invalid_argument);
}

TEST(FGOTest, SourceMeterTdcpRejectsMixedSelectors) {
    for (int mode = 0; mode < 3; ++mode) {
        FGOProcessor::FGOConfig c;
        c.use_source_tdcp_meter_sigma = true;
        c.use_official_tdcp_snr_type_sigma = mode == 0;
        c.use_official_tdcp_huber_k = mode == 1;
        c.use_official_tdcp_resl_atmosphere_cancellation = mode == 2;
        EXPECT_THROW(FGOProcessor(c).buildPseudorangeProblem({}, NavigationData{}), std::invalid_argument);
        EXPECT_THROW(FGOProcessor(c).optimizeProblem(FGOProcessor::FGOProblem{}), std::invalid_argument);
    }
}

TEST(FGOTest, OfficialTdcpWeightingIsDefaultOff) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_official_tdcp_snr_type_sigma);
    EXPECT_FALSE(config.use_source_tdcp_meter_sigma);
    EXPECT_DOUBLE_EQ(config.tdcp_sigma_m, 0.03);
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
    EXPECT_EQ(result.diagnostics.single_difference_tdcp_factors_inserted, 5u);
    EXPECT_LT(result.diagnostics.single_difference_doppler_residual_rms_mps,
              1e-5);
    EXPECT_LT(result.diagnostics.single_difference_tdcp_residual_rms_m,
              1e-5);
}

TEST(FGOTest, UndifferencedDopplerFactorsConstrainNoBaseVelocityStates) {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem();
    const auto satellites = makeSatelliteGeometry();
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const double dt = problem.epochs[1].time - problem.epochs[0].time;
    const Vector3d true_velocity = (true_positions[1] - true_positions[0]) / dt;

    for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
        const Vector3d delta =
            satellites[sat] - problem.epochs[1].position_ecef;
        const Vector3d los = delta / delta.norm();
        FGOProcessor::UndifferencedDopplerFactor factor;
        factor.epoch_index = 1;
        factor.satellite =
            SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
        factor.signal = SignalType::GPS_L1CA;
        factor.los = los;
        factor.residual_mps = los.dot(true_velocity);
        factor.sigma_mps = 0.2;
        factor.elevation_rad = 0.7;
        problem.undifferenced_doppler_factors.push_back(factor);
    }

    FGOProcessor::FGOConfig config;
    config.max_iterations = 10;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.use_velocity_states = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_robust_loss = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors, 6u);
    EXPECT_GE(result.diagnostics.graph_factors, 18u);
    EXPECT_LT(result.diagnostics.undifferenced_doppler_residual_rms_mps,
              1e-5);
}

TEST(FGOTest, CorrectedUndifferencedDopplerUsesClockBiasDifference) {
    FGOProcessor::FGOProblem problem = makeSyntheticProblem();
    const auto satellites = makeSatelliteGeometry();
    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.5, -4841694.0, 3985349.4),
    };
    const double dt = problem.epochs[1].time - problem.epochs[0].time;
    const Vector3d true_velocity = (true_positions[1] - true_positions[0]) / dt;
    constexpr double true_receiver_clock_drift_mps = 2.5;

    for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
        const Vector3d delta = satellites[sat] - true_positions[1];
        const Vector3d los_receiver_to_satellite = delta.normalized();
        FGOProcessor::UndifferencedDopplerFactor factor;
        factor.epoch_index = 1;
        factor.previous_epoch_index = 0;
        factor.satellite =
            SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
        factor.signal = SignalType::GPS_L1CA;
        factor.los = -los_receiver_to_satellite;
        factor.residual_mps =
            factor.los.dot(true_velocity) + true_receiver_clock_drift_mps;
        factor.sigma_mps = 0.2;
        factor.elevation_rad = 0.7;
        factor.dt_s = dt;
        factor.includes_receiver_clock_drift = true;
        factor.uses_rotated_satellite_state = true;
        problem.undifferenced_doppler_factors.push_back(factor);
    }

    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.use_velocity_states = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_corrected_undifferenced_doppler_factors = true;
    config.use_robust_loss = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.solution.size(), 2u);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors, 6u);
    // The synthetic problem also contains the deliberately offset
    // pseudorange seeds, so the coupled position/clock solve need not land
    // at machine-zero in one batch.  The contract row must nevertheless
    // converge to a small residual rather than the unmodelled clock-drift
    // scale (2.5 m/s).
    EXPECT_LT(result.diagnostics.undifferenced_doppler_residual_rms_mps,
              0.05);
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

TEST(FGOTest, ExternalDopplerDrMonitorMaterializesShadowFactorsOnly) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> rover_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113194.0, -4841695.0, 3985350.0),
    };
    const std::array<Vector3d, 2> base_positions = {
        rover_positions[0] + Vector3d(-320.0, 180.0, 45.0),
        rover_positions[1] + Vector3d(-320.0, 180.0, 45.0),
    };
    auto rover_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, rover_positions, 0.0);
    auto base_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, base_positions, 0.0);
    for (auto* epochs : {&rover_epochs, &base_epochs}) {
        for (auto& epoch : *epochs) {
            for (auto& observation : epoch.observations) {
                observation.has_doppler = true;
                observation.doppler = 0.0;
            }
        }
    }

    FGOProcessor::FGOConfig config;
    config.use_spp_seed = false;
    config.use_pseudorange_factors = false;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_double_difference_factors = true;
    config.use_single_difference_doppler_factors = false;
    config.monitor_external_doppler_dr = true;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;

    const auto problem = FGOProcessor(config).buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_positions[0]);

    EXPECT_FALSE(problem.single_difference_doppler_factors.empty());
    EXPECT_FALSE(config.use_single_difference_doppler_factors);
}

TEST(FGOTest, CandidateIntegrityMonitorMaterializesShadowFactorsOnly) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> rover_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113194.0, -4841695.0, 3985350.0),
    };
    const std::array<Vector3d, 2> base_positions = {
        rover_positions[0] + Vector3d(-320.0, 180.0, 45.0),
        rover_positions[1] + Vector3d(-320.0, 180.0, 45.0),
    };
    auto rover_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, rover_positions, 0.0);
    auto base_epochs =
        makeSyntheticDoubleDifferenceObservationEpochs(nav, base_positions, 0.0);
    for (auto* epochs : {&rover_epochs, &base_epochs}) {
        for (auto& epoch : *epochs) {
            for (auto& observation : epoch.observations) {
                observation.has_doppler = true;
                observation.doppler = 0.0;
            }
        }
    }

    FGOProcessor::FGOConfig config;
    config.use_spp_seed = false;
    config.use_pseudorange_factors = false;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_double_difference_factors = true;
    config.use_single_difference_doppler_factors = false;
    config.monitor_candidate_integrity_witness = true;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;

    const auto problem = FGOProcessor(config).buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_positions[0]);

    EXPECT_FALSE(problem.single_difference_doppler_factors.empty());
    EXPECT_FALSE(config.use_single_difference_doppler_factors);
    EXPECT_FALSE(config.monitor_external_doppler_dr);
}

TEST(FGOTest, ClockResilientTemporalCarrierShadowCancelsCommonClockJump) {
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
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_positions[0]);
    const auto baseline =
        FGOProcessor::buildClockResilientTemporalCarrierShadow(problem);
    ASSERT_FALSE(baseline.empty());

    auto jumped_problem = problem;
    constexpr double clock_jump_m = constants::SPEED_OF_LIGHT * 0.001;
    for (auto& observation : jumped_problem.carrier_observations) {
        if (observation.epoch_index == 1) {
            observation.corrected_carrier_m += clock_jump_m;
        }
    }
    const auto jumped =
        FGOProcessor::buildClockResilientTemporalCarrierShadow(jumped_problem);
    ASSERT_EQ(jumped.size(), baseline.size());

    using ObservationKey = std::tuple<std::size_t, SatelliteId, SignalType>;
    std::map<ObservationKey, const FGOProcessor::CarrierPhaseFactor*> observations;
    for (const auto& observation :
         problem.double_difference_pseudorange_observations) {
        observations[{observation.epoch_index, observation.satellite,
                      observation.signal}] = &observation;
    }

    double max_previous_current_los_delta = 0.0;
    for (std::size_t i = 0; i < baseline.size(); ++i) {
        const auto& factor = baseline[i];
        const auto& jumped_factor = jumped[i];
        EXPECT_EQ(jumped_factor.satellite, factor.satellite);
        EXPECT_EQ(jumped_factor.reference_satellite,
                  factor.reference_satellite);
        EXPECT_EQ(jumped_factor.signal, factor.signal);
        EXPECT_NEAR(jumped_factor.delta_carrier_m, factor.delta_carrier_m,
                    1e-9);

        const auto previous_satellite = observations.at(
            {factor.previous_epoch_index, factor.satellite, factor.signal});
        const auto previous_reference = observations.at(
            {factor.previous_epoch_index, factor.reference_satellite,
             factor.signal});
        const auto current_satellite = observations.at(
            {factor.current_epoch_index, factor.satellite, factor.signal});
        const auto current_reference = observations.at(
            {factor.current_epoch_index, factor.reference_satellite,
             factor.signal});
        const Vector3d previous_los =
            previous_satellite->los - previous_reference->los;
        const Vector3d current_los =
            current_satellite->los - current_reference->los;
        EXPECT_LT((factor.previous_los - previous_los).norm(), 1e-12);
        EXPECT_LT((factor.los - current_los).norm(), 1e-12);
        EXPECT_EQ(factor.target_ambiguity_index,
                  current_satellite->ambiguity_index);
        EXPECT_EQ(factor.reference_ambiguity_index,
                  current_reference->ambiguity_index);
        EXPECT_EQ(factor.arc_length_epochs, 2);
        max_previous_current_los_delta = std::max(
            max_previous_current_los_delta,
            (factor.previous_los - factor.los).norm());
    }
    EXPECT_GT(max_previous_current_los_delta, 1e-8);

    auto switched_reference_problem = problem;
    std::set<std::pair<SatelliteId, SignalType>> initial_references;
    for (const auto& factor : baseline) {
        initial_references.insert(
            {factor.reference_satellite, factor.signal});
    }
    for (auto& observation : switched_reference_problem.carrier_observations) {
        if (observation.epoch_index == 1 &&
            initial_references.count(
                {observation.satellite, observation.signal}) != 0) {
            observation.has_carrier_phase = false;
        }
    }
    const auto after_reference_switch =
        FGOProcessor::buildClockResilientTemporalCarrierShadow(
            switched_reference_problem);
    EXPECT_TRUE(after_reference_switch.empty())
        << "a new reference must start a fresh temporal arc";
}

TEST(FGOTest, ReportsSignedCommonGpsPseudorangeDeltaForClockAudit) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> rover_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113194.0, -4841695.0, 3985350.0),
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
    config.use_pseudorange_factors = true;
    config.use_double_difference_factors = true;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_satellites_per_epoch = 2;

    constexpr double jump_m = constants::SPEED_OF_LIGHT * 0.001;
    const auto build_with_jump = [&](double jump) {
        auto jumped = rover_epochs;
        for (auto& observation : jumped[1].observations) {
            observation.pseudorange += jump;
        }
        return FGOProcessor(config).buildDoubleDifferenceProblem(
            jumped, base_epochs, nav, base_positions[0]);
    };

    const auto positive = build_with_jump(jump_m);
    ASSERT_EQ(positive.gps_common_pseudorange_delta_m.size(), 2u);
    ASSERT_EQ(positive.gps_common_pseudorange_delta_satellites.size(), 2u);
    EXPECT_GT(positive.gps_common_pseudorange_delta_m[1], 1e5);
    EXPECT_GE(positive.gps_common_pseudorange_delta_satellites[1], 2);

    const auto negative = build_with_jump(-jump_m);
    ASSERT_EQ(negative.gps_common_pseudorange_delta_m.size(), 2u);
    EXPECT_LT(negative.gps_common_pseudorange_delta_m[1], -1e5);
    EXPECT_GE(negative.gps_common_pseudorange_delta_satellites[1], 2);
}

TEST(FGOTest, PredictedDdprQualityShadowIsClockSafeAndFailClosed) {
    FGOProcessor::FGOProblem problem;
    problem.epochs.resize(2);
    problem.epochs[0].time = GNSSTime(2300, 100000.0);
    problem.epochs[1].time = GNSSTime(2300, 100001.0);

    const Vector3d rover0(1113194.0, -4841695.0, 3985350.0);
    const Vector3d rover1 = rover0 + Vector3d(12.0, 4.0, -1.0);
    const Vector3d base = rover0 + Vector3d(-320.0, 180.0, 45.0);
    const Vector3d target_satellite(15600000.0, 7540000.0, 20140000.0);
    const Vector3d reference_satellite(-18760000.0, 2750000.0,
                                        18610000.0);
    const auto dd_range = [&](const Vector3d& rover) {
        return ((target_satellite - rover).norm() -
                (target_satellite - base).norm()) -
               ((reference_satellite - rover).norm() -
                (reference_satellite - base).norm());
    };
    const double geometry0 = dd_range(rover0);
    const double geometry1 = dd_range(rover1);
    const double dd_rate = geometry1 - geometry0;

    const auto make_factor = [&](std::size_t epoch, double geometry,
                                 double rover_clock_m,
                                 double base_clock_m) {
        FGOProcessor::DoubleDifferencePseudorangeFactor factor;
        factor.epoch_index = epoch;
        factor.satellite = SatelliteId(GNSSSystem::GPS, 5);
        factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 11);
        factor.signal = SignalType::GPS_L1CA;
        factor.rover_satellite_position_ecef = target_satellite;
        factor.rover_reference_position_ecef = reference_satellite;
        factor.base_satellite_position_ecef = target_satellite;
        factor.base_reference_position_ecef = reference_satellite;
        factor.base_position_ecef = base;
        const double rover_target = (target_satellite -
            (epoch == 0 ? rover0 : rover1)).norm();
        const double rover_reference = (reference_satellite -
            (epoch == 0 ? rover0 : rover1)).norm();
        const double base_target = (target_satellite - base).norm();
        const double base_reference = (reference_satellite - base).norm();
        factor.observed_dd_pseudorange_m =
            ((rover_target + rover_clock_m) -
             (base_target + base_clock_m)) -
            ((rover_reference + rover_clock_m) -
             (base_reference + base_clock_m));
        EXPECT_NEAR(factor.observed_dd_pseudorange_m, geometry, 1e-8);
        factor.sigma_m = 0.5;
        factor.rover_satellite_model.has_doppler_residual = true;
        factor.rover_reference_model.has_doppler_residual = true;
        factor.base_satellite_model.has_doppler_residual = true;
        factor.base_reference_model.has_doppler_residual = true;
        factor.rover_satellite_model.doppler_residual_mps = dd_rate;
        factor.rover_satellite_model.elevation_rad = 0.8;
        factor.rover_reference_model.elevation_rad = 0.9;
        factor.rover_satellite_model.snr_dbhz = 42.0;
        factor.rover_reference_model.snr_dbhz = 44.0;
        return factor;
    };
    problem.double_difference_pseudorange_factors.push_back(
        make_factor(0, geometry0, 25.0, -10.0));
    // A roughly one-millisecond common rover clock jump cancels in the DD.
    problem.double_difference_pseudorange_factors.push_back(
        make_factor(1, geometry1, 299817.458, -10.0));

    const std::vector<Vector3d> solved = {rover0, rover1};
    const std::vector<Vector3d> predicted = {rover0, rover1};
    const auto clean = FGOProcessor::analyzePredictedDdprQualityShadow(
        problem, solved, predicted, 0.2, 5.0, 1.5);
    ASSERT_EQ(clean.size(), 1u);
    EXPECT_TRUE(clean[0].doppler_evaluated);
    EXPECT_TRUE(clean[0].imu_geometry_evaluated);
    EXPECT_NEAR(clean[0].doppler_innovation_m, 0.0, 1e-8);
    EXPECT_NEAR(clean[0].imu_innovation_m, 0.0, 1e-8);
    EXPECT_NEAR(clean[0].previous_predicted_ddpr_residual_m, 0.0, 1e-8);
    EXPECT_NEAR(clean[0].current_predicted_ddpr_residual_m, 0.0, 1e-8);
    EXPECT_EQ(clean[0].pair_age_epochs, 2);
    EXPECT_EQ(clean[0].proposed_action,
              FGOProcessor::PredictedDdprQualityAction::Keep);

    auto outlier_problem = problem;
    outlier_problem.double_difference_pseudorange_factors[1]
        .observed_dd_pseudorange_m += 20.0;
    const auto outlier = FGOProcessor::analyzePredictedDdprQualityShadow(
        outlier_problem, solved, predicted, 0.2, 5.0, 1.5);
    ASSERT_EQ(outlier.size(), 1u);
    EXPECT_GT(outlier[0].normalized_doppler_innovation, 5.0);
    EXPECT_GT(outlier[0].normalized_imu_innovation, 5.0);
    EXPECT_NEAR(outlier[0].current_predicted_ddpr_residual_m, 20.0, 1e-8);
    EXPECT_EQ(outlier[0].proposed_action,
              FGOProcessor::PredictedDdprQualityAction::Downweight);

    auto unavailable_problem = problem;
    for (auto& factor :
         unavailable_problem.double_difference_pseudorange_factors) {
        factor.rover_satellite_model.has_doppler_residual = false;
        factor.rover_reference_model.has_doppler_residual = false;
        factor.base_satellite_model.has_doppler_residual = false;
        factor.base_reference_model.has_doppler_residual = false;
    }
    const std::vector<Vector3d> unavailable_positions(2, Vector3d::Zero());
    const auto unavailable =
        FGOProcessor::analyzePredictedDdprQualityShadow(
            unavailable_problem, unavailable_positions,
            unavailable_positions);
    ASSERT_EQ(unavailable.size(), 1u);
    EXPECT_FALSE(unavailable[0].doppler_evaluated);
    EXPECT_FALSE(unavailable[0].imu_geometry_evaluated);
    EXPECT_EQ(unavailable[0].proposed_action,
              FGOProcessor::PredictedDdprQualityAction::Unavailable);

    auto switched_reference_problem = problem;
    switched_reference_problem.double_difference_pseudorange_factors[1]
        .reference_satellite = SatelliteId(GNSSSystem::GPS, 12);
    const auto switched =
        FGOProcessor::analyzePredictedDdprQualityShadow(
            switched_reference_problem, solved, predicted);
    EXPECT_TRUE(switched.empty());
}

TEST(FGOTest, PredictedDdprBiasStateShadowPredictsPersistentBiasCausally) {
    const auto make_row = [](std::size_t previous_epoch,
                             double residual_m) {
        FGOProcessor::PredictedDdprQualityFactorDiagnostics row;
        row.previous_epoch_index = previous_epoch;
        row.current_epoch_index = previous_epoch + 1;
        row.satellite = SatelliteId(GNSSSystem::GPS, 5);
        row.reference_satellite = SatelliteId(GNSSSystem::GPS, 11);
        row.signal = SignalType::GPS_L1CA;
        row.dt_s = 1.0;
        row.pair_age_epochs = static_cast<int>(previous_epoch + 2);
        row.imu_geometry_evaluated = true;
        row.current_predicted_ddpr_residual_m = residual_m;
        row.imu_innovation_sigma_m = std::sqrt(0.5);
        return row;
    };
    const std::vector<FGOProcessor::PredictedDdprQualityFactorDiagnostics>
        rows = {make_row(0, 10.0), make_row(1, 10.0),
                make_row(2, 10.0), make_row(3, 10.0)};

    const auto diagnostics =
        FGOProcessor::analyzePredictedDdprBiasStateShadow(rows);
    ASSERT_EQ(diagnostics.size(), rows.size());
    EXPECT_TRUE(diagnostics[0].continuity_reset);
    EXPECT_FALSE(diagnostics[0].prediction_usable);
    EXPECT_FALSE(diagnostics[1].prediction_usable);
    ASSERT_TRUE(diagnostics[2].prediction_usable);
    EXPECT_EQ(diagnostics[2].prior_updates, 2);
    EXPECT_NEAR(diagnostics[2].prior_bias_m, 10.0, 0.2);
    EXPECT_LT(std::abs(diagnostics[2].corrected_residual_m), 0.2);
    EXPECT_GT(std::abs(diagnostics[2].raw_residual_m), 9.0);
}

TEST(FGOTest, PredictedDdprBiasStateShadowBoundsImpulseAndResetsOnGap) {
    const auto make_row = [](std::size_t previous_epoch,
                             double residual_m) {
        FGOProcessor::PredictedDdprQualityFactorDiagnostics row;
        row.previous_epoch_index = previous_epoch;
        row.current_epoch_index = previous_epoch + 1;
        row.satellite = SatelliteId(GNSSSystem::GPS, 5);
        row.reference_satellite = SatelliteId(GNSSSystem::GPS, 11);
        row.signal = SignalType::GPS_L1CA;
        row.dt_s = 1.0;
        row.pair_age_epochs = 2;
        row.imu_geometry_evaluated = true;
        row.current_predicted_ddpr_residual_m = residual_m;
        row.imu_innovation_sigma_m = std::sqrt(0.5);
        return row;
    };
    const std::vector<FGOProcessor::PredictedDdprQualityFactorDiagnostics>
        rows = {make_row(0, 10.0), make_row(1, 10.0),
                make_row(2, 40.0), make_row(3, 10.0),
                make_row(5, -8.0)};

    const auto diagnostics =
        FGOProcessor::analyzePredictedDdprBiasStateShadow(rows);
    ASSERT_EQ(diagnostics.size(), rows.size());
    ASSERT_TRUE(diagnostics[2].prediction_usable);
    EXPECT_NEAR(diagnostics[2].prior_bias_m, 10.0, 0.2)
        << "the current impulse must not leak into its own prediction";
    EXPECT_TRUE(diagnostics[2].update_clipped);
    ASSERT_TRUE(diagnostics[3].prediction_usable);
    EXPECT_LT(std::abs(diagnostics[3].prior_bias_m - 10.0), 1.0)
        << "one impulse must not become a persistent 40 m bias";
    EXPECT_TRUE(diagnostics[4].continuity_reset);
    EXPECT_FALSE(diagnostics[4].prediction_usable);
    EXPECT_EQ(diagnostics[4].prior_updates, 0);
    EXPECT_NEAR(diagnostics[4].prior_bias_m, 0.0, 1e-12);
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

TEST(FGOTest, ReportBatchFixAsFloatRelabelsWithoutChangingPositions) {
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

    const auto baseline =
        FGOProcessor(config).optimizeProblem(makeSyntheticProblem(false, true));
    ASSERT_EQ(baseline.solution.size(), 2u);
    EXPECT_EQ(baseline.solution.solutions[0].status, SolutionStatus::FIXED);
    EXPECT_EQ(baseline.solution.solutions[1].status, SolutionStatus::FIXED);

    config.report_batch_fix_as_float = true;
    const auto relabelled =
        FGOProcessor(config).optimizeProblem(makeSyntheticProblem(false, true));
    ASSERT_EQ(relabelled.solution.size(), 2u);
    EXPECT_EQ(relabelled.solution.solutions[0].status, SolutionStatus::FLOAT);
    EXPECT_EQ(relabelled.solution.solutions[1].status, SolutionStatus::FLOAT);
    // Only the reported status changes; the integer-constrained batch
    // position is retained.
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        EXPECT_NEAR((relabelled.solution.solutions[epoch].position_ecef -
                     baseline.solution.solutions[epoch].position_ecef)
                        .norm(),
                    0.0,
                    1e-9);
    }
}

TEST(FGOTest, NewArLeversAreDefaultOff) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.report_batch_fix_as_float);
    EXPECT_FALSE(config.allow_partial_fix_and_hold);
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

TEST(FGOTest, GeometryFreeSlipShadowTaintsExistingDdAmbiguityArc) {
    FGOProcessor::FGOProblem problem;
    for (int epoch = 0; epoch < 3; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + 0.2 * epoch);
        problem.epochs.push_back(seed);

        const auto add_factor = [&](SignalType signal,
                                    std::size_t ambiguity_index,
                                    double target_sd_phase_m,
                                    double reference_sd_phase_m) {
            FGOProcessor::DoubleDifferenceCarrierFactor factor;
            factor.epoch_index = static_cast<std::size_t>(epoch);
            factor.ambiguity_index = ambiguity_index;
            factor.satellite = SatelliteId(GNSSSystem::GPS, 1);
            factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 2);
            factor.signal = signal;
            factor.rover_satellite_model.raw_carrier_m = target_sd_phase_m;
            factor.base_satellite_model.raw_carrier_m = 0.0;
            factor.rover_reference_model.raw_carrier_m = reference_sd_phase_m;
            factor.base_reference_model.raw_carrier_m = 0.0;
            problem.double_difference_carrier_factors.push_back(factor);
        };

        const double slip_m = epoch == 0 ? 0.0 : 0.10;
        add_factor(SignalType::GPS_L1CA, 0, 10.0 + slip_m, 1.0);
        add_factor(SignalType::GPS_L2C, 1, 8.0, 2.0);
    }

    const auto shadow =
        FGOProcessor::analyzeGeometryFreeSlipShadow(problem, 0.05, 1.5);
    ASSERT_EQ(shadow.size(), 3u);
    EXPECT_EQ(shadow[0].event_pairs, 0);
    EXPECT_EQ(shadow[0].tainted_ambiguities, 0);
    EXPECT_EQ(shadow[1].event_pairs, 1);
    EXPECT_NEAR(shadow[1].max_jump_m, 0.10, 1e-12);
    EXPECT_EQ(shadow[1].tainted_ambiguities, 2);
    EXPECT_EQ(shadow[2].event_pairs, 0);
    EXPECT_EQ(shadow[2].tainted_ambiguities, 2);
}

TEST(FGOTest, GeometryFreeSlipShadowUsesDopplerToIsolateSignal) {
    FGOProcessor::FGOProblem problem;
    for (int epoch = 0; epoch < 3; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + 0.2 * epoch);
        problem.epochs.push_back(seed);

        const auto add_factor = [&](SignalType signal,
                                    std::size_t ambiguity_index,
                                    double target_sd_phase_m,
                                    double reference_sd_phase_m) {
            FGOProcessor::DoubleDifferenceCarrierFactor factor;
            factor.epoch_index = static_cast<std::size_t>(epoch);
            factor.ambiguity_index = ambiguity_index;
            factor.satellite = SatelliteId(GNSSSystem::GPS, 1);
            factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 2);
            factor.signal = signal;
            factor.rover_satellite_model.raw_carrier_m = target_sd_phase_m;
            factor.base_satellite_model.raw_carrier_m = 0.0;
            factor.rover_reference_model.raw_carrier_m = reference_sd_phase_m;
            factor.base_reference_model.raw_carrier_m = 0.0;
            factor.rover_satellite_model.has_doppler_residual = true;
            factor.base_satellite_model.has_doppler_residual = true;
            factor.rover_reference_model.has_doppler_residual = true;
            factor.base_reference_model.has_doppler_residual = true;
            problem.double_difference_carrier_factors.push_back(factor);
        };

        const double slip_m = epoch == 0 ? 0.0 : 0.40;
        add_factor(SignalType::GPS_L1CA, 0, 10.0 + slip_m, 1.0);
        add_factor(SignalType::GPS_L2C, 1, 8.0, 2.0);
    }

    const auto shadow =
        FGOProcessor::analyzeGeometryFreeSlipShadow(problem, 0.05, 1.5);
    ASSERT_EQ(shadow.size(), 3u);
    EXPECT_EQ(shadow[0].doppler_event_signals, 0);
    EXPECT_EQ(shadow[1].event_pairs, 1);
    EXPECT_EQ(shadow[1].doppler_event_signals, 1);
    EXPECT_NEAR(shadow[1].doppler_max_innovation_m, 0.40, 1e-12);
    EXPECT_EQ(shadow[1].doppler_isolated_event_pairs, 1);
    EXPECT_EQ(shadow[2].doppler_event_signals, 0);
    EXPECT_EQ(shadow[2].doppler_isolated_event_pairs, 0);
}

TEST(FGOTest, GeometryFreeSlipResetBreaksBothBandsAfterConfirmation) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position =
        rover_position + Vector3d(-320.0, 180.0, 45.0);
    const std::vector<double> rover_l1 = {0.0, 0.0, 0.40, 0.40};
    const std::vector<double> rover_l2 = {0.0, 0.0, 0.00, 0.00};
    const std::vector<double> base_bias(4, 0.0);
    const auto rover_epochs = makeGeometryFreeObservationEpochs(
        nav, rover_position, rover_l1, rover_l2);
    const auto base_epochs = makeGeometryFreeObservationEpochs(
        nav, base_position, base_bias, base_bias);

    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.use_code_minus_carrier_screening = false;
    config.use_multi_frequency_double_difference = true;
    config.use_geometry_free_cycle_slip_reset = true;
    config.geometry_free_cycle_slip_threshold_m = 0.05;
    const auto problem = FGOProcessor(config).buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    std::map<SignalType, std::map<std::size_t, std::size_t>> arcs;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        arcs[factor.signal][factor.epoch_index] = factor.ambiguity_index;
    }
    ASSERT_EQ(arcs[SignalType::GPS_L1CA].size(), 4u);
    ASSERT_EQ(arcs[SignalType::GPS_L2C].size(), 4u);
    EXPECT_EQ(arcs[SignalType::GPS_L1CA].at(0),
              arcs[SignalType::GPS_L1CA].at(1));
    EXPECT_EQ(arcs[SignalType::GPS_L1CA].at(1),
              arcs[SignalType::GPS_L1CA].at(2));
    EXPECT_NE(arcs[SignalType::GPS_L1CA].at(2),
              arcs[SignalType::GPS_L1CA].at(3));
    EXPECT_EQ(arcs[SignalType::GPS_L2C].at(0),
              arcs[SignalType::GPS_L2C].at(1));
    EXPECT_EQ(arcs[SignalType::GPS_L2C].at(1),
              arcs[SignalType::GPS_L2C].at(2));
    EXPECT_NE(arcs[SignalType::GPS_L2C].at(2),
              arcs[SignalType::GPS_L2C].at(3));
    // The same controlled L1 jump is present on both satellites; one is the
    // DD target and one the reference. Geometry-free detection cannot identify
    // the changed band, so both bands for both satellites are conservatively
    // reset one epoch after detection.
    EXPECT_EQ(problem.diagnostics.geometry_free_cycle_slip_resets, 4u);

    config.use_geometry_free_cycle_slip_reset = false;
    const auto control = FGOProcessor(config).buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);
    std::map<SignalType, std::map<std::size_t, std::size_t>> control_arcs;
    for (const auto& factor : control.double_difference_carrier_factors) {
        control_arcs[factor.signal][factor.epoch_index] =
            factor.ambiguity_index;
    }
    EXPECT_EQ(control_arcs[SignalType::GPS_L1CA].at(1),
              control_arcs[SignalType::GPS_L1CA].at(2));
    EXPECT_EQ(control_arcs[SignalType::GPS_L2C].at(1),
              control_arcs[SignalType::GPS_L2C].at(2));
    EXPECT_EQ(control.diagnostics.geometry_free_cycle_slip_resets, 0u);
}

TEST(FGOTest, GeometryFreeSlipResetDefersToReceiverArcBreak) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position =
        rover_position + Vector3d(-320.0, 180.0, 45.0);
    const std::vector<double> rover_l1 = {0.0, 0.0, 0.40, 0.40};
    const std::vector<double> rover_l2 = {0.0, 0.0, 0.00, 0.00};
    const std::vector<double> base_bias(4, 0.0);
    auto rover_epochs = makeGeometryFreeObservationEpochs(
        nav, rover_position, rover_l1, rover_l2);
    const auto base_epochs = makeGeometryFreeObservationEpochs(
        nav, base_position, base_bias, base_bias);
    for (auto& observation : rover_epochs[3].observations) {
        observation.lli = 1;
        observation.loss_of_lock = true;
    }

    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.use_code_minus_carrier_screening = false;
    config.use_multi_frequency_double_difference = true;
    config.use_geometry_free_cycle_slip_reset = true;
    const auto problem = FGOProcessor(config).buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    // The epoch-2 GF event was pending, but the epoch-3 receiver LLI already
    // created new arcs. The debounce must cancel rather than count a second,
    // redundant GF-driven reset.
    EXPECT_EQ(problem.diagnostics.geometry_free_cycle_slip_resets, 0u);
}

TEST(FGOTest, CmcJumpForcesAmbiguityArcBreak) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // Both satellites: rover bias jumps by 5 m at epoch 2 (base bias stays
    // at 0), so single-difference CMC jumps by -5 m there -- well past the
    // 1.0 m test threshold. No jump before or after.
    const std::vector<std::array<double, 2>> rover_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {5.0, 5.0}, {5.0, 5.0},
    };
    const std::vector<std::array<double, 2>> base_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
    };
    const auto rover_epochs =
        makeCmcObservationEpochs(nav, rover_position, rover_bias, 1.0);
    const auto base_epochs =
        makeCmcObservationEpochs(nav, base_position, base_bias, 1.0);

    FGOProcessor processor(makeCmcTestConfig());
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    const auto rows = trackedSatelliteRowsByEpoch(problem);
    ASSERT_EQ(rows.size(), 4u);
    for (const auto& [epoch_index, row] : rows) {
        EXPECT_TRUE(row.has_carrier_factor) << "epoch " << epoch_index;
        EXPECT_TRUE(row.has_pseudorange_factor) << "epoch " << epoch_index;
    }

    // No break between epoch 0 and 1 (no CMC change).
    EXPECT_EQ(rows.at(0).ambiguity_index, rows.at(1).ambiguity_index);
    // The jump at epoch 2 forces a NEW ambiguity for the tracked satellite.
    EXPECT_NE(rows.at(1).ambiguity_index, rows.at(2).ambiguity_index);
    // No further break at epoch 3 (bias held steady after the jump).
    EXPECT_EQ(rows.at(2).ambiguity_index, rows.at(3).ambiguity_index);

    // Both PRN1 and PRN2 are screened every epoch (only one of them is the
    // DD reference and gates factor emission, but the CMC pre-pass runs over
    // every tracked satellite), so the shared jump at epoch 2 is detected
    // twice.
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_jump_resets, 2u);
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_level_exclusions, 0u);
}

TEST(FGOTest, CmcScreeningDefaultOffIsNoOp) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    const std::vector<std::array<double, 2>> rover_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {5.0, 5.0}, {5.0, 5.0},
    };
    const std::vector<std::array<double, 2>> base_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
    };
    const auto rover_epochs =
        makeCmcObservationEpochs(nav, rover_position, rover_bias, 1.0);
    const auto base_epochs =
        makeCmcObservationEpochs(nav, base_position, base_bias, 1.0);

    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.use_code_minus_carrier_screening = false;  // default
    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    const auto rows = trackedSatelliteRowsByEpoch(problem);
    ASSERT_EQ(rows.size(), 4u);
    for (const auto& [epoch_index, row] : rows) {
        EXPECT_TRUE(row.has_carrier_factor) << "epoch " << epoch_index;
        EXPECT_TRUE(row.has_pseudorange_factor) << "epoch " << epoch_index;
    }
    // With screening off, the same CMC jump that broke the arc in
    // CmcJumpForcesAmbiguityArcBreak must NOT break it here: every epoch
    // keeps the same ambiguity_index.
    EXPECT_EQ(rows.at(0).ambiguity_index, rows.at(1).ambiguity_index);
    EXPECT_EQ(rows.at(1).ambiguity_index, rows.at(2).ambiguity_index);
    EXPECT_EQ(rows.at(2).ambiguity_index, rows.at(3).ambiguity_index);

    EXPECT_EQ(problem.diagnostics.code_minus_carrier_jump_resets, 0u);
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_level_exclusions, 0u);
}

TEST(FGOTest, CmcLevelExclusionSkipsBothFactorsWithoutUpdatingBaseline) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // Warmup = 3: epochs 0,1,2 seed/average the baseline to
    // (0 + 2 - 1) / 3 = 1/3 (CMC = base_bias - rover_bias). Epoch 3 and 4
    // both deviate by the SAME amount (baseline + threshold + margin) so
    // that IF the baseline had been updated after epoch 3's exclusion,
    // epoch 4 would no longer be excluded -- it must still be excluded,
    // proving the baseline was left untouched.
    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.code_minus_carrier_jump_threshold_m = 1000.0;  // isolate the level check
    config.code_minus_carrier_level_threshold_m = 0.4;
    config.code_minus_carrier_warmup_epochs = 3;
    config.code_minus_carrier_baseline_alpha = 0.5;

    // CMC[e] = base_bias[e] - rover_bias[e]:
    //   e0: 0 - 0 = 0
    //   e1: 2 - 0 = 2   (rover=0, base=2)
    //   e2: -1 - 0 = -1 (rover=0, base=-1)
    //   baseline after warmup = (0 + 2 - 1) / 3 = 1/3
    //   e3, e4: cmc = 1/3 + 0.4 + 0.2 = 0.9333... -> |delta| = 0.6 > 0.4: excluded both times
    const double baseline_after_warmup = (0.0 + 2.0 - 1.0) / 3.0;
    const double excluded_cmc = baseline_after_warmup + 0.4 + 0.2;
    const std::vector<std::array<double, 2>> base_bias = {
        {0.0, 0.0}, {2.0, 2.0}, {-1.0, -1.0},
        {excluded_cmc, excluded_cmc}, {excluded_cmc, excluded_cmc},
    };
    const std::vector<std::array<double, 2>> rover_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
    };
    const auto rover_epochs =
        makeCmcObservationEpochs(nav, rover_position, rover_bias, 1.0);
    const auto base_epochs =
        makeCmcObservationEpochs(nav, base_position, base_bias, 1.0);

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    const auto rows = trackedSatelliteRowsByEpoch(problem);
    // Epochs 0, 1, 2 (seeding/warmup) always emit factors.
    EXPECT_TRUE(rows.at(0).has_carrier_factor);
    EXPECT_TRUE(rows.at(1).has_carrier_factor);
    EXPECT_TRUE(rows.at(2).has_carrier_factor);
    // Epochs 3 and 4 are BOTH excluded -- if the baseline had moved after
    // epoch 3, epoch 4 (identical deviation) would no longer exceed the
    // threshold and would NOT be excluded.
    EXPECT_EQ(rows.find(3), rows.end());
    EXPECT_EQ(rows.find(4), rows.end());

    // No ambiguity arc break: only the level check is active here.
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_jump_resets, 0u);
    // Both PRN1 and PRN2 are screened -> each excluded epoch counts twice.
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_level_exclusions, 4u);
}

TEST(FGOTest, CmcBaselineEwmaUpdatesInSteadyState) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // Warmup = 1: epoch 0 seeds the baseline directly (count=1==warmup), so
    // epoch 1 onward is already steady-state.
    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.code_minus_carrier_jump_threshold_m = 1000.0;  // isolate the level check
    config.code_minus_carrier_level_threshold_m = 0.5;
    config.code_minus_carrier_warmup_epochs = 1;
    config.code_minus_carrier_baseline_alpha = 0.5;

    // baseline0 = cmc0 = 0.
    // e1: cmc1 = 0.4 (within 0.5 of baseline0=0) -> NOT excluded;
    //     baseline1 = 0.5*0 + 0.5*0.4 = 0.2.
    // e2: cmc2 = 0.65 (within 0.5 of baseline1=0.2, would be EXCLUDED if
    //     compared against the stale baseline0=0: |0.65-0|=0.65>0.5) ->
    //     proves the baseline moved. NOT excluded;
    //     baseline2 = 0.5*0.2 + 0.5*0.65 = 0.425.
    // e3: cmc3 = 0.9 (within 0.5 of baseline2=0.425: |0.9-0.425|=0.475<=0.5,
    //     but would be EXCLUDED against baseline1=0.2: |0.9-0.2|=0.7>0.5) ->
    //     proves the second EWMA update also took effect. NOT excluded.
    const std::vector<std::array<double, 2>> base_bias = {
        {0.0, 0.0}, {0.4, 0.4}, {0.65, 0.65}, {0.9, 0.9},
    };
    const std::vector<std::array<double, 2>> rover_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
    };
    const auto rover_epochs =
        makeCmcObservationEpochs(nav, rover_position, rover_bias, 1.0);
    const auto base_epochs =
        makeCmcObservationEpochs(nav, base_position, base_bias, 1.0);

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    const auto rows = trackedSatelliteRowsByEpoch(problem);
    ASSERT_EQ(rows.size(), 4u);
    for (const auto& [epoch_index, row] : rows) {
        EXPECT_TRUE(row.has_carrier_factor) << "epoch " << epoch_index;
    }
    EXPECT_EQ(problem.diagnostics.code_minus_carrier_level_exclusions, 0u);
}

TEST(FGOTest, CmcArcResetAlsoResetsBaseline) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // Warmup = 2, level threshold = 1.0: epochs 0-1 establish a baseline of
    // 0. At epoch 2 a loss-of-lock is injected on BOTH satellites (forcing
    // the pre-existing rover-side arc-break machinery to start a new
    // carrier arc) at the SAME epoch the CMC value jumps by 50 m -- an
    // enormous deviation that would clearly exceed the level threshold
    // against the OLD baseline. Because this port resets the CMC
    // baseline/prev/warmup state whenever the rover-side arc restarts (the
    // deliberate deviation from the reference, documented on
    // FGOConfig::use_code_minus_carrier_screening), epoch 2 must be treated
    // as a fresh baseline seed -- NOT excluded.
    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.code_minus_carrier_jump_threshold_m = 1000.0;  // isolate the level check
    config.code_minus_carrier_level_threshold_m = 1.0;
    config.code_minus_carrier_warmup_epochs = 2;
    config.code_minus_carrier_baseline_alpha = 0.5;

    const std::vector<std::array<double, 2>> base_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
    };
    // CMC = base_bias - rover_bias, so a rover bias of -50 at/after epoch 2
    // makes CMC jump to +50.
    const std::vector<std::array<double, 2>> rover_bias = {
        {0.0, 0.0}, {0.0, 0.0}, {-50.0, -50.0}, {-50.0, -50.0}, {-50.0, -50.0},
    };
    const std::vector<bool> loss_of_lock_epochs = {false, false, true, false, false};
    const auto rover_epochs = makeCmcObservationEpochs(
        nav, rover_position, rover_bias, 1.0, loss_of_lock_epochs);
    const auto base_epochs = makeCmcObservationEpochs(
        nav, base_position, base_bias, 1.0, loss_of_lock_epochs);

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    const auto rows = trackedSatelliteRowsByEpoch(problem);
    ASSERT_EQ(rows.size(), 5u);
    // Epoch 2's huge CMC deviation is NOT excluded: the arc reset (from
    // loss-of-lock) also reset the baseline, so epoch 2 just reseeds it.
    EXPECT_TRUE(rows.at(2).has_carrier_factor);
    EXPECT_TRUE(rows.at(2).has_pseudorange_factor);
    // The loss-of-lock itself still breaks the ambiguity arc as before.
    EXPECT_NE(rows.at(1).ambiguity_index, rows.at(2).ambiguity_index);
    // Epochs 3, 4 continue the new arc with the reseeded baseline (stable
    // value, count still ramping through warmup) -- also not excluded.
    EXPECT_TRUE(rows.at(3).has_carrier_factor);
    EXPECT_TRUE(rows.at(4).has_carrier_factor);
    EXPECT_EQ(rows.at(2).ambiguity_index, rows.at(3).ambiguity_index);
    EXPECT_EQ(rows.at(3).ambiguity_index, rows.at(4).ambiguity_index);

    EXPECT_EQ(problem.diagnostics.code_minus_carrier_level_exclusions, 0u);
}

// --- CMC-aware DD reference selection (FGOConfig::cmc_aware_reference_
// selection) -- select_reference() normally picks the group's highest-
// elevation satellite as the DD reference with no regard for
// cmc_level_exclude_this_epoch; every DD pair in the group is formed
// against that one satellite, so a CMC-excluded (multipath/NLOS) reference
// poisons the whole group at once. Needs a THIRD satellite (unlike the
// 2-satellite helper above) so that after the elevation-max satellite is
// excluded and a new reference is picked, a clean non-reference satellite
// remains to produce an observable DD factor pointing at the NEW reference.
std::vector<ObservationData> makeCmcObservationEpochsThreeSat(
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const std::vector<std::array<double, 3>>& carrier_bias_m,
    double dt_s) {
    std::vector<ObservationData> epochs;
    for (std::size_t epoch_index = 0; epoch_index < carrier_bias_m.size();
         ++epoch_index) {
        ObservationData epoch(
            GNSSTime(2300, 100100.0 + dt_s * static_cast<double>(epoch_index)));
        epoch.receiver_position = receiver_position;
        for (std::size_t sat_index = 0; sat_index < 3; ++sat_index) {
            Observation observation;
            const uint8_t prn = static_cast<uint8_t>(sat_index + 1);
            if (makeSyntheticGpsL1Observation(nav,
                                              SatelliteId(GNSSSystem::GPS, prn),
                                              epoch.time,
                                              receiver_position,
                                              carrier_bias_m[epoch_index][sat_index],
                                              observation)) {
                epoch.addObservation(observation);
            }
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

TEST(FGOTest, CmcAwareReferenceSelectionAvoidsExcludedReference) {
    const NavigationData nav = makeSyntheticGpsNavigation(3);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // Step 1: probe which satellite the plain elevation-max rule currently
    // picks as reference (geometry-driven, not controlled by this test --
    // same reasoning as makeCmcObservationEpochs's comment above), using a
    // single zero-bias epoch so no CMC exclusion is in play.
    const std::vector<std::array<double, 3>> zero_bias = {{0.0, 0.0, 0.0}};
    const auto probe_rover =
        makeCmcObservationEpochsThreeSat(nav, rover_position, zero_bias, 1.0);
    const auto probe_base =
        makeCmcObservationEpochsThreeSat(nav, base_position, zero_bias, 1.0);
    FGOProcessor probe_processor(makeCmcTestConfig());
    const auto probe_problem = probe_processor.buildDoubleDifferenceProblem(
        probe_rover, probe_base, nav, base_position);
    ASSERT_FALSE(probe_problem.double_difference_carrier_factors.empty());
    const SatelliteId elevation_reference =
        probe_problem.double_difference_carrier_factors.front().reference_satellite;
    const std::size_t excluded_sat_index =
        static_cast<std::size_t>(elevation_reference.prn) - 1;

    // Step 2: warmup (epochs 0-2) at zero CMC for all three satellites, then
    // epoch 3 pushes ONLY the elevation-max satellite's single-difference
    // CMC past the level threshold; the other two stay clean throughout.
    // Jump threshold set far above any deviation used here so only the
    // level check can fire (same isolation style as the level-exclusion
    // test above).
    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.code_minus_carrier_jump_threshold_m = 1000.0;
    config.code_minus_carrier_level_threshold_m = 0.4;
    config.code_minus_carrier_warmup_epochs = 3;
    config.code_minus_carrier_baseline_alpha = 0.5;

    std::vector<std::array<double, 3>> base_bias(4, std::array<double, 3>{0.0, 0.0, 0.0});
    base_bias[3][excluded_sat_index] = 1.0;  // baseline 0 + 1.0 > 0.4 threshold
    const std::vector<std::array<double, 3>> rover_bias(
        4, std::array<double, 3>{0.0, 0.0, 0.0});

    const auto rover_epochs =
        makeCmcObservationEpochsThreeSat(nav, rover_position, rover_bias, 1.0);
    const auto base_epochs =
        makeCmcObservationEpochsThreeSat(nav, base_position, base_bias, 1.0);

    // --- cmc_aware_reference_selection OFF (default): the CMC-excluded,
    // elevation-max satellite is still picked as reference at epoch 3 --
    // every surviving DD factor in the group references it (poisoned).
    {
        FGOProcessor processor(config);
        const auto problem = processor.buildDoubleDifferenceProblem(
            rover_epochs, base_epochs, nav, base_position);
        bool found_epoch3_factor = false;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index != 3) {
                continue;
            }
            found_epoch3_factor = true;
            EXPECT_TRUE(factor.reference_satellite == elevation_reference);
        }
        EXPECT_TRUE(found_epoch3_factor);
        EXPECT_EQ(problem.diagnostics.cmc_ref_avoided_count, 0u);
    }

    // --- cmc_aware_reference_selection ON: epoch 3's reference must move
    // away from the CMC-excluded elevation-max satellite, and the avoided-
    // reference counter must record it (once for the PR-factor loop, once
    // for the CP-factor loop -- both call select_reference for this group).
    {
        config.cmc_aware_reference_selection = true;
        FGOProcessor processor(config);
        const auto problem = processor.buildDoubleDifferenceProblem(
            rover_epochs, base_epochs, nav, base_position);
        bool found_epoch3_factor = false;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index != 3) {
                continue;
            }
            found_epoch3_factor = true;
            EXPECT_FALSE(factor.reference_satellite == elevation_reference);
        }
        EXPECT_TRUE(found_epoch3_factor);
        EXPECT_EQ(problem.diagnostics.cmc_ref_avoided_count, 2u);
    }
}

// --- Elevation-dependent DD sigma ("varerr", FGOConfig::use_elevation_
// dependent_sigma) -- port of the inuex35 reference's preprocess/prefit.py
// varerr_dd_sigma / buildfactor/factors.py pair_sigma. See the knob's doc
// comment in fgo.hpp for the full formula and dt_s/undifferenced-PR
// decisions.

TEST(FGOTest, ElevationDependentSigmaDefaultOffMatchesFlatFormula) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    const std::vector<std::array<double, 2>> bias = {{0.0, 0.0}, {0.0, 0.0}};
    const auto rover_epochs = makeCmcObservationEpochs(nav, rover_position, bias, 0.5);
    const auto base_epochs = makeCmcObservationEpochs(nav, base_position, bias, 0.5);

    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.use_code_minus_carrier_screening = false;
    ASSERT_FALSE(config.use_elevation_dependent_sigma);  // default OFF

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    // With the knob off, DD PR/CP sigma must be bit-for-bit the pre-existing
    // flat/elevation-power formula: band_scale(=1, single-freq GPS L1CA) *
    // sigma_m / sqrt(max(0.1, sin(own elevation))).
    ASSERT_FALSE(problem.double_difference_pseudorange_factors.empty());
    ASSERT_FALSE(problem.double_difference_carrier_factors.empty());
    for (const auto& f : problem.double_difference_pseudorange_factors) {
        const double sin_el = std::max(0.1, std::sin(f.elevation_rad));
        const double expected =
            std::max(1e-3, config.double_difference_pseudorange_sigma_m) /
            std::sqrt(sin_el);
        EXPECT_DOUBLE_EQ(f.sigma_m, expected) << "epoch " << f.epoch_index;
    }
    for (const auto& f : problem.double_difference_carrier_factors) {
        const double sin_el = std::max(0.1, std::sin(f.elevation_rad));
        const double expected =
            std::max(1e-4, config.double_difference_carrier_sigma_m) /
            std::sqrt(sin_el);
        EXPECT_DOUBLE_EQ(f.sigma_m, expected) << "epoch " << f.epoch_index;
    }
}

TEST(FGOTest, ElevationDependentSigmaMatchesPortedVarerrFormula) {
    const NavigationData nav = makeSyntheticGpsNavigation(2);
    const Vector3d rover_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = rover_position + Vector3d(-320.0, 180.0, 45.0);

    // dt_s = 0.5 s (deliberately different from the 0.2 s epoch-0 fallback)
    // so epoch 1's clock term proves the ACTUAL measured rover epoch spacing
    // is used, not just the fallback default.
    constexpr double kDtS = 0.5;
    const std::vector<std::array<double, 2>> bias = {{0.0, 0.0}, {0.0, 0.0}};
    const auto rover_epochs = makeCmcObservationEpochs(nav, rover_position, bias, kDtS);
    const auto base_epochs = makeCmcObservationEpochs(nav, base_position, bias, kDtS);

    FGOProcessor::FGOConfig config = makeCmcTestConfig();
    config.use_code_minus_carrier_screening = false;
    config.use_elevation_dependent_sigma = true;
    // Reference defaults (config.py): these ran DEFAULT-ON in every
    // reference benchmark.
    config.elevation_sigma_err_a_m = 0.001;
    config.elevation_sigma_err_b_m = 0.001;
    config.elevation_sigma_pseudorange_ratio = 100.0;
    config.elevation_sigma_clock_stability = 5e-12;

    FGOProcessor processor(config);
    const auto problem = processor.buildDoubleDifferenceProblem(
        rover_epochs, base_epochs, nav, base_position);

    ASSERT_FALSE(problem.double_difference_pseudorange_factors.empty());
    ASSERT_FALSE(problem.double_difference_carrier_factors.empty());

    const double el_min_rad = M_PI / 180.0 * std::max(1.0, config.min_elevation_deg);
    constexpr double kSpeedOfLightMps = 299792458.0;

    // Independent oracle re-implementing the exact ported formula (fgo.hpp's
    // use_elevation_dependent_sigma doc comment / varerrDdSigma in fgo.cpp),
    // so this test would catch a broken port even if it coincidentally
    // matched at one particular elevation.
    auto expectedSigma = [&](bool is_pseudorange, double el_pair_rad, double epoch_dt_s) {
        const double fact = is_pseudorange ? config.elevation_sigma_pseudorange_ratio : 1.0;
        const double a = fact * config.elevation_sigma_err_a_m;
        const double b = fact * config.elevation_sigma_err_b_m;
        const double d = kSpeedOfLightMps * config.elevation_sigma_clock_stability * epoch_dt_s;
        const double sinel = std::max(std::sin(el_pair_rad), 0.05);
        const double var_sd = 2.0 * (a * a + b * b / (sinel * sinel)) + d * d;
        return std::sqrt(2.0 * var_sd);
    };

    auto findReferenceElevationRad = [&](std::size_t epoch_index,
                                         const SatelliteId& reference_satellite,
                                         SignalType signal) {
        for (const auto& reference_observation :
             problem.double_difference_reference_observations) {
            if (reference_observation.epoch_index == epoch_index &&
                reference_observation.satellite == reference_satellite &&
                reference_observation.signal == signal) {
                return reference_observation.elevation_rad;
            }
        }
        ADD_FAILURE() << "no matching reference observation for epoch "
                      << epoch_index;
        return 0.0;
    };

    // Epoch 0 has no preceding epoch, so the port's epoch_dt_s is still at
    // its initial 0.2 s default (see the fgo.hpp doc comment); epoch 1's
    // measured spacing is exactly kDtS.
    for (const auto& f : problem.double_difference_pseudorange_factors) {
        const double epoch_dt_s = (f.epoch_index == 0) ? 0.2 : kDtS;
        const double reference_el_rad = findReferenceElevationRad(
            f.epoch_index, f.reference_satellite, f.signal);
        const double el_pair_rad =
            std::max(std::min(reference_el_rad, f.elevation_rad), el_min_rad);
        const double expected = expectedSigma(/*is_pseudorange=*/true, el_pair_rad, epoch_dt_s);
        EXPECT_NEAR(f.sigma_m, expected, 1e-9 * std::max(1.0, expected))
            << "PR epoch " << f.epoch_index;
    }
    double pr_sigma_at_epoch0 = 0.0;
    double cp_sigma_at_epoch0 = 0.0;
    for (const auto& f : problem.double_difference_carrier_factors) {
        const double epoch_dt_s = (f.epoch_index == 0) ? 0.2 : kDtS;
        const double reference_el_rad = findReferenceElevationRad(
            f.epoch_index, f.reference_satellite, f.signal);
        const double el_pair_rad =
            std::max(std::min(reference_el_rad, f.elevation_rad), el_min_rad);
        const double expected = expectedSigma(/*is_pseudorange=*/false, el_pair_rad, epoch_dt_s);
        EXPECT_NEAR(f.sigma_m, expected, 1e-9 * std::max(1.0, expected))
            << "CP epoch " << f.epoch_index;
        if (f.epoch_index == 0) cp_sigma_at_epoch0 = f.sigma_m;
    }
    for (const auto& f : problem.double_difference_pseudorange_factors) {
        if (f.epoch_index == 0) pr_sigma_at_epoch0 = f.sigma_m;
    }
    // PR is roughly eratio_pr (100x) the CP sigma at the same pair-elevation
    // -- not exactly 100x, because the additive clock term d^2 does NOT
    // scale by eratio_pr in the reference formula (see fgo.hpp's doc
    // comment); at these defaults d^2 is a small correction, so the ratio is
    // close to but not exactly 100.
    ASSERT_GT(cp_sigma_at_epoch0, 0.0);
    EXPECT_NEAR(pr_sigma_at_epoch0 / cp_sigma_at_epoch0, 100.0, 1.0);
}

TEST(FGOTest, LowCountRelaxSurplusQualityIsDefaultOffAndIndependent) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_low_count_ambiguity_resolution);
    EXPECT_FALSE(config.low_count_relax_surplus_quality);
    EXPECT_FALSE(config.low_count_require_separation_witness);
    EXPECT_EQ(config.low_count_min_candidates, 4);
    EXPECT_DOUBLE_EQ(config.low_count_max_float_separation_m, 0.1);
    EXPECT_DOUBLE_EQ(config.low_count_max_imu_prediction_separation_m, 0.1);
    // The relaxation and witness are inert unless the low-count path is on.
    EXPECT_FALSE(config.low_count_relax_surplus_quality &&
                 !config.use_low_count_ambiguity_resolution);
    EXPECT_FALSE(config.low_count_require_separation_witness &&
                 !config.use_low_count_ambiguity_resolution);
}

TEST(FGOTest, RawAndroidEpochIdentityPropagatesIntoRetainedEpochSeeds) {
    const NavigationData nav = makeSyntheticGpsNavigation(4);
    const std::array<Vector3d, 2> receiver_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.0, -4841694.0, 3985351.0)};
    auto observations = makeSyntheticDoubleDifferenceObservationEpochs(
        nav, receiver_positions, 0.0);
    ASSERT_EQ(observations.size(), 2U);
    ASSERT_EQ(observations[0].observations.size(), 4U);
    ASSERT_EQ(observations[1].observations.size(), 4U);
    observations[0].raw_source_index = 4U;
    observations[0].raw_utc_time_millis = 1'700'000'004'000LL;
    observations[1].raw_source_index = 6U;
    observations[1].raw_utc_time_millis = 1'700'000'006'000LL;

    FGOProcessor::FGOConfig config;
    config.use_spp_seed = false;
    config.use_multi_constellation = false;
    config.use_motion_factors = false;
    config.use_tdcp_factors = false;
    config.use_carrier_phase_factors = false;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    config.min_elevation_deg = -90.0;
    config.min_snr_dbhz = 0.0;
    config.min_satellites_per_epoch = 4;
    const FGOProcessor processor(config);
    const FGOProcessor::FGOProblem problem =
        processor.buildPseudorangeProblem(observations, nav);
    ASSERT_EQ(problem.epochs.size(), observations.size());
    EXPECT_EQ(problem.epochs[0].raw_source_index, 4U);
    EXPECT_EQ(problem.epochs[0].raw_utc_time_millis, 1'700'000'004'000LL);
    EXPECT_EQ(problem.epochs[1].raw_source_index, 6U);
    EXPECT_EQ(problem.epochs[1].raw_utc_time_millis, 1'700'000'006'000LL);
}
