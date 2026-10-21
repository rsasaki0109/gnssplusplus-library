// SPP measurement weighting: SNR slope/cap, GLONASS variance factor and the
// temporal inter-system-bias (ISB) prior. The library defaults must keep the
// historical behaviour; `gnss spp` opts into the steeper weighting.
#include <gtest/gtest.h>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "../apps/native/gnss_spp_defaults.hpp"

using namespace libgnss;

namespace {

// Synthetic GPS + Galileo + GLONASS constellation (24 satellites each) over a
// fixed receiver. Satellites come from Keplerian elements; GLONASS ones are
// state vectors sampled from such an orbit, so the processor and the scene
// share one propagator and the data are noise-free.
struct SyntheticMultiGnssScene {
    Vector3d truth = Vector3d(-3959000.0, 3354000.0, 3697000.0);  // near Tokyo
    double clock_bias_m = 3000.0;
    std::map<GNSSSystem, double> isb_m;  // receiver ISB against GPS per system
    NavigationData nav;

    SyntheticMultiGnssScene() {
        const GNSSTime toe(2300, 100000.0);
        int system_index = 0;
        for (const GNSSSystem system :
             {GNSSSystem::GPS, GNSSSystem::Galileo, GNSSSystem::GLONASS}) {
            for (int plane = 0; plane < 6; ++plane) {
                for (int slot = 0; slot < 4; ++slot) {
                    Ephemeris eph;
                    eph.valid = true;
                    eph.satellite = SatelliteId(system, plane * 4 + slot + 1);
                    eph.toe = eph.toc = toe;
                    eph.toes = toe.tow;
                    eph.week = 2300;
                    eph.sqrt_a = std::sqrt(26560000.0);
                    eph.e = 0.005;
                    eph.i0 = 55.0 * M_PI / 180.0;
                    eph.omega0 = plane * (M_PI / 3.0) + system_index * (M_PI / 9.0);
                    eph.omega = 0.0;
                    eph.m0 = slot * (M_PI / 2.0) + plane * (M_PI / 12.0) +
                             system_index * (M_PI / 5.0);
                    eph.omega_dot = -8.0e-9;
                    if (system == GNSSSystem::Galileo) {
                        eph.data_source_code = galileo_data_source::kClockE5bE1 | 1;
                    }
                    if (system == GNSSSystem::GLONASS) {
                        Vector3d pos, vel;
                        double clk = 0.0, drift = 0.0;
                        eph.satellite = SatelliteId(GNSSSystem::GPS, 1);
                        eph.calculateSatelliteState(toe, pos, vel, clk, drift);
                        eph.satellite = SatelliteId(GNSSSystem::GLONASS, plane * 4 + slot + 1);
                        eph.glonass_position = pos;
                        eph.glonass_velocity = vel;
                        eph.glonass_frequency_channel_present = true;
                    }
                    nav.addEphemeris(eph);
                }
            }
            ++system_index;
        }
    }

    // `max_per_system` caps how many (highest-PRN-first dropped) satellites of
    // a system are observed; systems missing from `systems` are not observed.
    ObservationData observe(const GNSSTime& time,
                            const std::set<GNSSSystem>& systems,
                            const std::map<GNSSSystem, std::size_t>& max_per_system = {},
                            const std::map<GNSSSystem, double>& extra_bias_m = {},
                            double snr = 45.0) const {
        ObservationData epoch;
        epoch.time = time;
        epoch.receiver_position = truth + Vector3d(50.0, -30.0, 20.0);
        const Vector3d up = truth.normalized();
        std::map<GNSSSystem, std::size_t> observed;
        for (const auto& [sat, ephs] : nav.ephemeris_data) {
            if (!systems.count(sat.system)) continue;
            const Ephemeris& eph = ephs.front();
            double tau = 0.075;
            double sat_clock = 0.0;
            Vector3d rotated = Vector3d::Zero();
            bool ok = true;
            for (int i = 0; i < 5; ++i) {
                Vector3d pos, vel;
                double drift;
                if (!eph.calculateSatelliteState(time - tau, pos, vel, sat_clock, drift)) {
                    ok = false;
                    break;
                }
                const double angle = constants::OMEGA_E * tau;
                Eigen::Matrix3d rot;
                rot << std::cos(angle), std::sin(angle), 0.0,
                      -std::sin(angle), std::cos(angle), 0.0,
                       0.0, 0.0, 1.0;
                rotated = rot * pos;
                tau = (rotated - truth).norm() / constants::SPEED_OF_LIGHT;
            }
            if (!ok) continue;
            const Vector3d los = (rotated - truth).normalized();
            if (std::asin(los.dot(up)) * 180.0 / M_PI < 20.0) continue;
            const auto limit = max_per_system.find(sat.system);
            if (limit != max_per_system.end() && observed[sat.system] >= limit->second) continue;
            ++observed[sat.system];

            Observation obs;
            obs.satellite = sat;
            switch (sat.system) {
                case GNSSSystem::Galileo: obs.signal = SignalType::GAL_E1; break;
                case GNSSSystem::GLONASS: obs.signal = SignalType::GLO_L1CA; break;
                default: obs.signal = SignalType::GPS_L1CA; break;
            }
            const auto isb = isb_m.find(sat.system);
            const auto extra = extra_bias_m.find(sat.system);
            obs.pseudorange = (rotated - truth).norm() + clock_bias_m -
                              sat_clock * constants::SPEED_OF_LIGHT +
                              (isb != isb_m.end() ? isb->second : 0.0) +
                              (extra != extra_bias_m.end() ? extra->second : 0.0);
            obs.snr = snr;
            obs.valid = true;
            obs.has_pseudorange = true;
            epoch.addObservation(obs);
        }
        return epoch;
    }
};

ProcessorConfig syntheticConfig() {
    ProcessorConfig config;
    config.elevation_mask = 15.0;
    config.snr_mask = 0.0;
    config.mode = PositioningMode::SPP;
    config.use_ionosphere_model = false;
    config.use_troposphere_model = false;
    return config;
}

SPPProcessor::SPPConfig isbPriorConfig() {
    SPPProcessor::SPPConfig config;
    config.isb_prior_sigma_m = 1.0;
    return config;
}

GNSSTime epochTime(int k) { return GNSSTime(2300, 100000.0 + 30.0 * k); }

const std::set<GNSSSystem> kGpsGal = {GNSSSystem::GPS, GNSSSystem::Galileo};
const std::set<GNSSSystem> kAllSystems = {GNSSSystem::GPS, GNSSSystem::Galileo,
                                          GNSSSystem::GLONASS};

// Code variance with no atmosphere model: ((sigma * snr_factor)^2 + 2.5^2 + 0.5^2) / sin(el)^2.
double expectedVarianceAtZenith(double snr_factor) {
    const double code_sigma = 3.0 * snr_factor;
    return code_sigma * code_sigma + 2.5 * 2.5 + 0.5 * 0.5;
}

double positionVarianceTrace(const PositionSolution& solution) {
    return solution.position_covariance.trace();
}

}  // namespace

TEST(SPPWeightingConfigTest, LibraryDefaultsKeepHistoricalWeighting) {
    const SPPProcessor::SPPConfig config;
    EXPECT_EQ(config.snr_weight_slope_db, 20.0);
    EXPECT_EQ(config.snr_weight_max_factor, 4.0);
    EXPECT_EQ(config.glonass_variance_factor, 1.0);
    EXPECT_EQ(config.isb_prior_sigma_m, 0.0);

    const spp_utils::MeasurementVarianceInputs inputs;
    EXPECT_EQ(inputs.snr_slope_db, 20.0);
    EXPECT_EQ(inputs.snr_max_factor, 4.0);
}

TEST(SPPWeightingConfigTest, GnssSppAppUsesSteeperWeightingAndLibraryDefaultsStay) {
    const auto app = libgnss_apps::gnssSppDefaultConfig();
    EXPECT_EQ(app.snr_weight_slope_db, 10.0);
    EXPECT_EQ(app.snr_weight_max_factor, 32.0);
    EXPECT_EQ(app.glonass_variance_factor, 4.0);
    EXPECT_EQ(app.isb_prior_sigma_m, 1.0);

    // Apart from the four weighting fields the app starts from the library defaults.
    const SPPProcessor::SPPConfig library;
    EXPECT_EQ(app.snr_reference_dbhz, library.snr_reference_dbhz);
    EXPECT_EQ(app.pseudorange_sigma, library.pseudorange_sigma);
    EXPECT_EQ(app.use_variance_model, library.use_variance_model);
    EXPECT_EQ(app.model_intersystem_bias, library.model_intersystem_bias);
    EXPECT_EQ(app.mrtklib_iflc_code_bias, library.mrtklib_iflc_code_bias);
    EXPECT_EQ(SPPProcessor::SPPConfig().snr_weight_slope_db, 20.0);
    EXPECT_EQ(SPPProcessor::SPPConfig().snr_weight_max_factor, 4.0);
    EXPECT_EQ(SPPProcessor::SPPConfig().glonass_variance_factor, 1.0);
    EXPECT_EQ(SPPProcessor::SPPConfig().isb_prior_sigma_m, 0.0);
}

TEST(SPPWeightingVarianceTest, DefaultsEqualHistoricalSnrFactor) {
    spp_utils::MeasurementVarianceInputs inputs;
    inputs.elevation_rad = M_PI / 2.0;
    inputs.snr_dbhz = 35.0;  // 10 dB under the 45 dB-Hz reference
    // 10^(10/20) = 3.162 is under the cap of 4.
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(std::sqrt(10.0)), 1e-9);
    inputs.snr_dbhz = 25.0;  // 10^(20/20) = 10 is clamped to 4
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(4.0), 1e-9);
}

TEST(SPPWeightingVarianceTest, SlopeSetsTheSnrFactorExponent) {
    spp_utils::MeasurementVarianceInputs inputs;
    inputs.elevation_rad = M_PI / 2.0;
    inputs.snr_dbhz = 40.0;  // 5 dB under the reference
    inputs.snr_max_factor = 1000.0;
    inputs.snr_slope_db = 20.0;
    const double slow = spp_utils::calculatePseudorangeVariance(inputs);
    EXPECT_NEAR(slow, expectedVarianceAtZenith(std::pow(10.0, 5.0 / 20.0)), 1e-9);
    inputs.snr_slope_db = 10.0;
    const double steep = spp_utils::calculatePseudorangeVariance(inputs);
    EXPECT_NEAR(steep, expectedVarianceAtZenith(std::pow(10.0, 5.0 / 10.0)), 1e-9);
    EXPECT_GT(steep, slow);
}

TEST(SPPWeightingVarianceTest, CapBoundsTheSnrFactor) {
    spp_utils::MeasurementVarianceInputs inputs;
    inputs.elevation_rad = M_PI / 2.0;
    inputs.snr_dbhz = 20.0;  // 25 dB under the reference, 10 dB slope -> 316
    inputs.snr_slope_db = 10.0;
    inputs.snr_max_factor = 4.0;
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(4.0), 1e-9);
    inputs.snr_max_factor = 32.0;
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(32.0), 1e-9);
    // A factor under the cap is not lifted to it.
    inputs.snr_dbhz = 40.0;
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(std::sqrt(10.0)), 1e-9);
    // A strong signal is never penalised, whatever the slope and cap.
    inputs.snr_dbhz = 47.0;
    EXPECT_NEAR(spp_utils::calculatePseudorangeVariance(inputs),
                expectedVarianceAtZenith(1.0), 1e-9);
}

// preprocessEpoch() rebuilds the variance on its own path: it must follow the
// same configuration as the solver.
TEST(SPPWeightingProcessorTest, PreprocessVarianceFollowsSnrConfigAndGlonassFactor) {
    SyntheticMultiGnssScene scene;
    const auto epoch = scene.observe(epochTime(0), kAllSystems, {}, {}, 35.0);

    // Each config converges to a position differing in the last digits, so the
    // per-satellite variance ratios hold to 1e-6 rather than to rounding.
    const auto preprocess = [&](const SPPProcessor::SPPConfig& config) {
        SPPProcessor processor(config);
        EXPECT_TRUE(processor.initialize(syntheticConfig()));
        return processor.preprocessEpoch(epoch, scene.nav).second;
    };
    const auto reference = preprocess(SPPProcessor::SPPConfig());
    ASSERT_GE(reference.size(), 12u);

    SPPProcessor::SPPConfig steep;
    steep.snr_weight_slope_db = 10.0;
    steep.snr_weight_max_factor = 32.0;
    const auto steep_result = preprocess(steep);
    ASSERT_EQ(steep_result.size(), reference.size());
    // 10 dB under the reference: factor^2 is 10 (20 dB slope) vs 100 (10 dB slope).
    const double expected_ratio =
        expectedVarianceAtZenith(10.0) / expectedVarianceAtZenith(std::sqrt(10.0));
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(steep_result[i].variance / reference[i].variance, expected_ratio, 1e-6) << i;
    }

    SPPProcessor::SPPConfig capped = steep;
    capped.snr_weight_max_factor = 4.0;  // the cap alone limits the factor again
    const auto capped_result = preprocess(capped);
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(capped_result[i].variance / reference[i].variance,
                    expectedVarianceAtZenith(4.0) / expectedVarianceAtZenith(std::sqrt(10.0)),
                    1e-6) << i;
    }

    SPPProcessor::SPPConfig glonass;
    glonass.glonass_variance_factor = 4.0;
    const auto glonass_result = preprocess(glonass);
    ASSERT_EQ(glonass_result.size(), reference.size());
    int glonass_rows = 0;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const bool is_glonass = reference[i].system_id == 1;
        glonass_rows += is_glonass ? 1 : 0;
        EXPECT_NEAR(glonass_result[i].variance / reference[i].variance,
                    is_glonass ? 4.0 : 1.0, 1e-6) << i;
        EXPECT_NEAR(glonass_result[i].weight * glonass_result[i].variance, 1.0, 1e-12) << i;
    }
    EXPECT_GE(glonass_rows, 4);
}

// The least-squares solve weights rows the same way: with one clock group and a
// uniform SNR every variance scales together, so the formal position variance
// scales exactly with the code variance.
TEST(SPPWeightingProcessorTest, SolverCovarianceFollowsSlopeAndCap) {
    SyntheticMultiGnssScene scene;
    const auto epoch = scene.observe(epochTime(0), {GNSSSystem::GPS}, {}, {}, 35.0);

    const auto trace = [&](double slope, double cap) {
        SPPProcessor::SPPConfig config;
        config.snr_weight_slope_db = slope;
        config.snr_weight_max_factor = cap;
        SPPProcessor processor(config);
        EXPECT_TRUE(processor.initialize(syntheticConfig()));
        const auto solution = processor.processEpoch(epoch, scene.nav);
        EXPECT_TRUE(solution.isValid());
        return positionVarianceTrace(solution);
    };
    const double reference = trace(20.0, 4.0);
    ASSERT_GT(reference, 0.0);
    const double base_variance = expectedVarianceAtZenith(std::sqrt(10.0));
    EXPECT_NEAR(trace(10.0, 32.0) / reference, expectedVarianceAtZenith(10.0) / base_variance,
                1e-6);
    EXPECT_NEAR(trace(10.0, 4.0) / reference, expectedVarianceAtZenith(4.0) / base_variance,
                1e-6);
    EXPECT_NEAR(trace(20.0, 32.0) / reference, 1.0, 1e-6);
}

TEST(SPPWeightingProcessorTest, SolverCovarianceFollowsGlonassFactor) {
    SyntheticMultiGnssScene scene;
    // GLONASS alone: no ISB column, so the factor rescales the whole solve.
    const auto epoch = scene.observe(epochTime(0), {GNSSSystem::GLONASS});

    const auto trace = [&](double factor) {
        SPPProcessor::SPPConfig config;
        config.glonass_variance_factor = factor;
        SPPProcessor processor(config);
        EXPECT_TRUE(processor.initialize(syntheticConfig()));
        const auto solution = processor.processEpoch(epoch, scene.nav);
        EXPECT_TRUE(solution.isValid());
        return positionVarianceTrace(solution);
    };
    const double reference = trace(1.0);
    ASSERT_GT(reference, 0.0);
    EXPECT_NEAR(trace(4.0) / reference, 4.0, 1e-6);
    EXPECT_NEAR(trace(0.5) / reference, 0.5, 1e-6);
}

TEST(SPPIsbTrackTest, FirstEstimateStartsTheTrack) {
    spp_utils::IsbTrack track;
    EXPECT_FALSE(spp_utils::isbTrackHasPrior(track));
    spp_utils::updateIsbTrack(track, 5.0);
    EXPECT_EQ(track.count, 1);
    EXPECT_DOUBLE_EQ(track.value, 5.0);
    EXPECT_EQ(track.rejects, 0);
}

TEST(SPPIsbTrackTest, UpdatesRunAsAMeanThenWithAGainFloor) {
    spp_utils::IsbTrack track;
    spp_utils::updateIsbTrack(track, 1.0);
    spp_utils::updateIsbTrack(track, 3.0);  // gain 1/2
    EXPECT_EQ(track.count, 2);
    EXPECT_DOUBLE_EQ(track.value, 2.0);
    spp_utils::updateIsbTrack(track, 5.0);  // gain 1/3
    EXPECT_EQ(track.count, 3);
    EXPECT_NEAR(track.value, 2.0 + (5.0 - 2.0) / 3.0, 1e-12);

    // The gain stops falling at 0.02 (count 50).
    spp_utils::IsbTrack settled;
    settled.value = 0.0;
    settled.count = 99;
    spp_utils::updateIsbTrack(settled, 1.0);
    EXPECT_EQ(settled.count, 100);
    EXPECT_NEAR(settled.value, 0.02, 1e-12);
}

TEST(SPPIsbTrackTest, PriorNeedsTwentyUpdates) {
    spp_utils::IsbTrack track;
    for (int i = 0; i < 19; ++i) {
        spp_utils::updateIsbTrack(track, 2.0);
    }
    EXPECT_EQ(track.count, 19);
    EXPECT_FALSE(spp_utils::isbTrackHasPrior(track));
    spp_utils::updateIsbTrack(track, 2.0);
    EXPECT_EQ(track.count, 20);
    EXPECT_TRUE(spp_utils::isbTrackHasPrior(track));
}

TEST(SPPIsbTrackTest, JumpOfEightMetresIsRejectedAndSevenPointNineIsAccepted) {
    spp_utils::IsbTrack track;
    spp_utils::updateIsbTrack(track, 0.0);
    spp_utils::updateIsbTrack(track, 7.9);
    EXPECT_EQ(track.count, 2);
    EXPECT_NEAR(track.value, 3.95, 1e-12);
    EXPECT_EQ(track.rejects, 0);

    spp_utils::IsbTrack other;
    spp_utils::updateIsbTrack(other, 0.0);
    spp_utils::updateIsbTrack(other, 8.0);
    EXPECT_EQ(other.count, 1);
    EXPECT_DOUBLE_EQ(other.value, 0.0);
    EXPECT_EQ(other.rejects, 1);
    spp_utils::updateIsbTrack(other, -8.0);
    EXPECT_EQ(other.count, 1);
    EXPECT_EQ(other.rejects, 2);
}

TEST(SPPIsbTrackTest, ThirtyOneConsecutiveJumpsRestartTheTrack) {
    spp_utils::IsbTrack track;
    for (int i = 0; i < 25; ++i) {
        spp_utils::updateIsbTrack(track, 1.0);
    }
    ASSERT_TRUE(spp_utils::isbTrackHasPrior(track));
    for (int i = 0; i < 30; ++i) {
        spp_utils::updateIsbTrack(track, 20.0);
    }
    EXPECT_EQ(track.rejects, 30);
    EXPECT_EQ(track.count, 25);
    EXPECT_DOUBLE_EQ(track.value, 1.0);
    spp_utils::updateIsbTrack(track, 20.0);  // the 31st
    EXPECT_EQ(track.count, 1);
    EXPECT_DOUBLE_EQ(track.value, 20.0);
    EXPECT_EQ(track.rejects, 0);
    EXPECT_FALSE(spp_utils::isbTrackHasPrior(track));
}

TEST(SPPIsbTrackTest, AnAcceptedEstimateClearsTheRejectRun) {
    spp_utils::IsbTrack track;
    spp_utils::updateIsbTrack(track, 1.0);
    for (int i = 0; i < 30; ++i) {
        spp_utils::updateIsbTrack(track, 20.0);
    }
    spp_utils::updateIsbTrack(track, 1.0);
    EXPECT_EQ(track.rejects, 0);
    for (int i = 0; i < 30; ++i) {
        spp_utils::updateIsbTrack(track, 20.0);
    }
    EXPECT_DOUBLE_EQ(track.value, 1.0);  // 30 more jumps are not enough after the reset
    EXPECT_EQ(track.count, 2);
}

// The processor feeds the tracks from groups with four or more satellites on
// GPS-referenced epochs only, and starts counting from the first such epoch.
TEST(SPPIsbProcessorTest, TracksOnlyWellDeterminedGroupsOnGpsReferencedEpochs) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    scene.isb_m[GNSSSystem::GLONASS] = -7.0;
    SPPProcessor processor(isbPriorConfig());
    ASSERT_TRUE(processor.initialize(syntheticConfig()));

    for (int k = 0; k < 5; ++k) {
        // GLONASS contributes three satellites only.
        const auto epoch = scene.observe(epochTime(k), kAllSystems,
                                         {{GNSSSystem::GLONASS, 3}});
        ASSERT_TRUE(processor.processEpoch(epoch, scene.nav).isValid()) << k;
    }
    const auto& tracks = processor.getIsbTracks();
    ASSERT_EQ(tracks.count(GNSSSystem::Galileo), 1u);
    EXPECT_EQ(tracks.at(GNSSSystem::Galileo).count, 5);
    EXPECT_NEAR(tracks.at(GNSSSystem::Galileo).value, 12.0, 0.5);
    EXPECT_EQ(tracks.count(GNSSSystem::GLONASS), 0u);
    EXPECT_EQ(tracks.count(GNSSSystem::GPS), 0u);

    // Without GPS the clock reference is another group: nothing is tracked.
    const auto no_gps = scene.observe(epochTime(5),
                                      {GNSSSystem::Galileo, GNSSSystem::GLONASS});
    ASSERT_TRUE(processor.processEpoch(no_gps, scene.nav).isValid());
    EXPECT_EQ(processor.getIsbTracks().at(GNSSSystem::Galileo).count, 5);
    EXPECT_EQ(processor.getIsbTracks().count(GNSSSystem::GLONASS), 0u);

    // Four GLONASS satellites are enough.
    const auto four = scene.observe(epochTime(6), kAllSystems, {{GNSSSystem::GLONASS, 4}});
    ASSERT_TRUE(processor.processEpoch(four, scene.nav).isValid());
    ASSERT_EQ(processor.getIsbTracks().count(GNSSSystem::GLONASS), 1u);
    EXPECT_EQ(processor.getIsbTracks().at(GNSSSystem::GLONASS).count, 1);
    EXPECT_EQ(processor.getIsbTracks().at(GNSSSystem::Galileo).count, 6);
}

TEST(SPPIsbProcessorTest, TracksStayEmptyWithoutAPrior) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    SPPProcessor processor;  // library default: isb_prior_sigma_m == 0
    ASSERT_TRUE(processor.initialize(syntheticConfig()));
    for (int k = 0; k < 3; ++k) {
        ASSERT_TRUE(processor.processEpoch(scene.observe(epochTime(k), kGpsGal), scene.nav)
                        .isValid());
    }
    EXPECT_TRUE(processor.getIsbTracks().empty());
}

TEST(SPPIsbProcessorTest, ResetClearsTheTracks) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    SPPProcessor processor(isbPriorConfig());
    ASSERT_TRUE(processor.initialize(syntheticConfig()));
    for (int k = 0; k < 3; ++k) {
        ASSERT_TRUE(processor.processEpoch(scene.observe(epochTime(k), kGpsGal), scene.nav)
                        .isValid());
    }
    ASSERT_FALSE(processor.getIsbTracks().empty());
    processor.reset();
    EXPECT_TRUE(processor.getIsbTracks().empty());
    ASSERT_TRUE(processor.initialize(syntheticConfig()));
    ASSERT_TRUE(processor.processEpoch(scene.observe(epochTime(3), kGpsGal), scene.nav)
                    .isValid());
    EXPECT_EQ(processor.getIsbTracks().at(GNSSSystem::Galileo).count, 1);
}

namespace {

// Runs `priming_epochs` full epochs, then one epoch where Galileo has only two
// satellites carrying `thin_offset_m` of extra bias. Returns the Galileo ISB the
// final solve estimated.
double thinGroupIsb(const SPPProcessor::SPPConfig& config,
                    int priming_epochs,
                    const std::set<GNSSSystem>& probe_systems,
                    double thin_offset_m,
                    GNSSSystem probed_group = GNSSSystem::Galileo,
                    const SyntheticMultiGnssScene* shared_scene = nullptr) {
    SyntheticMultiGnssScene local;
    const SyntheticMultiGnssScene& scene = shared_scene ? *shared_scene : local;
    SPPProcessor processor(config);
    EXPECT_TRUE(processor.initialize(syntheticConfig()));
    for (int k = 0; k < priming_epochs; ++k) {
        EXPECT_TRUE(processor.processEpoch(scene.observe(epochTime(k), kAllSystems), scene.nav)
                        .isValid());
    }
    const auto probe = scene.observe(epochTime(priming_epochs), probe_systems,
                                     {{probed_group, 2}}, {{probed_group, thin_offset_m}});
    EXPECT_TRUE(processor.processEpoch(probe, scene.nav).isValid());
    return processor.getSystemBiases().at(probed_group);
}

}  // namespace

// With only two Galileo satellites the ISB column absorbs a common offset
// exactly; the prior holds it near the tracked value instead.
TEST(SPPIsbProcessorTest, PriorHoldsAThinGroupNearTheTrackedBias) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    const double offset = 4.0;

    const double unconstrained = thinGroupIsb(SPPProcessor::SPPConfig(), 25, kGpsGal,
                                              offset, GNSSSystem::Galileo, &scene);
    EXPECT_NEAR(unconstrained, 12.0 + offset, 0.5);

    const double constrained = thinGroupIsb(isbPriorConfig(), 25, kGpsGal, offset,
                                            GNSSSystem::Galileo, &scene);
    EXPECT_NEAR(constrained, 12.0, 1.5);
    EXPECT_LT(constrained, unconstrained - 2.0);
}

TEST(SPPIsbProcessorTest, PriorNeedsTwentyTrackedEpochs) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    const double offset = 4.0;

    // After 19 tracked epochs the track holds 19 updates: no prior yet.
    const double nineteen = thinGroupIsb(isbPriorConfig(), 19, kGpsGal, offset,
                                         GNSSSystem::Galileo, &scene);
    EXPECT_NEAR(nineteen, 12.0 + offset, 0.5);
    // After 20 it is applied.
    const double twenty = thinGroupIsb(isbPriorConfig(), 20, kGpsGal, offset,
                                       GNSSSystem::Galileo, &scene);
    EXPECT_NEAR(twenty, 12.0, 1.5);
}

TEST(SPPIsbProcessorTest, PriorIsNotAppliedWithoutAGpsReference) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    scene.isb_m[GNSSSystem::GLONASS] = -7.0;
    const double offset = 4.0;

    // Probe without GPS: Galileo is the reference, GLONASS is the thin group.
    // The tracked GLONASS bias is against GPS, so it must not constrain this solve.
    SPPProcessor::SPPConfig no_prior;
    const double free_isb = thinGroupIsb(no_prior, 25, {GNSSSystem::Galileo, GNSSSystem::GLONASS},
                                         offset, GNSSSystem::GLONASS, &scene);
    const double with_prior = thinGroupIsb(isbPriorConfig(), 25,
                                           {GNSSSystem::Galileo, GNSSSystem::GLONASS}, offset,
                                           GNSSSystem::GLONASS, &scene);
    EXPECT_NEAR(with_prior, free_isb, 1e-6);
}

TEST(SPPIsbProcessorTest, PriorIsNotAppliedWhenTheSigmaIsZero) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;
    SPPProcessor::SPPConfig zero;
    zero.isb_prior_sigma_m = 0.0;
    const double isb = thinGroupIsb(zero, 25, kGpsGal, 4.0, GNSSSystem::Galileo, &scene);
    EXPECT_NEAR(isb, 16.0, 0.5);
}

// The prior also stiffens the ISB column of the reported covariance.
TEST(SPPIsbProcessorTest, PriorTightensTheReportedPositionCovariance) {
    SyntheticMultiGnssScene scene;
    scene.isb_m[GNSSSystem::Galileo] = 12.0;

    const auto thin_trace = [&](const SPPProcessor::SPPConfig& config) {
        SPPProcessor processor(config);
        EXPECT_TRUE(processor.initialize(syntheticConfig()));
        for (int k = 0; k < 25; ++k) {
            EXPECT_TRUE(processor.processEpoch(scene.observe(epochTime(k), kGpsGal), scene.nav)
                            .isValid());
        }
        const auto probe = scene.observe(epochTime(25), kGpsGal,
                                         {{GNSSSystem::GPS, 4}, {GNSSSystem::Galileo, 4}});
        const auto solution = processor.processEpoch(probe, scene.nav);
        EXPECT_TRUE(solution.isValid());
        return positionVarianceTrace(solution);
    };
    const double without = thin_trace(SPPProcessor::SPPConfig());
    const double with = thin_trace(isbPriorConfig());

    ASSERT_GT(without, 0.0);
    // Four GPS and four Galileo satellites leave the geometry weak once the ISB
    // column soaks up a degree of freedom; the prior's 1/sigma^2 on the diagonal
    // returns it to the position.
    EXPECT_LT(with, 0.9 * without) << with << " vs " << without;
}
