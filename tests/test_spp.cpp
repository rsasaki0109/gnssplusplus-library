#include <gtest/gtest.h>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/io/rinex.hpp>

#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <set>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

}  // namespace

class SPPTest : public ::testing::Test {
protected:
    void SetUp() override {
        processor_config_.elevation_mask = 15.0;
        processor_config_.snr_mask = 0.0;  // RINEX 2 sample data does not carry SNR.
        processor_config_.mode = PositioningMode::SPP;

        spp_processor_ = std::make_unique<SPPProcessor>();
        ASSERT_TRUE(spp_processor_->initialize(processor_config_));
        ASSERT_TRUE(loadNavigationData());
        ASSERT_TRUE(loadFirstEpoch());
    }

    bool loadNavigationData() {
        io::RINEXReader nav_reader;
        if (!nav_reader.open(sourcePath("data/navigation_static.nav"))) {
            return false;
        }
        return nav_reader.readNavigationData(nav_data_);
    }

    bool loadFirstEpoch() {
        io::RINEXReader obs_reader;
        if (!obs_reader.open(sourcePath("data/rover_static.obs"))) {
            return false;
        }
        if (!obs_reader.readHeader(obs_header_)) {
            return false;
        }
        if (!obs_reader.readObservationEpoch(obs_data_)) {
            return false;
        }
        if (obs_header_.approximate_position.norm() > 0.0) {
            obs_data_.receiver_position = obs_header_.approximate_position;
        }
        return true;
    }

    bool loadOdaibaMixedNavigation(NavigationData& nav_data) const {
        io::RINEXReader nav_reader;
        if (!nav_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base.nav"))) {
            return false;
        }
        return nav_reader.readNavigationData(nav_data);
    }

    bool loadOdaibaFirstEpoch(ObservationData& obs_data,
                              io::RINEXReader::RINEXHeader& header) const {
        io::RINEXReader obs_reader;
        if (!obs_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs"))) {
            return false;
        }
        if (!obs_reader.readHeader(header)) {
            return false;
        }
        if (!obs_reader.readObservationEpoch(obs_data)) {
            return false;
        }
        if (header.approximate_position.norm() > 0.0) {
            obs_data.receiver_position = header.approximate_position;
        }
        return true;
    }

    std::vector<ObservationData> loadEpochs(int max_epochs) const {
        io::RINEXReader obs_reader;
        if (!obs_reader.open(sourcePath("data/rover_static.obs"))) {
            return {};
        }

        io::RINEXReader::RINEXHeader header;
        if (!obs_reader.readHeader(header)) {
            return {};
        }

        std::vector<ObservationData> epochs;
        ObservationData epoch;
        while (static_cast<int>(epochs.size()) < max_epochs &&
               obs_reader.readObservationEpoch(epoch)) {
            if (header.approximate_position.norm() > 0.0) {
                epoch.receiver_position = header.approximate_position;
            }
            epochs.push_back(epoch);
        }
        return epochs;
    }

    ObservationData makeInsufficientObservationSet() const {
        ObservationData trimmed;
        trimmed.time = obs_data_.time;
        trimmed.receiver_position = obs_data_.receiver_position;

        for (const auto& obs : obs_data_.observations) {
            if (obs.signal != SignalType::GPS_L1CA ||
                !obs.valid ||
                !obs.has_pseudorange ||
                obs.pseudorange <= 0.0) {
                continue;
            }
            trimmed.addObservation(obs);
            if (trimmed.observations.size() == 3) {
                break;
            }
        }
        return trimmed;
    }

    std::unique_ptr<SPPProcessor> spp_processor_;
    ProcessorConfig processor_config_;
    io::RINEXReader::RINEXHeader obs_header_;
    ObservationData obs_data_;
    NavigationData nav_data_;
};

TEST_F(SPPTest, ProcessorInitialization) {
    ASSERT_NE(spp_processor_, nullptr);

    const auto config = spp_processor_->getSPPConfig();
    EXPECT_GT(config.max_iterations, 0);
    EXPECT_GT(config.pseudorange_sigma, 0.0);
}

TEST_F(SPPTest, BasicPositioning) {
    const auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);

    EXPECT_TRUE(solution.isValid());
    EXPECT_EQ(solution.status, SolutionStatus::SPP);
    EXPECT_GE(solution.num_satellites, 4);
    EXPECT_GT(solution.pdop, 0.0);
    EXPECT_LT(solution.pdop, 20.0);
}

TEST_F(SPPTest, InsufficientSatellites) {
    const ObservationData trimmed = makeInsufficientObservationSet();
    ASSERT_EQ(trimmed.observations.size(), 3u);

    const auto solution = spp_processor_->processEpoch(trimmed, nav_data_);

    EXPECT_FALSE(solution.isValid());
    EXPECT_LT(solution.num_satellites, 4);
}

TEST_F(SPPTest, QualityControl) {
    ObservationData degraded = obs_data_;
    SatelliteId downgraded_sat;
    bool changed = false;

    for (auto& obs : degraded.observations) {
        if (obs.signal != SignalType::GPS_L1CA || !obs.valid || !obs.has_pseudorange) {
            continue;
        }
        downgraded_sat = obs.satellite;
        obs.snr = -1.0;
        changed = true;
        break;
    }

    ASSERT_TRUE(changed);

    const auto solution = spp_processor_->processEpoch(degraded, nav_data_);

    EXPECT_TRUE(solution.isValid());

    bool found_downgraded = false;
    for (const auto& sat : solution.satellites_used) {
        if (sat == downgraded_sat) {
            found_downgraded = true;
            break;
        }
    }
    EXPECT_FALSE(found_downgraded);
}

TEST_F(SPPTest, DOPCalculation) {
    const auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);

    ASSERT_TRUE(solution.isValid());
    EXPECT_GT(solution.gdop, 0.0);
    EXPECT_GT(solution.pdop, 0.0);
    EXPECT_GT(solution.hdop, 0.0);
    EXPECT_GT(solution.vdop, 0.0);
    EXPECT_GE(solution.gdop, solution.pdop);
}

TEST_F(SPPTest, ProcessingStatistics) {
    const auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    const auto stats = spp_processor_->getStats();

    ASSERT_TRUE(solution.isValid());
    EXPECT_GE(solution.processing_time_ms, 0.0);
    EXPECT_GE(solution.iterations, 1);
    EXPECT_GE(solution.residual_rms, 0.0);
    EXPECT_EQ(stats.total_epochs, 1u);
    EXPECT_EQ(stats.valid_solutions, 1u);
    EXPECT_GE(stats.average_processing_time_ms, 0.0);
}

TEST_F(SPPTest, MultipleEpochs) {
    auto epochs = loadEpochs(10);
    ASSERT_EQ(epochs.size(), 10u);

    spp_processor_->reset();
    ASSERT_TRUE(spp_processor_->initialize(processor_config_));

    int valid_solutions = 0;
    for (const auto& epoch : epochs) {
        const auto solution = spp_processor_->processEpoch(epoch, nav_data_);
        if (solution.isValid()) {
            valid_solutions++;
        }
    }

    EXPECT_GE(valid_solutions, 8);
}

TEST_F(SPPTest, UtilityFunctions) {
    const Vector3d receiver_pos(4000000.0, 3000000.0, 2000000.0);
    const Vector3d satellite_pos(20000000.0, 15000000.0, 10000000.0);

    const double elevation = spp_utils::calculateElevation(receiver_pos, satellite_pos);
    EXPECT_GE(elevation, 0.0);
    EXPECT_LE(elevation, M_PI / 2);

    const double azimuth = spp_utils::calculateAzimuth(receiver_pos, satellite_pos);
    EXPECT_GE(azimuth, -M_PI);
    EXPECT_LE(azimuth, M_PI);

    const auto geodetic = spp_utils::ecefToGeodetic(receiver_pos);
    EXPECT_GE(geodetic.latitude, -M_PI / 2);
    EXPECT_LE(geodetic.latitude, M_PI / 2);
    EXPECT_GE(geodetic.longitude, -M_PI);
    EXPECT_LE(geodetic.longitude, M_PI);

    const auto ecef_back = spp_utils::geodeticToEcef(geodetic);
    EXPECT_NEAR(ecef_back(0), receiver_pos(0), 1.0);
    EXPECT_NEAR(ecef_back(1), receiver_pos(1), 1.0);
    EXPECT_NEAR(ecef_back(2), receiver_pos(2), 1.0);
}

TEST_F(SPPTest, ReadsMixedConstellationObservationEpochsFromOdaiba) {
    ObservationData odaiba_epoch;
    io::RINEXReader::RINEXHeader odaiba_header;
    ASSERT_TRUE(loadOdaibaFirstEpoch(odaiba_epoch, odaiba_header));

    std::set<GNSSSystem> systems;
    for (const auto& obs : odaiba_epoch.observations) {
        systems.insert(obs.satellite.system);
    }

    EXPECT_TRUE(systems.count(GNSSSystem::GPS));
    EXPECT_TRUE(systems.count(GNSSSystem::BeiDou));
    EXPECT_TRUE(systems.count(GNSSSystem::Galileo));
    EXPECT_TRUE(systems.count(GNSSSystem::QZSS));
}

TEST_F(SPPTest, ReadsNonGpsBroadcastEphemerisFromMixedNavigationFile) {
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    bool has_beidou = false;
    bool has_galileo = false;
    bool has_glonass = false;
    bool has_qzss = false;
    bool has_beidou_secondary_delay = false;
    for (const auto& [sat, ephs] : mixed_nav.ephemeris_data) {
        has_beidou = has_beidou || sat.system == GNSSSystem::BeiDou;
        has_galileo = has_galileo || sat.system == GNSSSystem::Galileo;
        has_glonass = has_glonass || sat.system == GNSSSystem::GLONASS;
        has_qzss = has_qzss || sat.system == GNSSSystem::QZSS;
        if (sat.system == GNSSSystem::BeiDou) {
            for (const auto& eph : ephs) {
                if (std::abs(eph.tgd_secondary) > 0.0) {
                    has_beidou_secondary_delay = true;
                    break;
                }
            }
        }
    }

    EXPECT_TRUE(has_beidou);
    EXPECT_TRUE(has_galileo);
    EXPECT_TRUE(has_glonass);
    EXPECT_TRUE(has_qzss);
    EXPECT_TRUE(has_beidou_secondary_delay);
}

TEST_F(SPPTest, ComputesGlonassBroadcastStateFromMixedNavigationFile) {
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    ObservationData odaiba_epoch;
    io::RINEXReader::RINEXHeader odaiba_header;
    ASSERT_TRUE(loadOdaibaFirstEpoch(odaiba_epoch, odaiba_header));

    bool solved_glonass = false;
    for (const auto& obs : odaiba_epoch.observations) {
        if (obs.satellite.system != GNSSSystem::GLONASS) {
            continue;
        }
        Vector3d pos;
        Vector3d vel;
        double clk = 0.0;
        double drift = 0.0;
        if (mixed_nav.calculateSatelliteState(obs.satellite, odaiba_epoch.time, pos, vel, clk, drift)) {
            EXPECT_TRUE(pos.allFinite());
            EXPECT_GT(pos.norm(), 1.5e7);
            solved_glonass = true;
            break;
        }
    }

    EXPECT_TRUE(solved_glonass);
}

TEST_F(SPPTest, ComputesBeiDouBroadcastStateFromMixedNavigationFile) {
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    ObservationData odaiba_epoch;
    io::RINEXReader::RINEXHeader odaiba_header;
    ASSERT_TRUE(loadOdaibaFirstEpoch(odaiba_epoch, odaiba_header));

    bool solved_beidou = false;
    for (const auto& obs : odaiba_epoch.observations) {
        if (obs.satellite.system != GNSSSystem::BeiDou) {
            continue;
        }
        Vector3d pos;
        Vector3d vel;
        double clk = 0.0;
        double drift = 0.0;
        if (mixed_nav.calculateSatelliteState(obs.satellite, odaiba_epoch.time, pos, vel, clk, drift)) {
            EXPECT_TRUE(pos.allFinite());
            EXPECT_GT(pos.norm(), 2.0e7);
            solved_beidou = true;
            break;
        }
    }

    EXPECT_TRUE(solved_beidou);
}

TEST_F(SPPTest, UsesMultipleConstellationsOnOdaibaEpoch) {
    ObservationData odaiba_epoch;
    io::RINEXReader::RINEXHeader odaiba_header;
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaFirstEpoch(odaiba_epoch, odaiba_header));
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    spp_processor_->reset();
    ASSERT_TRUE(spp_processor_->initialize(processor_config_));
    const auto solution = spp_processor_->processEpoch(odaiba_epoch, mixed_nav);

    ASSERT_TRUE(solution.isValid());
    std::set<GNSSSystem> used_systems;
    for (const auto& sat : solution.satellites_used) {
        used_systems.insert(sat.system);
    }
    EXPECT_TRUE(used_systems.count(GNSSSystem::GPS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::GLONASS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::BeiDou));
    EXPECT_TRUE(used_systems.count(GNSSSystem::Galileo));
    EXPECT_TRUE(used_systems.count(GNSSSystem::QZSS));
}

TEST_F(SPPTest, BeiDouEnabledSPPStaysCloseOnOdaibaSequence) {
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    io::RINEXReader obs_reader;
    ASSERT_TRUE(obs_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs")));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(obs_reader.readHeader(header));

    SPPProcessor::SPPConfig baseline_config;
    baseline_config.enable_beidou = false;
    SPPProcessor baseline_processor(baseline_config);
    ASSERT_TRUE(baseline_processor.initialize(processor_config_));

    SPPProcessor beidou_processor;
    ASSERT_TRUE(beidou_processor.initialize(processor_config_));

    ObservationData epoch;
    int count = 0;
    int beidou_improves_sat_count = 0;
    while (count < 20 && obs_reader.readObservationEpoch(epoch)) {
        if (header.approximate_position.norm() > 0.0) {
            epoch.receiver_position = header.approximate_position;
        }

        const auto baseline = baseline_processor.processEpoch(epoch, mixed_nav);
        const auto beidou = beidou_processor.processEpoch(epoch, mixed_nav);
        ASSERT_TRUE(baseline.isValid()) << count;
        ASSERT_TRUE(beidou.isValid()) << count;

        const double position_delta = (baseline.position_ecef - beidou.position_ecef).norm();
        EXPECT_LT(position_delta, 3.0) << count;
        EXPECT_GE(beidou.num_satellites, baseline.num_satellites) << count;
        if (beidou.num_satellites > baseline.num_satellites) {
            beidou_improves_sat_count++;
        }
        count++;
    }

    EXPECT_EQ(count, 20);
    EXPECT_GT(beidou_improves_sat_count, 0);
}
