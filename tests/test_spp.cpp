#include <gtest/gtest.h>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <set>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

bool sourcePathExists(const std::string& relative_path) {
    return std::filesystem::exists(sourcePath(relative_path));
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

SatelliteId parseSatelliteId(const std::string& token) {
    if (token.size() < 2) {
        return {};
    }
    const int prn = std::stoi(token.substr(1));
    switch (token[0]) {
        case 'G': return SatelliteId(GNSSSystem::GPS, prn);
        case 'R': return SatelliteId(GNSSSystem::GLONASS, prn);
        case 'E': return SatelliteId(GNSSSystem::Galileo, prn);
        case 'C': return SatelliteId(GNSSSystem::BeiDou, prn);
        case 'J': return SatelliteId(GNSSSystem::QZSS, prn);
        case 'I': return SatelliteId(GNSSSystem::NavIC, prn);
        default: return {};
    }
}

double expectedClaslibBroadcastVariance(const Ephemeris& eph) {
    const double accuracy =
        std::isfinite(eph.sv_accuracy) && eph.sv_accuracy >= 0.0 ? eph.sv_accuracy : 0.0;
    if (eph.satellite.system == GNSSSystem::Galileo) {
        const double sisa_cm = accuracy * 100.0;
        int index = 255;
        if (sisa_cm >= 0.0 && sisa_cm < 50.0) {
            index = static_cast<int>(std::ceil(sisa_cm));
        } else if (sisa_cm >= 50.0 && sisa_cm < 100.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 50.0) / 2.0)) + 50;
        } else if (sisa_cm >= 100.0 && sisa_cm < 200.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 100.0) / 4.0)) + 75;
        } else if (sisa_cm >= 200.0 && sisa_cm <= 600.0) {
            index = static_cast<int>(std::ceil((sisa_cm - 200.0) / 16.0)) + 100;
        }

        double value_cm = 6144.0 * 100.0;
        if (index >= 0 && index <= 49) {
            value_cm = static_cast<double>(index);
        } else if (index >= 50 && index <= 74) {
            value_cm = static_cast<double>(index - 50) * 2.0 + 50.0;
        } else if (index >= 75 && index <= 99) {
            value_cm = static_cast<double>(index - 75) * 4.0 + 100.0;
        } else if (index >= 100 && index <= 125) {
            value_cm = static_cast<double>(index - 100) * 16.0 + 200.0;
        }
        const double value_m = value_cm / 100.0;
        return value_m * value_m;
    }

    constexpr double kUraValues[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0, 96.0,
        192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0
    };
    for (const double value : kUraValues) {
        if (value >= accuracy) {
            return value * value;
        }
    }
    return 6144.0 * 6144.0;
}

}  // namespace

class SPPTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!sourcePathExists("data/navigation_static.nav") ||
            !sourcePathExists("data/rover_static.obs")) {
            GTEST_SKIP() << "repo static test data is not available";
        }
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

TEST_F(SPPTest, RespectsConfiguredElevationMask) {
    ProcessorConfig high_mask_config = processor_config_;
    high_mask_config.elevation_mask = 89.0;

    SPPProcessor high_mask_processor;
    ASSERT_TRUE(high_mask_processor.initialize(high_mask_config));

    const auto solution = high_mask_processor.processEpoch(obs_data_, nav_data_);

    EXPECT_FALSE(solution.isValid());
    EXPECT_EQ(solution.num_satellites, 0);
}

TEST_F(SPPTest, IonosphereFreeCodeModeProducesSolution) {
    SPPProcessor::SPPConfig iflc_config;
    iflc_config.use_ionosphere_free_code = true;
    SPPProcessor iflc_processor(iflc_config);
    ASSERT_TRUE(iflc_processor.initialize(processor_config_));

    const auto solution = iflc_processor.processEpoch(obs_data_, nav_data_);

    EXPECT_TRUE(solution.isValid());
    EXPECT_GE(solution.num_satellites, 4);
}

TEST_F(SPPTest, ConfiguredZeroInitialPositionBypassesHeaderSeed) {
    const auto dump_path =
        std::filesystem::temp_directory_path() / "libgnss_spp_zero_seed_config_test.csv";
    std::filesystem::remove(dump_path);
    setenv("GNSS_SPP_ITERATION_DUMP", dump_path.c_str(), 1);

    SPPProcessor::SPPConfig config;
    config.use_zero_initial_position = true;
    SPPProcessor processor(config);
    ASSERT_TRUE(processor.initialize(processor_config_));

    const auto solution = processor.processEpoch(obs_data_, nav_data_);
    unsetenv("GNSS_SPP_ITERATION_DUMP");

    ASSERT_TRUE(solution.isValid());

    std::ifstream dump(dump_path);
    ASSERT_TRUE(dump.good());
    std::string header_line;
    std::string row_line;
    ASSERT_TRUE(static_cast<bool>(std::getline(dump, header_line)));
    ASSERT_TRUE(static_cast<bool>(std::getline(dump, row_line)));
    std::filesystem::remove(dump_path);

    const auto header = splitCsvLine(header_line);
    const auto row = splitCsvLine(row_line);
    ASSERT_EQ(header.size(), row.size());

    const auto pos_x_it = std::find(header.begin(), header.end(), "pos_before_x_m");
    const auto pos_y_it = std::find(header.begin(), header.end(), "pos_before_y_m");
    const auto pos_z_it = std::find(header.begin(), header.end(), "pos_before_z_m");
    ASSERT_NE(pos_x_it, header.end());
    ASSERT_NE(pos_y_it, header.end());
    ASSERT_NE(pos_z_it, header.end());

    const auto pos_x_index = static_cast<size_t>(std::distance(header.begin(), pos_x_it));
    const auto pos_y_index = static_cast<size_t>(std::distance(header.begin(), pos_y_it));
    const auto pos_z_index = static_cast<size_t>(std::distance(header.begin(), pos_z_it));
    EXPECT_NEAR(std::stod(row[pos_x_index]), 0.0, 1e-9);
    EXPECT_NEAR(std::stod(row[pos_y_index]), 0.0, 1e-9);
    EXPECT_NEAR(std::stod(row[pos_z_index]), 0.0, 1e-9);
}

TEST_F(SPPTest, PntposCodeWeightIncludesBroadcastEphemerisVariance) {
    const auto dump_path =
        std::filesystem::temp_directory_path() / "libgnss_spp_pntpos_weight_test.csv";
    std::filesystem::remove(dump_path);
    setenv("GNSS_SPP_ITERATION_DUMP", dump_path.c_str(), 1);

    SPPProcessor::SPPConfig config;
    config.use_pntpos_code_weight = true;
    SPPProcessor processor(config);
    ASSERT_TRUE(processor.initialize(processor_config_));

    const auto solution = processor.processEpoch(obs_data_, nav_data_);
    unsetenv("GNSS_SPP_ITERATION_DUMP");

    ASSERT_TRUE(solution.isValid());

    std::ifstream dump(dump_path);
    ASSERT_TRUE(dump.good());
    std::string header_line;
    std::string row_line;
    ASSERT_TRUE(static_cast<bool>(std::getline(dump, header_line)));
    ASSERT_TRUE(static_cast<bool>(std::getline(dump, row_line)));
    std::filesystem::remove(dump_path);

    const auto header = splitCsvLine(header_line);
    const auto row = splitCsvLine(row_line);
    ASSERT_EQ(header.size(), row.size());

    const auto sat_it = std::find(header.begin(), header.end(), "sat");
    const auto eph_var_it = std::find(header.begin(), header.end(), "ephemeris_variance_m2");
    const auto variance_it = std::find(header.begin(), header.end(), "variance_m2");
    ASSERT_NE(sat_it, header.end());
    ASSERT_NE(eph_var_it, header.end());
    ASSERT_NE(variance_it, header.end());

    const auto sat_index = static_cast<size_t>(std::distance(header.begin(), sat_it));
    const auto eph_var_index = static_cast<size_t>(std::distance(header.begin(), eph_var_it));
    const auto variance_index = static_cast<size_t>(std::distance(header.begin(), variance_it));
    const SatelliteId sat = parseSatelliteId(row[sat_index]);
    const Ephemeris* eph = nav_data_.getEphemeris(sat, obs_data_.time);
    ASSERT_NE(eph, nullptr);

    const double ephemeris_variance = std::stod(row[eph_var_index]);
    const double total_variance = std::stod(row[variance_index]);
    EXPECT_NEAR(ephemeris_variance, expectedClaslibBroadcastVariance(*eph), 1e-9);
    EXPECT_GT(total_variance, ephemeris_variance);
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

TEST_F(SPPTest, ComputesGalileoBroadcastStateFromMixedNavigationFile) {
    NavigationData mixed_nav;
    ASSERT_TRUE(loadOdaibaMixedNavigation(mixed_nav));

    ObservationData odaiba_epoch;
    io::RINEXReader::RINEXHeader odaiba_header;
    ASSERT_TRUE(loadOdaibaFirstEpoch(odaiba_epoch, odaiba_header));

    bool solved_galileo = false;
    for (const auto& obs : odaiba_epoch.observations) {
        if (obs.satellite.system != GNSSSystem::Galileo) {
            continue;
        }
        Vector3d pos;
        Vector3d vel;
        double clk = 0.0;
        double drift = 0.0;
        if (mixed_nav.calculateSatelliteState(obs.satellite, odaiba_epoch.time, pos, vel, clk, drift)) {
            EXPECT_TRUE(pos.allFinite());
            EXPECT_GT(pos.norm(), 2.0e7);
            solved_galileo = true;
            break;
        }
    }

    EXPECT_TRUE(solved_galileo);
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
    bool used_bds2 = false;
    bool used_bds3 = false;
    for (const auto& sat : solution.satellites_used) {
        used_systems.insert(sat.system);
        used_bds2 = used_bds2 || signal_policy::isBeiDou2Satellite(sat);
        used_bds3 = used_bds3 || signal_policy::isBeiDou3Satellite(sat);
    }
    EXPECT_TRUE(used_systems.count(GNSSSystem::GPS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::GLONASS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::BeiDou));
    EXPECT_TRUE(used_systems.count(GNSSSystem::QZSS));
    EXPECT_GE(used_systems.size(), 4U);
    EXPECT_TRUE(solution.receiver_clock_biases_m.count(ReceiverClockBiasGroup::GPS));
    EXPECT_TRUE(solution.receiver_clock_biases_m.count(ReceiverClockBiasGroup::GLONASS));
    if (used_bds2) {
        EXPECT_TRUE(solution.receiver_clock_biases_m.count(ReceiverClockBiasGroup::BeiDou2));
    }
    if (used_bds3) {
        EXPECT_TRUE(solution.receiver_clock_biases_m.count(ReceiverClockBiasGroup::BeiDou3));
    }
}

TEST_F(SPPTest, BeiDouEnabledSPPStaysCloseOnOdaibaSequence) {
    if (!sourcePathExists("data/driving/Tokyo_Data/Odaiba/base.nav") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs")) {
        GTEST_SKIP() << "repo Odaiba test data is not available";
    }
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
