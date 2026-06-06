#include <gtest/gtest.h>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/models/ionosphere.hpp>

#include <memory>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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

std::filesystem::path tempFilePath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

void writeTextFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream output(path);
    output << contents;
}

double solveKeplerForTest(double mean_anomaly, double eccentricity) {
    double eccentric_anomaly = mean_anomaly;
    for (int i = 0; i < 30; ++i) {
        const double previous = eccentric_anomaly;
        eccentric_anomaly -=
            (eccentric_anomaly - eccentricity * std::sin(eccentric_anomaly) -
             mean_anomaly) /
            (1.0 - eccentricity * std::cos(eccentric_anomaly));
        if (std::abs(eccentric_anomaly - previous) < 1e-14) {
            break;
        }
    }
    return eccentric_anomaly;
}

Vector3d expectedBeiDouGeoPosition(const Ephemeris& eph,
                                   const GNSSTime& time,
                                   bool use_geo_omega) {
    constexpr double kBeiDouEarthMu = 3.986004418e14;
    constexpr double kBeiDouEarthRotationRate = 7.292115e-5;
    constexpr double kSinMinus5Deg = -0.0871557427476582;
    constexpr double kCosMinus5Deg = 0.9961946980917456;

    const double semi_major_axis = eph.sqrt_a * eph.sqrt_a;
    const double tk = time - eph.toe;
    const double n0 = std::sqrt(kBeiDouEarthMu /
                                (semi_major_axis * semi_major_axis *
                                 semi_major_axis));
    const double mean_anomaly = eph.m0 + (n0 + eph.delta_n) * tk;
    const double eccentric_anomaly = solveKeplerForTest(mean_anomaly, eph.e);
    const double sin_e = std::sin(eccentric_anomaly);
    const double cos_e = std::cos(eccentric_anomaly);
    const double true_anomaly =
        std::atan2(std::sqrt(1.0 - eph.e * eph.e) * sin_e, cos_e - eph.e);
    const double phi = true_anomaly + eph.omega;
    const double two_phi = 2.0 * phi;
    const double u = phi + eph.cus * std::sin(two_phi) +
                     eph.cuc * std::cos(two_phi);
    const double r = semi_major_axis * (1.0 - eph.e * cos_e) +
                     eph.crs * std::sin(two_phi) +
                     eph.crc * std::cos(two_phi);
    const double inclination =
        eph.i0 + eph.idot * tk +
        eph.cis * std::sin(two_phi) + eph.cic * std::cos(two_phi);
    const double toe_seconds = eph.toes != 0.0 ? eph.toes : eph.toe.tow;
    const double omega =
        use_geo_omega
            ? eph.omega0 + eph.omega_dot * tk -
                  kBeiDouEarthRotationRate * toe_seconds
            : eph.omega0 + (eph.omega_dot - kBeiDouEarthRotationRate) * tk -
                  kBeiDouEarthRotationRate * toe_seconds;

    const double x_orb = r * std::cos(u);
    const double y_orb = r * std::sin(u);
    const double xg = x_orb * std::cos(omega) -
                      y_orb * std::cos(inclination) * std::sin(omega);
    const double yg = x_orb * std::sin(omega) +
                      y_orb * std::cos(inclination) * std::cos(omega);
    const double zg = y_orb * std::sin(inclination);
    const double sin_rot = std::sin(kBeiDouEarthRotationRate * tk);
    const double cos_rot = std::cos(kBeiDouEarthRotationRate * tk);

    Vector3d position;
    position(0) = xg * cos_rot + yg * sin_rot * kCosMinus5Deg +
                  zg * sin_rot * kSinMinus5Deg;
    position(1) = -xg * sin_rot + yg * cos_rot * kCosMinus5Deg +
                  zg * cos_rot * kSinMinus5Deg;
    position(2) = -yg * kSinMinus5Deg + zg * kCosMinus5Deg;
    return position;
}

}  // namespace

TEST(NavigationTest, BeiDouGeoBroadcastStateUsesGeoRotationFrame) {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::BeiDou, 1);
    eph.valid = true;
    eph.week = 2323;
    eph.toe = GNSSTime(2323, 553800.0);
    eph.toc = eph.toe;
    eph.toes = 553800.0;
    eph.sqrt_a = std::sqrt(42164000.0);
    eph.e = 0.0012;
    eph.i0 = 0.08;
    eph.omega0 = 1.25;
    eph.omega = 0.61;
    eph.m0 = 0.42;
    eph.delta_n = 1.0e-9;
    eph.idot = -2.0e-10;
    eph.omega_dot = -2.6e-9;
    eph.cuc = 1.0e-6;
    eph.cus = -2.0e-6;
    eph.crc = 120.0;
    eph.crs = -30.0;
    eph.cic = 2.0e-7;
    eph.cis = -3.0e-7;

    const GNSSTime eval_time = eph.toe + 120.0;
    Vector3d position;
    Vector3d velocity;
    double clock_bias = 0.0;
    double clock_drift = 0.0;
    ASSERT_TRUE(eph.calculateSatelliteState(
        eval_time, position, velocity, clock_bias, clock_drift));

    const Vector3d expected =
        expectedBeiDouGeoPosition(eph, eval_time, true);
    const Vector3d non_geo_omega =
        expectedBeiDouGeoPosition(eph, eval_time, false);
    EXPECT_LT((position - expected).norm(), 1e-3);
    EXPECT_GT((position - non_geo_omega).norm(), 100000.0);
}

TEST(IonosphereModelTest, KlobucharUsesElevationInSemicircles) {
    const double latitude = 35.0 * M_PI / 180.0;
    const double longitude = 139.0 * M_PI / 180.0;
    const double azimuth = 120.0 * M_PI / 180.0;
    const double elevation = 30.0 * M_PI / 180.0;
    const double tow = 16000.0;

    const double delay_m = models::ionoDelayKlobuchar(
        latitude, longitude, azimuth, elevation, tow, nullptr, nullptr);

    // Standard Klobuchar broadcast-model calculation with elevation expressed
    // in semicircles. A previous rad/pi/pi conversion inflated this case to
    // about 12.04 m.
    EXPECT_NEAR(delay_m, 7.691021650363509, 1e-9);
}

TEST(SPPUtilsTest, ResidualInlierMaskRejectsLargeSingleOutlier) {
    const auto mask = spp_utils::calculateResidualInlierMask(
        {0.2, -0.1, 0.0, 0.3, -0.2, 35.0}, 3.0, 1.0);

    ASSERT_EQ(mask.size(), 6U);
    EXPECT_TRUE(mask[0]);
    EXPECT_TRUE(mask[1]);
    EXPECT_TRUE(mask[2]);
    EXPECT_TRUE(mask[3]);
    EXPECT_TRUE(mask[4]);
    EXPECT_FALSE(mask[5]);
}

TEST(SPPUtilsTest, ResidualInlierMaskDropsNonFiniteResidualWhenRedundant) {
    const auto mask = spp_utils::calculateResidualInlierMask(
        {0.0, 0.2, -0.1, 0.1, -0.2, std::numeric_limits<double>::quiet_NaN()},
        3.0,
        1.0);

    ASSERT_EQ(mask.size(), 6U);
    EXPECT_TRUE(mask[0]);
    EXPECT_TRUE(mask[1]);
    EXPECT_TRUE(mask[2]);
    EXPECT_TRUE(mask[3]);
    EXPECT_TRUE(mask[4]);
    EXPECT_FALSE(mask[5]);
}

TEST(SPPUtilsTest, SelectRaimFdeCandidateRequiresMeaningfulImprovement) {
    EXPECT_EQ(spp_utils::selectRaimFdeCandidate(
                  12.0, {11.5, 11.0, 3.0, 10.0}, 0.25, 1.0),
              2);
    EXPECT_EQ(spp_utils::selectRaimFdeCandidate(
                  12.0, {11.5, 11.0, 10.5}, 0.25, 1.0),
              -1);
    EXPECT_EQ(spp_utils::selectRaimFdeCandidate(
                  0.0, {1.0, 2.0}, 0.25, 1.0),
              -1);
}

TEST(SPPUtilsTest, PseudorangeVariancePenalizesLowElevationAndWeakSnr) {
    spp_utils::MeasurementVarianceInputs strong_high;
    strong_high.elevation_rad = 60.0 * M_PI / 180.0;
    strong_high.snr_dbhz = 45.0;
    strong_high.pseudorange_sigma_m = 3.0;
    strong_high.ionosphere_delay_m = 3.0;
    strong_high.troposphere_delay_m = 2.3;
    strong_high.ionosphere_corrected = true;
    strong_high.troposphere_corrected = true;

    auto weak_low = strong_high;
    weak_low.elevation_rad = 10.0 * M_PI / 180.0;
    weak_low.snr_dbhz = 30.0;

    const double high_variance = spp_utils::calculatePseudorangeVariance(strong_high);
    const double low_variance = spp_utils::calculatePseudorangeVariance(weak_low);

    EXPECT_TRUE(std::isfinite(high_variance));
    EXPECT_TRUE(std::isfinite(low_variance));
    EXPECT_GT(low_variance, high_variance);
}

TEST(SPPUtilsTest, PseudorangeVariancePenalizesUnmodeledAtmosphere) {
    spp_utils::MeasurementVarianceInputs corrected;
    corrected.elevation_rad = 45.0 * M_PI / 180.0;
    corrected.snr_dbhz = 45.0;
    corrected.pseudorange_sigma_m = 3.0;
    corrected.ionosphere_delay_m = 2.0;
    corrected.troposphere_delay_m = 2.3;
    corrected.ionosphere_corrected = true;
    corrected.troposphere_corrected = true;

    auto unmodeled = corrected;
    unmodeled.ionosphere_corrected = false;
    unmodeled.troposphere_corrected = false;

    EXPECT_GT(spp_utils::calculatePseudorangeVariance(unmodeled),
              spp_utils::calculatePseudorangeVariance(corrected));
}

TEST(SPPUtilsTest, IonosphereFreeCoefficientsMatchGpsL1L2) {
    const auto coefficients = spp_utils::ionosphereFreeCoefficients(
        constants::GPS_L1_FREQ, constants::GPS_L2_FREQ);

    EXPECT_NEAR(coefficients.first, 2.54572778016316, 1e-12);
    EXPECT_NEAR(coefficients.second, -1.5457277801631601, 1e-12);
    EXPECT_NEAR(coefficients.first + coefficients.second, 1.0, 1e-12);
}

TEST(SPPUtilsTest, IonosphereFreePseudorangeCancelsFirstOrderIonosphere) {
    const double true_range = 20200000.0;
    const double ionosphere_l1 = 5.0;
    const double f1 = constants::GPS_L1_FREQ;
    const double f2 = constants::GPS_L2_FREQ;
    const double ionosphere_l2 = ionosphere_l1 * (f1 / f2) * (f1 / f2);

    const double p1 = true_range + ionosphere_l1;
    const double p2 = true_range + ionosphere_l2;
    const double iflc = spp_utils::calculateIonosphereFreePseudorange(p1, p2, f1, f2);

    EXPECT_NEAR(iflc, true_range, 1e-8);
}

TEST(SPPProductTest, ProcessorLoadsIonexAndDcbProducts) {
    const auto ionex_path = tempFilePath("libgnss_spp_ionex_test.ionex");
    const auto dcb_path = tempFilePath("libgnss_spp_dcb_test.bsx");
    std::filesystem::remove(ionex_path);
    std::filesystem::remove(dcb_path);

    const std::string ionex_text =
        "     1.0           I                   G                   IONEX VERSION / TYPE\n"
        "  3600                                                      INTERVAL\n"
        "    -1                                                      EXPONENT\n"
        "    0.0    0.0    0.0                                       LAT1 / LAT2 / DLAT\n"
        "    0.0   10.0   10.0                                       LON1 / LON2 / DLON\n"
        "  450.0  450.0    0.0                                       HGT1 / HGT2 / DHGT\n"
        "                                                            END OF HEADER\n"
        "    1                                                      START OF TEC MAP\n"
        " 2026     3    26     0     0     0                        EPOCH OF CURRENT MAP\n"
        "    0.0    0.0   10.0   10.0  450.0                        LAT/LON1/LON2/DLON/H\n"
        "  100 200\n"
        "                                                            END OF TEC MAP\n";
    writeTextFile(ionex_path, ionex_text);

    const std::string dcb_text =
        "%=BIA 1.00 TEST TEST 2024:002:00000 TEST\n"
        "+BIAS/SOLUTION\n"
        "*BIAS SVN PRN STATION OBS1 OBS2 BEGIN END UNIT EST STDDEV\n"
        " OSB G01 C1C C2W 2024:002:00000 2024:003:00000 ns 0.250 0.050\n"
        " OSB G01 C2W C1C 2024:002:00000 2024:003:00000 m 0.125 0.020\n"
        "-BIAS/SOLUTION\n";
    writeTextFile(dcb_path, dcb_text);

    SPPProcessor processor;
    EXPECT_FALSE(processor.hasLoadedIONEXProducts());
    EXPECT_FALSE(processor.hasLoadedDCBProducts());

    ASSERT_TRUE(processor.loadIONEXProducts(ionex_path.string()));
    ASSERT_TRUE(processor.loadDCBProducts(dcb_path.string()));
    EXPECT_TRUE(processor.hasLoadedIONEXProducts());
    EXPECT_TRUE(processor.hasLoadedDCBProducts());
    EXPECT_EQ(processor.getLoadedIONEXMapCount(), 1U);
    EXPECT_EQ(processor.getLoadedDCBEntryCount(), 2U);
    EXPECT_EQ(processor.getLastAppliedIonexCorrections(), 0);
    EXPECT_EQ(processor.getLastAppliedDcbCorrections(), 0);

    std::filesystem::remove(ionex_path);
    std::filesystem::remove(dcb_path);
}

TEST(SPPProductTest, ProcessorLoadsPreciseProducts) {
    const auto sp3_path = tempFilePath("libgnss_spp_precise_test.sp3");
    const auto clk_path = tempFilePath("libgnss_spp_precise_test.clk");
    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);

    const std::string sp3_text =
        "*  2026 03 26 00 00 00.00000000\n"
        "PG01   20200.000000   14000.000000   21700.000000       123.000000\n"
        "*  2026 03 26 00 15 00.00000000\n"
        "PG01   20201.500000   14001.500000   21701.500000       223.000000\n";
    const std::string clk_text =
        "     3.00           C                   RINEX VERSION / TYPE\n"
        "END OF HEADER\n"
        "AS G01 2026 03 26 00 00 00.00000000  2  1.230000000000E-04  1.000000000000E-12\n"
        "AS G01 2026 03 26 00 15 00.00000000  2  2.230000000000E-04  1.000000000000E-12\n";
    writeTextFile(sp3_path, sp3_text);
    writeTextFile(clk_path, clk_text);

    SPPProcessor processor;
    EXPECT_FALSE(processor.hasLoadedPreciseProducts());

    ASSERT_TRUE(processor.loadPreciseProducts(sp3_path.string(), clk_path.string()));
    EXPECT_TRUE(processor.hasLoadedPreciseProducts());
    EXPECT_EQ(processor.getLoadedPreciseSatelliteCount(), 1U);
    EXPECT_EQ(processor.getLastAppliedPreciseOrbitClockMeasurements(), 0);

    std::filesystem::remove(sp3_path);
    std::filesystem::remove(clk_path);
}

TEST(SPPProductTest, ProcessorLoadsSSRProducts) {
    const auto ssr_path = tempFilePath("libgnss_spp_ssr_test.csv");
    std::filesystem::remove(ssr_path);

    const std::string ssr_text =
        "# week,tow,sat,dx,dy,dz,dclock_m\n"
        "2414,345600.0,G01,0.10,-0.20,0.30,0.40,cbias:2=-0.12\n"
        "2414,345660.0,G01,0.20,-0.30,0.40,0.50,cbias:2=-0.13\n";
    writeTextFile(ssr_path, ssr_text);

    SPPProcessor processor;
    EXPECT_FALSE(processor.hasLoadedSSRProducts());

    ASSERT_TRUE(processor.loadSSRProducts(ssr_path.string()));
    EXPECT_TRUE(processor.hasLoadedSSRProducts());
    EXPECT_EQ(processor.getLoadedSSRSatelliteCount(), 1U);
    EXPECT_EQ(processor.getLastAppliedSSROrbitClockCorrections(), 0);
    EXPECT_EQ(processor.getLastAppliedSSRCodeBiasCorrections(), 0);

    std::filesystem::remove(ssr_path);
}

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
    for (const auto& sat : solution.satellites_used) {
        used_systems.insert(sat.system);
    }
    EXPECT_TRUE(used_systems.count(GNSSSystem::GPS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::GLONASS));
    EXPECT_TRUE(used_systems.count(GNSSSystem::BeiDou));
    EXPECT_TRUE(used_systems.count(GNSSSystem::QZSS));
    EXPECT_GE(used_systems.size(), 4U);
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
