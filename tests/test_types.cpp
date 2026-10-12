#include <gtest/gtest.h>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/core/types.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

using namespace libgnss;

class TypesTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TypesTest, GNSSTimeBasicOperations) {
    GNSSTime time1(2000, 345600.0);  // GPS week 2000, TOW 345600
    GNSSTime time2(2000, 345700.0);  // 100 seconds later
    
    EXPECT_EQ(time1.week, 2000);
    EXPECT_DOUBLE_EQ(time1.tow, 345600.0);
    
    // Test time difference
    double diff = time2 - time1;
    EXPECT_DOUBLE_EQ(diff, 100.0);
    
    // Test comparison operators
    EXPECT_TRUE(time1 < time2);
    EXPECT_FALSE(time1 == time2);
}

TEST_F(TypesTest, SatelliteIdOperations) {
    SatelliteId gps_sat(GNSSSystem::GPS, 1);
    SatelliteId glo_sat(GNSSSystem::GLONASS, 1);
    SatelliteId gps_sat2(GNSSSystem::GPS, 2);
    
    EXPECT_EQ(gps_sat.system, GNSSSystem::GPS);
    EXPECT_EQ(gps_sat.prn, 1);
    
    // Test comparison operators
    EXPECT_TRUE(gps_sat < glo_sat);  // GPS < GLONASS
    EXPECT_TRUE(gps_sat < gps_sat2); // PRN 1 < PRN 2
    EXPECT_FALSE(gps_sat == glo_sat);
    
    // Test string conversion
    std::string sat_str = gps_sat.toString();
    EXPECT_FALSE(sat_str.empty());
}

TEST_F(TypesTest, CoordinateConversions) {
    // Test ECEF coordinates
    ECEFCoord ecef(4000000.0, 3000000.0, 2000000.0);
    Vector3d vec = ecef.toVector();
    
    EXPECT_DOUBLE_EQ(vec(0), 4000000.0);
    EXPECT_DOUBLE_EQ(vec(1), 3000000.0);
    EXPECT_DOUBLE_EQ(vec(2), 2000000.0);
    
    ECEFCoord ecef2;
    ecef2.fromVector(vec);
    EXPECT_DOUBLE_EQ(ecef2.x, ecef.x);
    EXPECT_DOUBLE_EQ(ecef2.y, ecef.y);
    EXPECT_DOUBLE_EQ(ecef2.z, ecef.z);
}

TEST_F(TypesTest, GeodeticCoordinates) {
    GeodeticCoord geo(0.7854, 1.5708, 100.0);  // 45°, 90°, 100m
    
    EXPECT_DOUBLE_EQ(geo.latitude, 0.7854);
    EXPECT_DOUBLE_EQ(geo.longitude, 1.5708);
    EXPECT_DOUBLE_EQ(geo.height, 100.0);
}

TEST_F(TypesTest, Constants) {
    EXPECT_DOUBLE_EQ(constants::SPEED_OF_LIGHT, 299792458.0);
    EXPECT_GT(constants::GPS_L1_FREQ, 1.5e9);
    EXPECT_GT(constants::WGS84_A, 6.3e6);
    EXPECT_GT(constants::WGS84_F, 0.0);
    EXPECT_LT(constants::WGS84_F, 1.0);
}

TEST_F(TypesTest, PropagatedSolutionIsValidWithoutCurrentSatellitesButNeverFixed) {
    PositionSolution solution;
    solution.status = SolutionStatus::PROPAGATED;
    solution.position_ecef = Vector3d(1.0, 2.0, 3.0);
    solution.num_satellites = 0;

    EXPECT_TRUE(solution.isValid());
    EXPECT_FALSE(solution.isFixed());

    solution.position_ecef.x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(solution.isValid());
}

TEST_F(TypesTest, DefaultPositionSolutionVectorsAreVisiblyInvalid) {
    // Eigen does not value-initialize; an unassigned position/velocity must be
    // NaN (never stack garbage) so failed solutions are deterministic and
    // cannot masquerade as finite coordinates.
    const PositionSolution solution;
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isnan(solution.position_ecef(i)));
        EXPECT_TRUE(std::isnan(solution.velocity_ecef(i)));
        EXPECT_TRUE(std::isnan(solution.velocity_ned(i)));
    }
    EXPECT_FALSE(solution.has_velocity);
    EXPECT_FALSE(solution.isValid());

    PositionSolution propagated;
    propagated.status = SolutionStatus::PROPAGATED;
    EXPECT_FALSE(propagated.isValid());
    propagated.position_ecef = Vector3d(1.0, 2.0, 3.0);
    EXPECT_TRUE(propagated.isValid());
}

TEST_F(TypesTest, PositionSolutionCovariancesDefaultToInvalid) {
    // Value-initialize: const default initialization is ill-formed without a
    // user-provided default constructor (strict clang/macOS builds).
    const PositionSolution solution{};

    // An omitted covariance must be rejected by consumers rather than being
    // interpreted as whatever coefficients Eigen happened to leave behind.
    EXPECT_FALSE(solution.position_covariance.allFinite());
    EXPECT_FALSE(solution.velocity_covariance.allFinite());
}

TEST_F(TypesTest, SolutionFileRoundTripsFgoQualityColumns) {
    Solution solution;
    PositionSolution sol;
    sol.time = GNSSTime(2300, 12345.0);
    sol.status = SolutionStatus::FIXED;
    sol.position_ecef = Vector3d(1.0, 2.0, 3.0);
    sol.position_geodetic = GeodeticCoord(0.1, 0.2, 3.5);
    sol.num_satellites = 12;
    sol.pdop = 1.25;
    sol.ratio = 4.5;
    sol.num_fixed_ambiguities = 9;
    sol.iterations = 7;
    solution.addSolution(sol);

    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_roundtrip_test.pos";
    std::filesystem::remove(path);

    ASSERT_TRUE(solution.writeToFile(path.string()));

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 1u);
    const PositionSolution& loaded_sol = loaded.solutions.front();
    EXPECT_DOUBLE_EQ(loaded_sol.ratio, sol.ratio);
    EXPECT_EQ(loaded_sol.num_fixed_ambiguities, sol.num_fixed_ambiguities);
    EXPECT_EQ(loaded_sol.iterations, sol.iterations);
}

TEST_F(TypesTest, SolutionLoaderMapsNamedOptionalColumns) {
    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_named_optional_columns_test.pos";
    std::filesystem::remove(path);

    {
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open());
        file << "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
                "Status NumSat PDOP Ratio Baseline(m) RTKIter RTKObs RTKPhaseObs "
                "RTKCodeObs RTKOutliers RTKPrefitRMS(m) RTKPrefitMax(m) "
                "RTKPostSuppressRMS(m) RTKPostSuppressMax(m) RTKUpdateNIS "
                "RTKUpdateNISPerObs RTKUpdateNISRejected\n";
        file << "2300 12346.000 1 2 3 5 6 7 4 10 1.2 8.5 12.25 4 "
                "20 11 9 3 0.5 2.0 0.25 1.5 7.0 0.35 1\n";
    }

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 1u);
    const PositionSolution& sol = loaded.solutions.front();
    EXPECT_DOUBLE_EQ(sol.ratio, 8.5);
    EXPECT_DOUBLE_EQ(sol.baseline_length, 12.25);
    EXPECT_EQ(sol.iterations, 4);
    EXPECT_EQ(sol.num_fixed_ambiguities, 0);
    EXPECT_EQ(sol.rtk_update_observations, 20);
    EXPECT_EQ(sol.rtk_update_phase_observations, 11);
    EXPECT_EQ(sol.rtk_update_code_observations, 9);
    EXPECT_EQ(sol.rtk_update_suppressed_outliers, 3);
    EXPECT_DOUBLE_EQ(sol.rtk_update_prefit_residual_rms_m, 0.5);
    EXPECT_DOUBLE_EQ(sol.rtk_update_prefit_residual_max_m, 2.0);
    EXPECT_DOUBLE_EQ(sol.rtk_update_post_suppression_residual_rms_m, 0.25);
    EXPECT_DOUBLE_EQ(sol.rtk_update_post_suppression_residual_max_m, 1.5);
    EXPECT_DOUBLE_EQ(sol.rtk_update_normalized_innovation_squared, 7.0);
    EXPECT_DOUBLE_EQ(sol.rtk_update_normalized_innovation_squared_per_observation, 0.35);
    EXPECT_EQ(sol.rtk_update_rejected_by_innovation_gate, 1);
}

TEST_F(TypesTest, SolutionLoaderParsesRtklibPositionVelocityAndCovariance) {
    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_rtklib_input_test.pos";
    std::filesystem::remove(path);

    {
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open());
        file << "%  GPST latitude(deg) longitude(deg) height(m) Q ns "
                "sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) age(s) ratio "
                "vn(m/s) ve(m/s) vu(m/s) sdvn sdve sdvu sdvne sdveu sdvun\n";
        file << "2025/07/08 19:34:18.499 40.0 -105.0 1600.0 1 21 "
                "0.01 0.02 0.03 -0.004 0.005 -0.006 0.0 4.5 "
                "1.0 2.0 3.0 0.1 0.2 0.3 -0.04 0.05 -0.06\n";
    }

    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 1u);
    const PositionSolution& sol = loaded.solutions.front();
    EXPECT_EQ(sol.time.week, 2374);
    EXPECT_NEAR(sol.time.tow, 243258.499, 1e-6);
    EXPECT_EQ(sol.status, SolutionStatus::FIXED);
    EXPECT_EQ(sol.num_satellites, 21);
    EXPECT_DOUBLE_EQ(sol.ratio, 4.5);
    EXPECT_NEAR(sol.position_geodetic.latitude, 40.0 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(sol.position_geodetic.longitude, -105.0 * M_PI / 180.0, 1e-12);
    EXPECT_TRUE(sol.position_covariance.allFinite());
    EXPECT_NEAR(sol.position_covariance.trace(), 0.01 * 0.01 + 0.02 * 0.02 +
                                                     0.03 * 0.03,
                1e-12);
    ASSERT_TRUE(sol.has_velocity);
    EXPECT_NEAR(sol.velocity_ned.x(), 1.0, 1e-12);
    EXPECT_NEAR(sol.velocity_ned.y(), 2.0, 1e-12);
    EXPECT_NEAR(sol.velocity_ned.z(), -3.0, 1e-12);
    EXPECT_TRUE(sol.velocity_ecef.allFinite());
    EXPECT_NEAR(sol.velocity_ecef.norm(), std::sqrt(14.0), 1e-12);
    EXPECT_NEAR(sol.velocity_covariance.trace(), 0.1 * 0.1 + 0.2 * 0.2 +
                                                     0.3 * 0.3,
                1e-12);
}

namespace {

Matrix3d ecefToEnuTestRotation(double lat, double lon) {
    Matrix3d rotation;
    rotation << -std::sin(lon), std::cos(lon), 0.0,
        -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
        std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat);
    return rotation;
}

// Solution at lat 35 deg / lon 139 deg whose local ENU covariance is known:
// sdn = 0.01, sde = 0.02, sdu = 0.03 m, cov(E,N) = -4.9e-5 (sdne = -0.007).
PositionSolution makeSolutionWithEnuCovariance() {
    PositionSolution sol;
    sol.time = GNSSTime(2300, 12345.0);
    sol.status = SolutionStatus::FIXED;
    const double lat = 35.0 * M_PI / 180.0;
    const double lon = 139.0 * M_PI / 180.0;
    sol.position_geodetic = GeodeticCoord(lat, lon, 40.0);
    sol.position_ecef = geodetic2ecef(lat, lon, 40.0);
    sol.num_satellites = 12;
    sol.pdop = 1.5;
    sol.ratio = 4.5;
    const Matrix3d rotation = ecefToEnuTestRotation(lat, lon);
    Matrix3d enu;
    enu << 0.02 * 0.02, -4.9e-5, 0.0,
        -4.9e-5, 0.01 * 0.01, 0.0,
        0.0, 0.0, 0.03 * 0.03;
    sol.position_covariance = rotation.transpose() * enu * rotation;
    return sol;
}

std::vector<std::string> splitTokens(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) tokens.push_back(token);
    return tokens;
}

std::vector<std::string> readLines(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) lines.push_back(line);
    return lines;
}

}  // namespace

TEST_F(TypesTest, SolutionWriterDefaultOutputHasNoUncertaintyColumns) {
    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_default_test.pos";
    {
        io::SolutionWriter writer;
        ASSERT_TRUE(writer.open(path.string()));
        writer.writeEpoch(makeSolutionWithEnuCovariance());
        // An age argument must not matter when the option is off.
        writer.writeEpoch(makeSolutionWithEnuCovariance(), 1.0);
    }
    const auto lines = readLines(path);
    std::filesystem::remove(path);
    ASSERT_EQ(lines.size(), 4u);
    EXPECT_EQ(lines[1].substr(lines[1].size() - 20), "RTKUpdateNISRejected");
    EXPECT_EQ(lines[1].find("sdn(m)"), std::string::npos);
    EXPECT_EQ(splitTokens(lines[2]).size(), 25u);
    EXPECT_EQ(lines[2], lines[3]);
}

TEST_F(TypesTest, SolutionWriterAppendsUncertaintyColumnsAndKeepsLegacyColumns) {
    const std::filesystem::path off_path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_unc_off_test.pos";
    const std::filesystem::path on_path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_unc_on_test.pos";
    const PositionSolution sol = makeSolutionWithEnuCovariance();
    {
        io::SolutionWriter off;
        ASSERT_TRUE(off.open(off_path.string()));
        off.writeEpoch(sol);
        io::SolutionWriter on;
        ASSERT_TRUE(on.open(on_path.string(), io::SolutionWriter::Format::POS, true));
        on.writeEpoch(sol, 0.25);
    }
    const auto off_lines = readLines(off_path);
    const auto on_lines = readLines(on_path);
    std::filesystem::remove(off_path);
    std::filesystem::remove(on_path);

    ASSERT_EQ(on_lines.size(), 3u);
    const auto header = splitTokens(on_lines[1].substr(1));
    ASSERT_EQ(header.size(), 25u + 7u);
    const std::vector<std::string> expected_names = {
        "sdn(m)", "sde(m)", "sdu(m)", "sdne(m)", "sdeu(m)", "sdun(m)", "DiffAge(s)"};
    for (size_t i = 0; i < expected_names.size(); ++i) {
        EXPECT_EQ(header[25 + i], expected_names[i]);
    }

    // Everything before the appended columns is byte-identical to the legacy row.
    EXPECT_EQ(on_lines[2].compare(0, off_lines[2].size(), off_lines[2]), 0);
    const auto row = splitTokens(on_lines[2]);
    ASSERT_EQ(row.size(), 25u + 7u);
    EXPECT_NEAR(std::stod(row[25]), 0.01, 1e-5);    // sdn
    EXPECT_NEAR(std::stod(row[26]), 0.02, 1e-5);    // sde
    EXPECT_NEAR(std::stod(row[27]), 0.03, 1e-5);    // sdu
    EXPECT_NEAR(std::stod(row[28]), -std::sqrt(4.9e-5), 1e-5);  // sdne (signed sqrt of cov)
    EXPECT_NEAR(std::stod(row[29]), 0.0, 1e-5);     // sdeu
    EXPECT_NEAR(std::stod(row[30]), 0.0, 1e-5);     // sdun
    EXPECT_NEAR(std::stod(row[31]), 0.25, 1e-9);    // DiffAge(s)
}

TEST_F(TypesTest, SolutionWriterUncertaintyColumnsAreNanWhenUnavailable) {
    PositionSolution sol = makeSolutionWithEnuCovariance();
    sol.position_covariance =
        Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const auto tokens = splitTokens(io::SolutionWriter::formatUncertaintyColumns(
        sol, std::numeric_limits<double>::quiet_NaN()));
    ASSERT_EQ(tokens.size(), 7u);
    for (const auto& token : tokens) EXPECT_EQ(token, "nan");

    // Negative variance on one axis is "nan" for that axis only.
    PositionSolution partial = makeSolutionWithEnuCovariance();
    Matrix3d bad = Matrix3d::Identity() * 0.0001;
    bad(2, 2) = -1.0;  // negative Up variance in the local frame
    const Matrix3d rotation = ecefToEnuTestRotation(35.0 * M_PI / 180.0, 139.0 * M_PI / 180.0);
    partial.position_covariance = rotation.transpose() * bad * rotation;
    const auto partial_tokens = splitTokens(io::SolutionWriter::formatUncertaintyColumns(partial, 2.0));
    ASSERT_EQ(partial_tokens.size(), 7u);
    EXPECT_EQ(partial_tokens[2], "nan");
    EXPECT_NE(partial_tokens[0], "nan");
    EXPECT_EQ(partial_tokens[6], "2.000");
}

TEST_F(TypesTest, SolutionWriterFixDiagnosticColumnsAreOptInAndAppendedLast) {
    const std::filesystem::path off_path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_fixdiag_off_test.pos";
    const std::filesystem::path diag_path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_fixdiag_on_test.pos";
    const std::filesystem::path both_path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_fixdiag_both_test.pos";
    // Float reference 3 m east, 4 m north and 1.5 m below the fixed position.
    PositionSolution fixed = makeSolutionWithEnuCovariance();
    const double lat = 35.0 * M_PI / 180.0;
    const double lon = 139.0 * M_PI / 180.0;
    const Matrix3d rotation = ecefToEnuTestRotation(lat, lon);
    fixed.rtk_float_reference_valid = true;
    fixed.rtk_float_position_ecef =
        fixed.position_ecef + rotation.transpose() * Vector3d(3.0, 4.0, -1.5);
    PositionSolution float_epoch = fixed;
    float_epoch.status = SolutionStatus::FLOAT;  // reference present but not FIXED
    PositionSolution no_reference = makeSolutionWithEnuCovariance();  // FIXED, no record
    {
        io::SolutionWriter off;
        ASSERT_TRUE(off.open(off_path.string()));
        io::SolutionWriter diag;
        ASSERT_TRUE(diag.open(diag_path.string(), io::SolutionWriter::Format::POS, false, true));
        io::SolutionWriter both;
        ASSERT_TRUE(both.open(both_path.string(), io::SolutionWriter::Format::POS, true, true));
        for (const auto* sol : {&fixed, &float_epoch, &no_reference}) {
            off.writeEpoch(*sol);
            diag.writeEpoch(*sol);
            both.writeEpoch(*sol, 0.25);
        }
    }
    const auto off_lines = readLines(off_path);
    const auto diag_lines = readLines(diag_path);
    const auto both_lines = readLines(both_path);
    std::filesystem::remove(off_path);
    std::filesystem::remove(diag_path);
    std::filesystem::remove(both_path);

    ASSERT_EQ(diag_lines.size(), 5u);
    ASSERT_EQ(both_lines.size(), 5u);
    const auto header = splitTokens(diag_lines[1].substr(1));
    ASSERT_EQ(header.size(), 25u + 2u);
    EXPECT_EQ(header[25], "FloatFixH(m)");
    EXPECT_EQ(header[26], "FloatFixU(m)");
    const auto both_header = splitTokens(both_lines[1].substr(1));
    ASSERT_EQ(both_header.size(), 25u + 7u + 2u);
    EXPECT_EQ(both_header[31], "DiffAge(s)");
    EXPECT_EQ(both_header[32], "FloatFixH(m)");
    EXPECT_EQ(both_header[33], "FloatFixU(m)");

    // Legacy columns are byte-identical; the new columns come last.
    for (size_t i = 2; i < 5; ++i) {
        EXPECT_EQ(diag_lines[i].compare(0, off_lines[i].size(), off_lines[i]), 0);
        EXPECT_EQ(both_lines[i].compare(0, off_lines[i].size(), off_lines[i]), 0);
    }
    const auto fixed_row = splitTokens(diag_lines[2]);
    ASSERT_EQ(fixed_row.size(), 27u);
    EXPECT_NEAR(std::stod(fixed_row[25]), 5.0, 1e-4);
    EXPECT_NEAR(std::stod(fixed_row[26]), 1.5, 1e-4);
    const auto both_row = splitTokens(both_lines[2]);
    ASSERT_EQ(both_row.size(), 34u);
    EXPECT_EQ(both_row[32], fixed_row[25]);
    EXPECT_EQ(both_row[33], fixed_row[26]);
    for (size_t i = 3; i < 5; ++i) {
        const auto row = splitTokens(diag_lines[i]);
        ASSERT_EQ(row.size(), 27u);
        EXPECT_EQ(row[25], "nan");
        EXPECT_EQ(row[26], "nan");
    }

    // The loader tolerates the trailing columns.
    Solution loaded;
    const std::filesystem::path roundtrip =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_fixdiag_roundtrip_test.pos";
    {
        io::SolutionWriter writer;
        ASSERT_TRUE(writer.open(roundtrip.string(), io::SolutionWriter::Format::POS, true, true));
        writer.writeEpoch(fixed, 0.0);
        writer.writeEpoch(float_epoch);
    }
    ASSERT_TRUE(loaded.loadFromFile(roundtrip.string()));
    std::filesystem::remove(roundtrip);
    ASSERT_EQ(loaded.solutions.size(), 2u);
    EXPECT_DOUBLE_EQ(loaded.solutions[0].ratio, 4.5);
    EXPECT_NEAR(loaded.solutions[0].positionCovarianceEnu()(0, 0), 0.02 * 0.02, 1e-8);
}

TEST_F(TypesTest, SolutionLoaderRoundTripsUncertaintyColumnsAndToleratesNan) {
    const std::filesystem::path path =
        std::filesystem::path(GNSSPP_BINARY_DIR) / "solution_writer_unc_roundtrip_test.pos";
    PositionSolution good = makeSolutionWithEnuCovariance();
    PositionSolution unavailable = makeSolutionWithEnuCovariance();
    unavailable.time = GNSSTime(2300, 12346.0);
    unavailable.position_covariance =
        Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
    {
        io::SolutionWriter writer;
        ASSERT_TRUE(writer.open(path.string(), io::SolutionWriter::Format::POS, true));
        writer.writeEpoch(good, 0.0);
        writer.writeEpoch(unavailable);
    }
    Solution loaded;
    ASSERT_TRUE(loaded.loadFromFile(path.string()));
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.solutions.size(), 2u);
    EXPECT_DOUBLE_EQ(loaded.solutions[0].ratio, 4.5);
    const Matrix3d enu = loaded.solutions[0].positionCovarianceEnu();
    ASSERT_TRUE(enu.allFinite());
    EXPECT_NEAR(enu(0, 0), 0.02 * 0.02, 1e-8);
    EXPECT_NEAR(enu(1, 1), 0.01 * 0.01, 1e-8);
    EXPECT_NEAR(enu(2, 2), 0.03 * 0.03, 1e-8);
    EXPECT_NEAR(enu(0, 1), -4.9e-5, 1e-8);
    // NaN columns neither abort the row parse nor invent a covariance.
    EXPECT_DOUBLE_EQ(loaded.solutions[1].ratio, 4.5);
    EXPECT_FALSE(loaded.solutions[1].position_covariance.allFinite());
}
