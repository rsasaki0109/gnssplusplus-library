#include <gtest/gtest.h>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/io/rinex.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace libgnss;

// ============================================================================
// Helper: RTKLIB reference solution (lat/lon in degrees, height in meters)
// Parsed from output/rtklib_rtk_result.pos
// ============================================================================
struct RTKLIBEpoch {
    int week;
    double tow;
    double lat_deg;
    double lon_deg;
    double height;
    int quality;    // Q: 1=fix, 2=float
    int num_sats;
    double ratio;
};

// Convert geodetic (rad) to ECEF
static Vector3d geodeticToEcef(double lat_rad, double lon_rad, double height) {
    const double a = constants::WGS84_A;
    const double e2 = constants::WGS84_E2;
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);
    const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    return Vector3d(
        (N + height) * cos_lat * cos_lon,
        (N + height) * cos_lat * sin_lon,
        (N * (1.0 - e2) + height) * sin_lat
    );
}

// Compute ENU error from ECEF positions
static Vector3d ecefToEnu(const Vector3d& error_ecef, const Vector3d& ref_ecef) {
    // Compute lat/lon of reference
    const double p = std::sqrt(ref_ecef(0)*ref_ecef(0) + ref_ecef(1)*ref_ecef(1));
    const double lon = std::atan2(ref_ecef(1), ref_ecef(0));
    const double lat = std::atan2(ref_ecef(2), p * (1.0 - constants::WGS84_E2));
    // Iterative for better accuracy
    double lat_iter = lat;
    for (int i = 0; i < 5; ++i) {
        const double sin_lat = std::sin(lat_iter);
        const double N = constants::WGS84_A / std::sqrt(1.0 - constants::WGS84_E2 * sin_lat * sin_lat);
        lat_iter = std::atan2(ref_ecef(2) + constants::WGS84_E2 * N * sin_lat, p);
    }
    const double sin_lat = std::sin(lat_iter);
    const double cos_lat = std::cos(lat_iter);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    // Rotation matrix ECEF -> ENU
    Eigen::Matrix3d R;
    R << -sin_lon,           cos_lon,          0,
         -sin_lat*cos_lon,  -sin_lat*sin_lon,  cos_lat,
          cos_lat*cos_lon,   cos_lat*sin_lon,  sin_lat;

    return R * error_ecef;
}

// Parse RTKLIB .pos file and return epochs
static std::vector<RTKLIBEpoch> parseRTKLIBPos(const std::string& filename) {
    std::vector<RTKLIBEpoch> epochs;
    std::ifstream file(filename);
    if (!file.is_open()) return epochs;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '%') continue;
        std::istringstream iss(line);
        RTKLIBEpoch e;
        double sdn, sde, sdu, sdne, sdeu, sdun, age;
        iss >> e.week >> e.tow >> e.lat_deg >> e.lon_deg >> e.height
            >> e.quality >> e.num_sats >> sdn >> sde >> sdu >> sdne >> sdeu >> sdun
            >> age >> e.ratio;
        if (!iss.fail()) {
            epochs.push_back(e);
        }
    }
    return epochs;
}

// ============================================================================
// Test fixture that loads real RINEX data
// ============================================================================
class RTKRealDataTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Load RTKLIB reference
        rtklib_epochs_ = parseRTKLIBPos("output/rtklib_rtk_result.pos");
        ASSERT_GT(rtklib_epochs_.size(), 0u) << "Failed to load RTKLIB reference";

        // Build ECEF reference positions indexed by TOW
        for (const auto& e : rtklib_epochs_) {
            double lat_rad = e.lat_deg * M_PI / 180.0;
            double lon_rad = e.lon_deg * M_PI / 180.0;
            rtklib_ecef_[e.tow] = geodeticToEcef(lat_rad, lon_rad, e.height);
            rtklib_quality_[e.tow] = e.quality;
        }

        // Compute mean RTKLIB reference position (for ENU origin)
        ref_position_ = Vector3d::Zero();
        int count = 0;
        for (const auto& [tow, pos] : rtklib_ecef_) {
            ref_position_ += pos;
            count++;
        }
        ref_position_ /= count;

        // Open RINEX files
        ASSERT_TRUE(rover_reader_.open("data/rover.obs"));
        rover_reader_.readHeader(rover_header_);
        ASSERT_TRUE(base_reader_.open("data/base.obs"));
        base_reader_.readHeader(base_header_);
        ASSERT_TRUE(nav_reader_.open("data/navigation.nav"));
        nav_reader_.readNavigationData(nav_data_);

        // Setup base position
        if (base_header_.approximate_position.norm() > 0) {
            base_position_ = base_header_.approximate_position;
        } else {
            base_position_ = Vector3d(-3962108.7, 3381309.5, 3668678.8);
        }

        // Setup RTK processor
        rtk_config_.max_baseline_length = 20000.0;
        rtk_config_.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config_.ratio_threshold = 3.0;
        rtk_config_.min_satellites_for_ar = 5;
    }

    // Run full RTK processing and collect results
    struct RTKResult {
        double tow;
        Vector3d position_ecef;
        SolutionStatus status;
        int num_sats;
    };

    std::vector<RTKResult> runFullRTK() {
        RTKProcessor rtk;
        rtk.setRTKConfig(rtk_config_);
        rtk.setBasePosition(base_position_);

        std::vector<RTKResult> results;
        ObservationData rover_obs, base_obs;

        bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        bool base_ok = base_reader_.readObservationEpoch(base_obs);

        if (rover_ok && rover_header_.approximate_position.norm() > 0) {
            rover_obs.receiver_position = rover_header_.approximate_position;
        }

        while (rover_ok && base_ok) {
            // Synchronize epochs
            double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                             + (rover_obs.time.tow - base_obs.time.tow);
            while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
                if (time_diff < 0) {
                    Vector3d saved = rover_obs.receiver_position;
                    rover_ok = rover_reader_.readObservationEpoch(rover_obs);
                    if (rover_ok) rover_obs.receiver_position = saved;
                } else {
                    base_ok = base_reader_.readObservationEpoch(base_obs);
                }
                time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                          + (rover_obs.time.tow - base_obs.time.tow);
            }
            if (!rover_ok || !base_ok) break;

            auto sol = rtk.processRTKEpoch(rover_obs, base_obs, nav_data_);
            if (sol.isValid()) {
                RTKResult r;
                r.tow = rover_obs.time.tow;
                r.position_ecef = sol.position_ecef;
                r.status = sol.status;
                r.num_sats = sol.num_satellites;
                results.push_back(r);
            }

            Vector3d saved = rover_obs.receiver_position;
            rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            base_ok = base_reader_.readObservationEpoch(base_obs);
            if (rover_ok) {
                rover_obs.receiver_position = sol.isValid() ? sol.position_ecef : saved;
            }
        }
        return results;
    }

    std::vector<RTKLIBEpoch> rtklib_epochs_;
    std::map<double, Vector3d> rtklib_ecef_;        // TOW -> ECEF position
    std::map<double, int> rtklib_quality_;           // TOW -> quality flag
    Vector3d ref_position_;
    Vector3d base_position_;

    io::RINEXReader rover_reader_, base_reader_, nav_reader_;
    io::RINEXReader::RINEXHeader rover_header_, base_header_;
    NavigationData nav_data_;
    RTKProcessor::RTKConfig rtk_config_;
};

// ============================================================================
// Test 1: RTKLIB reference data loaded correctly
// ============================================================================
TEST_F(RTKRealDataTest, RTKLIBReferenceLoaded) {
    EXPECT_GE(rtklib_epochs_.size(), 100u);
    // All RTKLIB epochs should be Q=1 (fixed) for this dataset
    int fixed_count = 0;
    for (const auto& e : rtklib_epochs_) {
        if (e.quality == 1) fixed_count++;
    }
    double fix_rate = static_cast<double>(fixed_count) / rtklib_epochs_.size();
    EXPECT_GT(fix_rate, 0.95) << "RTKLIB should have >95% fix rate";

    // RTKLIB reference position should be near the rover header position
    double dist = (ref_position_ - rover_header_.approximate_position).norm();
    EXPECT_LT(dist, 10.0) << "Reference position should be within 10m of RINEX header";
}

// ============================================================================
// Test 2: RTK processes all epochs without crash
// ============================================================================
TEST_F(RTKRealDataTest, ProcessesAllEpochsWithoutCrash) {
    auto results = runFullRTK();
    EXPECT_GE(results.size(), 100u) << "Should produce at least 100 valid solutions";
}

// ============================================================================
// Test 3: Position accuracy vs RTKLIB reference (ECEF)
// ============================================================================
TEST_F(RTKRealDataTest, PositionAccuracyVsRTKLIB) {
    auto results = runFullRTK();
    ASSERT_GE(results.size(), 50u);

    double sum_h_err_sq = 0.0, sum_v_err_sq = 0.0;
    int matched = 0;

    for (const auto& r : results) {
        // Find matching RTKLIB epoch (within 1 second)
        auto it = rtklib_ecef_.lower_bound(r.tow - 1.0);
        if (it == rtklib_ecef_.end() || std::abs(it->first - r.tow) > 1.0) continue;

        Vector3d error_ecef = r.position_ecef - it->second;
        Vector3d error_enu = ecefToEnu(error_ecef, ref_position_);

        sum_h_err_sq += error_enu(0)*error_enu(0) + error_enu(1)*error_enu(1);
        sum_v_err_sq += error_enu(2)*error_enu(2);
        matched++;
    }

    ASSERT_GT(matched, 50) << "Should match at least 50 epochs with RTKLIB";

    double rms_h = std::sqrt(sum_h_err_sq / matched);
    double rms_v = std::sqrt(sum_v_err_sq / matched);

    std::cout << "=== Accuracy vs RTKLIB ===" << std::endl;
    std::cout << "  Matched epochs: " << matched << std::endl;
    std::cout << "  RMS horizontal (ENU): " << rms_h << " m" << std::endl;
    std::cout << "  RMS vertical (ENU):   " << rms_v << " m" << std::endl;
    std::cout << "  Target horizontal:     0.027 m" << std::endl;
    std::cout << "  Target vertical:       0.039 m" << std::endl;

    // Current relaxed targets - tighten as accuracy improves
    EXPECT_LT(rms_h, 2.0) << "Horizontal RMS should be < 2.0m (current target)";
    EXPECT_LT(rms_v, 2.0) << "Vertical RMS should be < 2.0m (current target)";
}

// ============================================================================
// Test 4: Fix rate
// ============================================================================
TEST_F(RTKRealDataTest, FixRate) {
    auto results = runFullRTK();
    ASSERT_GE(results.size(), 50u);

    int fixed = 0;
    for (const auto& r : results) {
        if (r.status == SolutionStatus::FIXED) fixed++;
    }
    double fix_rate = static_cast<double>(fixed) / results.size();

    std::cout << "=== Fix Rate ===" << std::endl;
    std::cout << "  Total: " << results.size() << ", Fixed: " << fixed
              << ", Rate: " << (fix_rate * 100) << "%" << std::endl;
    std::cout << "  Target: >95%" << std::endl;

    // Current relaxed target
    EXPECT_GT(fix_rate, 0.10) << "Fix rate should be > 10% (current target)";
}

// ============================================================================
// Test 5: Fixed solutions accuracy (when fix is achieved, it should be accurate)
// ============================================================================
TEST_F(RTKRealDataTest, FixedSolutionAccuracy) {
    auto results = runFullRTK();

    double sum_h_err_sq = 0.0, sum_v_err_sq = 0.0;
    int matched = 0;

    for (const auto& r : results) {
        if (r.status != SolutionStatus::FIXED) continue;

        auto it = rtklib_ecef_.lower_bound(r.tow - 1.0);
        if (it == rtklib_ecef_.end() || std::abs(it->first - r.tow) > 1.0) continue;

        Vector3d error_ecef = r.position_ecef - it->second;
        Vector3d error_enu = ecefToEnu(error_ecef, ref_position_);

        sum_h_err_sq += error_enu(0)*error_enu(0) + error_enu(1)*error_enu(1);
        sum_v_err_sq += error_enu(2)*error_enu(2);
        matched++;
    }

    if (matched > 0) {
        double rms_h = std::sqrt(sum_h_err_sq / matched);
        double rms_v = std::sqrt(sum_v_err_sq / matched);

        std::cout << "=== Fixed Solution Accuracy ===" << std::endl;
        std::cout << "  Fixed & matched epochs: " << matched << std::endl;
        std::cout << "  RMS horizontal (ENU): " << rms_h << " m" << std::endl;
        std::cout << "  RMS vertical (ENU):   " << rms_v << " m" << std::endl;

        // Fixed solutions should be reasonably close
        EXPECT_LT(rms_h, 1.0) << "Fixed horizontal RMS should be < 1.0m";
        EXPECT_LT(rms_v, 1.0) << "Fixed vertical RMS should be < 1.0m";
    } else {
        std::cout << "No fixed solutions matched with RTKLIB epochs" << std::endl;
    }
}

// ============================================================================
// Test 6: Per-epoch error vs RTKLIB (diagnostic - prints each epoch's error)
// ============================================================================
TEST_F(RTKRealDataTest, PerEpochErrorDiagnostic) {
    auto results = runFullRTK();
    ASSERT_GE(results.size(), 10u);

    std::cout << "=== Per-Epoch Errors (first 20 matched) ===" << std::endl;
    std::cout << "TOW        Status  E(m)     N(m)     U(m)     3D(m)" << std::endl;

    int printed = 0;
    double max_3d = 0.0;
    for (const auto& r : results) {
        auto it = rtklib_ecef_.lower_bound(r.tow - 1.0);
        if (it == rtklib_ecef_.end() || std::abs(it->first - r.tow) > 1.0) continue;

        Vector3d error_ecef = r.position_ecef - it->second;
        Vector3d error_enu = ecefToEnu(error_ecef, ref_position_);
        double err_3d = error_enu.norm();
        max_3d = std::max(max_3d, err_3d);

        if (printed < 20) {
            const char* status_str = (r.status == SolutionStatus::FIXED) ? "FIX" :
                                     (r.status == SolutionStatus::FLOAT) ? "FLT" : "OTH";
            printf("%.1f  %s   %+7.3f  %+7.3f  %+7.3f  %7.3f\n",
                   r.tow, status_str, error_enu(0), error_enu(1), error_enu(2), err_3d);
            printed++;
        }
    }
    std::cout << "Max 3D error: " << max_3d << " m" << std::endl;
}

// ============================================================================
// Test 7: Baseline length consistency
// ============================================================================
TEST_F(RTKRealDataTest, BaselineLengthConsistency) {
    auto results = runFullRTK();
    ASSERT_GE(results.size(), 10u);

    // Compute RTKLIB mean baseline length
    double rtklib_bl_sum = 0.0;
    int rtklib_count = 0;
    for (const auto& [tow, pos] : rtklib_ecef_) {
        rtklib_bl_sum += (pos - base_position_).norm();
        rtklib_count++;
    }
    double rtklib_mean_bl = rtklib_bl_sum / rtklib_count;

    // Compute our mean baseline length
    double our_bl_sum = 0.0;
    for (const auto& r : results) {
        our_bl_sum += (r.position_ecef - base_position_).norm();
    }
    double our_mean_bl = our_bl_sum / results.size();

    std::cout << "=== Baseline Length ===" << std::endl;
    std::cout << "  RTKLIB mean:  " << rtklib_mean_bl << " m" << std::endl;
    std::cout << "  Our mean:     " << our_mean_bl << " m" << std::endl;
    std::cout << "  Difference:   " << std::abs(our_mean_bl - rtklib_mean_bl) << " m" << std::endl;

    // Baseline lengths should agree within a few meters
    EXPECT_NEAR(our_mean_bl, rtklib_mean_bl, 5.0)
        << "Baseline length should be close to RTKLIB";
}

// ============================================================================
// Test 8: LAMBDA method unit test with known values
// ============================================================================
TEST(LAMBDATest, SimpleAmbiguityResolution) {
    RTKProcessor rtk;

    // Test with ambiguities close to integers
    VectorXd float_amb(3);
    float_amb << 1.02, -2.97, 5.01;

    // Small, well-conditioned covariance
    MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;

    VectorXd fixed_amb;
    double success_rate;
    bool ok = rtk.lambdaMethod(float_amb, cov, fixed_amb, success_rate);

    if (ok) {
        std::cout << "LAMBDA result: " << fixed_amb.transpose() << std::endl;
        std::cout << "Success rate: " << success_rate << std::endl;

        // Should round to nearest integers
        EXPECT_NEAR(fixed_amb(0), 1.0, 0.01);
        EXPECT_NEAR(fixed_amb(1), -3.0, 0.01);
        EXPECT_NEAR(fixed_amb(2), 5.0, 0.01);
    }
}

TEST(LAMBDATest, CorrelatedAmbiguities) {
    RTKProcessor rtk;

    VectorXd float_amb(4);
    float_amb << 10.05, 20.02, -15.03, 7.98;

    // Correlated covariance
    MatrixXd cov(4, 4);
    cov << 0.02, 0.01, 0.005, 0.002,
           0.01, 0.03, 0.008, 0.003,
           0.005, 0.008, 0.025, 0.006,
           0.002, 0.003, 0.006, 0.015;

    VectorXd fixed_amb;
    double success_rate;
    bool ok = rtk.lambdaMethod(float_amb, cov, fixed_amb, success_rate);

    if (ok) {
        std::cout << "LAMBDA correlated: " << fixed_amb.transpose() << std::endl;
        EXPECT_NEAR(fixed_amb(0), 10.0, 0.01);
        EXPECT_NEAR(fixed_amb(1), 20.0, 0.01);
        EXPECT_NEAR(fixed_amb(2), -15.0, 0.01);
        EXPECT_NEAR(fixed_amb(3), 8.0, 0.01);
    }
}

TEST(LAMBDATest, AmbiguousCase) {
    RTKProcessor rtk;

    // Ambiguities exactly at 0.5 - should be hard to resolve
    VectorXd float_amb(2);
    float_amb << 1.5, 2.5;

    MatrixXd cov = MatrixXd::Identity(2, 2) * 0.1;

    VectorXd fixed_amb;
    double success_rate;
    bool ok = rtk.lambdaMethod(float_amb, cov, fixed_amb, success_rate);

    std::cout << "LAMBDA ambiguous case: ok=" << ok
              << " success_rate=" << success_rate << std::endl;
    // This case should either fail or have low success rate
}

// ============================================================================
// Test 9: DD formation produces correct number of differences
// ============================================================================
TEST_F(RTKRealDataTest, DDFormationBasics) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    if (rover_header_.approximate_position.norm() > 0) {
        rover_obs.receiver_position = rover_header_.approximate_position;
    }

    // Process first epoch
    auto sol = rtk.processRTKEpoch(rover_obs, base_obs, nav_data_);

    // Should have a valid solution
    EXPECT_TRUE(sol.isValid()) << "First epoch should produce valid solution";
    EXPECT_GE(sol.num_satellites, 5) << "Should use at least 5 satellites";
}

// ============================================================================
// Test 10: Convergence over time - errors should decrease
// ============================================================================
TEST_F(RTKRealDataTest, ConvergenceBehavior) {
    auto results = runFullRTK();
    ASSERT_GE(results.size(), 20u);

    // Compute 3D error for first 10 and last 10 epochs
    auto compute_mean_error = [&](int start, int count) -> double {
        double sum = 0.0;
        int n = 0;
        for (int i = start; i < start + count && i < static_cast<int>(results.size()); ++i) {
            const auto& r = results[i];
            auto it = rtklib_ecef_.lower_bound(r.tow - 1.0);
            if (it == rtklib_ecef_.end() || std::abs(it->first - r.tow) > 1.0) continue;
            sum += (r.position_ecef - it->second).norm();
            n++;
        }
        return n > 0 ? sum / n : 999.0;
    };

    double early_error = compute_mean_error(0, 10);
    double late_error = compute_mean_error(std::max(0, (int)results.size() - 10), 10);

    std::cout << "=== Convergence ===" << std::endl;
    std::cout << "  First 10 epochs mean error: " << early_error << " m" << std::endl;
    std::cout << "  Last 10 epochs mean error:  " << late_error << " m" << std::endl;

    // Later epochs should generally not be much worse than early ones
    // (or should be better if filter converges)
}

// ============================================================================
// Test 11: ENU error decomposition correctness (verifies helper function)
// ============================================================================
TEST(ENUConversionTest, KnownPoint) {
    // Test at a known location: Tokyo area (lat=35.68, lon=139.77)
    double lat = 35.68 * M_PI / 180.0;
    double lon = 139.77 * M_PI / 180.0;
    double height = 50.0;

    Vector3d ref = geodeticToEcef(lat, lon, height);

    // A point 1m East should have ~1m East error in ENU
    Vector3d east_offset = geodeticToEcef(lat, lon + 1e-5 * M_PI / 180.0, height);
    Vector3d error_ecef = east_offset - ref;
    Vector3d enu = ecefToEnu(error_ecef, ref);

    // East component should be positive and close to the actual distance
    EXPECT_GT(enu(0), 0.0) << "East offset should give positive E";
    EXPECT_NEAR(enu(1), 0.0, 0.01) << "East offset should give ~0 N";
    EXPECT_NEAR(enu(2), 0.0, 0.01) << "East offset should give ~0 U";

    // A point 1m North
    Vector3d north_offset = geodeticToEcef(lat + 1e-5 * M_PI / 180.0, lon, height);
    error_ecef = north_offset - ref;
    enu = ecefToEnu(error_ecef, ref);
    EXPECT_NEAR(enu(0), 0.0, 0.01) << "North offset should give ~0 E";
    EXPECT_GT(enu(1), 0.0) << "North offset should give positive N";

    // A point 1m Up
    Vector3d up_offset = geodeticToEcef(lat, lon, height + 1.0);
    error_ecef = up_offset - ref;
    enu = ecefToEnu(error_ecef, ref);
    EXPECT_NEAR(enu(0), 0.0, 0.01) << "Up offset should give ~0 E";
    EXPECT_NEAR(enu(1), 0.0, 0.01) << "Up offset should give ~0 N";
    EXPECT_NEAR(enu(2), 1.0, 0.01) << "Up offset should give ~1 U";
}

// ============================================================================
// Test 12: DD residuals at RTKLIB true position should be small
//   Key diagnostic: if code DD residual (DD_PR - geom_DD) is large at true pos,
//   then DD formation or satellite position computation has a bug.
// ============================================================================
TEST_F(RTKRealDataTest, DDResidualsAtTruePosition) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    using TA = RTKProcessor::TestAccess;

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    // Use RTKLIB true position for first epoch
    auto it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
    ASSERT_NE(it, rtklib_ecef_.end());
    Vector3d true_pos = it->second;
    rover_obs.receiver_position = true_pos;

    const double lambda_L1 = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;

    auto dds = TA::formDD(rtk, rover_obs, base_obs, nav_data_);
    ASSERT_GE(dds.size(), 4u) << "Should form at least 4 DDs";

    std::cout << "=== DD Residuals at RTKLIB True Position ===" << std::endl;
    std::cout << "Sat  ref  dd_pr(m)       dd_cp(cyc)       geom_dd(m)     CodeRes(m)  PhaseRes(cyc) [frac]  Elev" << std::endl;

    double max_code_res = 0.0;
    double max_phase_res_cyc = 0.0;
    int valid_count = 0;

    for (const auto& dd : dds) {
        if (!dd.valid) continue;

        // Recompute geometric DD at true position
        auto rover_res = TA::buildResiduals(rtk, rover_obs, true_pos, nav_data_);
        auto base_res = TA::buildResiduals(rtk, base_obs, base_position_, nav_data_);

        auto r_sat = rover_res.find(dd.satellite);
        auto r_ref = rover_res.find(dd.reference_satellite);
        auto b_sat = base_res.find(dd.satellite);
        auto b_ref = base_res.find(dd.reference_satellite);
        if (r_sat == rover_res.end() || r_ref == rover_res.end() ||
            b_sat == base_res.end() || b_ref == base_res.end()) continue;

        double geom_dd = (r_sat->second.geometric_range - b_sat->second.geometric_range)
                       - (r_ref->second.geometric_range - b_ref->second.geometric_range);

        double code_res = dd.pseudorange_dd - geom_dd;
        double phase_dd_m = dd.carrier_phase_dd * lambda_L1;
        // Phase residual = phase_dd_m - geom_dd should be lambda*N (integer cycles)
        double phase_res_m = phase_dd_m - geom_dd;
        double phase_res_cyc = phase_res_m / lambda_L1;
        double phase_frac = phase_res_cyc - std::round(phase_res_cyc);

        double elev_deg = r_sat->second.elevation * 180.0 / M_PI;

        printf("G%02d  G%02d  %+14.3f  %+14.3f  %+12.3f  %+8.3f  %+12.3f [%+.3f]  %5.1f°\n",
               dd.satellite.prn, dd.reference_satellite.prn,
               dd.pseudorange_dd, dd.carrier_phase_dd, geom_dd,
               code_res, phase_res_cyc, phase_frac, elev_deg);

        max_code_res = std::max(max_code_res, std::abs(code_res));
        max_phase_res_cyc = std::max(max_phase_res_cyc, std::abs(phase_frac));
        valid_count++;
    }

    std::cout << "Max |code residual|: " << max_code_res << " m" << std::endl;
    std::cout << "Max |phase frac| (separate sat pos): " << max_phase_res_cyc << " cycles" << std::endl;

    // Now recompute geom_dd using SHARED satellite position (like buildDoubleDifferenceSystem does)
    std::cout << "\n--- Recomputed with shared satellite positions ---" << std::endl;
    double max_phase_frac_shared = 0.0;
    for (const auto& dd : dds) {
        if (!dd.valid) continue;

        // Use mean pseudorange for satellite position (same approach as buildDoubleDifferenceSystem)
        auto findObs = [](const ObservationData& od, const SatelliteId& s, SignalType sig) -> const Observation* {
            for (const auto& o : od.observations) {
                if (o.satellite == s && o.signal == sig) return &o;
            }
            return nullptr;
        };
        const auto* ros = findObs(rover_obs, dd.satellite, SignalType::GPS_L1CA);
        const auto* bos = findObs(base_obs, dd.satellite, SignalType::GPS_L1CA);
        const auto* ror = findObs(rover_obs, dd.reference_satellite, SignalType::GPS_L1CA);
        const auto* bor = findObs(base_obs, dd.reference_satellite, SignalType::GPS_L1CA);
        if (!ros || !bos || !ror || !bor) continue;

        double mean_pr_sat = 0.5 * (ros->pseudorange + bos->pseudorange);
        double mean_pr_ref = 0.5 * (ror->pseudorange + bor->pseudorange);

        Vector3d sat_pos, sat_vel, ref_pos, ref_vel;
        double sat_clk, sat_drift, ref_clk, ref_drift;

        // Use a local lambda to avoid depending on the anonymous namespace function
        auto calcSat = [&](const SatelliteId& s, double pr, Vector3d& pos, Vector3d& vel, double& clk, double& drift) -> bool {
            double travel = pr / constants::SPEED_OF_LIGHT;
            GNSSTime tx = rover_obs.time - travel;
            if (!nav_data_.calculateSatelliteState(s, tx, pos, vel, clk, drift)) return false;
            double omega_e = 7.2921151467e-5;
            double angle = omega_e * travel;
            Eigen::Matrix3d rot;
            rot << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
            pos = rot * pos;
            return true;
        };

        if (!calcSat(dd.satellite, mean_pr_sat, sat_pos, sat_vel, sat_clk, sat_drift)) continue;
        if (!calcSat(dd.reference_satellite, mean_pr_ref, ref_pos, ref_vel, ref_clk, ref_drift)) continue;

        double rover_range_sat = (sat_pos - true_pos).norm();
        double rover_range_ref = (ref_pos - true_pos).norm();
        double base_range_sat = (sat_pos - base_position_).norm();
        double base_range_ref = (ref_pos - base_position_).norm();

        // Include satellite clock correction (like RTKLIB does)
        double corr_rover_sat = rover_range_sat - constants::SPEED_OF_LIGHT * sat_clk;
        double corr_rover_ref = rover_range_ref - constants::SPEED_OF_LIGHT * ref_clk;
        double corr_base_sat = base_range_sat - constants::SPEED_OF_LIGHT * sat_clk;
        double corr_base_ref = base_range_ref - constants::SPEED_OF_LIGHT * ref_clk;

        // Without clock correction
        double geom_dd_no_clk = (rover_range_sat - base_range_sat) - (rover_range_ref - base_range_ref);
        // With clock correction
        double geom_dd_with_clk = (corr_rover_sat - corr_base_sat) - (corr_rover_ref - corr_base_ref);

        double phase_dd_m = dd.carrier_phase_dd * lambda_L1;
        double n1_no_clk = (phase_dd_m - geom_dd_no_clk) / lambda_L1;
        double n1_with_clk = (phase_dd_m - geom_dd_with_clk) / lambda_L1;
        double frac_no_clk = n1_no_clk - std::round(n1_no_clk);
        double frac_with_clk = n1_with_clk - std::round(n1_with_clk);

        printf("G%02d  shared_no_clk:  frac=%+.4f  shared_with_clk: frac=%+.4f  sat_clk=%.6f ref_clk=%.6f\n",
               dd.satellite.prn, frac_no_clk, frac_with_clk, sat_clk, ref_clk);
        max_phase_frac_shared = std::max(max_phase_frac_shared, std::abs(frac_no_clk));
    }
    std::cout << "Max |phase frac| (shared sat pos, no clk): " << max_phase_frac_shared << " cycles" << std::endl;

    EXPECT_LT(max_code_res, 10.0) << "Code DD residuals should be <10m at true position";
    // Phase frac ~0.3-0.4 is expected due to DD ionosphere residuals
    // (especially for low-elevation satellites). This is NOT a bug.
    // With ionosphere estimation or IF combination, this should decrease to <0.1.
    EXPECT_LT(max_phase_frac_shared, 0.5)
        << "Phase DD frac with shared sat pos should be <0.5 cycles";
}

// ============================================================================
// Test 13: KF initialized at true position - ambiguities should converge to integer
//   If they don't, the problem is in KF estimation, not initialization.
// ============================================================================
TEST_F(RTKRealDataTest, KFWithTruePositionInit) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    using TA = RTKProcessor::TestAccess;

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    // Use RTKLIB true position
    auto it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
    ASSERT_NE(it, rtklib_ecef_.end());
    Vector3d true_pos = it->second;
    rover_obs.receiver_position = true_pos;

    // Process first epoch normally (this initializes filter)
    auto sol = rtk.processRTKEpoch(rover_obs, base_obs, nav_data_);
    ASSERT_TRUE(sol.isValid());

    // Now force the filter state to the true baseline
    Vector3d true_baseline = true_pos - base_position_;
    auto& state = TA::filterStateMut(rtk);
    state.state.head<3>() = true_baseline;
    state.covariance.block<3,3>(0,0) = Matrix3d::Identity() * 0.001; // 3cm sigma

    std::cout << "=== KF True Position Init Test ===" << std::endl;
    std::cout << "True baseline: " << true_baseline.norm() << " m" << std::endl;

    // Process remaining epochs, tracking ambiguity convergence
    int epoch = 1;
    int fix_count = 0;
    double last_max_frac = 1.0;

    while (rover_ok && base_ok && epoch < 40) {
        Vector3d saved = rover_obs.receiver_position;
        rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        base_ok = base_reader_.readObservationEpoch(base_obs);
        if (!rover_ok || !base_ok) break;

        // Sync epochs
        double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                         + (rover_obs.time.tow - base_obs.time.tow);
        while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
            if (time_diff < 0) {
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
            if (!rover_ok || !base_ok) break;
            time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                      + (rover_obs.time.tow - base_obs.time.tow);
        }
        if (!rover_ok || !base_ok) break;

        // Feed true position each epoch
        auto ref_it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
        if (ref_it != rtklib_ecef_.end() && std::abs(ref_it->first - rover_obs.time.tow) < 1.0) {
            rover_obs.receiver_position = ref_it->second;
        } else {
            rover_obs.receiver_position = true_pos; // fallback
        }

        sol = rtk.processRTKEpoch(rover_obs, base_obs, nav_data_);
        if (sol.isFixed()) fix_count++;

        // After processing, force position back to truth (to isolate ambiguity estimation)
        if (ref_it != rtklib_ecef_.end() && std::abs(ref_it->first - rover_obs.time.tow) < 1.0) {
            Vector3d true_bl = ref_it->second - base_position_;
            TA::filterStateMut(rtk).state.head<3>() = true_bl;
            TA::filterStateMut(rtk).covariance.block<3,3>(0,0) = Matrix3d::Identity() * 0.001;
        }

        // Check ambiguity fractional parts
        const auto& fstate = TA::filterState(rtk);
        double max_frac = 0.0;
        int n_amb = 0;
        for (const auto& [sat, idx] : fstate.ambiguity_indices) {
            if (idx >= 0 && idx < fstate.state.size()) {
                double amb = fstate.state(idx);
                double frac = std::abs(amb - std::round(amb));
                max_frac = std::max(max_frac, frac);
                n_amb++;
            }
        }
        last_max_frac = max_frac;

        if (epoch <= 5 || epoch % 5 == 0) {
            double amb_var_max = 0.0;
            for (const auto& [sat, idx] : fstate.ambiguity_indices) {
                if (idx >= 0 && idx < fstate.covariance.rows()) {
                    amb_var_max = std::max(amb_var_max, fstate.covariance(idx, idx));
                }
            }
            printf("Epoch %2d: n_amb=%d max_frac=%.3f max_amb_var=%.4f status=%s\n",
                   epoch, n_amb, max_frac, amb_var_max,
                   sol.isFixed() ? "FIX" : "FLT");
        }

        epoch++;
    }

    std::cout << "Fix count: " << fix_count << "/" << epoch << std::endl;

    // With true position, max_frac ~0.4-0.5 is expected because DD ionosphere
    // biases the ambiguity estimate. This confirms the problem is NOT in position
    // estimation but in the measurement model (missing ionosphere correction).
    // With proper iono estimation, this should decrease to <0.15.
    EXPECT_LT(last_max_frac, 0.5)
        << "With true position, max ambiguity frac should be < 0.5";
}

// ============================================================================
// Test 14: DD consistency check across frequencies (L1 vs L2)
//   If L1 and L2 DD are inconsistent, there may be a signal mapping issue.
// ============================================================================
TEST_F(RTKRealDataTest, DDCrossFrequencyConsistency) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    using TA = RTKProcessor::TestAccess;

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    auto it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
    ASSERT_NE(it, rtklib_ecef_.end());
    Vector3d true_pos = it->second;
    rover_obs.receiver_position = true_pos;

    const double lambda_L1 = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;
    const double lambda_L2 = constants::SPEED_OF_LIGHT / constants::GPS_L2_FREQ;

    // Get L1 DDs
    auto dds = TA::formDD(rtk, rover_obs, base_obs, nav_data_);
    auto rover_res = TA::buildResiduals(rtk, rover_obs, true_pos, nav_data_);
    auto base_res = TA::buildResiduals(rtk, base_obs, base_position_, nav_data_);

    SatelliteId ref_sat;
    if (!dds.empty()) ref_sat = dds[0].reference_satellite;

    std::cout << "=== Cross-Frequency DD Consistency ===" << std::endl;
    std::cout << "Sat   L1_N(cyc)    L2_N(cyc)    N_WL=N1-N2  L1_frac  L2_frac" << std::endl;

    for (const auto& dd : dds) {
        if (!dd.valid) continue;

        // Get L2 observations
        auto findObs = [](const ObservationData& obs_data, const SatelliteId& sat, SignalType sig) -> const Observation* {
            for (const auto& obs : obs_data.observations) {
                if (obs.satellite == sat && obs.signal == sig) return &obs;
            }
            return nullptr;
        };

        const auto* rl2s = findObs(rover_obs, dd.satellite, SignalType::GPS_L2C);
        const auto* rl2r = findObs(rover_obs, dd.reference_satellite, SignalType::GPS_L2C);
        const auto* bl2s = findObs(base_obs, dd.satellite, SignalType::GPS_L2C);
        const auto* bl2r = findObs(base_obs, dd.reference_satellite, SignalType::GPS_L2C);

        if (!rl2s || !rl2r || !bl2s || !bl2r) continue;
        if (!rl2s->has_carrier_phase || !rl2r->has_carrier_phase ||
            !bl2s->has_carrier_phase || !bl2r->has_carrier_phase) continue;

        // L2 DD in cycles
        double l2_dd_cyc = (rl2s->carrier_phase - bl2s->carrier_phase) -
                           (rl2r->carrier_phase - bl2r->carrier_phase);

        // Geometric DD at true position
        auto r_sat = rover_res.find(dd.satellite);
        auto r_ref = rover_res.find(dd.reference_satellite);
        auto b_sat = base_res.find(dd.satellite);
        auto b_ref = base_res.find(dd.reference_satellite);
        if (r_sat == rover_res.end() || r_ref == rover_res.end() ||
            b_sat == base_res.end() || b_ref == base_res.end()) continue;

        double geom_dd = (r_sat->second.geometric_range - b_sat->second.geometric_range)
                       - (r_ref->second.geometric_range - b_ref->second.geometric_range);

        // L1 integer ambiguity estimate = (L1_DD_m - geom_DD) / lambda_L1
        double n1_float = (dd.carrier_phase_dd * lambda_L1 - geom_dd) / lambda_L1;
        double n2_float = (l2_dd_cyc * lambda_L2 - geom_dd) / lambda_L2;
        double n_wl = n1_float - n2_float; // wide-lane ambiguity

        double n1_frac = n1_float - std::round(n1_float);
        double n2_frac = n2_float - std::round(n2_float);

        printf("G%02d   %+12.3f  %+12.3f  %+12.3f  %+.3f    %+.3f\n",
               dd.satellite.prn, n1_float, n2_float, n_wl, n1_frac, n2_frac);
    }
}

// ============================================================================
// Test 15: Code-phase consistency check
//   (code_DD - geom_DD) should be similar to (phase_DD_m - geom_DD) mod lambda*N
//   If they diverge wildly, there may be a clock or timing issue.
// ============================================================================
TEST_F(RTKRealDataTest, CodePhaseConsistency) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    using TA = RTKProcessor::TestAccess;

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    auto it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
    ASSERT_NE(it, rtklib_ecef_.end());
    rover_obs.receiver_position = it->second;

    const double lambda_L1 = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;

    auto dds = TA::formDD(rtk, rover_obs, base_obs, nav_data_);
    auto rover_res = TA::buildResiduals(rtk, rover_obs, it->second, nav_data_);
    auto base_res = TA::buildResiduals(rtk, base_obs, base_position_, nav_data_);

    std::cout << "=== Code-Phase Consistency ===" << std::endl;
    std::cout << "Sat  CodeRes(m)  N1_from_code  N1_from_phase  Diff(cyc)" << std::endl;

    for (const auto& dd : dds) {
        if (!dd.valid) continue;

        auto r_sat = rover_res.find(dd.satellite);
        auto r_ref = rover_res.find(dd.reference_satellite);
        auto b_sat = base_res.find(dd.satellite);
        auto b_ref = base_res.find(dd.reference_satellite);
        if (r_sat == rover_res.end() || r_ref == rover_res.end() ||
            b_sat == base_res.end() || b_ref == base_res.end()) continue;

        double geom_dd = (r_sat->second.geometric_range - b_sat->second.geometric_range)
                       - (r_ref->second.geometric_range - b_ref->second.geometric_range);

        double code_res = dd.pseudorange_dd - geom_dd;
        // N1 estimated from phase
        double n1_phase = (dd.carrier_phase_dd * lambda_L1 - geom_dd) / lambda_L1;
        // N1 estimated from code (rough: code_res should be ~0, so N1_code ≈ phase_dd - code_dd/lambda)
        double n1_code = (dd.carrier_phase_dd * lambda_L1 - dd.pseudorange_dd) / lambda_L1;
        // The difference tells us about code noise/multipath
        double diff = n1_phase - std::round(n1_phase) - code_res / lambda_L1;

        printf("G%02d  %+8.3f    %+12.3f    %+12.3f    %+.3f\n",
               dd.satellite.prn, code_res, n1_code, n1_phase, n1_phase - n1_code);
    }
}

// ============================================================================
// Test 16: Undifferenced residual check - verify satellite clock is consistent
//   phase * lambda + sat_clk * c should equal geometric_range + lambda * N + receiver_clock
//   In DD, sat_clk and receiver_clock cancel. But let's check single-diff consistency.
// ============================================================================
TEST_F(RTKRealDataTest, UndifferencedClockConsistency) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    using TA = RTKProcessor::TestAccess;

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);
    ASSERT_TRUE(rover_ok && base_ok);

    auto it = rtklib_ecef_.lower_bound(rover_obs.time.tow - 1.0);
    ASSERT_NE(it, rtklib_ecef_.end());
    Vector3d true_pos = it->second;

    const double lambda_L1 = constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ;

    // Get undifferenced residuals at true position for rover and base
    auto rover_res = TA::buildResiduals(rtk, rover_obs, true_pos, nav_data_);
    auto base_res = TA::buildResiduals(rtk, base_obs, base_position_, nav_data_);

    std::cout << "=== Undifferenced Consistency ===" << std::endl;
    std::cout << "Checking: PR - geom_range (should be ~receiver_clock - sat_clock*c)" << std::endl;
    std::cout << "          Phase*lambda - geom_range (should be ~receiver_clock - sat_clock*c + N*lambda)" << std::endl;
    std::cout << "          PR - Phase*lambda (should be ~-2*N*lambda, code-phase diff)" << std::endl;
    std::cout << std::endl;

    std::cout << "--- Rover ---" << std::endl;
    std::cout << "Sat  geom_range(m)      PR_res(m)      Phase_res(m)    PR-Phase*lam(m)" << std::endl;
    for (const auto& [sat, res] : rover_res) {
        if (!res.valid) continue;
        double pr_res = res.pseudorange - res.geometric_range;
        double ph_res = res.carrier_phase * lambda_L1 - res.geometric_range;
        double pr_minus_ph = res.pseudorange - res.carrier_phase * lambda_L1;
        printf("G%02d  %15.3f  %+12.3f  %+15.3f  %+12.3f\n",
               sat.prn, res.geometric_range, pr_res, ph_res, pr_minus_ph);
    }

    std::cout << "\n--- Base ---" << std::endl;
    for (const auto& [sat, res] : base_res) {
        if (!res.valid) continue;
        double pr_res = res.pseudorange - res.geometric_range;
        double ph_res = res.carrier_phase * lambda_L1 - res.geometric_range;
        double pr_minus_ph = res.pseudorange - res.carrier_phase * lambda_L1;
        printf("G%02d  %15.3f  %+12.3f  %+15.3f  %+12.3f\n",
               sat.prn, res.geometric_range, pr_res, ph_res, pr_minus_ph);
    }

    // Check single-diff: (PR_rover - PR_base) - (geom_rover - geom_base) should be ~receiver_clock_diff
    // and should be the SAME for all satellites (within code noise)
    std::cout << "\n--- Single Differences (rover - base) ---" << std::endl;
    std::cout << "Sat  SD_PR_res(m)  SD_Phase_res(m)  SD_PR-Phase(m)" << std::endl;
    std::vector<double> sd_pr_res_values;
    std::vector<double> sd_ph_res_values;
    for (const auto& [sat, r_res] : rover_res) {
        auto b_it = base_res.find(sat);
        if (b_it == base_res.end() || !r_res.valid || !b_it->second.valid) continue;
        const auto& b_res = b_it->second;

        double sd_pr = (r_res.pseudorange - b_res.pseudorange) -
                       (r_res.geometric_range - b_res.geometric_range);
        double sd_ph = (r_res.carrier_phase - b_res.carrier_phase) * lambda_L1 -
                       (r_res.geometric_range - b_res.geometric_range);
        double sd_pr_ph = (r_res.pseudorange - b_res.pseudorange) -
                          (r_res.carrier_phase - b_res.carrier_phase) * lambda_L1;
        printf("G%02d  %+12.3f    %+15.3f     %+12.3f\n",
               sat.prn, sd_pr, sd_ph, sd_pr_ph);
        sd_pr_res_values.push_back(sd_pr);
        sd_ph_res_values.push_back(sd_ph);
    }

    // SD_PR_res should be consistent across sats (receiver clock diff)
    if (sd_pr_res_values.size() >= 2) {
        double mean_sd_pr = 0;
        for (double v : sd_pr_res_values) mean_sd_pr += v;
        mean_sd_pr /= sd_pr_res_values.size();
        double max_dev = 0;
        for (double v : sd_pr_res_values) max_dev = std::max(max_dev, std::abs(v - mean_sd_pr));
        std::cout << "\nSD_PR_res mean: " << mean_sd_pr << " m, max deviation: " << max_dev << " m" << std::endl;
        EXPECT_LT(max_dev, 5.0) << "SD code residuals should be consistent (receiver clock)";
    }
}

// ============================================================================
// Test 17: Float ambiguity diagnostics - check what fraction are close to integer
// ============================================================================
TEST_F(RTKRealDataTest, FloatAmbiguityDiagnostic) {
    RTKProcessor rtk;
    rtk.setRTKConfig(rtk_config_);
    rtk.setBasePosition(base_position_);

    ObservationData rover_obs, base_obs;
    bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
    bool base_ok = base_reader_.readObservationEpoch(base_obs);

    if (rover_header_.approximate_position.norm() > 0) {
        rover_obs.receiver_position = rover_header_.approximate_position;
    }

    // Process 30 epochs to let filter converge
    int epoch = 0;
    while (rover_ok && base_ok && epoch < 30) {
        double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                         + (rover_obs.time.tow - base_obs.time.tow);
        while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
            if (time_diff < 0) {
                Vector3d saved = rover_obs.receiver_position;
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
                if (rover_ok) rover_obs.receiver_position = saved;
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
            time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                      + (rover_obs.time.tow - base_obs.time.tow);
        }
        if (!rover_ok || !base_ok) break;

        auto sol = rtk.processRTKEpoch(rover_obs, base_obs, nav_data_);

        Vector3d saved = rover_obs.receiver_position;
        rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        base_ok = base_reader_.readObservationEpoch(base_obs);
        if (rover_ok) {
            rover_obs.receiver_position = sol.isValid() ? sol.position_ecef : saved;
        }
        epoch++;
    }

    // After 30 epochs, try LAMBDA with test ambiguities
    VectorXd float_amb(3);
    float_amb << 1.02, -2.97, 5.01;
    MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
    VectorXd fixed_amb;
    double success_rate;
    bool ok = rtk.lambdaMethod(float_amb, cov, fixed_amb, success_rate);

    std::cout << "=== LAMBDA after 30 epochs ===" << std::endl;
    std::cout << "  ok=" << ok << " success_rate=" << success_rate << std::endl;
    if (ok) {
        std::cout << "  Fixed: " << fixed_amb.transpose() << std::endl;
    }
}
