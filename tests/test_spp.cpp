#include <gtest/gtest.h>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/navigation.hpp>

using namespace libgnss;

class SPPTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize SPP processor
        spp_processor_ = std::make_unique<SPPProcessor>();
        
        // Create test configuration
        ProcessorConfig config;
        config.elevation_mask = 15.0;
        config.snr_mask = 35.0;
        config.mode = PositioningMode::SPP;
        
        spp_processor_->initialize(config);
        
        // Create test observation data
        createTestObservations();
        createTestNavigation();
    }
    
    void createTestObservations() {
        obs_data_.time = GNSSTime(2000, 345600.0);
        obs_data_.receiver_position = Vector3d(4000000.0, 3000000.0, 2000000.0);
        
        // Add multiple GPS satellites with realistic pseudoranges
        std::vector<int> prns = {1, 2, 3, 5, 6, 7, 9, 12};
        std::vector<double> ranges = {20123456.7, 21234567.8, 22345678.9, 
                                    23456789.0, 24567890.1, 25678901.2, 
                                    26789012.3, 27890123.4};
        
        for (size_t i = 0; i < prns.size(); ++i) {
            Observation obs(SatelliteId(GNSSSystem::GPS, prns[i]), SignalType::GPS_L1CA);
            obs.pseudorange = ranges[i];
            obs.carrier_phase = ranges[i] / (constants::SPEED_OF_LIGHT / constants::GPS_L1_FREQ);
            obs.snr = 45.0 - i * 2.0;  // Decreasing SNR
            obs.valid = true;
            obs_data_.addObservation(obs);
        }
    }
    
    void createTestNavigation() {
        // Create simplified ephemeris data for test satellites
        std::vector<int> prns_to_create = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 20};
        for (int prn : prns_to_create) {
            Ephemeris eph;
            eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
            eph.toe = GNSSTime(2000, 345600.0);
            eph.toc = GNSSTime(2000, 345600.0);
            eph.week = 2000;
            
            // Simplified orbital parameters (not realistic but sufficient for testing)
            eph.sqrt_a = 5153.0;  // sqrt(semi-major axis) in sqrt(m)
            eph.e = 0.01;         // eccentricity
            eph.i0 = 0.97;        // inclination (rad)
            eph.omega0 = prn * 0.5; // right ascension (rad)
            eph.omega = 0.0;      // argument of perigee (rad)
            eph.m0 = prn * 0.3;   // mean anomaly (rad)
            eph.delta_n = 0.0;
            eph.idot = 0.0;
            eph.omega_dot = 0.0;
            
            // Clock parameters
            eph.af0 = 1e-6;  // clock bias
            eph.af1 = 0.0;   // clock drift
            eph.af2 = 0.0;   // clock drift rate
            eph.tgd = 0.0;   // group delay
            
            eph.valid = true;
            nav_data_.addEphemeris(eph);
        }
    }
    
    std::unique_ptr<SPPProcessor> spp_processor_;
    ObservationData obs_data_;
    NavigationData nav_data_;
};

TEST_F(SPPTest, ProcessorInitialization) {
    EXPECT_NE(spp_processor_, nullptr);
    
    // Test configuration
    auto config = spp_processor_->getSPPConfig();
    EXPECT_GT(config.max_iterations, 0);
    EXPECT_GT(config.pseudorange_sigma, 0.0);
}

TEST_F(SPPTest, BasicPositioning) {
    auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    
    EXPECT_TRUE(solution.isValid());
    EXPECT_EQ(solution.status, SolutionStatus::SPP);
    EXPECT_GE(solution.num_satellites, 4);
    EXPECT_GT(solution.pdop, 0.0);
    EXPECT_LT(solution.pdop, 20.0);  // Reasonable PDOP
}

TEST_F(SPPTest, InsufficientSatellites) {
    // Remove satellites to have less than 4
    obs_data_.observations.resize(3);
    
    auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    
    EXPECT_FALSE(solution.isValid());
    EXPECT_LT(solution.num_satellites, 4);
}

TEST_F(SPPTest, QualityControl) {
    // Add low SNR observation
    Observation bad_obs(SatelliteId(GNSSSystem::GPS, 20), SignalType::GPS_L1CA);
    bad_obs.pseudorange = 28000000.0;
    bad_obs.snr = 25.0;  // Below threshold
    bad_obs.valid = true;
    obs_data_.addObservation(bad_obs);
    
    auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    
    // Should still be valid but low SNR satellite should be filtered out
    EXPECT_TRUE(solution.isValid());
    
    // Check that the low SNR satellite is not in the solution
    bool found_bad_sat = false;
    for (const auto& sat : solution.satellites_used) {
        if (sat.prn == 20) {
            found_bad_sat = true;
            break;
        }
    }
    EXPECT_FALSE(found_bad_sat);
}

TEST_F(SPPTest, DOPCalculation) {
    auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    
    EXPECT_TRUE(solution.isValid());
    EXPECT_GT(solution.gdop, 0.0);
    EXPECT_GT(solution.pdop, 0.0);
    EXPECT_GT(solution.hdop, 0.0);
    EXPECT_GT(solution.vdop, 0.0);
    
    // GDOP should be >= PDOP
    EXPECT_GE(solution.gdop, solution.pdop);
}

TEST_F(SPPTest, ProcessingStatistics) {
    auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
    
    EXPECT_TRUE(solution.isValid());
    EXPECT_GT(solution.processing_time_ms, 0.0);
    EXPECT_GE(solution.iterations, 1);
    EXPECT_GE(solution.residual_rms, 0.0);
}

TEST_F(SPPTest, MultipleEpochs) {
    int valid_solutions = 0;
    
    for (int i = 0; i < 10; ++i) {
        // Update time
        obs_data_.time = GNSSTime(2000, 345600.0 + i * 30.0);
        
        // Add some noise to pseudoranges
        for (auto& obs : obs_data_.observations) {
            obs.pseudorange += (i % 2 == 0 ? 1.0 : -1.0) * 0.5;
        }
        
        auto solution = spp_processor_->processEpoch(obs_data_, nav_data_);
        
        if (solution.isValid()) {
            valid_solutions++;
        }
    }
    
    EXPECT_GT(valid_solutions, 8);  // Should have most solutions valid
}

// Test SPP utility functions
TEST_F(SPPTest, UtilityFunctions) {
    Vector3d receiver_pos(4000000.0, 3000000.0, 2000000.0);
    Vector3d satellite_pos(20000000.0, 15000000.0, 10000000.0);
    
    // Test elevation calculation
    double elevation = spp_utils::calculateElevation(receiver_pos, satellite_pos);
    EXPECT_GE(elevation, 0.0);
    EXPECT_LE(elevation, M_PI/2);
    
    // Test azimuth calculation
    double azimuth = spp_utils::calculateAzimuth(receiver_pos, satellite_pos);
    EXPECT_GE(azimuth, 0.0);
    EXPECT_LT(azimuth, 2*M_PI);
    
    // Test coordinate conversion
    auto geodetic = spp_utils::ecefToGeodetic(receiver_pos);
    EXPECT_GE(geodetic.latitude, -M_PI/2);
    EXPECT_LE(geodetic.latitude, M_PI/2);
    EXPECT_GE(geodetic.longitude, -M_PI);
    EXPECT_LT(geodetic.longitude, M_PI);
    
    auto ecef_back = spp_utils::geodeticToEcef(geodetic);
    EXPECT_NEAR(ecef_back(0), receiver_pos(0), 1.0);
    EXPECT_NEAR(ecef_back(1), receiver_pos(1), 1.0);
    EXPECT_NEAR(ecef_back(2), receiver_pos(2), 1.0);
}
