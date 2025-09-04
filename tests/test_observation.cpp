#include <gtest/gtest.h>
#include <libgnss++/core/observation.hpp>

using namespace libgnss;

class ObservationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test observation data
        obs_data_.time = GNSSTime(2000, 345600.0);
        obs_data_.receiver_position = Vector3d(4000000.0, 3000000.0, 2000000.0);
        
        // Add GPS satellite observations
        Observation obs1(SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L1CA);
        obs1.pseudorange = 20000000.0;
        obs1.carrier_phase = 105263157.9;
        obs1.snr = 45.0;
        obs1.valid = true;
        obs_data_.addObservation(obs1);
        
        Observation obs2(SatelliteId(GNSSSystem::GPS, 2), SignalType::GPS_L1CA);
        obs2.pseudorange = 21000000.0;
        obs2.carrier_phase = 110526315.8;
        obs2.snr = 42.0;
        obs2.valid = true;
        obs_data_.addObservation(obs2);
        
        // Add GLONASS satellite observation
        Observation obs3(SatelliteId(GNSSSystem::GLONASS, 1), SignalType::GLO_L1CA);
        obs3.pseudorange = 22000000.0;
        obs3.carrier_phase = 115789473.7;
        obs3.snr = 40.0;
        obs3.valid = true;
        obs_data_.addObservation(obs3);
    }
    
    ObservationData obs_data_;
};

TEST_F(ObservationTest, BasicObservationData) {
    EXPECT_EQ(obs_data_.observations.size(), 3);
    EXPECT_EQ(obs_data_.getNumSatellites(), 3);
    EXPECT_FALSE(obs_data_.isEmpty());
}

TEST_F(ObservationTest, GetObservationsBySatellite) {
    SatelliteId gps1(GNSSSystem::GPS, 1);
    auto gps1_obs = obs_data_.getObservations(gps1);
    
    EXPECT_EQ(gps1_obs.size(), 1);
    EXPECT_EQ(gps1_obs[0].satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(gps1_obs[0].satellite.prn, 1);
}

TEST_F(ObservationTest, GetObservationsBySystem) {
    auto gps_obs = obs_data_.getObservations(GNSSSystem::GPS);
    auto glo_obs = obs_data_.getObservations(GNSSSystem::GLONASS);
    
    EXPECT_EQ(gps_obs.size(), 2);
    EXPECT_EQ(glo_obs.size(), 1);
}

TEST_F(ObservationTest, GetObservationsBySignal) {
    auto l1ca_obs = obs_data_.getObservations(SignalType::GPS_L1CA);
    
    EXPECT_EQ(l1ca_obs.size(), 2);  // Two GPS L1 C/A observations
}

TEST_F(ObservationTest, FilterBySNR) {
    auto filtered_obs = obs_data_.filterBySNR(43.0);  // SNR >= 43 dB-Hz
    
    EXPECT_EQ(filtered_obs.size(), 1);  // Only GPS PRN 1 has SNR >= 43
    EXPECT_EQ(filtered_obs[0].satellite.prn, 1);
}

TEST_F(ObservationTest, HasObservation) {
    SatelliteId gps1(GNSSSystem::GPS, 1);
    SatelliteId gps99(GNSSSystem::GPS, 99);
    
    EXPECT_TRUE(obs_data_.hasObservation(gps1, SignalType::GPS_L1CA));
    EXPECT_FALSE(obs_data_.hasObservation(gps99, SignalType::GPS_L1CA));
}

TEST_F(ObservationTest, GetUniquesatellites) {
    auto satellites = obs_data_.getSatellites();
    
    EXPECT_EQ(satellites.size(), 3);
    
    // Check that we have the expected satellites
    bool has_gps1 = false, has_gps2 = false, has_glo1 = false;
    for (const auto& sat : satellites) {
        if (sat.system == GNSSSystem::GPS && sat.prn == 1) has_gps1 = true;
        if (sat.system == GNSSSystem::GPS && sat.prn == 2) has_gps2 = true;
        if (sat.system == GNSSSystem::GLONASS && sat.prn == 1) has_glo1 = true;
    }
    
    EXPECT_TRUE(has_gps1);
    EXPECT_TRUE(has_gps2);
    EXPECT_TRUE(has_glo1);
}

TEST_F(ObservationTest, QualityControl) {
    // Apply quality control with high SNR threshold
    obs_data_.applyQualityControl(0.0, 44.0);  // min_elevation=0, min_snr=44
    
    EXPECT_EQ(obs_data_.observations.size(), 1);  // Only GPS PRN 1 should remain
    EXPECT_EQ(obs_data_.observations[0].satellite.prn, 1);
}

TEST_F(ObservationTest, EpochStats) {
    auto stats = obs_data_.getStats();
    
    EXPECT_EQ(stats.total_observations, 3);
    EXPECT_EQ(stats.valid_observations, 3);
    EXPECT_EQ(stats.num_satellites, 3);
    EXPECT_EQ(stats.num_systems, 2);  // GPS and GLONASS
    EXPECT_GT(stats.average_snr, 40.0);
}

class ObservationSeriesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create multiple epochs
        for (int i = 0; i < 5; ++i) {
            ObservationData epoch;
            epoch.time = GNSSTime(2000, 345600.0 + i * 30.0);  // 30-second intervals
            
            Observation obs(SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L1CA);
            obs.pseudorange = 20000000.0 + i * 1000.0;
            obs.valid = true;
            epoch.addObservation(obs);
            
            series_.addEpoch(epoch);
        }
    }
    
    ObservationSeries series_;
};

TEST_F(ObservationSeriesTest, BasicSeries) {
    EXPECT_EQ(series_.size(), 5);
    EXPECT_FALSE(series_.isEmpty());
}

TEST_F(ObservationSeriesTest, GetEpochByTime) {
    GNSSTime target_time(2000, 345630.0);  // Second epoch
    const auto* epoch = series_.getEpoch(target_time);
    
    EXPECT_NE(epoch, nullptr);
    EXPECT_EQ(epoch->time.tow, 345630.0);
}

TEST_F(ObservationSeriesTest, GetTimeSpan) {
    auto [start, end] = series_.getTimeSpan();
    
    EXPECT_EQ(start.tow, 345600.0);
    EXPECT_EQ(end.tow, 345720.0);  // 345600 + 4*30
}

TEST_F(ObservationSeriesTest, GetEpochsInRange) {
    GNSSTime start_time(2000, 345630.0);
    GNSSTime end_time(2000, 345690.0);
    
    auto epochs = series_.getEpochs(start_time, end_time);
    
    EXPECT_EQ(epochs.size(), 3);  // Should include epochs at 345630, 345660, 345690
}
