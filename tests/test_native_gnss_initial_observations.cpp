#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_gnss_initial_observations.hpp>

using namespace libgnss;
namespace {
struct ColdFixture {
    std::vector<ObservationData> raw;
    FGOProcessor::FGOProblem source;
    ColdFixture() {
        const std::array<int,5> seconds{0,1,2,6,7};
        for (std::size_t i=0;i<5;++i) {
            ObservationData epoch(GNSSTime(2300,100000.0+seconds[i]));
            epoch.raw_source_index=i; epoch.raw_utc_time_millis=1700000000000LL+1000*seconds[i];
            epoch.receiver_position=Vector3d(6378137.0+i*i,2*i,0);
            epoch.receiver_clock_bias=100.0/constants::SPEED_OF_LIGHT;
            epoch.receiver_clock_drift_mps=3.5;
            Observation row; row.pseudorange=21000000+i; row.doppler=100+i;
            epoch.observations.push_back(row); raw.push_back(epoch);
            FGOProcessor::EpochSeed e; e.time=epoch.time; e.raw_source_index=i;
            e.raw_utc_time_millis=epoch.raw_utc_time_millis; e.position_ecef=epoch.receiver_position;
            source.epochs.push_back(e);
            raw_p_seed::RawPNoDopplerSeed seed; seed.epoch_index=i; seed.raw_source_index=i;
            seed.time=epoch.time; seed.raw_utc_time_millis=epoch.raw_utc_time_millis;
            seed.status=raw_p_seed::SeedAdapterStatus::Accepted;
            seed.position_ecef=epoch.receiver_position; seed.velocity_ecef_mps=Vector3d(99,98,97);
            seed.has_position=seed.has_velocity=seed.has_clock=seed.has_clock_rate=true;
            seed.clock_bias_m=100; seed.clock_rate_mps=3.5;
            seed.c7_clock_mapping_supported=true; seed.reference_clock_group=GNSSSystem::GPS;
            seed.clock_bias_component_available[0]=true; seed.clock_bias_components_m[0]=100;
            seed.temporal_initial_guess=i==1; seed.initial_guess_left_source=0; seed.initial_guess_right_source=2;
            source.native_raw_p_no_doppler_seeds.push_back(seed);
        }
    }
    auto input() const { return native_gnss_initial::fromNativeSeeds(raw,source); }
};
}

TEST(NativeGnssInitialObservations, UsesNominalIntervalGradientIncludingIrregularUtcGap) {
    ColdFixture f; const auto out=f.input();
    EXPECT_DOUBLE_EQ(out.interval_s,1.0);
    const std::array<double,5> dx{1,2,4,6,7};
    ASSERT_EQ(out.states.velocity_ecef_mps.size(),5U);
    for (std::size_t i=0;i<5;++i) {
        EXPECT_EQ(out.states.velocity_ecef_mps[i],Vector3d(dx[i],2,0));
        EXPECT_EQ(out.seeds[i].velocity_ecef_mps,out.states.velocity_ecef_mps[i]);
        EXPECT_TRUE(out.seeds[i].has_velocity);
        EXPECT_EQ(out.seeds[i].position_ecef,f.source.native_raw_p_no_doppler_seeds[i].position_ecef);
        EXPECT_DOUBLE_EQ(out.seeds[i].clock_bias_m,100);
        EXPECT_DOUBLE_EQ(out.seeds[i].clock_rate_mps,3.5);
        EXPECT_DOUBLE_EQ(out.states.observations[i].observations[0].pseudorange,21000000+i);
        EXPECT_DOUBLE_EQ(out.states.observations[i].observations[0].doppler,100+i);
        EXPECT_EQ(f.source.native_raw_p_no_doppler_seeds[i].velocity_ecef_mps,Vector3d(99,98,97));
    }
    EXPECT_TRUE(out.seeds[1].temporal_initial_guess);
    EXPECT_EQ(out.seeds[1].initial_guess_left_source,0U);
    EXPECT_EQ(out.seeds[1].initial_guess_right_source,2U);
    EXPECT_TRUE(out.states.attitude_body_to_nav.empty());
}

TEST(NativeGnssInitialObservations, RejectsBrokenTimeAndNativePositionProvenance) {
    { ColdFixture f; f.raw.pop_back(); EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.epochs[1].raw_utc_time_millis++; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.native_raw_p_no_doppler_seeds[1].raw_source_index=0; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.epochs[1].time=f.raw[1].time+0.001; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.raw[1].receiver_position.x()+=1; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.native_raw_p_no_doppler_seeds[1].status=raw_p_seed::SeedAdapterStatus::RawPResultRejected;
      EXPECT_THROW(f.input(),std::invalid_argument); }
}

TEST(NativeGnssInitialObservations, RejectsClockUnitDriftAndMappingMismatch) {
    { ColdFixture f; f.raw[1].receiver_clock_bias=100; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.raw[1].receiver_clock_drift_mps=4; EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.native_raw_p_no_doppler_seeds[1].clock_bias_components_m[0]=101;
      EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.native_raw_p_no_doppler_seeds[1].reference_clock_group=GNSSSystem::Galileo;
      EXPECT_THROW(f.input(),std::invalid_argument); }
    { ColdFixture f; f.source.native_raw_p_no_doppler_seeds[1].clock_rate_mps=std::numeric_limits<double>::quiet_NaN();
      EXPECT_THROW(f.input(),std::invalid_argument); }
}
