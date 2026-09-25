#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_gnss_imu_initial_handoff.hpp>

using namespace libgnss;
namespace {
struct InitialFixture {
    std::vector<ObservationData> raw;
    FGOProcessor::FGOProblem problem;
    FGOProcessor::FGOResult result;
    InitialFixture() {
        problem.imu.valid = true;
        problem.imu.nav_origin_ecef = Vector3d(6378137,0,0);
        result.diagnostics.converged = true;
        for (std::size_t i=0; i<2; ++i) {
            ObservationData epoch(GNSSTime(2300,300000.0+i));
            epoch.raw_source_index=i;
            epoch.raw_utc_time_millis=1700000000000LL+1000*i;
            epoch.receiver_position=Vector3d(6378130,0,0);
            Observation row; row.pseudorange=21000000+i; row.doppler=1200+i;
            epoch.observations.push_back(row); raw.push_back(epoch);
            PositionSolution solution; solution.time=epoch.time;
            solution.position_ecef=Vector3d(6378137,2*i,3*i);
            result.solution.solutions.push_back(solution);
            result.epoch_velocities_ecef_mps.push_back(Vector3d(4,2,3));
            result.epoch_clock_bias_components_m.push_back({100000,2,3,4,5,6,7});
            result.epoch_clock_drift_mps.push_back(12.5);
            FGOProcessor::EpochSeed seed; seed.time=epoch.time; seed.raw_source_index=i;
            seed.raw_utc_time_millis=epoch.raw_utc_time_millis; seed.position_ecef=solution.position_ecef;
            problem.epochs.push_back(seed);
            problem.imu.stop_velocity_seeds_nav.push_back(Vector3d(2,3,4));
            problem.imu.epoch_heading_attitude_times.push_back(epoch.time);
            problem.imu.epoch_heading_attitudes_body_to_nav.push_back(
                Eigen::AngleAxisd(0.5,Vector3d::UnitZ()).toRotationMatrix());
        }
        problem.native_source_clock_c0d_gnss_first_c_handoff_m=result.epoch_clock_bias_components_m;
        problem.native_source_clock_c0d_gnss_first_d_handoff_mps=result.epoch_clock_drift_mps;
    }
    auto handoff() const { return native_imu_refinement::fromGnssResult(raw,problem,result); }
};
}

TEST(NativeGnssImuInitialHandoff, PreservesRawMeasurementsAndCombinesDistinctStageStates) {
    InitialFixture f; const auto out=f.handoff();
    ASSERT_EQ(out.observations.size(),2U);
    EXPECT_DOUBLE_EQ(out.observations[1].observations[0].pseudorange,21000001);
    EXPECT_DOUBLE_EQ(out.observations[1].observations[0].doppler,1201);
    EXPECT_EQ(out.velocity_ecef_mps[0],Vector3d(4,2,3));
    EXPECT_EQ(out.attitude_body_to_nav[0],f.problem.imu.epoch_heading_attitudes_body_to_nav[0]);
    EXPECT_EQ(out.observations[1].receiver_position,f.result.solution.solutions[1].position_ecef);
    EXPECT_NEAR(out.observations[1].receiver_clock_bias*constants::SPEED_OF_LIGHT,100000,1e-10);
    EXPECT_DOUBLE_EQ(out.observations[1].receiver_clock_drift_mps,12.5);
    EXPECT_EQ(out.clock_components_m[1],f.result.epoch_clock_bias_components_m[1]);
    EXPECT_DOUBLE_EQ(f.raw[1].receiver_clock_bias,0);
    EXPECT_EQ(f.raw[1].receiver_position,Vector3d(6378130,0,0));
    EXPECT_TRUE(f.result.epoch_attitude_rpy_rad.empty());
}

TEST(NativeGnssImuInitialHandoff, RejectsWrongStageMissingEpochsAndTimeJoins) {
    { InitialFixture f; f.result.diagnostics.converged=false; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.diagnostics.imu_intervals=1; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.epoch_attitude_rpy_rad.push_back(Vector3d::Zero()); EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.epoch_velocities_ecef_mps.pop_back(); EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.imu.epoch_heading_attitude_times[1]=f.raw[1].time+0.001; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.epochs[1].raw_utc_time_millis++; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.raw[1].raw_source_index=0; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.solution.solutions[1].time=f.raw[1].time+0.001; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.raw[1].raw_utc_time_millis=f.raw[0].raw_utc_time_millis;
      f.problem.epochs[1].raw_utc_time_millis=f.raw[1].raw_utc_time_millis; EXPECT_THROW(f.handoff(),std::invalid_argument); }
}

TEST(NativeGnssImuInitialHandoff, RejectsFrameClockPositionAndRotationInconsistency) {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    { InitialFixture f; f.problem.imu.nav_origin_lat_rad=90; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.imu.stop_velocity_seeds_nav[0]=Vector3d(4,2,3); EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.native_source_clock_c0d_gnss_first_c_handoff_m[1][6]+=1; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.epoch_clock_drift_mps[1]=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.result.epoch_velocities_ecef_mps[1].x()=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.epochs[1].position_ecef.x()+=0.001; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.imu.epoch_heading_attitudes_body_to_nav[0](0,0)=2; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { InitialFixture f; f.problem.imu.epoch_heading_attitudes_body_to_nav[0]=Matrix3d::Identity();
      f.problem.imu.epoch_heading_attitudes_body_to_nav[0](0,0)=-1; EXPECT_THROW(f.handoff(),std::invalid_argument); }
}
