#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_stage_position_offset.hpp>

using namespace libgnss;
namespace {
struct StageOffsetFixture {
    std::vector<ObservationData> raw;
    FGOProcessor::FGOProblem problem;
    FGOProcessor::FGOResult result;
    StageOffsetFixture() {
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


TEST(NativeStagePositionOffset, GnssUsesVelocityHeadingAndPreservesAllOtherStates) {
    StageOffsetFixture f;
    const auto before=f.handoff();
    const auto out=native_imu_refinement::fromGnssResultWithPositionOffset(f.raw,f.problem,f.result,"mi8");
    const Vector3d expected_enu(-1.45/std::sqrt(13.0),-0.55/std::sqrt(13.0),0);
    for (std::size_t i=0;i<2;++i) {
        EXPECT_LT((out.displacement_enu_m[i]-expected_enu).norm(),1e-12);
        const Vector3d expected=f.result.solution.solutions[i].position_ecef+Vector3d(0,expected_enu.x(),expected_enu.y());
        EXPECT_LT((out.states.observations[i].receiver_position-expected).norm(),1e-9);
        EXPECT_EQ(out.states.velocity_ecef_mps[i],before.velocity_ecef_mps[i]);
        EXPECT_EQ(out.states.attitude_body_to_nav[i],before.attitude_body_to_nav[i]);
        EXPECT_EQ(out.states.clock_components_m[i],before.clock_components_m[i]);
        EXPECT_DOUBLE_EQ(out.states.observations[i].receiver_clock_bias,before.observations[i].receiver_clock_bias);
        EXPECT_DOUBLE_EQ(out.states.observations[i].receiver_clock_drift_mps,before.observations[i].receiver_clock_drift_mps);
        EXPECT_DOUBLE_EQ(out.states.observations[i].observations[0].pseudorange,21000000+i);
        EXPECT_EQ(f.result.solution.solutions[i].position_ecef,f.problem.epochs[i].position_ecef);
    }
}

TEST(NativeStagePositionOffset, ImuUsesOptimizedAttitudeAndDoesNotAccumulateOnRepeatedCalls) {
    StageOffsetFixture f;
    f.result.epoch_attitude_rpy_rad.assign(2,Vector3d::Zero());
    f.result.epoch_velocity_nav_mps.assign(2,Vector3d(2,3,4));
    const auto out=native_imu_refinement::fromResultWithPositionOffset(f.raw,f.problem,f.result,"mi8");
    const auto again=native_imu_refinement::fromResultWithPositionOffset(f.raw,f.problem,f.result,"mi8");
    const auto before=native_imu_refinement::fromResult(f.raw,f.problem,f.result);
    for (std::size_t i=0;i<2;++i) {
        EXPECT_LT((out.displacement_enu_m[i]-Vector3d(0.35,-0.25,0)).norm(),1e-12);
        EXPECT_LT((out.states.observations[i].receiver_position-before.observations[i].receiver_position-Vector3d(0,0.35,-0.25)).norm(),1e-9);
        EXPECT_EQ(out.states.observations[i].receiver_position,again.states.observations[i].receiver_position);
        EXPECT_EQ(out.states.velocity_ecef_mps[i],before.velocity_ecef_mps[i]);
        EXPECT_EQ(out.states.attitude_body_to_nav[i],before.attitude_body_to_nav[i]);
        EXPECT_EQ(out.states.clock_components_m[i],before.clock_components_m[i]);
        EXPECT_DOUBLE_EQ(out.states.observations[i].receiver_clock_drift_mps,12.5);
        EXPECT_DOUBLE_EQ(out.states.observations[i].observations[0].doppler,1200+i);
    }
}

TEST(NativeStagePositionOffset, RejectsUnknownPhoneBrokenKeysAndUnobservableGnssHeading) {
    { StageOffsetFixture f;
      EXPECT_THROW(native_imu_refinement::fromGnssResultWithPositionOffset(f.raw,f.problem,f.result,"unknown"),std::invalid_argument); }
    { StageOffsetFixture f; f.raw[1].raw_utc_time_millis++;
      EXPECT_THROW(native_imu_refinement::fromGnssResultWithPositionOffset(f.raw,f.problem,f.result,"mi8"),std::invalid_argument); }
    { StageOffsetFixture f; f.result.epoch_velocities_ecef_mps.assign(2,Vector3d::Zero());
      f.problem.imu.stop_velocity_seeds_nav.assign(2,Vector3d::Zero());
      EXPECT_THROW(native_imu_refinement::fromGnssResultWithPositionOffset(f.raw,f.problem,f.result,"mi8"),std::invalid_argument); }
    { StageOffsetFixture f; f.result.epoch_attitude_rpy_rad.assign(2,Vector3d::Zero());
      f.result.epoch_velocity_nav_mps.assign(2,Vector3d(2,3,4)); f.result.diagnostics.converged=false;
      EXPECT_THROW(native_imu_refinement::fromResultWithPositionOffset(f.raw,f.problem,f.result,"mi8"),std::invalid_argument); }
}
