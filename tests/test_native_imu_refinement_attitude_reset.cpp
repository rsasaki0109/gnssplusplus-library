#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_imu_refinement_attitude_reset.hpp>

using namespace libgnss;
namespace {
struct ResetFixture {
    std::vector<ObservationData> raw;
    FGOProcessor::FGOProblem problem;
    FGOProcessor::FGOResult result;
    explicit ResetFixture(std::size_t count) {
        problem.imu.valid=true;
        problem.imu.nav_origin_ecef=Vector3d(6378137,0,0);
        result.diagnostics.converged=true;
        for (std::size_t i=0;i<count;++i) {
            ObservationData epoch(GNSSTime(2300,300000.0+i));
            epoch.raw_source_index=i; epoch.raw_utc_time_millis=1700000000000LL+1000*i;
            Observation obs; obs.pseudorange=21000000+i; obs.doppler=100+i;
            epoch.observations.push_back(obs); raw.push_back(epoch);
            FGOProcessor::EpochSeed seed; seed.time=epoch.time;
            seed.raw_source_index=i; seed.raw_utc_time_millis=epoch.raw_utc_time_millis;
            problem.epochs.push_back(seed);
            PositionSolution solution; solution.time=epoch.time;
            solution.position_ecef=Vector3d(6378137,2*i,3*i);
            result.solution.solutions.push_back(solution);
            result.epoch_velocity_nav_mps.push_back(Vector3d(i%2 ? 3:1,0,0));
            result.epoch_attitude_rpy_rad.push_back(Vector3d(0.1,0.2,0.3));
            result.epoch_clock_bias_components_m.push_back({100000,2,3,4,5,6,7});
            result.epoch_clock_drift_mps.push_back(12.5);
        }
    }
    auto reset() const { return native_imu_refinement::fromResultWithAttitudeReset(raw,problem,result); }
};
}

TEST(NativeImuRefinementAttitudeReset, ChangesOnlyAttitudeAndPreservesUnsmoothVelocityAndClock) {
    ResetFixture f(30);
    const auto ordinary=native_imu_refinement::fromResult(f.raw,f.problem,f.result);
    const auto reset=f.reset();
    ASSERT_EQ(reset.states.attitude_body_to_nav.size(),30U);
    EXPECT_EQ(reset.low_speed_count,0U); EXPECT_EQ(reset.nearest_fill_count,0U);
    for (std::size_t i=0;i<30;++i) {
        const auto& rotation=reset.states.attitude_body_to_nav[i];
        EXPECT_LT((rotation*Vector3d::UnitX()+Vector3d::UnitX()).norm(),1e-12);
        EXPECT_LT((rotation*Vector3d::UnitZ()-Vector3d::UnitZ()).norm(),1e-12);
        EXPECT_NEAR(rotation.determinant(),1,1e-12);
        EXPECT_EQ(reset.states.velocity_ecef_mps[i],ordinary.velocity_ecef_mps[i]);
        EXPECT_EQ(reset.states.velocity_ecef_mps[i],Vector3d(0,i%2 ? 3:1,0));
        EXPECT_EQ(reset.states.clock_components_m[i],ordinary.clock_components_m[i]);
        EXPECT_EQ(reset.states.observations[i].receiver_position,ordinary.observations[i].receiver_position);
        EXPECT_DOUBLE_EQ(reset.states.observations[i].receiver_clock_bias,ordinary.observations[i].receiver_clock_bias);
        EXPECT_DOUBLE_EQ(reset.states.observations[i].receiver_clock_drift_mps,12.5);
        EXPECT_DOUBLE_EQ(reset.states.observations[i].observations[0].pseudorange,21000000+i);
        EXPECT_EQ(f.result.epoch_attitude_rpy_rad[i],Vector3d(0.1,0.2,0.3));
    }
}

TEST(NativeImuRefinementAttitudeReset, UsesNearestHeadingThroughSmoothedLowSpeedGap) {
    ResetFixture f(80);
    for (std::size_t i=0;i<80;++i)
        f.result.epoch_velocity_nav_mps[i]=i<20 ? Vector3d(2,0,0) : (i>=60 ? Vector3d(0,2,0) : Vector3d::Zero());
    const auto out=f.reset();
    EXPECT_EQ(out.low_speed_count,29U); EXPECT_EQ(out.nearest_fill_count,29U);
    // Window 20 leaves valid courses at 25 and 55 (speed exactly 0.5).
    // The equidistant gap sample 40 uses the later course, not interpolation.
    EXPECT_LT((out.states.attitude_body_to_nav[39]*Vector3d::UnitX()+Vector3d::UnitX()).norm(),1e-12);
    EXPECT_LT((out.states.attitude_body_to_nav[40]*Vector3d::UnitX()+Vector3d::UnitY()).norm(),1e-12);
    EXPECT_EQ(out.states.velocity_ecef_mps[40],Vector3d::Zero());
}

TEST(NativeImuRefinementAttitudeReset, RejectsUnobservableOrIncompleteStatesWithoutChangingInput) {
    { ResetFixture f(2); f.result.epoch_velocity_nav_mps.assign(2,Vector3d::Zero());
      EXPECT_THROW(f.reset(),std::invalid_argument);
      EXPECT_EQ(f.result.epoch_attitude_rpy_rad[0],Vector3d(0.1,0.2,0.3)); }
    { ResetFixture f(2); f.result.epoch_velocity_nav_mps.pop_back(); EXPECT_THROW(f.reset(),std::invalid_argument); }
    { ResetFixture f(2); f.result.epoch_velocity_nav_mps[0].x()=std::numeric_limits<double>::quiet_NaN();
      EXPECT_THROW(f.reset(),std::invalid_argument); }
    { ResetFixture f(2); f.result.solution.solutions[1].time=f.raw[1].time+0.001; EXPECT_THROW(f.reset(),std::invalid_argument); }
    { ResetFixture f(2); f.result.diagnostics.converged=false; EXPECT_THROW(f.reset(),std::invalid_argument); }
}
