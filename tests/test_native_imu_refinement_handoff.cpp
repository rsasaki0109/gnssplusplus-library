#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_imu_refinement_handoff.hpp>
#include <libgnss++/algorithms/native_imu_refinement_rebuild.hpp>

using namespace libgnss;
namespace {
struct Fixture {
    std::vector<ObservationData> raw;
    FGOProcessor::FGOProblem problem;
    FGOProcessor::FGOResult result;
    Fixture() {
        problem.imu.valid = true;
        problem.imu.nav_origin_ecef = Vector3d(6378137, 0, 0);
        result.diagnostics.converged = true;
        for (std::size_t i=0; i<2; ++i) {
            ObservationData epoch(GNSSTime(2300, 300000.0 + i));
            epoch.receiver_position = Vector3d(6378130, 0, 0);
            epoch.raw_source_index = i;
            epoch.raw_utc_time_millis = 1700000000000LL + 1000*i;
            Observation row; row.pseudorange = 21000000.0 + i;
            epoch.observations.push_back(row); raw.push_back(epoch);
            FGOProcessor::EpochSeed seed;
            seed.time = epoch.time; seed.raw_source_index = i;
            seed.raw_utc_time_millis = epoch.raw_utc_time_millis;
            problem.epochs.push_back(seed);
            PositionSolution solution;
            solution.time = epoch.time;
            solution.position_ecef = Vector3d(6378137, 2.0*i, 3.0*i);
            result.solution.solutions.push_back(solution);
            result.epoch_velocity_nav_mps.push_back(Vector3d(2,3,4));
            result.epoch_attitude_rpy_rad.push_back(Vector3d(0,0,1.5707963267948966));
            result.epoch_clock_drift_mps.push_back(12.5);
            result.epoch_clock_bias_components_m.push_back({100000, 2,3,4,5,6,7});
        }
    }
    auto handoff() const { return native_imu_refinement::fromResult(raw,problem,result); }
};
}

TEST(NativeImuRefinementHandoff, PreservesRawDataAndConvertsStateUnitsAndFrames) {
    Fixture f;
    const auto out = f.handoff();
    ASSERT_EQ(out.observations.size(), 2U);
    EXPECT_EQ(out.observations[1].raw_utc_time_millis, f.raw[1].raw_utc_time_millis);
    EXPECT_DOUBLE_EQ(out.observations[1].observations[0].pseudorange,21000001.0);
    EXPECT_DOUBLE_EQ(f.raw[1].receiver_clock_bias,0.0);
    EXPECT_DOUBLE_EQ(f.raw[1].receiver_position.x(),6378130.0);
    EXPECT_NEAR(out.observations[1].receiver_clock_bias*constants::SPEED_OF_LIGHT,100000,1e-10);
    EXPECT_DOUBLE_EQ(out.observations[1].receiver_clock_drift_mps,12.5);
    EXPECT_EQ(out.clock_components_m[1],f.result.epoch_clock_bias_components_m[1]);
    EXPECT_LT((out.velocity_ecef_mps[1]-Vector3d(4,2,3)).norm(),1e-12);
    EXPECT_LT((out.attitude_body_to_nav[1]*Vector3d::UnitX()-Vector3d::UnitY()).norm(),1e-12);
    EXPECT_EQ(out.observations[1].receiver_position,f.result.solution.solutions[1].position_ecef);
}

TEST(NativeImuRefinementHandoff, RejectsEpochMismatchWithoutNearestTimeOrMissingStateFill) {
    { Fixture f; f.problem.epochs[1].raw_utc_time_millis++; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.solution.solutions[1].time = f.raw[1].time + 0.001; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.raw[1].raw_source_index=0; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.raw[1].raw_utc_time_millis=f.raw[0].raw_utc_time_millis;
      f.problem.epochs[1].raw_utc_time_millis=f.raw[1].raw_utc_time_millis; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_velocity_nav_mps.pop_back(); EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_attitude_rpy_rad.clear(); EXPECT_THROW(f.handoff(),std::invalid_argument); }
}

TEST(NativeImuRefinementHandoff, RejectsInvalidSolveFrameAndState) {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    { Fixture f; f.result.diagnostics.converged=false; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.problem.imu.valid=false; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.problem.imu.nav_origin_lat_rad=90; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_clock_bias_components_m[1][6]=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_clock_drift_mps[1]=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_velocity_nav_mps[1].x()=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.epoch_attitude_rpy_rad[1].x()=nan; EXPECT_THROW(f.handoff(),std::invalid_argument); }
    { Fixture f; f.result.solution.solutions[1].position_ecef.setZero(); EXPECT_THROW(f.handoff(),std::invalid_argument); }
}

TEST(NativeImuRefinementHandoff, DopplerMaskUsesOptimizedVelocityAndRetainsExactBoundary) {
    Fixture f;
    auto input=f.handoff();
    auto geometry=f.problem;
    for (std::size_t i=0;i<2;++i) {
        input.observations[i].observations[0].doppler=100;
        FGOProcessor::UndifferencedDopplerFactor row;
        row.epoch_index=i;
        row.satellite=input.observations[i].observations[0].satellite;
        row.signal=input.observations[i].observations[0].signal;
        row.los=Vector3d::UnitX();
        row.residual_mps=4+12.5+3+(i ? 0.001 : 0.0);
        geometry.undifferenced_doppler_factors.push_back(row);
    }
    const auto report=native_imu_refinement::maskDoppler(input,geometry,1.0);
    EXPECT_EQ(report.retained,1U); EXPECT_EQ(report.rejected,1U);
    EXPECT_DOUBLE_EQ(input.observations[0].observations[0].doppler,100);
    EXPECT_TRUE(std::isnan(input.observations[1].observations[0].doppler));
    EXPECT_DOUBLE_EQ(input.observations[1].observations[0].pseudorange,21000001);
    // A geometry/ephemeris miss remains a miss; it is not supplied a fake D row.
    geometry.undifferenced_doppler_factors.clear();
    const auto missing=native_imu_refinement::maskDoppler(input,geometry,1.0);
    EXPECT_EQ(missing.retained,0U);
}

TEST(NativeImuRefinementHandoff, DopplerMaskRejectsBrokenGeometryBeforeMutation) {
    Fixture f; auto input=f.handoff(); auto geometry=f.problem;
    input.observations[0].observations[0].doppler=100;
    FGOProcessor::UndifferencedDopplerFactor row;
    row.epoch_index=0; row.satellite=input.observations[0].observations[0].satellite;
    row.signal=input.observations[0].observations[0].signal;
    row.los=Vector3d::UnitX(); row.residual_mps=16.5;
    geometry.undifferenced_doppler_factors={row,row};
    EXPECT_THROW(native_imu_refinement::maskDoppler(input,geometry,1),std::invalid_argument);
    EXPECT_DOUBLE_EQ(input.observations[0].observations[0].doppler,100);
    geometry.undifferenced_doppler_factors={row};
    geometry.epochs[0].raw_utc_time_millis++;
    EXPECT_THROW(native_imu_refinement::maskDoppler(input,geometry,1),std::invalid_argument);
    EXPECT_THROW(native_imu_refinement::maskDoppler(input,geometry,0),std::invalid_argument);
}

TEST(NativeImuRefinementHandoff, InitialDopplerThresholdPreservesTwentyMetreBoundary) {
    Fixture f; auto input=f.handoff(); auto geometry=f.problem;
    for (std::size_t i=0;i<2;++i) {
        input.observations[i].observations[0].doppler=100;
        FGOProcessor::UndifferencedDopplerFactor row;
        row.epoch_index=i; row.satellite=input.observations[i].observations[0].satellite;
        row.signal=input.observations[i].observations[0].signal;
        row.los=Vector3d::UnitX(); row.residual_mps=4+12.5+20+(i?0.001:0);
        geometry.undifferenced_doppler_factors.push_back(row);
    }
    const auto threshold=nativeImuDopplerResidualThreshold(NativeImuObservationPhase::Initialization);
    auto final_input=input;
    const auto initial=native_imu_refinement::maskDoppler(input,geometry,1,threshold);
    EXPECT_EQ(initial.retained,1U); EXPECT_EQ(initial.rejected,1U);
    const auto final=native_imu_refinement::maskDoppler(final_input,geometry,1,
        nativeImuDopplerResidualThreshold(NativeImuObservationPhase::Final));
    EXPECT_EQ(final.retained,0U);
    EXPECT_THROW(native_imu_refinement::maskDoppler(input,geometry,1,0),std::invalid_argument);
    EXPECT_THROW(nativeImuDopplerResidualThreshold(static_cast<NativeImuObservationPhase>(99)),std::invalid_argument);
    EXPECT_DOUBLE_EQ(nativeImuCodeResidualThreshold(NativeImuObservationPhase::Initialization,false),30);
    EXPECT_DOUBLE_EQ(nativeImuCodeResidualThreshold(NativeImuObservationPhase::Final,false),15);
}
