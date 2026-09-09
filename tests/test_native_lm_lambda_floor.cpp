#include <gtest/gtest.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <libgnss++/algorithms/native_lm_lambda_floor.hpp>

TEST(NativeLmFloor, DisabledUnchangedEnabledChangesOnlyLowerBound) {
    gtsam::LevenbergMarquardtParams p;
    const auto before = p;
    libgnss::applyNativeLmLambdaFloor(p, false, false);
    EXPECT_DOUBLE_EQ(p.lambdaLowerBound, before.lambdaLowerBound);
    EXPECT_THROW(libgnss::applyNativeLmLambdaFloor(p, true, false), std::invalid_argument);
    libgnss::applyNativeLmLambdaFloor(p, true, true);
    EXPECT_DOUBLE_EQ(p.lambdaLowerBound, 1e-8);
    EXPECT_DOUBLE_EQ(p.lambdaInitial, before.lambdaInitial);
    EXPECT_DOUBLE_EQ(p.lambdaUpperBound, before.lambdaUpperBound);
    EXPECT_DOUBLE_EQ(p.lambdaFactor, before.lambdaFactor);
    EXPECT_EQ(p.getLinearSolverType(), before.getLinearSolverType());
    EXPECT_EQ(p.getDiagonalDamping(), before.getDiagonalDamping());
    p.lambdaInitial = 0.;
    EXPECT_THROW(libgnss::applyNativeLmLambdaFloor(p, true, true), std::invalid_argument);
}
