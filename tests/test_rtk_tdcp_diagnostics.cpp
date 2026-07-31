#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>

namespace libgnss::rtk_tdcp_diagnostics {
namespace {

TEST(RTKTdcpDiagnosticsTest, TrapezoidIntegratedDopplerMatchesPhaseChange) {
    const auto result = evaluate(10.0, 10.21, 1.0, 1.1, 0.2, false);
    EXPECT_EQ(result.status, Status::VALID);
    EXPECT_NEAR(result.residual_m, 0.0, 1e-12);
}

TEST(RTKTdcpDiagnosticsTest, ReportsSignedResidual) {
    const auto result = evaluate(10.0, 10.25, 1.0, 1.0, 0.2, false);
    EXPECT_EQ(result.status, Status::VALID);
    EXPECT_NEAR(result.residual_m, 0.05, 1e-12);
}

TEST(RTKTdcpDiagnosticsTest, RejectsGapAndLossOfLock) {
    EXPECT_EQ(evaluate(1.0, 2.0, 1.0, 1.0, 2.1, false).status,
              Status::INVALID_GAP);
    EXPECT_EQ(evaluate(1.0, 2.0, 1.0, 1.0, 1.0, true).status,
              Status::LOSS_OF_LOCK);
}

TEST(RTKTdcpDiagnosticsTest, RejectsNonFiniteInput) {
    EXPECT_EQ(evaluate(1.0, std::numeric_limits<double>::quiet_NaN(),
                       1.0, 1.0, 1.0, false).status,
              Status::INVALID_INPUT);
}

}  // namespace
}  // namespace libgnss::rtk_tdcp_diagnostics
