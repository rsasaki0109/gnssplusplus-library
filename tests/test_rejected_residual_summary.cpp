#include <gtest/gtest.h>
#include <libgnss++/algorithms/rejected_residual_summary.hpp>
#include <limits>

TEST(RejectedResidualSummary, SignsAndNonfiniteAreSeparate) {
    libgnss::RejectedResidualSummary summary;
    for (double x : {21., -30., 0., std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) summary.observe(x);
    EXPECT_EQ(summary.positive,1U);
    EXPECT_EQ(summary.negative,1U);
    EXPECT_EQ(summary.zero,1U);
    EXPECT_EQ(summary.nonfinite,2U);
    EXPECT_DOUBLE_EQ(summary.max_abs_m,30.);
}
