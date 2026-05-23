#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_parity.hpp>

namespace libgnss::algorithms::madoca_parity {
namespace {

TEST(MadocaParitySkeleton, OracleToleranceIsDefined) {
    EXPECT_DOUBLE_EQ(kOracleTolerance, 1e-6);
}

}  // namespace
}  // namespace libgnss::algorithms::madoca_parity
