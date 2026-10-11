#include <gtest/gtest.h>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/solution.hpp>

#include <cmath>

using namespace libgnss;

namespace {

// Tokyo (35.67 N, 139.79 E): the ECEF axes are far from the local ENU axes,
// so reading ECEF XX/YY/ZZ as E/N/U is visibly wrong there.
constexpr double kLatRad = 35.67 * M_PI / 180.0;
constexpr double kLonRad = 139.79 * M_PI / 180.0;

Matrix3d enuToEcefRotation() {
    Matrix3d ecef_to_enu;
    ecef_to_enu.col(0) = ecef2enu(Vector3d::UnitX(), kLatRad, kLonRad);
    ecef_to_enu.col(1) = ecef2enu(Vector3d::UnitY(), kLatRad, kLonRad);
    ecef_to_enu.col(2) = ecef2enu(Vector3d::UnitZ(), kLatRad, kLonRad);
    return ecef_to_enu.transpose();
}

PositionSolution solutionWithEnuSigmas(double sigma_e, double sigma_n, double sigma_u) {
    PositionSolution solution;
    solution.position_ecef = geodetic2ecef(kLatRad, kLonRad, 40.0);
    const Matrix3d enu = Vector3d(sigma_e * sigma_e, sigma_n * sigma_n, sigma_u * sigma_u)
                             .asDiagonal();
    const Matrix3d r = enuToEcefRotation();
    solution.position_covariance = r * enu * r.transpose();
    return solution;
}

}  // namespace

TEST(SolutionCovarianceFrame, EnuCovarianceRecoversLocalSigmas) {
    const PositionSolution solution = solutionWithEnuSigmas(0.02, 0.03, 0.10);
    const Matrix3d enu = solution.positionCovarianceEnu();
    EXPECT_NEAR(enu(0, 0), 0.02 * 0.02, 1e-12);
    EXPECT_NEAR(enu(1, 1), 0.03 * 0.03, 1e-12);
    EXPECT_NEAR(enu(2, 2), 0.10 * 0.10, 1e-12);
    EXPECT_NEAR(enu(0, 1), 0.0, 1e-12);
    EXPECT_NEAR(enu(1, 2), 0.0, 1e-12);
    // The ECEF diagonal is not the local one at this latitude.
    EXPECT_GT(std::abs(solution.position_covariance(2, 2) - enu(2, 2)), 1e-3);
}

TEST(SolutionCovarianceFrame, AccuracyHelpersUseTheLocalFrame) {
    const PositionSolution solution = solutionWithEnuSigmas(0.02, 0.03, 0.10);
    EXPECT_NEAR(solution.getHorizontalAccuracy(),
                2.0 * std::sqrt(0.02 * 0.02 + 0.03 * 0.03), 1e-9);
    EXPECT_NEAR(solution.getVerticalAccuracy(), 2.0 * 0.10, 1e-9);
    EXPECT_NEAR(solution.get3DAccuracy(),
                2.0 * std::sqrt(0.02 * 0.02 + 0.03 * 0.03 + 0.10 * 0.10), 1e-9);
}

TEST(SolutionCovarianceFrame, MissingCovarianceStaysNaN) {
    PositionSolution solution;
    solution.position_ecef = geodetic2ecef(kLatRad, kLonRad, 40.0);
    EXPECT_FALSE(solution.positionCovarianceEnu().allFinite());
    EXPECT_TRUE(std::isnan(solution.getHorizontalAccuracy()));

    PositionSolution no_position;
    no_position.position_covariance = Matrix3d::Identity();
    EXPECT_FALSE(no_position.positionCovarianceEnu().allFinite());
}
