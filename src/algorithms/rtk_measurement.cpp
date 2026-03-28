#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <cmath>

namespace libgnss {
namespace rtk_measurement {

Eigen::MatrixXd buildDoubleDifferenceCovariance(const std::vector<int>& block_sizes,
                                                const std::vector<double>& reference_variances,
                                                const std::vector<double>& satellite_variances,
                                                int observation_count) {
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(observation_count, observation_count);
    int offset = 0;
    for (size_t block = 0; block < block_sizes.size(); ++block) {
        for (int row = 0; row < block_sizes[block]; ++row) {
            for (int col = 0; col < block_sizes[block]; ++col) {
                covariance(offset + row, offset + col) = reference_variances[offset + row];
                if (row == col) {
                    covariance(offset + row, offset + col) += satellite_variances[offset + row];
                }
            }
        }
        offset += block_sizes[block];
    }
    return covariance;
}

int suppressOutlierRows(Eigen::VectorXd& residuals,
                        Eigen::MatrixXd& design_matrix,
                        double threshold) {
    int suppressed = 0;
    for (int row = 0; row < residuals.size(); ++row) {
        if (std::abs(residuals(row)) > threshold) {
            residuals(row) = 0.0;
            design_matrix.row(row).setZero();
            ++suppressed;
        }
    }
    return suppressed;
}

}  // namespace rtk_measurement
}  // namespace libgnss
