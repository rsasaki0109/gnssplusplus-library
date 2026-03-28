#pragma once

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace rtk_measurement {

Eigen::MatrixXd buildDoubleDifferenceCovariance(const std::vector<int>& block_sizes,
                                                const std::vector<double>& reference_variances,
                                                const std::vector<double>& satellite_variances,
                                                int observation_count);

int suppressOutlierRows(Eigen::VectorXd& residuals,
                        Eigen::MatrixXd& design_matrix,
                        double threshold);

}  // namespace rtk_measurement
}  // namespace libgnss
