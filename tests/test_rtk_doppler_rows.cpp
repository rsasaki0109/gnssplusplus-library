#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <vector>

// navi.776 B2: unit tests for the between-satellite SD Doppler observation
// model. The rows are built by RTKProcessor::buildMeasurementBlocks (needs
// live RINEX data), so these tests exercise the same math contract on
// hand-built blocks: H touches only the velocity tail states, the common
// receiver clock drift cancels in the SD residual, and a Kalman update over
// the assembled system recovers a known velocity.

namespace libgnss {
namespace {

using rtk_measurement::MeasurementBlock;
using rtk_measurement::MeasurementKind;
using rtk_measurement::MeasurementRow;

// Simple synthetic geometry: unit LOS vectors spread over the sky.
std::vector<Eigen::Vector3d> syntheticLosVectors() {
    std::vector<Eigen::Vector3d> los = {
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.6, 0.0, 0.8),
        Eigen::Vector3d(0.0, 0.6, 0.8),
        Eigen::Vector3d(-0.6, 0.0, 0.8),
        Eigen::Vector3d(0.0, -0.6, 0.8),
        Eigen::Vector3d(0.42, 0.42, 0.8),
        Eigen::Vector3d(-0.42, 0.42, 0.8),
    };
    for (auto& e : los) e.normalize();
    return los;
}

// Builds the SD Doppler block exactly the way buildMeasurementBlocks does:
// satellite 0 is the reference; residual = (rr_obs_s - rr_pred_s) -
// (rr_obs_ref - rr_pred_ref) evaluated at v_hat; H columns on the velocity
// tail get e_ref - e_s. rr terms follow d(rr)/d(v_rx) = -e.
MeasurementBlock buildSyntheticDopplerBlock(const std::vector<Eigen::Vector3d>& los,
                                            const Eigen::Vector3d& true_velocity,
                                            const Eigen::Vector3d& velocity_estimate,
                                            double receiver_clock_drift_mps,
                                            int velocity_state_index) {
    MeasurementBlock block;
    block.kind = MeasurementKind::DOPPLER;
    block.frequency_index = 0;

    const Eigen::Vector3d& e_ref = los[0];
    // Observed range rate contains the true velocity and the receiver clock
    // drift; the prediction is evaluated at the current estimate and has no
    // drift term (it cancels in the SD).
    const double rr_obs_ref = -e_ref.dot(true_velocity) + receiver_clock_drift_mps;
    const double rr_pred_ref = -e_ref.dot(velocity_estimate);

    for (size_t s = 1; s < los.size(); ++s) {
        const Eigen::Vector3d& e_sat = los[s];
        const double rr_obs_sat = -e_sat.dot(true_velocity) + receiver_clock_drift_mps;
        const double rr_pred_sat = -e_sat.dot(velocity_estimate);

        MeasurementRow row;
        row.residual = (rr_obs_sat - rr_pred_sat) - (rr_obs_ref - rr_pred_ref);
        for (int axis = 0; axis < 3; ++axis) {
            row.state_coefficients.push_back(
                {velocity_state_index + axis, e_ref(axis) - e_sat(axis)});
        }
        row.reference_variance = 0.04;
        row.satellite_variance = 0.04;
        block.rows.push_back(std::move(row));
    }
    return block;
}

TEST(RTKDopplerRowTest, RowsTouchOnlyVelocityTailStates) {
    const auto los = syntheticLosVectors();
    constexpr int kStates = 10;
    constexpr int kVelocityIndex = 7;
    const auto block = buildSyntheticDopplerBlock(
        los, Eigen::Vector3d(1.0, -2.0, 0.5), Eigen::Vector3d::Zero(), 0.0, kVelocityIndex);
    const auto system = rtk_measurement::assembleMeasurementSystem({block}, kStates);

    ASSERT_EQ(system.design_matrix.rows(), static_cast<int>(los.size()) - 1);
    for (int row = 0; row < system.design_matrix.rows(); ++row) {
        for (int col = 0; col < kVelocityIndex; ++col) {
            EXPECT_EQ(system.design_matrix(row, col), 0.0)
                << "row " << row << " col " << col;
        }
        EXPECT_NE(system.design_matrix.row(row).segment<3>(kVelocityIndex).norm(), 0.0);
    }
}

TEST(RTKDopplerRowTest, CommonReceiverClockDriftCancels) {
    const auto los = syntheticLosVectors();
    const Eigen::Vector3d v_true(3.0, -1.0, 0.2);
    const Eigen::Vector3d v_hat(2.5, -0.5, 0.0);

    const auto block_no_drift =
        buildSyntheticDopplerBlock(los, v_true, v_hat, 0.0, 3);
    const auto block_with_drift =
        buildSyntheticDopplerBlock(los, v_true, v_hat, 123.456, 3);

    ASSERT_EQ(block_no_drift.rows.size(), block_with_drift.rows.size());
    for (size_t i = 0; i < block_no_drift.rows.size(); ++i) {
        EXPECT_NEAR(block_no_drift.rows[i].residual,
                    block_with_drift.rows[i].residual, 1e-12);
    }
}

TEST(RTKDopplerRowTest, SyntheticVelocityRecoveredThroughKalmanUpdate) {
    const auto los = syntheticLosVectors();
    const Eigen::Vector3d v_true(4.0, -2.0, 0.3);
    const Eigen::Vector3d v_hat(0.0, 0.0, 0.0);  // zero estimate: error = v_true
    constexpr int kStates = 6;
    constexpr int kVelocityIndex = 3;

    const auto block =
        buildSyntheticDopplerBlock(los, v_true, v_hat, 42.0, kVelocityIndex);
    auto system = rtk_measurement::assembleMeasurementSystem({block}, kStates);

    Eigen::VectorXd state = Eigen::VectorXd::Zero(kStates);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(kStates, kStates);
    // Only the velocity states carry covariance; the head states are
    // inactive (zero state + zero covariance), like a KINEMATIC baseline
    // with the position block untouched by Doppler rows.
    covariance.block<3, 3>(kVelocityIndex, kVelocityIndex) =
        100.0 * Eigen::Matrix3d::Identity();

    std::vector<bool> force_active(kStates, false);
    for (int i = 0; i < 3; ++i) force_active[kVelocityIndex + i] = true;

    const auto result = rtk_update::applyMeasurementUpdate(
        state, covariance, system, 30.0, 3, 0.0, force_active);

    ASSERT_TRUE(result.ok);
    const Eigen::Vector3d recovered = state.segment<3>(kVelocityIndex);
    EXPECT_NEAR((recovered - v_true).norm(), 0.0, 0.05)
        << "recovered " << recovered.transpose();
    // Head states must be untouched.
    EXPECT_EQ(state.head<3>().norm(), 0.0);
}

TEST(RTKDopplerRowTest, ZeroVelocityErrorYieldsZeroResiduals) {
    const auto los = syntheticLosVectors();
    const Eigen::Vector3d v(1.5, 2.5, -0.5);
    const auto block = buildSyntheticDopplerBlock(los, v, v, 7.0, 3);
    for (const auto& row : block.rows) {
        EXPECT_NEAR(row.residual, 0.0, 1e-12);
    }
}

}  // namespace
}  // namespace libgnss
