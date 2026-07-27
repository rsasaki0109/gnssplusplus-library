#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/core/types.hpp>

namespace libgnss {

/**
 * @brief Single IMU sample (accelerometer + gyroscope), raw sensor axes.
 *
 * Mirrors the field shape of Observation/ObservationData (observation.hpp)
 * for interface consistency: a GNSSTime timestamp plus payload vectors.
 */
struct ImuSample {
    GNSSTime time;                    ///< GPS week + time-of-week, seconds
    Eigen::Vector3d accel_raw = Eigen::Vector3d::Zero();       ///< m/s^2, raw sensor axes
    Eigen::Vector3d gyro_raw_radps = Eigen::Vector3d::Zero();  ///< rad/s (deg/s converted at load), raw sensor axes
};

/**
 * @brief Time series of IMU samples.
 *
 * Same shape as ObservationSeries (observation.hpp) for interface
 * consistency: a flat, time-sorted vector plus small helpers.
 */
class ImuSeries {
public:
    std::vector<ImuSample> samples;

    /** @brief Get samples in [start, end] (inclusive), assuming sorted input. */
    std::vector<ImuSample> getSamples(const GNSSTime& start, const GNSSTime& end) const;

    /** @brief Number of samples. */
    size_t size() const { return samples.size(); }

    /** @brief Whether the series has no samples. */
    bool isEmpty() const { return samples.empty(); }

    /** @brief Sort samples by time (stable, ascending). */
    void sortByTime();

    /**
     * @brief Shift every sample time by offset_s (navi.776 C: constant
     * GNSS-IMU time-offset application). Positive = IMU timestamps move
     * later. GPS week rollover is normalized via GNSSTime's own +/-
     * operators. Zero is an exact no-op.
     */
    void shiftTime(double offset_s);
};

/**
 * @brief Explicit, dataset-provided raw-axis -> body-FLU remapping.
 *
 * Mirrors run_ppc_imu_dropout_bridge_matrix.py's --forward-axis/--lateral-axis/
 * --forward-sign/--lateral-sign CLI parameters, generalized from 2D
 * (forward/lateral) to a full 3-axis body remapping. Each entry says: "body
 * axis X (Forward/Left/Up) comes from raw column <src>, sign <s>". Default
 * construction is the identity mapping (raw axis order already FLU).
 */
struct ImuAxisConvention {
    int forward_source_axis = 0;   ///< 0=x, 1=y, 2=z of the raw accel/gyro vector
    double forward_sign = 1.0;
    int left_source_axis = 1;
    double left_sign = 1.0;
    int up_source_axis = 2;
    double up_sign = 1.0;

    /** @brief Remap a raw sensor-axis vector into body FLU (Forward, Left, Up). */
    Eigen::Vector3d apply(const Eigen::Vector3d& raw) const {
        return Eigen::Vector3d(forward_sign * raw(forward_source_axis),
                                left_sign * raw(left_source_axis),
                                up_sign * raw(up_source_axis));
    }
};

/**
 * @brief Result of loadImuCsv(): success flag, row count, and the resolved
 * header-name-to-column mapping (for diagnostics/logging).
 */
struct ImuCsvLoadResult {
    bool ok = false;
    std::string error;
    int row_count = 0;
    std::string time_column;
    std::string week_column;
    std::array<std::string, 3> accel_columns;
    std::array<std::string, 3> gyro_columns;
};

/**
 * @brief Load a PPC-Dataset-style imu.csv file via header-name matching.
 *
 * Direct C++ port of scripts/analyze_ppc_imu_coverage.py's
 * normalize_header()/field_lookup()/find_column() candidate-list matching:
 * header names are stripped of whitespace/case/punctuation before matching
 * against a fixed candidate list per logical field (time, week, accel XYZ,
 * gyro XYZ). Gyro values are converted from deg/s (as stored in the file) to
 * rad/s once, here, at load time. Populates `out.samples` with RAW sensor
 * axes only -- axis remapping to a body convention is a separate, explicit
 * step (ImuAxisConvention::apply), never silently assumed.
 *
 * @param path  Path to the imu.csv file
 * @param out   Loaded series (raw accel m/s^2, raw gyro rad/s)
 * @return      Load result: ok/error, resolved column names, row count
 */
ImuCsvLoadResult loadImuCsv(const std::string& path, ImuSeries& out);

/**
 * @brief Load rtklibexplorer/GNSS_IMU's cleaned `imu_*_sf.csv` format.
 *
 * Column 0 is a GPST-referenced Unix-epoch timestamp, accelerometer columns
 * are in g, and gyro columns are already rad/s. This loader converts them to
 * GNSSTime, m/s^2, and rad/s respectively but deliberately leaves the raw
 * sensor axes untouched; mounting rotation and FRD/FLU conversion remain an
 * explicit caller responsibility.
 */
ImuCsvLoadResult loadRtklibExplorerImuCsv(const std::string& path, ImuSeries& out);

}  // namespace libgnss
