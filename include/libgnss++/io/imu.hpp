#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
    // Android-only provenance. A negative value means that the source did
    // not expose a monotonic elapsedRealtimeNanos clock. Keeping this value
    // alongside the converted GPST timestamp lets a later GNSS-anchor sync
    // stage use the original clock without reverse-engineering it.
    std::int64_t elapsed_realtime_nanos = -1;
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
 * @brief One raw Android GNSS clock anchor used by the IMU synchronizer.
 *
 * The pair is deliberately kept in the original integer domains.  The
 * upstream MATLAB path interpolates UTC milliseconds as a function of
 * ``ChipsetElapsedRealtimeNanos``; converting either clock to a rounded
 * floating-point timestamp before interpolation would lose that contract.
 */
struct AndroidGnssTimeAnchor {
    std::int64_t utc_time_ms = 0;
    std::int64_t elapsed_realtime_nanos = 0;
};

/**
 * @brief Diagnostics returned by loadAndroidGnssTimeAnchors().
 */
struct AndroidGnssTimeAnchorLoadResult {
    bool ok = false;
    std::string error;
    std::size_t input_rows = 0;
    std::size_t raw_rows = 0;
    std::size_t unsupported_rows = 0;
    std::size_t duplicate_utc_timestamps = 0;
    std::size_t unique_anchors = 0;
    std::int64_t first_utc_time_ms = 0;
    std::int64_t last_utc_time_ms = 0;
    std::int64_t first_elapsed_realtime_nanos = 0;
    std::int64_t last_elapsed_realtime_nanos = 0;
};

/**
 * @brief Extract the raw GNSS UTC/elapsed clock anchors from device_gnss.csv.
 *
 * Only the raw ``MessageType=Raw`` rows and the two clock columns are read;
 * no position, truth, result, or sample coordinates are accepted.  One
 * anchor is retained per UTC epoch, matching the upstream unique() contract.
 * The returned elapsed clock must be strictly increasing so linear
 * interpolation/extrapolation is deterministic and fail-closed.
 */
AndroidGnssTimeAnchorLoadResult loadAndroidGnssTimeAnchors(
    const std::string& path,
    std::vector<AndroidGnssTimeAnchor>& anchors);

/**
 * @brief Contract for loading the raw Android ``device_imu.csv`` streams.
 *
 * Android timestamps are retained as two distinct clocks: ``utcTimeMillis``
 * identifies the GNSS epoch and ``elapsedRealtimeNanos`` is the monotonic
 * clock used to align accelerometer samples to gyro anchors.  The raw
 * UncalGyro measurements are already radians/second and UncalAccel is
 * metres/second^2; neither is silently re-scaled.  The GPS-UTC offset is
 * explicit because the GSDC 2023 data is UTC and the native time type is
 * GPST.  Alignment never extrapolates: interior acceleration samples are
 * linearly interpolated in elapsed time, while an endpoint uses a nearest
 * sample only when it is within the fixed bound.
 */
struct AndroidImuCsvConfig {
    double gps_utc_leap_seconds = 18.0;
    double maximum_accel_pair_offset_s = 0.025;
    bool allow_endpoint_nearest = true;
    // Raw IMU inference must opt into the GNSS elapsed->UTC synchronization
    // contract.  The low-level parser keeps a legacy direct-UTC mode for
    // isolated parser callers; the native raw entrypoint sets this true and
    // fails closed when anchors are absent or invalid.
    bool require_gnss_elapsed_anchor = false;
    double imu_sync_coefficient = 0.5;
};

/**
 * @brief Diagnostics returned by loadAndroidImuCsv().
 */
struct AndroidImuCsvLoadResult {
    bool ok = false;
    std::string error;
    std::size_t total_rows = 0;
    std::size_t accel_rows = 0;
    std::size_t gyro_rows = 0;
    std::size_t unsupported_rows = 0;
    std::size_t duplicate_accel_timestamps = 0;
    std::size_t duplicate_gyro_timestamps = 0;
    std::size_t paired_rows = 0;
    std::size_t exact_elapsed_matches = 0;
    std::size_t interpolated_rows = 0;
    std::size_t endpoint_nearest_rows = 0;
    std::size_t omitted_rows = 0;
    std::int64_t first_gyro_utc_ms = 0;
    std::int64_t last_gyro_utc_ms = 0;
    std::int64_t first_gyro_elapsed_ns = 0;
    std::int64_t last_gyro_elapsed_ns = 0;
    double median_abs_pair_offset_ms = 0.0;
    double maximum_abs_pair_offset_ms = 0.0;
    std::size_t gnss_anchor_points = 0;
    std::size_t gnss_anchor_exact_rows = 0;
    std::size_t gnss_anchor_interpolated_rows = 0;
    std::size_t gnss_anchor_extrapolated_rows = 0;
    double first_mapped_utc_time_ms = 0.0;
    double last_mapped_utc_time_ms = 0.0;
    double imu_sync_coefficient = 0.5;
    double first_dt_s = 0.0;
    double last_dt_s = 0.0;
    bool dt_tail_repeated = false;
    bool elapsed_clock_preserved = false;
    // True only when the supplied GNSS elapsed->UTC anchors were used for
    // every synchronized IMU timestamp.  Direct UTC conversion is never
    // reported as an anchored raw inference.
    bool gnss_elapsed_anchor_applied = false;
};

/**
 * @brief Load a PPC-Dataset-style imu.csv file via header-name matching.
 *
 * Direct C++ port of scripts/analysis/analyze_ppc_imu_coverage.py's
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
 * @brief Load and align raw Android UncalAccel/UncalGyro measurements.
 *
 * This is a raw-only adapter for the GSDC Android schema.  It mirrors the
 * upstream gyro-anchored synchronization shape while keeping the native path
 * fail-closed at file boundaries: duplicate UTC timestamps retain the first
 * row (the MATLAB ``unique`` behavior), acceleration is interpolated on the
 * common elapsedRealtimeNanos clock, and unbracketed samples are only
 * nearest-held within AndroidImuCsvConfig::maximum_accel_pair_offset_s.
 * Output samples contain the gyro UTC time converted to GPST using the
 * supplied GNSS elapsed->UTC anchors when the opt-in requirement is enabled,
 * the preserved elapsedRealtimeNanos anchor, and raw sensor axes; mounting is
 * an explicit caller operation via ImuAxisConvention or a documented matrix.
 * The anchor path mirrors MATLAB interp1(...,"linear","extrap") and reports
 * exact/interpolated/extrapolated rows plus the frozen sync coefficient.
 */
AndroidImuCsvLoadResult loadAndroidImuCsv(
    const std::string& path,
    ImuSeries& out,
    const AndroidImuCsvConfig& config = {},
    const std::vector<AndroidGnssTimeAnchor>& gnss_time_anchors = {});

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
