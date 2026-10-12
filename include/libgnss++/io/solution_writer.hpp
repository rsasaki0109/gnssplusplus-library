#pragma once

/**
 * @file solution_writer.hpp
 * @brief Streaming solution output writer for GNSS positioning results.
 *
 * Provides epoch-by-epoch solution file output, complementing the batch
 * Solution::writeToFile() method for real-time or large-dataset use cases.
 */

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/solution.hpp>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <cmath>

namespace libgnss {
namespace io {

/**
 * @brief Streaming position solution writer.
 *
 * Opens an output file, writes a header line, and appends one line per
 * epoch as solutions arrive.  The file is flushed after each epoch so
 * that partial results are always available on disk.
 *
 * Usage:
 * @code
 *   libgnss::io::SolutionWriter writer;
 *   writer.open("output.pos");
 *   while (...) {
 *       auto sol = rtk.process(...);
 *       writer.writeEpoch(sol);
 *   }
 *   writer.close();
 * @endcode
 */
class SolutionWriter {
public:
    /**
     * @brief Output format for the solution file.
     */
    enum class Format {
        POS,   ///< LibGNSS++ .pos format (GPS week, TOW, ECEF, LLH, status)
        LLH,   ///< Latitude, longitude, height only
        XYZ    ///< ECEF XYZ only
    };

    SolutionWriter() = default;

    explicit SolutionWriter(const std::string& filename, Format fmt = Format::POS,
                            bool uncertainty_columns = false,
                            bool fix_diagnostic_columns = false) {
        open(filename, fmt, uncertainty_columns, fix_diagnostic_columns);
    }

    ~SolutionWriter() { close(); }

    // Non-copyable
    SolutionWriter(const SolutionWriter&) = delete;
    SolutionWriter& operator=(const SolutionWriter&) = delete;

    // Movable
    SolutionWriter(SolutionWriter&&) = default;
    SolutionWriter& operator=(SolutionWriter&&) = default;

    /**
     * @brief Open output file.
     * @param filename Path to the output file.
     * @param fmt      Output format (default: POS).
     * @param uncertainty_columns Append the per-epoch uncertainty columns
     *        (see kUncertaintyHeaderColumns) to POS rows and the POS header.
     *        Ignored for LLH/XYZ. Default false keeps the output byte-identical
     *        to earlier releases.
     * @param fix_diagnostic_columns Append the float-fix distance columns
     *        (see kFixDiagnosticHeaderColumns) after any other columns.
     *        Ignored for LLH/XYZ. Default false keeps the output
     *        byte-identical to earlier releases.
     * @return true if the file was opened successfully.
     */
    bool open(const std::string& filename, Format fmt = Format::POS,
              bool uncertainty_columns = false,
              bool fix_diagnostic_columns = false) {
        close();
        format_ = fmt;
        uncertainty_columns_ = uncertainty_columns && fmt == Format::POS;
        fix_diagnostic_columns_ = fix_diagnostic_columns && fmt == Format::POS;
        file_.open(filename);
        if (!file_.is_open()) return false;
        writeHeader();
        return true;
    }

    /**
     * @brief Check whether the writer has an open file.
     */
    bool isOpen() const { return file_.is_open(); }

    /**
     * @brief Write the file header.
     *
     * Called automatically by open(); exposed publicly for cases where
     * the caller wants to re-write the header mid-file.
     */
    void writeHeader() {
        if (!file_.is_open()) return;
        switch (format_) {
        case Format::POS:
            file_ << "% LibGNSS++ Position Solution\n"
                  << "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) "
                     "Status NumSat PDOP Ratio Baseline(m) RTKIter RTKObs RTKPhaseObs "
                     "RTKCodeObs RTKOutliers RTKPrefitRMS(m) RTKPrefitMax(m) "
                     "RTKPostSuppressRMS(m) RTKPostSuppressMax(m) RTKUpdateNIS "
                     "RTKUpdateNISPerObs RTKUpdateNISRejected";
            if (uncertainty_columns_) {
                file_ << ' ' << kUncertaintyHeaderColumns;
            }
            if (fix_diagnostic_columns_) {
                file_ << ' ' << kFixDiagnosticHeaderColumns;
            }
            file_ << "\n";
            break;
        case Format::LLH:
            file_ << "% GPS_Week GPS_TOW Lat(deg) Lon(deg) Height(m) Status\n";
            break;
        case Format::XYZ:
            file_ << "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Status\n";
            break;
        }
        file_.flush();
    }

    /// Names of the columns appended by the uncertainty option (POS only).
    static constexpr const char* kUncertaintyHeaderColumns =
        "sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) DiffAge(s)";

    /// Names of the columns appended by the fix-diagnostics option (POS only).
    static constexpr const char* kFixDiagnosticHeaderColumns = "FloatFixH(m) FloatFixU(m)";

    /**
     * @brief Format the float-fix distance columns for one epoch (each preceded by a space).
     *
     * FloatFixH is the horizontal distance and FloatFixU the absolute vertical
     * difference, in the local ENU frame at the FIXED position, between the
     * epoch's float position (computed before ambiguity resolution, recorded
     * by RTKProcessor on FIXED solutions) and the FIXED position. Both are
     * "nan" for any other status or when the float reference is unavailable.
     * A diagnostic only, not an integrity bound.
     */
    static std::string formatFixDiagnosticColumns(const PositionSolution& sol) {
        double d_h = std::numeric_limits<double>::quiet_NaN();
        double d_u = d_h;
        if (sol.status == SolutionStatus::FIXED && sol.rtk_float_reference_valid &&
            sol.rtk_float_position_ecef.allFinite() && sol.position_ecef.allFinite() &&
            sol.position_ecef.norm() > 1e6) {
            double lat = 0.0, lon = 0.0, h = 0.0;
            ecef2geodetic(sol.position_ecef, lat, lon, h);
            const Vector3d diff =
                ecef2enu(sol.rtk_float_position_ecef - sol.position_ecef, lat, lon);
            d_h = std::hypot(diff.x(), diff.y());
            d_u = std::abs(diff.z());
        }
        std::ostringstream out;
        out << std::fixed << std::setprecision(5);
        for (const double value : {d_h, d_u}) {
            out << ' ';
            if (std::isfinite(value)) {
                out << value;
            } else {
                out << "nan";
            }
        }
        return out.str();
    }

    /**
     * @brief Format the uncertainty columns for one epoch (each preceded by a space).
     *
     * sdn/sde/sdu are 1-sigma standard deviations in metres from the local
     * ENU position covariance (PositionSolution::positionCovarianceEnu()).
     * sdne/sdeu/sdun use the RTKLIB signed-square-root convention
     * (sign(cov) * sqrt(|cov|)). The last field is the age of the
     * differential correction in seconds (rover time minus the base
     * observation time used). Any value that is unavailable (non-finite
     * covariance, negative variance, age not supplied) is written as "nan".
     */
    static std::string formatUncertaintyColumns(const PositionSolution& sol,
                                                double differential_age_s) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const Matrix3d cov = sol.positionCovarianceEnu();  // rows/cols: E, N, U
        const auto sigma = [&](double variance) {
            return std::isfinite(variance) && variance >= 0.0 ? std::sqrt(variance) : nan;
        };
        const auto signed_sqrt = [&](double covariance) {
            return std::isfinite(covariance) ? std::copysign(std::sqrt(std::abs(covariance)), covariance)
                                             : nan;
        };
        const double values[] = {
            sigma(cov(1, 1)),        // sdn
            sigma(cov(0, 0)),        // sde
            sigma(cov(2, 2)),        // sdu
            signed_sqrt(cov(1, 0)),  // sdne
            signed_sqrt(cov(0, 2)),  // sdeu
            signed_sqrt(cov(2, 1)),  // sdun
            std::isfinite(differential_age_s) ? differential_age_s : nan,
        };
        std::ostringstream out;
        out << std::fixed;
        for (int i = 0; i < 7; ++i) {
            out << ' ';
            if (std::isfinite(values[i])) {
                const int precision = i == 6 ? 3 : 5;
                // Avoid printing "-0.00000" for tiny negative cross terms.
                const double value =
                    std::abs(values[i]) < 0.5 * std::pow(10.0, -precision) ? 0.0 : values[i];
                out << std::setprecision(precision) << value;
            } else {
                out << "nan";
            }
        }
        return out.str();
    }

    /**
     * @brief Write a single epoch's solution.
     * @param sol The position solution for this epoch.
     * @param differential_age_s Age of differential in seconds for the
     *        uncertainty columns (NaN when unknown). Unused unless the
     *        writer was opened with uncertainty_columns.
     */
    void writeEpoch(const PositionSolution& sol,
                    double differential_age_s = std::numeric_limits<double>::quiet_NaN()) {
        if (!file_.is_open()) return;

        file_ << std::fixed;

        switch (format_) {
        case Format::POS:
            file_ << sol.time.week << " "
                  << std::setprecision(3) << sol.time.tow << " "
                  << std::setprecision(4)
                  << sol.position_ecef(0) << " "
                  << sol.position_ecef(1) << " "
                  << sol.position_ecef(2) << " "
                  << std::setprecision(9)
                  << sol.position_geodetic.latitude  * 180.0 / M_PI << " "
                  << sol.position_geodetic.longitude * 180.0 / M_PI << " "
                  << std::setprecision(4)
                  << sol.position_geodetic.height << " "
                  << static_cast<int>(sol.status) << " "
                  << sol.num_satellites << " "
                  << std::setprecision(2) << sol.pdop << " "
                  << std::setprecision(1) << sol.ratio << " "
                  << std::setprecision(4) << sol.baseline_length << " "
                  << sol.iterations << " "
                  << sol.rtk_update_observations << " "
                  << sol.rtk_update_phase_observations << " "
                  << sol.rtk_update_code_observations << " "
                  << sol.rtk_update_suppressed_outliers << " "
                  << std::setprecision(4) << sol.rtk_update_prefit_residual_rms_m << " "
                  << sol.rtk_update_prefit_residual_max_m << " "
                  << sol.rtk_update_post_suppression_residual_rms_m << " "
                  << sol.rtk_update_post_suppression_residual_max_m << " "
                  << sol.rtk_update_normalized_innovation_squared << " "
                  << sol.rtk_update_normalized_innovation_squared_per_observation << " "
                  << sol.rtk_update_rejected_by_innovation_gate;
            if (uncertainty_columns_) {
                file_ << formatUncertaintyColumns(sol, differential_age_s);
            }
            if (fix_diagnostic_columns_) {
                file_ << formatFixDiagnosticColumns(sol);
            }
            file_ << "\n";
            break;

        case Format::LLH:
            file_ << sol.time.week << " "
                  << std::setprecision(3) << sol.time.tow << " "
                  << std::setprecision(9)
                  << sol.position_geodetic.latitude  * 180.0 / M_PI << " "
                  << sol.position_geodetic.longitude * 180.0 / M_PI << " "
                  << std::setprecision(4)
                  << sol.position_geodetic.height << " "
                  << static_cast<int>(sol.status)
                  << "\n";
            break;

        case Format::XYZ:
            file_ << sol.time.week << " "
                  << std::setprecision(3) << sol.time.tow << " "
                  << std::setprecision(4)
                  << sol.position_ecef(0) << " "
                  << sol.position_ecef(1) << " "
                  << sol.position_ecef(2) << " "
                  << static_cast<int>(sol.status)
                  << "\n";
            break;
        }

        file_.flush();
        epochs_written_++;
    }

    /**
     * @brief Flush and close the output file.
     */
    void close() {
        if (file_.is_open()) {
            file_.flush();
            file_.close();
        }
    }

    /**
     * @brief Number of epochs written so far.
     */
    size_t epochsWritten() const { return epochs_written_; }

private:
    std::ofstream file_;
    Format format_ = Format::POS;
    bool uncertainty_columns_ = false;
    bool fix_diagnostic_columns_ = false;
    size_t epochs_written_ = 0;
};

} // namespace io
} // namespace libgnss
