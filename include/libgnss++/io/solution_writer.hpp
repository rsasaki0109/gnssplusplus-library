#pragma once

/**
 * @file solution_writer.hpp
 * @brief Streaming solution output writer for GNSS positioning results.
 *
 * Provides epoch-by-epoch solution file output, complementing the batch
 * Solution::writeToFile() method for real-time or large-dataset use cases.
 */

#include <libgnss++/core/solution.hpp>
#include <fstream>
#include <iomanip>
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

    explicit SolutionWriter(const std::string& filename, Format fmt = Format::POS) {
        open(filename, fmt);
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
     * @return true if the file was opened successfully.
     */
    bool open(const std::string& filename, Format fmt = Format::POS) {
        close();
        format_ = fmt;
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
                     "RTKUpdateNISPerObs RTKUpdateNISRejected\n";
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

    /**
     * @brief Write a single epoch's solution.
     * @param sol The position solution for this epoch.
     */
    void writeEpoch(const PositionSolution& sol) {
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
                  << sol.rtk_update_rejected_by_innovation_gate
                  << "\n";
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
    size_t epochs_written_ = 0;
};

} // namespace io
} // namespace libgnss
