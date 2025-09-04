#pragma once

#include "core/processor.hpp"
#include "core/solution.hpp"
#include "core/observation.hpp"
#include "core/navigation.hpp"
#include "algorithms/spp.hpp"
#include "algorithms/rtk.hpp"
#include "algorithms/ppp.hpp"
#include "io/rinex.hpp"
#include "io/rtcm.hpp"

/**
 * @file gnss.hpp
 * @brief Main header file for LibGNSS++ library
 * 
 * This header provides the complete API for the LibGNSS++ library.
 * Include this file to access all GNSS processing capabilities.
 */

namespace libgnss {

/**
 * @brief Main GNSS processor class
 * 
 * This class provides a high-level interface for GNSS positioning
 * and navigation processing. It supports multiple positioning modes
 * and can process both real-time and post-processing data.
 */
class GNSSProcessor {
public:
    /**
     * @brief Positioning modes supported by the processor
     */
    enum class Mode {
        SPP,    ///< Single Point Positioning
        RTK,    ///< Real-Time Kinematic
        PPP     ///< Precise Point Positioning
    };

    /**
     * @brief Constructor
     */
    GNSSProcessor();

    /**
     * @brief Destructor
     */
    ~GNSSProcessor();

    /**
     * @brief Load configuration from file
     * @param config_file Path to configuration file
     * @return true if successful, false otherwise
     */
    bool loadConfig(const std::string& config_file);

    /**
     * @brief Set positioning mode
     * @param mode Positioning mode to use
     */
    void setMode(Mode mode);

    /**
     * @brief Process RINEX observation and navigation files
     * @param obs_file Path to RINEX observation file
     * @param nav_file Path to RINEX navigation file
     * @return Solution object containing positioning results
     */
    Solution processFile(const std::string& obs_file, const std::string& nav_file);

    /**
     * @brief Process single epoch of observations
     * @param obs Observation data for current epoch
     * @param nav Navigation data
     * @return Position solution for current epoch
     */
    PositionSolution processEpoch(const ObservationData& obs, const NavigationData& nav);

    /**
     * @brief Get current processor statistics
     * @return Statistics object
     */
    ProcessorStats getStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace libgnss
