#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "types.hpp"
#include "solution.hpp"
#include "observation.hpp"
#include "navigation.hpp"

namespace libgnss {

/**
 * @brief Configuration parameters for GNSS processing
 */
struct ProcessorConfig {
    // General settings
    double elevation_mask = 15.0;      ///< Elevation mask in degrees
    double snr_mask = 35.0;            ///< SNR mask in dB-Hz
    int max_satellites = 32;           ///< Maximum number of satellites to use
    
    // Positioning settings
    PositioningMode mode = PositioningMode::SPP;
    bool use_ionosphere_model = true;
    bool use_troposphere_model = true;
    
    // RTK settings
    double baseline_length_max = 50000.0;  ///< Maximum baseline length in meters
    int ambiguity_resolution_mode = 1;     ///< 0=off, 1=continuous, 2=instantaneous
    
    // PPP settings
    bool use_precise_orbits = true;
    bool use_precise_clocks = true;
    std::string orbit_file_path;
    std::string clock_file_path;
    
    // Quality control
    double position_variance_threshold = 100.0;  ///< Position variance threshold
    int min_satellites = 4;                      ///< Minimum satellites for solution
    
    // Output settings
    CoordinateSystem output_coordinate_system = CoordinateSystem::WGS84;
    bool output_velocity = false;
    bool output_statistics = true;
};

/**
 * @brief Processing statistics
 */
struct ProcessorStats {
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    
    size_t total_epochs = 0;
    size_t valid_solutions = 0;
    size_t fixed_solutions = 0;  ///< For RTK mode
    
    double average_pdop = 0.0;
    double average_satellites = 0.0;
    
    // Processing performance
    double average_processing_time_ms = 0.0;
    size_t total_observations_processed = 0;
    
    // Quality metrics
    double rms_horizontal = 0.0;
    double rms_vertical = 0.0;
    
    void reset() {
        *this = ProcessorStats{};
    }
};

/**
 * @brief Base class for GNSS processors
 */
class ProcessorBase {
public:
    ProcessorBase() = default;
    virtual ~ProcessorBase() = default;
    
    /**
     * @brief Initialize processor with configuration
     */
    virtual bool initialize(const ProcessorConfig& config) = 0;
    
    /**
     * @brief Process single epoch
     */
    virtual PositionSolution processEpoch(
        const ObservationData& obs, 
        const NavigationData& nav
    ) = 0;
    
    /**
     * @brief Get processing statistics
     */
    virtual ProcessorStats getStats() const = 0;
    
    /**
     * @brief Reset processor state
     */
    virtual void reset() = 0;
    
protected:
    ProcessorConfig config_;
    ProcessorStats stats_;
};

/**
 * @brief Factory for creating GNSS processors
 */
class ProcessorFactory {
public:
    /**
     * @brief Create processor based on positioning mode
     */
    static std::unique_ptr<ProcessorBase> create(PositioningMode mode);
    
    /**
     * @brief Get list of available processor types
     */
    static std::vector<std::string> getAvailableTypes();
};

} // namespace libgnss
