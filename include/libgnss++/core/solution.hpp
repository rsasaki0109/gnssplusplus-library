#pragma once

#include <vector>
#include <string>
#include <chrono>
#include "types.hpp"

namespace libgnss {

/**
 * @brief Single position solution
 */
struct PositionSolution {
    GNSSTime time;
    SolutionStatus status = SolutionStatus::NONE;
    
    // Position
    Vector3d position_ecef;         ///< Position in ECEF coordinates
    GeodeticCoord position_geodetic; ///< Position in geodetic coordinates
    Matrix3d position_covariance;   ///< Position covariance matrix
    
    // Velocity (if available)
    Vector3d velocity_ecef;         ///< Velocity in ECEF coordinates
    Vector3d velocity_ned;          ///< Velocity in NED coordinates
    Matrix3d velocity_covariance;   ///< Velocity covariance matrix
    bool has_velocity = false;
    
    // Clock
    double receiver_clock_bias = 0.0;     ///< Receiver clock bias in seconds
    double receiver_clock_drift = 0.0;    ///< Receiver clock drift in s/s
    double clock_variance = 0.0;          ///< Clock bias variance
    
    // Quality indicators
    double pdop = 999.9;            ///< Position dilution of precision
    double hdop = 999.9;            ///< Horizontal dilution of precision
    double vdop = 999.9;            ///< Vertical dilution of precision
    double gdop = 999.9;            ///< Geometric dilution of precision
    
    int num_satellites = 0;         ///< Number of satellites used
    int num_frequencies = 0;        ///< Number of frequencies used
    
    // RTK specific
    double baseline_length = 0.0;   ///< Baseline length for RTK
    double ratio = 0.0;             ///< Ambiguity ratio test value
    int num_fixed_ambiguities = 0;  ///< Number of fixed ambiguities
    
    // Processing statistics
    double processing_time_ms = 0.0; ///< Processing time in milliseconds
    int iterations = 0;             ///< Number of iterations
    double residual_rms = 0.0;      ///< RMS of residuals
    
    // Satellite information
    std::vector<SatelliteId> satellites_used;
    std::vector<double> satellite_elevations;
    std::vector<double> satellite_residuals;
    
    /**
     * @brief Check if solution is valid
     */
    bool isValid() const {
        return status != SolutionStatus::NONE && num_satellites >= 4;
    }
    
    /**
     * @brief Check if solution is fixed (RTK)
     */
    bool isFixed() const {
        return status == SolutionStatus::FIXED || status == SolutionStatus::PPP_FIXED;
    }
    
    /**
     * @brief Get horizontal accuracy (95% confidence)
     */
    double getHorizontalAccuracy() const;
    
    /**
     * @brief Get vertical accuracy (95% confidence)
     */
    double getVerticalAccuracy() const;
    
    /**
     * @brief Get 3D position accuracy (95% confidence)
     */
    double get3DAccuracy() const;
    
    /**
     * @brief Convert to string representation
     */
    std::string toString() const;
    
    /**
     * @brief Convert to NMEA format
     */
    std::string toNMEA() const;
};

/**
 * @brief Collection of position solutions
 */
class Solution {
public:
    std::vector<PositionSolution> solutions;
    
    /**
     * @brief Add position solution
     */
    void addSolution(const PositionSolution& sol) {
        solutions.push_back(sol);
    }
    
    /**
     * @brief Get solution by time
     */
    const PositionSolution* getSolution(const GNSSTime& time) const;
    
    /**
     * @brief Get solutions in time range
     */
    std::vector<PositionSolution> getSolutions(const GNSSTime& start, 
                                             const GNSSTime& end) const;
    
    /**
     * @brief Filter solutions by status
     */
    std::vector<PositionSolution> filterByStatus(SolutionStatus status) const;
    
    /**
     * @brief Sort solutions by time
     */
    void sortByTime();
    
    /**
     * @brief Get time span of solutions
     */
    std::pair<GNSSTime, GNSSTime> getTimeSpan() const;
    
    /**
     * @brief Calculate statistics
     */
    struct SolutionStatistics {
        size_t total_epochs = 0;
        size_t valid_solutions = 0;
        size_t fixed_solutions = 0;
        size_t float_solutions = 0;
        
        double availability_rate = 0.0;        ///< Valid solution rate
        double fix_rate = 0.0;                 ///< RTK fix rate
        
        double mean_hdop = 0.0;
        double mean_vdop = 0.0;
        double mean_pdop = 0.0;
        
        double rms_horizontal = 0.0;           ///< Horizontal RMS error
        double rms_vertical = 0.0;             ///< Vertical RMS error
        double rms_3d = 0.0;                   ///< 3D RMS error
        
        double mean_processing_time = 0.0;     ///< Average processing time
        double mean_satellites = 0.0;          ///< Average number of satellites
        
        GNSSTime first_fix_time;               ///< Time to first fix
        double convergence_time = 0.0;         ///< PPP convergence time
    };
    
    SolutionStatistics calculateStatistics(const Vector3d& reference_position = Vector3d::Zero()) const;
    
    /**
     * @brief Write solutions to file
     */
    bool writeToFile(const std::string& filename, const std::string& format = "pos") const;
    
    /**
     * @brief Write solutions in NMEA format
     */
    bool writeNMEA(const std::string& filename) const;
    
    /**
     * @brief Write solutions in KML format for visualization
     */
    bool writeKML(const std::string& filename) const;
    
    /**
     * @brief Load solutions from file
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief Clear all solutions
     */
    void clear() {
        solutions.clear();
    }
    
    /**
     * @brief Check if solution set is empty
     */
    bool isEmpty() const {
        return solutions.empty();
    }
    
    /**
     * @brief Get number of solutions
     */
    size_t size() const {
        return solutions.size();
    }
    
    /**
     * @brief Get last solution
     */
    const PositionSolution* getLastSolution() const {
        return solutions.empty() ? nullptr : &solutions.back();
    }
    
    /**
     * @brief Remove solutions older than specified time
     */
    void removeOldSolutions(const GNSSTime& cutoff_time);
    
    /**
     * @brief Interpolate position at given time
     */
    bool interpolatePosition(const GNSSTime& time, 
                           Vector3d& position, 
                           Matrix3d& covariance) const;
    
    /**
     * @brief Smooth solutions using post-processing
     */
    void applySmoothingFilter(double process_noise = 1.0, double measurement_noise = 1.0);
    
    /**
     * @brief Detect and remove outliers
     */
    void removeOutliers(double threshold_sigma = 3.0);
};

/**
 * @brief Real-time solution monitor
 */
class SolutionMonitor {
public:
    /**
     * @brief Quality assessment result
     */
    struct QualityAssessment {
        bool position_valid = false;
        bool velocity_valid = false;
        bool clock_valid = false;
        
        double position_innovation = 0.0;
        double velocity_innovation = 0.0;
        double clock_innovation = 0.0;
        
        bool geometry_adequate = false;
        bool convergence_achieved = false;
        
        std::string warning_message;
    };
    
    /**
     * @brief Assess solution quality
     */
    QualityAssessment assessQuality(const PositionSolution& solution,
                                  const PositionSolution* previous_solution = nullptr) const;
    
    /**
     * @brief Check for position jumps
     */
    bool detectPositionJump(const PositionSolution& current,
                          const PositionSolution& previous,
                          double threshold_meters = 10.0) const;
    
    /**
     * @brief Monitor convergence for PPP
     */
    bool checkConvergence(const std::vector<PositionSolution>& recent_solutions,
                        double horizontal_threshold = 0.1,
                        double vertical_threshold = 0.2,
                        size_t min_epochs = 10) const;
    
    /**
     * @brief Generate quality report
     */
    std::string generateQualityReport(const Solution& solution) const;
};

} // namespace libgnss
