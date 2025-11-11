#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include "../core/observation.hpp"
#include "../core/navigation.hpp"

namespace libgnss {
namespace io {

/**
 * @brief RINEX file reader/writer
 * 
 * Supports RINEX 2.x and 3.x formats for observation and navigation files
 */
class RINEXReader {
public:
    /**
     * @brief RINEX file type
     */
    enum class FileType {
        OBSERVATION,
        NAVIGATION,
        METEOROLOGICAL,
        CLOCK,
        UNKNOWN
    };
    
    /**
     * @brief RINEX version information
     */
    struct RINEXHeader {
        double version = 0.0;
        FileType file_type = FileType::UNKNOWN;
        std::string satellite_system;
        std::string program;
        std::string run_by;
        std::string date;
        std::string marker_name;
        std::string marker_number;
        std::string observer;
        std::string agency;
        std::string receiver_number;
        std::string receiver_type;
        std::string receiver_version;
        std::string antenna_number;
        std::string antenna_type;
        Vector3d approximate_position;
        Vector3d antenna_delta;
        std::vector<std::string> observation_types;
        double interval = 0.0;
        GNSSTime first_obs;
        GNSSTime last_obs;
        int leap_seconds = 0;
        int num_satellites = 0;
        std::map<std::string, std::string> comments;
    };
    
    RINEXReader() = default;
    ~RINEXReader() = default;
    
    /**
     * @brief Open RINEX file
     */
    bool open(const std::string& filename);
    
    /**
     * @brief Close file
     */
    void close();
    
    /**
     * @brief Read header
     */
    bool readHeader(RINEXHeader& header);
    
    /**
     * @brief Read observation data
     */
    bool readObservationEpoch(ObservationData& obs_data);
    
    /**
     * @brief Read all observation data
     */
    bool readAllObservations(ObservationSeries& obs_series);
    
    /**
     * @brief Read navigation data
     */
    bool readNavigationData(NavigationData& nav_data);
    
    /**
     * @brief Get file type
     */
    FileType getFileType() const { return header_.file_type; }
    
    /**
     * @brief Get RINEX version
     */
    double getVersion() const { return header_.version; }
    
    /**
     * @brief Check if file is open
     */
    bool isOpen() const { return file_.is_open(); }
    
    /**
     * @brief Get current line number
     */
    int getCurrentLine() const { return current_line_; }

private:
    std::ifstream file_;
    RINEXHeader header_;
    int current_line_ = 0;
    
    /**
     * @brief Parse header line
     */
    bool parseHeaderLine(const std::string& line, RINEXHeader& header);
    
    /**
     * @brief Parse observation epoch (RINEX 2.x)
     */
    bool parseObservationEpochV2(const std::string& line, ObservationData& obs_data);
    
    /**
     * @brief Parse observation epoch (RINEX 3.x)
     */
    bool parseObservationEpochV3(const std::string& line, ObservationData& obs_data);
    
    /**
     * @brief Parse navigation message
     */
    bool parseNavigationMessage(const std::vector<std::string>& lines, Ephemeris& eph);
    
    /**
     * @brief Parse time string
     */
    GNSSTime parseTime(const std::string& time_str, double version);
    
    /**
     * @brief Parse satellite ID
     */
    SatelliteId parseSatelliteId(const std::string& sat_str, double version);
    
    /**
     * @brief Parse observation value
     */
    bool parseObservationValue(const std::string& obs_str, Observation& obs);
    
    /**
     * @brief Skip to next epoch
     */
    bool skipToNextEpoch();
    
    /**
     * @brief Read line from file
     */
    bool readLine(std::string& line);
};

/**
 * @brief RINEX file writer
 */
class RINEXWriter {
public:
    RINEXWriter() = default;
    ~RINEXWriter() = default;
    
    /**
     * @brief Create observation file
     */
    bool createObservationFile(const std::string& filename,
                             const RINEXReader::RINEXHeader& header);
    
    /**
     * @brief Create navigation file
     */
    bool createNavigationFile(const std::string& filename,
                            const RINEXReader::RINEXHeader& header);
    
    /**
     * @brief Write observation epoch
     */
    bool writeObservationEpoch(const ObservationData& obs_data);
    
    /**
     * @brief Write navigation message
     */
    bool writeNavigationMessage(const Ephemeris& eph);
    
    /**
     * @brief Close file
     */
    void close();

private:
    std::ofstream file_;
    RINEXReader::RINEXHeader header_;
    
    /**
     * @brief Write header
     */
    bool writeHeader(const RINEXReader::RINEXHeader& header);
    
    /**
     * @brief Format time string
     */
    std::string formatTime(const GNSSTime& time, double version);
    
    /**
     * @brief Format satellite ID
     */
    std::string formatSatelliteId(const SatelliteId& sat, double version);
    
    /**
     * @brief Format observation value
     */
    std::string formatObservationValue(const Observation& obs);
};

/**
 * @brief RINEX utility functions
 */
namespace rinex_utils {
    
    /**
     * @brief Detect RINEX file type
     */
    RINEXReader::FileType detectFileType(const std::string& filename);
    
    /**
     * @brief Get RINEX version from file
     */
    double getVersion(const std::string& filename);
    
    /**
     * @brief Convert observation type string to SignalType
     */
    SignalType stringToSignalType(const std::string& obs_type, GNSSSystem system);
    
    /**
     * @brief Convert SignalType to observation type string
     */
    std::string signalTypeToString(SignalType signal, GNSSSystem system, double version);
    
    /**
     * @brief Validate RINEX filename
     */
    bool validateFilename(const std::string& filename);
    
    /**
     * @brief Generate RINEX filename
     */
    std::string generateFilename(const std::string& station,
                               const GNSSTime& time,
                               RINEXReader::FileType type,
                               double version = 3.0);
    
    /**
     * @brief Merge RINEX observation files
     */
    bool mergeObservationFiles(const std::vector<std::string>& input_files,
                             const std::string& output_file);
    
    /**
     * @brief Split RINEX file by time
     */
    bool splitFileByTime(const std::string& input_file,
                       const std::string& output_prefix,
                       double interval_hours);
    
    /**
     * @brief Quality check RINEX file
     */
    struct QualityReport {
        bool valid_header = false;
        bool valid_data = false;
        size_t total_epochs = 0;
        size_t valid_epochs = 0;
        size_t total_observations = 0;
        size_t valid_observations = 0;
        std::vector<std::string> warnings;
        std::vector<std::string> errors;
    };
    
    QualityReport checkQuality(const std::string& filename);
}

} // namespace io
} // namespace libgnss
