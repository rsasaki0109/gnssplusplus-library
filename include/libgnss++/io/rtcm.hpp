#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "../core/observation.hpp"
#include "../core/navigation.hpp"

namespace libgnss {
namespace io {

class NTRIPClient;

/**
 * @brief RTCM message types
 */
enum class RTCMMessageType : uint16_t {
    RTCM_UNKNOWN = 0,
    // RTCM 3.x messages
    RTCM_1001 = 1001,  ///< L1-only GPS RTK observables
    RTCM_1002 = 1002,  ///< Extended L1-only GPS RTK observables
    RTCM_1003 = 1003,  ///< L1&L2 GPS RTK observables
    RTCM_1004 = 1004,  ///< Extended L1&L2 GPS RTK observables
    RTCM_1005 = 1005,  ///< Stationary RTK reference station ARP
    RTCM_1006 = 1006,  ///< Stationary RTK reference station ARP with antenna height
    RTCM_1007 = 1007,  ///< Antenna descriptor
    RTCM_1008 = 1008,  ///< Antenna descriptor & serial number
    RTCM_1009 = 1009,  ///< L1-only GLONASS RTK observables
    RTCM_1010 = 1010,  ///< Extended L1-only GLONASS RTK observables
    RTCM_1011 = 1011,  ///< L1&L2 GLONASS RTK observables
    RTCM_1012 = 1012,  ///< Extended L1&L2 GLONASS RTK observables
    RTCM_1019 = 1019,  ///< GPS ephemeris
    RTCM_1020 = 1020,  ///< GLONASS ephemeris
    RTCM_1033 = 1033,  ///< Receiver and antenna descriptors
    RTCM_1057 = 1057,  ///< GPS SSR orbit correction
    RTCM_1058 = 1058,  ///< GPS SSR clock correction
    RTCM_1059 = 1059,  ///< GPS SSR code bias
    RTCM_1060 = 1060,  ///< GPS SSR combined orbit/clock correction
    RTCM_1061 = 1061,  ///< GPS SSR URA
    RTCM_1062 = 1062,  ///< GPS SSR high-rate clock correction
    RTCM_1063 = 1063,  ///< GLONASS SSR orbit correction
    RTCM_1064 = 1064,  ///< GLONASS SSR clock correction
    RTCM_1065 = 1065,  ///< GLONASS SSR code bias
    RTCM_1066 = 1066,  ///< GLONASS SSR combined orbit/clock correction
    RTCM_1067 = 1067,  ///< GLONASS SSR URA
    RTCM_1068 = 1068,  ///< GLONASS SSR high-rate clock correction
    RTCM_1074 = 1074,  ///< GPS MSM4
    RTCM_1075 = 1075,  ///< GPS MSM5
    RTCM_1076 = 1076,  ///< GPS MSM6
    RTCM_1077 = 1077,  ///< GPS MSM7
    RTCM_1084 = 1084,  ///< GLONASS MSM4
    RTCM_1085 = 1085,  ///< GLONASS MSM5
    RTCM_1086 = 1086,  ///< GLONASS MSM6
    RTCM_1087 = 1087,  ///< GLONASS MSM7
    RTCM_1094 = 1094,  ///< Galileo MSM4
    RTCM_1095 = 1095,  ///< Galileo MSM5
    RTCM_1096 = 1096,  ///< Galileo MSM6
    RTCM_1097 = 1097,  ///< Galileo MSM7
    RTCM_1240 = 1240,  ///< Galileo SSR orbit correction
    RTCM_1241 = 1241,  ///< Galileo SSR clock correction
    RTCM_1242 = 1242,  ///< Galileo SSR code bias
    RTCM_1243 = 1243,  ///< Galileo SSR combined orbit/clock correction
    RTCM_1244 = 1244,  ///< Galileo SSR URA
    RTCM_1245 = 1245,  ///< Galileo SSR high-rate clock correction
    RTCM_1246 = 1246,  ///< QZSS SSR orbit correction
    RTCM_1247 = 1247,  ///< QZSS SSR clock correction
    RTCM_1248 = 1248,  ///< QZSS SSR code bias
    RTCM_1249 = 1249,  ///< QZSS SSR combined orbit/clock correction
    RTCM_1250 = 1250,  ///< QZSS SSR URA
    RTCM_1251 = 1251,  ///< QZSS SSR high-rate clock correction
    RTCM_1258 = 1258,  ///< BeiDou SSR orbit correction
    RTCM_1259 = 1259,  ///< BeiDou SSR clock correction
    RTCM_1260 = 1260,  ///< BeiDou SSR code bias
    RTCM_1261 = 1261,  ///< BeiDou SSR combined orbit/clock correction
    RTCM_1262 = 1262,  ///< BeiDou SSR URA
    RTCM_1263 = 1263,  ///< BeiDou SSR high-rate clock correction
    RTCM_1124 = 1124,  ///< BeiDou MSM4
    RTCM_1125 = 1125,  ///< BeiDou MSM5
    RTCM_1126 = 1126,  ///< BeiDou MSM6
    RTCM_1127 = 1127   ///< BeiDou MSM7
};

/**
 * @brief RTCM message structure
 */
struct RTCMMessage {
    std::vector<uint8_t> data;
    RTCMMessageType type = RTCMMessageType::RTCM_UNKNOWN;
    uint16_t length = 0; // Payload length
    uint32_t crc = 0;
    bool valid = false;

    RTCMMessage() = default;
    RTCMMessage(RTCMMessageType msg_type, const std::vector<uint8_t>& msg_data)
        : data(msg_data), type(msg_type), length(msg_data.size()) {}
};

/**
 * @brief Decoded RTCM SSR orbit/clock correction
 *
 * Orbit fields are radial/along/cross-track deltas and rates in the RTCM SSR
 * reference frame. Clock fields follow the RTCM SSR polynomial coefficients in
 * meters, meters/second, and meters/second^2.
 */
struct RTCMSSRCorrection {
    SatelliteId satellite;
    GNSSTime time;
    double update_interval_seconds = 0.0;
    uint8_t issue_of_data = 0;
    int provider_id = 0;
    int solution_id = 0;
    bool reference_datum = false;
    int iode = -1;
    int iodcrc = -1;
    bool has_orbit = false;
    bool has_clock = false;
    bool has_code_bias = false;
    bool has_ura = false;
    bool has_high_rate_clock = false;
    Vector3d orbit_delta_rac_m = Vector3d::Zero();
    Vector3d orbit_rate_rac_mps = Vector3d::Zero();
    Vector3d clock_delta_poly = Vector3d::Zero();
    std::map<uint8_t, double> code_bias_m;
    uint8_t ura_index = 63;
    double ura_sigma_m = 0.0;
    double high_rate_clock_m = 0.0;
};

/**
 * @brief RTCM decoder/encoder
 */
class RTCMProcessor {
public:
    RTCMProcessor() = default;
    ~RTCMProcessor() = default;

    /**
     * @brief Clear internal state like reference position
     */
    void clear();
    
    /**
     * @brief Decode RTCM data stream
     * @param buffer Input data buffer
     * @param size Buffer size
     * @return Vector of decoded messages
     */
    std::vector<RTCMMessage> decode(const uint8_t* buffer, size_t size);
    
    
    /**
     * @brief Encode observation data to RTCM
     * @param obs_data Observation data to encode
     * @param message_type RTCM message type to use
     * @return Encoded RTCM message
     */
    RTCMMessage encodeObservations(const ObservationData& obs_data,
                                 RTCMMessageType message_type = RTCMMessageType::RTCM_1004);
    
    /**
     * @brief Encode ephemeris to RTCM
     * @param ephemeris Ephemeris data to encode
     * @return Encoded RTCM message
     */
    RTCMMessage encodeEphemeris(const Ephemeris& ephemeris);
    
    /**
     * @brief Set reference station position
     * @param position Reference station position (ECEF)
     */
    void setReferencePosition(const Vector3d& position) {
        reference_position_ = position;
        has_reference_position_ = true;
    }
    
    /**
     * @brief Get reference station position
     * @return Reference station position if available
     */
    Vector3d getReferencePosition() const { return reference_position_; }
    
    /**
     * @brief Check if reference position is available
     */
    bool hasReferencePosition() const { return has_reference_position_; }
    
    /**
     * @brief Get processing statistics
     */
    struct RTCMStats {
        size_t total_messages = 0;
        size_t valid_messages = 0;
        size_t crc_errors = 0;
        size_t decode_errors = 0;
        std::map<RTCMMessageType, size_t> message_counts;
    };
    
    RTCMStats getStats() const { return stats_; }

    /**
     * @brief Decode an RTCM observation payload into ObservationData
     */
    bool decodeObservationData(const RTCMMessage& message, ObservationData& obs_data);

    /**
     * @brief Decode an RTCM ephemeris payload into NavigationData
     */
    bool decodeNavigationData(const RTCMMessage& message, NavigationData& nav_data);

    /**
     * @brief Decode an RTCM SSR payload into orbit/clock corrections
     */
    bool decodeSSRCorrections(const RTCMMessage& message,
                              std::vector<RTCMSSRCorrection>& corrections);

    /**
     * @brief Cache GLONASS frequency channel for later MSM decoding
     */
    void setGlonassFrequencyChannel(const SatelliteId& sat, int channel) {
        glonass_frequency_channels_[sat] = channel;
    }

    /**
     * @brief Check whether a GLONASS frequency channel is cached
     */
    bool hasGlonassFrequencyChannel(const SatelliteId& sat) const {
        return glonass_frequency_channels_.find(sat) != glonass_frequency_channels_.end();
    }
    
    /**
     * @brief Reset statistics
     */
    void resetStats() { stats_ = RTCMStats{}; }

private:
    Vector3d reference_position_;
    bool has_reference_position_ = false;
    RTCMStats stats_;
    std::map<SatelliteId, int> glonass_frequency_channels_;
    
    // Internal decoding functions
    bool decodeObservationMessage(const RTCMMessage& message, ObservationData& obs_data);
    bool decodeEphemerisMessage(const RTCMMessage& message, NavigationData& nav_data);
    
    // CRC calculation
    uint32_t calculateCRC24(const uint8_t* data, size_t length);
    
    // Bit manipulation utilities
    int64_t getBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits);
    void setBits(uint8_t* data, int pos, int len, uint32_t value);
    
    // Message parsing utilities
    bool parseHeader(const uint8_t* data, RTCMMessageType& type, uint16_t& length);
    std::vector<uint8_t> extractPayload(const uint8_t* data, size_t total_length);
};

/**
 * @brief RTCM stream reader
 */
class RTCMReader {
public:
    RTCMReader() = default;
    ~RTCMReader();
    
    /**
     * @brief Open RTCM stream (file or network)
     * @param source Stream source (filename or URL)
     * @return true if successful
     */
    bool open(const std::string& source);
    
    /**
     * @brief Close stream
     */
    void close();
    
    /**
     * @brief Read next message
     * @param message Output message
     * @return true if message read successfully
     */
    bool readMessage(RTCMMessage& message);
    
    /**
     * @brief Check if stream is open
     */
    bool isOpen() const { return is_open_; }
    
    /**
     * @brief Get stream statistics
     */
    RTCMProcessor::RTCMStats getStats() const;

private:
    RTCMProcessor processor_;
    bool is_open_ = false;
    std::vector<uint8_t> buffer_;
    size_t buffer_pos_ = 0;
    NTRIPClient* ntrip_client_ = nullptr;
    int serial_fd_ = -1;
    int tcp_fd_ = -1;
    
    // Stream-specific implementations would go here
    bool readFromFile(const std::string& filename);
    bool readFromNetwork(const std::string& url);
    bool readFromSerial(const std::string& source);
    bool readFromTcp(const std::string& source);
};

/**
 * @brief RTCM utility functions
 */
namespace rtcm_utils {
    
    /**
     * @brief Convert GPS time to RTCM time
     */
    uint32_t gpsTimeToRTCMTime(const GNSSTime& gps_time);
    
    /**
     * @brief Convert RTCM time to GPS time
     */
    GNSSTime rtcmTimeToGPSTime(uint32_t rtcm_time, uint16_t week = 0);
    
    /**
     * @brief Get message type name
     */
    std::string getMessageTypeName(RTCMMessageType type);
    
    /**
     * @brief Check if message type is observation message
     */
    bool isObservationMessage(RTCMMessageType type);
    
    /**
     * @brief Check if message type is ephemeris message
     */
    bool isEphemerisMessage(RTCMMessageType type);

    /**
     * @brief Check if message type is an RTCM SSR correction message
     */
    bool isSSRMessage(RTCMMessageType type);

    /**
     * @brief Check if message type is station message
     */
    bool isStationMessage(RTCMMessageType type);
    
    /**
     * @brief Get GNSS system from message type
     */
    GNSSSystem getSystemFromMessageType(RTCMMessageType type);
}

} // namespace io
} // namespace libgnss
