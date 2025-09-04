#pragma once

#include <string>
#include <vector>
#include <memory>
#include "../core/observation.hpp"
#include "../core/navigation.hpp"

namespace libgnss {
namespace io {

/**
 * @brief RTCM message types
 */
enum class RTCMMessageType : uint16_t {
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
    RTCM_1124 = 1124,  ///< BeiDou MSM4
    RTCM_1125 = 1125,  ///< BeiDou MSM5
    RTCM_1126 = 1126,  ///< BeiDou MSM6
    RTCM_1127 = 1127   ///< BeiDou MSM7
};

/**
 * @brief RTCM message structure
 */
struct RTCMMessage {
    RTCMMessageType type;
    uint16_t length;
    std::vector<uint8_t> data;
    uint32_t crc;
    bool valid = false;
    
    RTCMMessage() = default;
    RTCMMessage(RTCMMessageType msg_type, const std::vector<uint8_t>& msg_data)
        : type(msg_type), length(msg_data.size()), data(msg_data) {}
};

/**
 * @brief RTCM decoder/encoder
 */
class RTCMProcessor {
public:
    RTCMProcessor() = default;
    ~RTCMProcessor() = default;
    
    /**
     * @brief Decode RTCM data stream
     * @param buffer Input data buffer
     * @param size Buffer size
     * @return Vector of decoded messages
     */
    std::vector<RTCMMessage> decode(const uint8_t* buffer, size_t size);
    
    /**
     * @brief Decode single RTCM message
     * @param message RTCM message to decode
     * @param obs_data Output observation data (for observation messages)
     * @param nav_data Output navigation data (for ephemeris messages)
     * @return true if successfully decoded
     */
    bool decodeMessage(const RTCMMessage& message,
                      ObservationData* obs_data = nullptr,
                      NavigationData* nav_data = nullptr);
    
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
     * @brief Reset statistics
     */
    void resetStats() { stats_ = RTCMStats{}; }

private:
    Vector3d reference_position_;
    bool has_reference_position_ = false;
    RTCMStats stats_;
    
    // Internal decoding functions
    bool decodeObservationMessage(const RTCMMessage& message, ObservationData& obs_data);
    bool decodeEphemerisMessage(const RTCMMessage& message, NavigationData& nav_data);
    bool decodeStationMessage(const RTCMMessage& message);
    
    // CRC calculation
    uint32_t calculateCRC24(const uint8_t* data, size_t length);
    bool verifyCRC(const RTCMMessage& message);
    
    // Bit manipulation utilities
    uint32_t getBits(const uint8_t* data, int pos, int len);
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
    ~RTCMReader() = default;
    
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
    RTCMProcessor::RTCMStats getStats() const { return processor_.getStats(); }

private:
    RTCMProcessor processor_;
    bool is_open_ = false;
    std::vector<uint8_t> buffer_;
    size_t buffer_pos_ = 0;
    
    // Stream-specific implementations would go here
    bool readFromFile(const std::string& filename);
    bool readFromNetwork(const std::string& url);
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
     * @brief Get GNSS system from message type
     */
    GNSSSystem getSystemFromMessageType(RTCMMessageType type);
}

} // namespace io
} // namespace libgnss
