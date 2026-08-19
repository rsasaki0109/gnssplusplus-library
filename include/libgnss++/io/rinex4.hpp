#pragma once

#include <libgnss++/core/navigation.hpp>
#include <array>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace libgnss::io::rinex4 {

inline constexpr double kVersion = 4.02;

/**
 * @brief Metadata carried by a RINEX 4 navigation data-record header.
 *
 * The source field is a satellite identifier for EPH records (for example
 * G01) and may be constellation-only for STO/EOP/ION records (for example
 * R).  The body parser is intentionally kept in RINEXReader; this type only
 * describes the format boundary.
 */
struct NavigationRecordHeader {
    std::string record_type;
    std::string source;
    std::string message_type;
    std::string subtype;
    char system = '\0';
    int prn = 0;
    NavigationMessageType navigation_message_type = NavigationMessageType::Unknown;
};

/**
 * @brief Parse a RINEX 4 navigation data-record header line.
 *
 * Returns false for malformed syntax.  Known-but-not-yet-supported GNSS
 * constellations remain syntactically valid so callers can report them as an
 * unsupported EPH message rather than treating them as malformed input.
 */
bool parseNavigationRecordHeader(const std::string& line,
                                 NavigationRecordHeader& record);

/**
 * @brief Check whether an EPH message can reuse the existing body parser.
 */
bool supportsEphemerisMessage(char system, const std::string& message_type);

/**
 * @brief Convert a RINEX 4 navigation message token to typed provenance.
 */
NavigationMessageType navigationMessageTypeFromRinexToken(const std::string& token);

/**
 * @brief Return the canonical RINEX token for typed navigation provenance.
 */
const char* navigationMessageTypeName(NavigationMessageType type);

/**
 * @brief Check whether a typed EPH message can reuse the existing body parser.
 */
bool supportsEphemerisMessage(char system, NavigationMessageType type);

/**
 * @brief Calendar fields used by RINEX 4 system-data records.
 *
 * These fields deliberately remain in the originating system time scale;
 * converting them to GPS time would give STO/EOP/ION records an ambiguous
 * meaning before their constellation and message context are applied.
 */
struct CalendarTime {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
};

/**
 * @brief Strictly parsed RINEX 4 A16/A17 GLONASS CDMA body.
 *
 * Epoch and transmission time remain in the originating UTC scale here;
 * RINEXReader converts them with the existing leap-second policy before
 * storing the resulting Ephemeris.
 */
struct GlonassCdmaEphemerisRecord {
    NavigationRecordHeader header;
    CalendarTime toc;
    double minus_tau_n = 0.0;  ///< First clock field is -TauN in A16/A17.
    double gamma_n = 0.0;
    double beta = 0.0;
    std::array<double, 3> position_km{};  ///< A16/A17 X,Y,Z positions (km).
    std::array<double, 3> velocity_km_per_s{};  ///< X,Y,Z velocities (km/s).
    std::array<double, 3> acceleration_km_per_s2{};  ///< X,Y,Z accelerations (km/s2).
    int signal_health = 0;  ///< 0 healthy, 1 unhealthy.
    int data_validity = 0;  ///< 0 valid, 1 invalid.
    std::optional<double> tgd_l2ocp;  ///< L1OC delay; blank is absent.
    std::optional<double> isc_l3ocp;  ///< L3OC inter-signal correction; blank is absent.
    int satellite_type = 0;  ///< Satellite type M.
    int source_flags = 0;  ///< Four-bit RE/RT source flag value.
    double aode = 0.0;  ///< Age of ephemeris data EE (days).
    double aodc = 0.0;  ///< Age of clock data ET (days).
    int attitude_flag = 0;  ///< 0 nominal yaw steering, 1 rate-limited manoeuvre.
    double tin = 0.0;  ///< Attitude reference time (UTC(SU) seconds of day).
    double tau1 = 0.0;  ///< Attitude time constant Tau1 (seconds).
    double tau2 = 0.0;  ///< Attitude time constant Tau2 (seconds).
    double yaw_angle = 0.0;  ///< Initial yaw angle (radians).
    int sign_flag = 0;  ///< Yaw sign flag.
    double angular_rate = 0.0;  ///< Initial angular rate (rad/s).
    double angular_acceleration = 0.0;  ///< Initial angular acceleration (rad/s2).
    double max_angular_rate = 0.0;  ///< Maximum angular rate (rad/s).
    std::array<double, 3> phase_center_m{};  ///< A16/A17 X,Y,Z offsets (m).
    int urai_orbit = 0;  ///< Orbit URAI index FE.
    int urai_clock = 0;  ///< Clock URAI index FT.
    double transmission_time_utc_week = 0.0;  ///< UTC week seconds t_tm.
};

/**
 * @brief Parse a complete RINEX 4 A16/A17 GLONASS CDMA body transactionally.
 */
bool parseGlonassCdmaEphemerisRecord(
    const NavigationRecordHeader& header,
    const std::vector<std::string>& body,
    GlonassCdmaEphemerisRecord& record);

/**
 * @brief Strictly parsed RINEX 4 SBAS (GEO) ephemeris body.
 *
 * SBAS broadcasts a GEO element set rather than Keplerian orbital elements.
 * The position/velocity/acceleration fields are in the ECEF frame; RINEXReader
 * converts them into the Ephemeris struct used by satellite-state propagation.
 * The body reuses the fixed-width three-character source + calendar prefix
 * used by GLONASS CDMA; the header PRN is the on-file PRN (S21 means PRN 121)
 * and must be offset by +100 when building the SatelliteId.
 */
struct SbasEphemerisRecord {
    NavigationRecordHeader header;
    CalendarTime toc;
    double x_position_km = 0.0;   ///< X position (km).
    double y_position_km = 0.0;   ///< Y position (km).
    double z_position_km = 0.0;   ///< Z position (km).
    double x_velocity_m_per_s = 0.0;  ///< X velocity (m/s).
    double y_velocity_m_per_s = 0.0;  ///< Y velocity (m/s).
    double z_velocity_m_per_s = 0.0;  ///< Z velocity (m/s).
    double x_acceleration_m_per_s2 = 0.0;  ///< X acceleration (m/s^2).
    double y_acceleration_m_per_s2 = 0.0;  ///< Y acceleration (m/s^2).
    double z_acceleration_m_per_s2 = 0.0;  ///< Z acceleration (m/s^2).
    double clock_bias_s = 0.0;   ///< af0 clock bias (s).
    double clock_drift_s_per_s = 0.0;  ///< af1 clock drift (s/s).
    double time_of_frame = 0.0;  ///< Time of frame (GPS seconds of week).
    int health = 0;              ///< Satellite health flag.
    int accuracy_index = 0;      ///< Broadcast accuracy index.
};

/**
 * @brief Parse a complete RINEX 4 SBAS ephemeris body transactionally.
 */
bool parseSbasEphemerisRecord(
    const NavigationRecordHeader& header,
    const std::vector<std::string>& body,
    SbasEphemerisRecord& record);

struct SystemTimeOffsetRecord {
    NavigationRecordHeader header;
    CalendarTime reference_epoch;
    std::string correction_type;
    std::string sbas_id;
    std::string utc_id;
    double transmission_time = 0.0;
    double a0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
};

struct EarthOrientationRecord {
    NavigationRecordHeader header;
    CalendarTime reference_epoch;
    double x_p = 0.0;
    double x_p_rate = 0.0;
    double x_p_acceleration = 0.0;
    double y_p = 0.0;
    double y_p_rate = 0.0;
    double y_p_acceleration = 0.0;
    double transmission_time = 0.0;
    double delta_ut1 = 0.0;
    double delta_ut1_rate = 0.0;
    double delta_ut1_acceleration = 0.0;
};

struct KlobucharIonosphere {
    CalendarTime transmit_time;
    std::array<double, 4> alpha{};
    std::array<double, 4> beta{};
};

struct NequickGIonosphere {
    CalendarTime transmit_time;
    std::array<double, 3> ai{};
    int disturbance_flags = 0;
};

struct BdgimIonosphere {
    CalendarTime transmit_time;
    std::array<double, 9> alpha{};  ///< A37 Alpha_1 through Alpha_9.
};

struct NavicKlobucharIonosphere {
    CalendarTime transmit_time;
    double issue_of_data = 0.0;
    std::array<double, 4> alpha{};
    std::array<double, 4> beta{};
    std::array<double, 4> region_bounds{};  ///< [longitude_min, longitude_max, latitude_min, latitude_max].
};

struct NavicNequickNIonosphere {
    CalendarTime transmit_time;
    double issue_of_data = 0.0;
    std::array<std::array<double, 3>, 3> coefficients{};  ///< Region order, each [a0, a1, a2].
    std::array<std::array<double, 4>, 3> region_bounds{};  ///< Region order, each [longitude_min, longitude_max, MODIP_min, MODIP_max].
    std::array<int, 3> disturbance_flags{};  ///< Disturbance flag in region order.
};

struct GlonassCdmaIonosphere {
    CalendarTime transmit_time;
    double c_a = 0.0;
    double c_f10_7 = 0.0;
    double c_ap = 0.0;
};

using IonospherePayload = std::variant<KlobucharIonosphere,
                                       NequickGIonosphere,
                                       BdgimIonosphere,
                                       NavicKlobucharIonosphere,
                                       NavicNequickNIonosphere,
                                       GlonassCdmaIonosphere>;

struct IonosphereRecord {
    NavigationRecordHeader header;
    IonospherePayload payload;
};

/**
 * @brief RINEX-specific sidecar for parsed STO/EOP/ION records.
 *
 * The sidecar is kept separate from NavigationData's broadcast ephemeris and
 * legacy Klobuchar selection fields.  A record is appended only after its
 * complete body validates, so malformed records cannot leave partial state.
 */
struct SystemData {
    std::vector<SystemTimeOffsetRecord> system_time_offsets;
    std::vector<EarthOrientationRecord> earth_orientation_parameters;
    std::vector<IonosphereRecord> ionosphere_records;

    void clear() {
        system_time_offsets.clear();
        earth_orientation_parameters.clear();
        ionosphere_records.clear();
    }

    bool empty() const {
        return system_time_offsets.empty() &&
               earth_orientation_parameters.empty() &&
               ionosphere_records.empty();
    }
};

/**
 * @brief Parse a complete RINEX 4 STO record body transactionally.
 */
bool parseSystemTimeOffsetRecord(const NavigationRecordHeader& header,
                                 const std::vector<std::string>& body,
                                 SystemTimeOffsetRecord& record);

/**
 * @brief Parse a complete RINEX 4 EOP record body transactionally.
 */
bool parseEarthOrientationRecord(const NavigationRecordHeader& header,
                                 const std::vector<std::string>& body,
                                 EarthOrientationRecord& record);

/**
 * @brief Parse a complete RINEX 4 ION record body transactionally.
 */
bool parseIonosphereRecord(const NavigationRecordHeader& header,
                           const std::vector<std::string>& body,
                           IonosphereRecord& record);

/**
 * @brief Parsed metadata from a RINEX 4 observation epoch record.
 */
struct ObservationEpochHeader {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    int flag = 0;
    int record_count = 0;
    double receiver_clock_offset = 0.0;
    bool has_receiver_clock_offset = false;
    int extra_second_digits = 0;
    bool has_date = false;
};

/**
 * @brief Parse a RINEX 4 observation epoch/event record.
 *
 * The required calendar, seconds, flag, and count fields are read as tokens
 * so the optional picosecond extension cannot shift the flag or count.  The
 * optional clock estimate and extension are then checked at their fixed
 * Table A3 locations.
 */
bool parseObservationEpochHeader(const std::string& line,
                                 ObservationEpochHeader& epoch);

/**
 * @brief Return true only for CompactRINEX filename suffixes.
 */
bool isCompactRinexPath(const std::string& filename);

}  // namespace libgnss::io::rinex4
