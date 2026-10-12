#pragma once

#include <optional>
#include <vector>

#include "../core/navigation.hpp"
#include "../core/observation.hpp"
#include "rtcm.hpp"

namespace libgnss {
namespace io {

class RTCMStreamDecoder {
public:
    struct Event {
        RTCMMessageType message_type = RTCMMessageType::RTCM_UNKNOWN;
        bool has_observation = false;
        ObservationData observation;
        // RTCM multiple-message bit (MSM) / synchronous GNSS flag (legacy):
        // true when more observation messages of the same epoch follow.
        bool observation_more_follows = false;
        bool has_navigation = false;
        NavigationData navigation;
        bool has_station_position = false;
        Vector3d station_position = Vector3d::Zero();
    };

    RTCMStreamDecoder() = default;
    ~RTCMStreamDecoder() = default;

    void clear();

    /**
     * Explicit GPST reference (seed) for the stream. While a reference exists it is
     * passed to the processor before every message (GLONASS MSM epochs and 1020
     * ephemerides are dated from it, not from the host clock), TOW-only observations
     * (week 0, or BeiDou's week-1 wrap) get the week nearest to it, and it advances to
     * every observation time resolved or decoded that way. Without a reference TOW-only
     * parts stay week 0 and GLONASS uses the host clock; nothing derived from the host
     * clock ever seeds or advances the reference. Throws std::invalid_argument for a
     * non-normalized time.
     */
    void setReferenceTime(const GNSSTime& time);
    std::optional<GNSSTime> referenceTime() const { return reference_time_; }

    /**
     * Seed time from broadcast ephemerides: the latest GPS toe (full week + TOW); empty
     * when there is no GPS ephemeris with a resolved week. Shared by gnss_live and the
     * G1 checker so they cannot diverge.
     */
    static std::optional<GNSSTime> referenceFromNavigation(const NavigationData& nav_data);

    /** Week of a TOW-only time (week 0, or 1 after BeiDou's +14 s wrap) that puts it nearest to the reference. */
    static GNSSTime resolveTowOnlyTime(const GNSSTime& time, const GNSSTime& reference);

    bool pushFrame(const uint8_t* buffer, size_t size, std::vector<Event>& events);

    const NavigationData& getNavigationData() const { return navigation_data_; }
    bool hasStationPosition() const { return processor_.hasReferencePosition(); }
    Vector3d getStationPosition() const { return processor_.getReferencePosition(); }
    const RTCMProcessor& getProcessor() const { return processor_; }

private:
    RTCMProcessor processor_;
    NavigationData navigation_data_;
    std::optional<GNSSTime> reference_time_;
};

}  // namespace io
}  // namespace libgnss
