#include <libgnss++/io/rtcm_stream.hpp>

#include <cmath>
#include <stdexcept>

namespace libgnss {
namespace io {

namespace {

void mergeNavigationData(NavigationData& dst, const NavigationData& src) {
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            // Streams rebroadcast every ephemeris periodically; keep one copy of each record.
            dst.addEphemerisIfNew(eph);
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
}

bool isNormalizedGpst(const GNSSTime& t) {
    return t.week > 0 && std::isfinite(t.tow) && t.tow >= 0.0 && t.tow < 604800.0;
}

bool isReferenceEphemerisSystem(GNSSSystem system) {
    return system == GNSSSystem::GPS || system == GNSSSystem::Galileo ||
           system == GNSSSystem::BeiDou || system == GNSSSystem::QZSS;
}

}  // namespace

GNSSTime RTCMStreamDecoder::resolveTowOnlyTime(const GNSSTime& time, const GNSSTime& reference) {
    const double total = static_cast<double>(time.week) * 604800.0 + time.tow;
    double tow = std::fmod(total, 604800.0);
    if (tow < 0.0) {
        tow += 604800.0;
    }
    const double reference_total = static_cast<double>(reference.week) * 604800.0 + reference.tow;
    long week = std::lround((reference_total - tow) / 604800.0);
    if (week < 0) {
        week = 0;
    }
    return GNSSTime(static_cast<int>(week), tow);
}

void RTCMStreamDecoder::clear() {
    processor_.clear();
    processor_.resetStats();
    navigation_data_.clear();
    reference_time_.reset();
}

void RTCMStreamDecoder::setReferenceTime(const GNSSTime& time) {
    if (!isNormalizedGpst(time)) {
        throw std::invalid_argument("expected normalized finite GPST reference");
    }
    reference_time_ = time;
}

std::optional<GNSSTime> RTCMStreamDecoder::referenceFromNavigation(const NavigationData& nav_data) {
    std::optional<GNSSTime> latest;
    for (const auto& [satellite, ephemerides] : nav_data.ephemeris_data) {
        if (satellite.system != GNSSSystem::GPS) {
            continue;
        }
        for (const auto& eph : ephemerides) {
            if (isNormalizedGpst(eph.toe) && (!latest || eph.toe > *latest)) {
                latest = eph.toe;
            }
        }
    }
    return latest;
}

bool RTCMStreamDecoder::pushFrame(const uint8_t* buffer, size_t size, std::vector<Event>& events) {
    events.clear();
    if (reference_time_) {
        processor_.setReferenceTime(*reference_time_);
    }
    const auto messages = processor_.decode(buffer, size);
    for (const auto& message : messages) {
        Event event;
        event.message_type = message.type;
        if (reference_time_) {
            processor_.setReferenceTime(*reference_time_);
        }

        if (rtcm_utils::isObservationMessage(message.type)) {
            event.has_observation = processor_.decodeObservationData(message, event.observation);
            event.observation_more_follows = processor_.lastObservationMoreFollows();
            if (event.has_observation && reference_time_) {
                // Decoded with the reference (GLONASS) or TOW-only: either way the resulting
                // time is stream-derived, never host-clock-derived.
                if (event.observation.time.week <= 1) {
                    event.observation.time = resolveTowOnlyTime(event.observation.time, *reference_time_);
                }
                if (isNormalizedGpst(event.observation.time)) {
                    reference_time_ = event.observation.time;
                }
            }
        }

        if (rtcm_utils::isEphemerisMessage(message.type)) {
            event.has_navigation = processor_.decodeNavigationData(message, event.navigation);
            if (event.has_navigation) {
                mergeNavigationData(navigation_data_, event.navigation);
                if (!reference_time_) {
                    // Only GPS/Galileo/BeiDou/QZSS ephemerides seed; GLONASS (1020) times
                    // come from the host clock when no reference exists.
                    for (const auto& [satellite, ephemerides] : event.navigation.ephemeris_data) {
                        if (!isReferenceEphemerisSystem(satellite.system)) {
                            continue;
                        }
                        for (const auto& eph : ephemerides) {
                            if (!reference_time_ && isNormalizedGpst(eph.toe)) {
                                reference_time_ = eph.toe;
                            }
                        }
                    }
                }
            }
        }

        if (rtcm_utils::isStationMessage(message.type) && processor_.hasReferencePosition()) {
            event.has_station_position = true;
            event.station_position = processor_.getReferencePosition();
        }

        events.push_back(std::move(event));
    }
    return !events.empty();
}

}  // namespace io
}  // namespace libgnss
