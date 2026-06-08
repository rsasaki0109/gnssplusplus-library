#include <libgnss++/io/rtcm_stream.hpp>

namespace libgnss {
namespace io {

namespace {

void mergeNavigationData(NavigationData& dst, const NavigationData& src) {
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            dst.addEphemeris(eph);
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
}

int inferObservationWeekContext(const NavigationData& nav_data) {
    for (const auto& [satellite, ephemerides] : nav_data.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            if (eph.week > 0) {
                return eph.week;
            }
            if (eph.toe.week > 0) {
                return eph.toe.week;
            }
            if (eph.toc.week > 0) {
                return eph.toc.week;
            }
        }
    }
    return 0;
}

}  // namespace

void RTCMStreamDecoder::clear() {
    processor_.clear();
    processor_.resetStats();
    navigation_data_.clear();
    observation_week_context_ = 0;
}

bool RTCMStreamDecoder::pushFrame(const uint8_t* buffer, size_t size, std::vector<Event>& events) {
    events.clear();
    const auto messages = processor_.decode(buffer, size);
    for (const auto& message : messages) {
        Event event;
        event.message_type = message.type;

        if (rtcm_utils::isObservationMessage(message.type)) {
            event.has_observation = processor_.decodeObservationData(message, event.observation);
            if (event.has_observation) {
                if (event.observation.time.week == 0 && observation_week_context_ > 0) {
                    event.observation.time.week = observation_week_context_;
                } else if (event.observation.time.week > 0) {
                    observation_week_context_ = event.observation.time.week;
                }
            }
        }

        if (rtcm_utils::isEphemerisMessage(message.type)) {
            event.has_navigation = processor_.decodeNavigationData(message, event.navigation);
            if (event.has_navigation) {
                mergeNavigationData(navigation_data_, event.navigation);
                const int week = inferObservationWeekContext(navigation_data_);
                if (week > 0) {
                    observation_week_context_ = week;
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
