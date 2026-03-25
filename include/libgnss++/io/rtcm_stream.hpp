#pragma once

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
        bool has_navigation = false;
        NavigationData navigation;
        bool has_station_position = false;
        Vector3d station_position = Vector3d::Zero();
    };

    RTCMStreamDecoder() = default;
    ~RTCMStreamDecoder() = default;

    void clear();
    bool pushFrame(const uint8_t* buffer, size_t size, std::vector<Event>& events);

    const NavigationData& getNavigationData() const { return navigation_data_; }
    bool hasStationPosition() const { return processor_.hasReferencePosition(); }
    Vector3d getStationPosition() const { return processor_.getReferencePosition(); }
    const RTCMProcessor& getProcessor() const { return processor_; }

private:
    RTCMProcessor processor_;
    NavigationData navigation_data_;
    int observation_week_context_ = 0;
};

}  // namespace io
}  // namespace libgnss
