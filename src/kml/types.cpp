#include "libgnss++/kml/types.hpp"

namespace libgnss {
namespace kml {

std::string altitudeModeStr(AltitudeMode mode) {
    switch (mode) {
        case AltitudeMode::ClampToGround:    return "clampToGround";
        case AltitudeMode::RelativeToGround: return "relativeToGround";
        case AltitudeMode::Absolute:         return "absolute";
    }
    return "clampToGround";
}

std::string flyToModeStr(FlyToMode mode) {
    switch (mode) {
        case FlyToMode::Smooth: return "smooth";
        case FlyToMode::Bounce: return "bounce";
    }
    return "smooth";
}

}  // namespace kml
}  // namespace libgnss
