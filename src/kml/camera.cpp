#include "libgnss++/kml/camera.hpp"

#include <cstdio>
#include <sstream>

namespace libgnss {
namespace kml {

KmlString createLookAt(const Location& location,
                       const LookAtParams& cam,
                       AltitudeMode mode) {
    std::ostringstream ss;
    char buf[64];

    ss << "<LookAt>\n";
    std::snprintf(buf, sizeof(buf), "%.8f", location.lat);
    ss << "<latitude>" << buf << "</latitude>\n";
    std::snprintf(buf, sizeof(buf), "%.8f", location.lon);
    ss << "<longitude>" << buf << "</longitude>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", location.alt);
    ss << "<altitude>" << buf << "</altitude>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.heading);
    ss << "<heading>" << buf << "</heading>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.tilt);
    ss << "<tilt>" << buf << "</tilt>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.range);
    ss << "<range>" << buf << "</range>\n";
    ss << "<altitudeMode>" << altitudeModeStr(mode) << "</altitudeMode>\n";
    ss << "</LookAt>\n";

    return ss.str();
}

KmlString createCamera(const Location& location,
                       const CameraParams& cam,
                       AltitudeMode mode) {
    std::ostringstream ss;
    char buf[64];

    ss << "<Camera>\n";
    std::snprintf(buf, sizeof(buf), "%.8f", location.lat);
    ss << "<latitude>" << buf << "</latitude>\n";
    std::snprintf(buf, sizeof(buf), "%.8f", location.lon);
    ss << "<longitude>" << buf << "</longitude>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", location.alt);
    ss << "<altitude>" << buf << "</altitude>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.heading);
    ss << "<heading>" << buf << "</heading>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.tilt);
    ss << "<tilt>" << buf << "</tilt>\n";
    std::snprintf(buf, sizeof(buf), "%.3f", cam.roll);
    ss << "<roll>" << buf << "</roll>\n";
    ss << "<altitudeMode>" << altitudeModeStr(mode) << "</altitudeMode>\n";
    ss << "</Camera>\n";

    return ss.str();
}

}  // namespace kml
}  // namespace libgnss
