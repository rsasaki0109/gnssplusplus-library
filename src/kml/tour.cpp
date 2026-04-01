#include "libgnss++/kml/tour.hpp"

#include <cstdio>
#include <sstream>

namespace libgnss {
namespace kml {

KmlString wrapTour(const KmlString& content, const std::string& name) {
    std::ostringstream ss;
    ss << "<gx:Tour>\n"
       << "<name>" << name << "</name>\n"
       << "<gx:Playlist>\n"
       << content
       << "</gx:Playlist>\n"
       << "</gx:Tour>\n";
    return ss.str();
}

KmlString wrapFlyTo(const KmlString& content, double duration, FlyToMode mode) {
    std::ostringstream ss;
    char buf[32];

    std::snprintf(buf, sizeof(buf), "%.3f", duration);
    ss << "<gx:FlyTo>\n"
       << "<gx:duration>" << buf << "</gx:duration>\n"
       << "<gx:flyToMode>" << flyToModeStr(mode) << "</gx:flyToMode>\n"
       << content
       << "</gx:FlyTo>\n";
    return ss.str();
}

KmlString wrapFolder(const KmlString& content, const std::string& name) {
    std::ostringstream ss;
    ss << "<Folder>\n"
       << "<name>" << name << "</name>\n"
       << content
       << "</Folder>\n";
    return ss.str();
}

KmlString createWait(double duration) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", duration);
    return std::string("<gx:Wait>\n<gx:duration>") + buf + "</gx:duration>\n</gx:Wait>\n";
}

}  // namespace kml
}  // namespace libgnss
