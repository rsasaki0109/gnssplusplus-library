#include "libgnss++/visibility/kml_tour.hpp"

#include <fstream>
#include <iomanip>

namespace libgnss {
namespace visibility {

bool writeKmlTour(const std::string& filepath,
                  const std::vector<KmlWaypoint>& waypoints,
                  double fov,
                  double dt) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(6);

    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<kml xmlns=\"http://www.opengis.net/kml/2.2\"\n"
        << "  xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n"
        << "  <gx:Tour>\n"
        << "    <name>fisheye_" << static_cast<int>(fov) << "</name>\n"
        << "    <gx:Playlist>\n";

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        double duration = (i == 0) ? 0.0 : dt;
        double heading = wrapTo360(wp.heading + 180.0);

        ofs << "      <gx:FlyTo>\n"
            << "        <gx:duration>" << duration << "</gx:duration>\n"
            << "        <gx:flyToMode>smooth</gx:flyToMode>\n"
            << "        <Camera>\n"
            << "          <longitude>" << wp.lon << "</longitude>\n"
            << "          <latitude>" << wp.lat << "</latitude>\n"
            << "          <altitude>" << wp.alt << "</altitude>\n"
            << "          <heading>" << heading << "</heading>\n"
            << "          <tilt>180</tilt>\n"
            << "          <roll>0</roll>\n"
            << "          <altitudeMode>absolute</altitudeMode>\n"
            << "          <gx:horizFov>" << fov << "</gx:horizFov>\n"
            << "        </Camera>\n"
            << "      </gx:FlyTo>\n";
    }

    ofs << "    </gx:Playlist>\n"
        << "  </gx:Tour>\n"
        << "</kml>\n";

    return ofs.good();
}

}  // namespace visibility
}  // namespace libgnss
