#include "libgnss++/kml/elements.hpp"
#include "libgnss++/kml/utils.hpp"
#include "libgnss++/kml/tour.hpp"

#include <cstdio>
#include <sstream>

namespace libgnss {
namespace kml {

namespace {

std::string fmtCoord(const Location& loc) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.8f,%.8f,%.3f", loc.lon, loc.lat, loc.alt);
    return buf;
}

}  // namespace

KmlString createPoints(const std::vector<std::string>& point_ids,
                       const std::vector<Location>& locations,
                       double scale,
                       const std::vector<Color>& colors,
                       double alpha,
                       AltitudeMode mode) {
    size_t n = locations.size();
    std::string alt_str = altitudeModeStr(mode);
    std::string icon = "http://maps.google.com/mapfiles/kml/pal2/icon18.png";

    std::ostringstream ss;

    // Styles
    for (size_t i = 0; i < n; ++i) {
        const auto& id = (point_ids.size() == 1 && n > 1) ? point_ids[0] : point_ids[i];
        const auto& col = (colors.size() == 1 && n > 1) ? colors[0] : colors[i];
        std::string hex = col2hex(col, alpha);

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.1f", scale);

        ss << "<Style id=\"PS" << id << "\">\n"
           << "<IconStyle id=\"PIS" << id << "\">\n"
           << "<color>" << hex << "</color>\n"
           << "<scale>" << buf << "</scale>\n"
           << "<Icon><href>" << icon << "</href></Icon>\n"
           << "</IconStyle>\n"
           << "</Style>\n";
    }

    // Points
    std::ostringstream points;
    for (size_t i = 0; i < n; ++i) {
        const auto& id = (point_ids.size() == 1 && n > 1) ? point_ids[0] : point_ids[i];

        points << "<Placemark>\n"
               << "<styleUrl>#PS" << id << "</styleUrl>\n"
               << "<Point id=\"P" << id << "\">\n"
               << "<altitudeMode>" << alt_str << "</altitudeMode>\n"
               << "<coordinates>" << fmtCoord(locations[i]) << "</coordinates>\n"
               << "</Point>\n"
               << "</Placemark>\n";
    }

    std::string point_str = points.str();
    if (n > 1) {
        point_str = wrapFolder(point_str, "Points");
    }

    ss << point_str;
    return ss.str();
}

KmlString createLine(const std::string& name,
                     const std::vector<Location>& locations,
                     double line_width,
                     const Color& color,
                     double alpha,
                     int draw_order,
                     AltitudeMode mode) {
    std::string hex = col2hex(color, alpha);
    std::string alt_str = altitudeModeStr(mode);

    char lw_buf[16];
    std::snprintf(lw_buf, sizeof(lw_buf), "%.1f", line_width);

    std::ostringstream ss;
    ss << "<Placemark>\n"
       << "<name>" << name << "</name>\n"
       << "<Style>\n"
       << "<LineStyle>\n"
       << "<color>" << hex << "</color>\n"
       << "<width>" << lw_buf << "</width>\n"
       << "</LineStyle>\n"
       << "</Style>\n"
       << "<LineString>\n"
       << "<altitudeMode>" << alt_str << "</altitudeMode>\n"
       << "<gx:drawOrder>" << draw_order << "</gx:drawOrder>\n"
       << "<coordinates>\n";

    for (const auto& loc : locations) {
        ss << fmtCoord(loc) << "\n";
    }

    ss << "</coordinates>\n"
       << "</LineString>\n"
       << "</Placemark>\n";

    return ss.str();
}

KmlString createCubes(const std::vector<std::string>& cube_ids,
                      const std::vector<Location>& locations,
                      double side_length,
                      const std::vector<Color>& colors,
                      double alpha,
                      AltitudeMode mode) {
    size_t n = locations.size();
    std::string alt_str = altitudeModeStr(mode);
    constexpr double poly_line_width = 2.0;

    std::ostringstream ss;

    // Styles
    for (size_t i = 0; i < n; ++i) {
        const auto& id = (cube_ids.size() == 1 && n > 1) ? cube_ids[0] : cube_ids[i];
        const auto& col = (colors.size() == 1 && n > 1) ? colors[0] : colors[i];
        std::string hex = col2hex(col, alpha);

        char lw_buf[16];
        std::snprintf(lw_buf, sizeof(lw_buf), "%.1f", poly_line_width);

        ss << "<Style id=\"CS" << id << "\">\n"
           << "<PolyStyle id=\"CPS" << id << "\">\n"
           << "<color>" << hex << "</color>\n"
           << "</PolyStyle>\n"
           << "<LineStyle>\n"
           << "<color>" << hex << "</color>\n"
           << "<width>" << lw_buf << "</width>\n"
           << "</LineStyle>\n"
           << "</Style>\n";
    }

    // Cubes
    std::ostringstream cubes;
    for (size_t i = 0; i < n; ++i) {
        const auto& id = (cube_ids.size() == 1 && n > 1) ? cube_ids[0] : cube_ids[i];
        auto cv = calcCubeVertices(locations[i], side_length);
        int visible = locations[i].isNaN() ? 0 : 1;

        for (int j = 0; j < CubeVertices::NUM_FACES; ++j) {
            cubes << "<Placemark id=\"CPM" << (j + 1) << "-" << id << "\">\n"
                  << "<visibility>" << visible << "</visibility>\n"
                  << "<styleUrl>#CS" << id << "</styleUrl>\n"
                  << "<Polygon>\n"
                  << "<altitudeMode>" << alt_str << "</altitudeMode>\n"
                  << "<outerBoundaryIs>\n"
                  << "<LinearRing id=\"CPL" << (j + 1) << "-" << id << "\">\n"
                  << "<coordinates>\n";

            for (int k = 0; k < CubeVertices::VERTS_PER_FACE; ++k) {
                cubes << fmtCoord(cv.vertices[cv.faces[j][k]]) << "\n";
            }
            // Close the ring
            cubes << fmtCoord(cv.vertices[cv.faces[j][0]]) << "\n";

            cubes << "</coordinates>\n"
                  << "</LinearRing>\n"
                  << "</outerBoundaryIs>\n"
                  << "</Polygon>\n"
                  << "</Placemark>\n";
        }
    }

    std::string cube_str = cubes.str();
    if (n > 1) {
        cube_str = wrapFolder(cube_str, "Cubes");
    }

    ss << cube_str;
    return ss.str();
}

KmlString createModels(const std::vector<std::string>& model_ids,
                       const std::vector<std::string>& files,
                       const std::vector<Location>& locations,
                       const std::vector<Orientation>& orientations,
                       double scale,
                       AltitudeMode mode) {
    size_t n = model_ids.size();
    std::string alt_str = altitudeModeStr(mode);

    char scale_buf[16];
    std::snprintf(scale_buf, sizeof(scale_buf), "%.3f", scale);

    std::ostringstream models;
    for (size_t i = 0; i < n; ++i) {
        const auto& ori = (orientations.size() == 1) ? orientations[0] : orientations[i];

        models << "<Placemark>\n"
               << "<name>" << model_ids[i] << "</name>\n"
               << "<Model id=\"" << model_ids[i] << "\">\n"
               << "<altitudeMode>" << alt_str << "</altitudeMode>\n"
               << "<Location id=\"" << model_ids[i] << "Loc\">\n";

        char buf[64];
        std::snprintf(buf, sizeof(buf), "<latitude>%.8f</latitude>\n", locations[i].lat);
        models << buf;
        std::snprintf(buf, sizeof(buf), "<longitude>%.8f</longitude>\n", locations[i].lon);
        models << buf;
        std::snprintf(buf, sizeof(buf), "<altitude>%.3f</altitude>\n", locations[i].alt);
        models << buf;

        models << "</Location>\n"
               << "<Orientation id=\"" << model_ids[i] << "Ori\">\n";

        std::snprintf(buf, sizeof(buf), "<heading>%.3f</heading>\n", ori.heading);
        models << buf;
        std::snprintf(buf, sizeof(buf), "<tilt>%.3f</tilt>\n", ori.tilt);
        models << buf;
        std::snprintf(buf, sizeof(buf), "<roll>%.3f</roll>\n", ori.roll);
        models << buf;

        models << "</Orientation>\n"
               << "<Scale>\n"
               << "<x>" << scale_buf << "</x>\n"
               << "<y>" << scale_buf << "</y>\n"
               << "<z>" << scale_buf << "</z>\n"
               << "</Scale>\n"
               << "<Link id=\"" << model_ids[i] << "Link\">\n"
               << "<href>" << files[i] << "</href>\n"
               << "</Link>\n"
               << "</Model>\n"
               << "</Placemark>\n";
    }

    std::string model_str = models.str();
    if (n > 1) {
        model_str = wrapFolder(model_str, "Models");
    }

    return model_str;
}

KmlString createModel(const std::string& model_id,
                      const std::string& file,
                      const Location& location,
                      const Orientation& orientation,
                      double scale,
                      AltitudeMode mode) {
    return createModels({model_id}, {file}, {location}, {orientation}, scale, mode);
}

KmlString createScreenOverlay(const std::string& screen_id,
                              const std::string& file) {
    std::ostringstream ss;
    ss << "<ScreenOverlay id=\"SO" << screen_id << "\">\n"
       << "<name>" << screen_id << "</name>\n"
       << "<Icon>\n"
       << "<href>" << file << "</href>\n"
       << "</Icon>\n"
       << "<overlayXY x=\"0.99\" y=\"0.99\" xunits=\"fraction\" yunits=\"fraction\"/>\n"
       << "<screenXY x=\"0.99\" y=\"0.99\" xunits=\"fraction\" yunits=\"fraction\"/>\n"
       << "<rotation>0</rotation>\n"
       << "<size x=\"0\" y=\"0\" xunits=\"pixels\" yunits=\"pixels\"/>\n"
       << "</ScreenOverlay>\n";
    return ss.str();
}

}  // namespace kml
}  // namespace libgnss
