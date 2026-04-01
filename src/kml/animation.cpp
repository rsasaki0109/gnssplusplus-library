#include "libgnss++/kml/animation.hpp"
#include "libgnss++/kml/utils.hpp"

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

KmlString animatedUpdatePoints(const std::vector<std::string>& point_ids,
                               double duration,
                               const std::vector<Location>& locations,
                               double delay,
                               const std::vector<Color>& colors,
                               double alpha) {
    size_t n = point_ids.size();
    bool update_color = !colors.empty();

    std::ostringstream ss;
    char buf[64];

    std::snprintf(buf, sizeof(buf), "%.3f", duration);
    ss << "<gx:AnimatedUpdate>\n"
       << "<gx:duration>" << buf << "</gx:duration>\n"
       << "<Update>\n"
       << "<Change>\n";

    for (size_t i = 0; i < n; ++i) {
        ss << "<Point targetId=\"P" << point_ids[i] << "\">\n"
           << "<coordinates>" << fmtCoord(locations[i]) << "</coordinates>\n"
           << "</Point>\n";

        if (update_color) {
            const auto& col = (colors.size() == 1 && n > 1) ? colors[0] : colors[i];
            std::string hex = col2hex(col, alpha);
            ss << "<IconStyle targetId=\"PIS" << point_ids[i] << "\">\n"
               << "<color>" << hex << "</color>\n"
               << "</IconStyle>\n";
        }
    }

    std::snprintf(buf, sizeof(buf), "%.3f", delay);
    ss << "</Change>\n"
       << "</Update>\n"
       << "<gx:delayedStart>" << buf << "</gx:delayedStart>\n"
       << "</gx:AnimatedUpdate>\n";

    return ss.str();
}

KmlString animatedUpdateModels(const std::vector<std::string>& model_ids,
                               double duration,
                               const std::vector<Location>& locations,
                               const std::vector<Orientation>& orientations,
                               const std::vector<std::string>& files,
                               double delay) {
    size_t n = model_ids.size();
    bool update_ori = !orientations.empty();
    bool update_file = !files.empty();

    std::ostringstream ss;
    char buf[64];

    std::snprintf(buf, sizeof(buf), "%.3f", duration);
    ss << "<gx:AnimatedUpdate>\n"
       << "<gx:duration>" << buf << "</gx:duration>\n"
       << "<Update>\n"
       << "<Change>\n";

    for (size_t i = 0; i < n; ++i) {
        // Location update
        ss << "<Location targetId=\"" << model_ids[i] << "Loc\">\n";
        std::snprintf(buf, sizeof(buf), "%.8f", locations[i].lat);
        ss << "<latitude>" << buf << "</latitude>\n";
        std::snprintf(buf, sizeof(buf), "%.8f", locations[i].lon);
        ss << "<longitude>" << buf << "</longitude>\n";
        std::snprintf(buf, sizeof(buf), "%.3f", locations[i].alt);
        ss << "<altitude>" << buf << "</altitude>\n"
           << "</Location>\n";

        // Orientation update
        if (update_ori) {
            const auto& ori = (orientations.size() == 1 && n > 1) ? orientations[0] : orientations[i];
            ss << "<Orientation targetId=\"" << model_ids[i] << "Ori\">\n";
            std::snprintf(buf, sizeof(buf), "%.3f", ori.heading);
            ss << "<heading>" << buf << "</heading>\n";
            std::snprintf(buf, sizeof(buf), "%.3f", ori.tilt);
            ss << "<tilt>" << buf << "</tilt>\n";
            std::snprintf(buf, sizeof(buf), "%.3f", ori.roll);
            ss << "<roll>" << buf << "</roll>\n"
               << "</Orientation>\n";
        }

        // Link (model file) update
        if (update_file) {
            ss << "<Link targetId=\"" << model_ids[i] << "Link\">\n"
               << "<href>" << files[i] << "</href>\n"
               << "</Link>\n";
        }
    }

    std::snprintf(buf, sizeof(buf), "%.3f", delay);
    ss << "</Change>\n"
       << "</Update>\n"
       << "<gx:delayedStart>" << buf << "</gx:delayedStart>\n"
       << "</gx:AnimatedUpdate>\n";

    return ss.str();
}

KmlString animatedUpdateModel(const std::string& model_id,
                              double duration,
                              const Location& location,
                              const std::optional<Orientation>& orientation,
                              const std::optional<std::string>& file,
                              double delay) {
    std::vector<Orientation> oris;
    if (orientation) oris.push_back(*orientation);

    std::vector<std::string> files;
    if (file) files.push_back(*file);

    return animatedUpdateModels({model_id}, duration, {location}, oris, files, delay);
}

KmlString animatedUpdateCubes(const std::vector<std::string>& cube_ids,
                              double duration,
                              const std::vector<Location>& locations,
                              double side_length,
                              const std::vector<Color>& colors,
                              double alpha,
                              double delay) {
    size_t n = cube_ids.size();
    bool update_color = !colors.empty();

    std::ostringstream ss;
    char buf[64];

    std::snprintf(buf, sizeof(buf), "%.3f", duration);
    ss << "<gx:AnimatedUpdate>\n"
       << "<gx:duration>" << buf << "</gx:duration>\n"
       << "<Update>\n"
       << "<Change>\n";

    for (size_t i = 0; i < n; ++i) {
        const auto& id = (cube_ids.size() == 1 && n > 1) ? cube_ids[0] : cube_ids[i];
        auto cv = calcCubeVertices(locations[i], side_length);
        bool visible = !locations[i].isNaN();

        for (int j = 0; j < CubeVertices::NUM_FACES; ++j) {
            // Update visibility
            ss << "<Placemark targetId=\"CPM" << (j + 1) << "-" << id << "\">\n"
               << "<visibility>" << (visible ? 1 : 0) << "</visibility>\n"
               << "</Placemark>\n";

            if (visible) {
                // Update coordinates
                ss << "<LinearRing targetId=\"CPL" << (j + 1) << "-" << id << "\">\n"
                   << "<coordinates>\n";
                for (int k = 0; k < CubeVertices::VERTS_PER_FACE; ++k) {
                    ss << fmtCoord(cv.vertices[cv.faces[j][k]]) << "\n";
                }
                ss << fmtCoord(cv.vertices[cv.faces[j][0]]) << "\n";
                ss << "</coordinates>\n"
                   << "</LinearRing>\n";

                // Update color
                if (update_color) {
                    const auto& col = (colors.size() == 1 && n > 1) ? colors[0] : colors[i];
                    std::string hex = col2hex(col, alpha);
                    ss << "<PolyStyle targetId=\"CPS" << id << "\">\n"
                       << "<color>" << hex << "</color>\n"
                       << "</PolyStyle>\n";
                }
            }
        }
    }

    std::snprintf(buf, sizeof(buf), "%.3f", delay);
    ss << "</Change>\n"
       << "</Update>\n"
       << "<gx:delayedStart>" << buf << "</gx:delayedStart>\n"
       << "</gx:AnimatedUpdate>\n";

    return ss.str();
}

}  // namespace kml
}  // namespace libgnss
