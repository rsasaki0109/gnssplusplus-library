#include "libgnss++/kml/writer.hpp"

#include <fstream>
#include <cstdlib>
#include <filesystem>

namespace libgnss {
namespace kml {

bool writeKml(const std::string& filepath,
              const KmlString& content,
              const std::string& name) {
    std::string doc_name = name.empty() ? std::filesystem::path(filepath).stem().string() : name;

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) return false;

    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" "
        << "xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n"
        << "<Document>\n"
        << "<name>" << doc_name << "</name>\n"
        << content
        << "</Document>\n"
        << "</kml>\n";

    return ofs.good();
}

bool writeKmz(const std::string& filepath,
              const KmlString& content,
              const std::string& name,
              const std::vector<std::string>& extra_files) {
    namespace fs = std::filesystem;

    // Derive .kml temp path from .kmz path
    fs::path kmz_path(filepath);
    fs::path kml_path = kmz_path;
    kml_path.replace_extension(".kml");

    if (!writeKml(kml_path.string(), content, name)) {
        return false;
    }

    // Build zip command
    std::string cmd = "zip -j \"" + filepath + "\" \"" + kml_path.string() + "\"";
    for (const auto& f : extra_files) {
        cmd += " \"" + f + "\"";
    }
    cmd += " > /dev/null 2>&1";

    int ret = std::system(cmd.c_str());

    // Remove temp .kml
    std::error_code ec;
    fs::remove(kml_path, ec);

    return (ret == 0);
}

}  // namespace kml
}  // namespace libgnss
