#pragma once

#include <sstream>
#include <string>

namespace libgnss_apps {

// Minimal JSON string escaping shared by the native CLI summary writers
// (gnss_fgo, gnss_spp, gnss_visibility, gnss_smartphone_fgo).
inline std::string jsonEscape(const std::string& value) {
    std::ostringstream escaped;
    for (char ch : value) {
        switch (ch) {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default: escaped << ch; break;
        }
    }
    return escaped.str();
}

}  // namespace libgnss_apps
