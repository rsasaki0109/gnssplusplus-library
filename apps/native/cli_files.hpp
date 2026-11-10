#pragma once

// Input-file errors that name the file and the cause, e.g.
// "cannot open rover observation file: rover.obs (no such file)" or
// "not a RINEX file (no RINEX VERSION / TYPE line): base.obs".
// Each helper returns an empty string on success and the message otherwise.

#include <filesystem>
#include <string>
#include <system_error>

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/io/rinex.hpp>

namespace libgnss_apps {

inline std::string openFailureReason(const std::string& path) {
    std::error_code error;
    const auto status = std::filesystem::status(path, error);
    if (error || !std::filesystem::exists(status)) {
        return "no such file";
    }
    if (std::filesystem::is_directory(status)) {
        return "is a directory";
    }
    return "could not be opened";
}

inline std::string cannotOpenMessage(const std::string& what, const std::string& path) {
    return "cannot open " + what + ": " + path + " (" + openFailureReason(path) + ")";
}

/// A directory opens as a stream on POSIX and then reads as empty.
inline bool isDirectory(const std::string& path) {
    std::error_code error;
    return std::filesystem::is_directory(path, error);
}

/// Opens an observation RINEX file and reads its header.
inline std::string openRinexObservation(libgnss::io::RINEXReader& reader,
                                        libgnss::io::RINEXReader::RINEXHeader& header,
                                        const std::string& what,
                                        const std::string& path) {
    if (isDirectory(path) || !reader.open(path)) {
        return cannotOpenMessage(what, path);
    }
    if (!reader.readHeader(header)) {
        return "failed to read the header of " + what + ": " + path;
    }
    if (header.version <= 0.0) {
        return "not a RINEX file (no RINEX VERSION / TYPE line): " + path;
    }
    return {};
}

/// Reads a navigation RINEX file.
inline std::string readRinexNavigation(libgnss::io::RINEXReader& reader,
                                       libgnss::NavigationData& nav_data,
                                       const std::string& path) {
    if (isDirectory(path) || !reader.open(path)) {
        return cannotOpenMessage("navigation file", path);
    }
    if (!reader.readNavigationData(nav_data)) {
        return "failed to read navigation data: " + path;
    }
    return {};
}

}  // namespace libgnss_apps
