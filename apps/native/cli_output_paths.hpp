#pragma once

// Output files are prepared before processing: the directory they go into is
// created when missing (like RTKLIB and the Python tools), and a path that
// cannot be written there fails at once instead of after a long run, when
// the results are written.

#include <filesystem>
#include <initializer_list>
#include <string>
#include <system_error>

namespace libgnss_apps {

/// Empty on success (an empty path is an output that was not asked for).
inline std::string prepareOutputFile(const std::string& path) {
    if (path.empty()) {
        return {};
    }
    std::error_code error;
    if (std::filesystem::is_directory(path, error)) {
        return "output path is a directory: " + path;
    }
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (parent.empty() || std::filesystem::is_directory(parent, error)) {
        return {};
    }
    std::filesystem::create_directories(parent, error);
    if (error) {
        return "cannot create the output directory " + parent.string() + " for " + path + ": " +
               error.message();
    }
    return {};
}

inline std::string prepareOutputFiles(std::initializer_list<std::string> paths) {
    for (const std::string& path : paths) {
        std::string error = prepareOutputFile(path);
        if (!error.empty()) {
            return error;
        }
    }
    return {};
}

}  // namespace libgnss_apps
