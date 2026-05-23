#pragma once

#include <string>
#include <vector>

namespace libgnss::external::madocalib {

struct PostposOptions {
    std::string obs_path;
    std::string nav_path;
    std::vector<std::string> auxiliary_input_paths;
    std::vector<std::string> mdciono_paths;
    std::string out_path;
    std::string config_path;
    std::string antenna_path;
    std::string start_time;
    std::string end_time;
    double time_interval_seconds = 0.0;
    int trace_level = 0;
};

bool isAvailable();
std::string defaultRootDir();
std::string defaultSampleConfigPath();

int runPostpos(const PostposOptions& options, std::string* error_message = nullptr);

}  // namespace libgnss::external::madocalib
