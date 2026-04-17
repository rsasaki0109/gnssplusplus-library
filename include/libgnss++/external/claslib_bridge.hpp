#pragma once

#include <string>

namespace libgnss::external::claslib {

struct PostposOptions {
    std::string obs_path;
    std::string nav_path;
    std::string ssr_path;
    std::string out_path;
    std::string config_path;
    std::string data_dir;
    std::string grid_path;
    std::string blq_path;
    std::string eop_path;
    std::string receiver_antenna_path;
    std::string isb_path;
    std::string phase_cycle_path;
    std::string receiver_antenna_type;
    int l6_gps_week = 0;
    double time_interval_seconds = 0.0;
    int trace_level = 0;
    bool output_ecef_xyz = true;
};

bool isAvailable();
std::string defaultRootDir();
std::string defaultDataDir();
std::string defaultStaticConfigPath();

int runPostpos(const PostposOptions& options, std::string* error_message = nullptr);

}  // namespace libgnss::external::claslib
