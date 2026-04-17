#include <libgnss++/external/claslib_bridge.hpp>

#include <cstring>
#include <filesystem>
#include <sstream>
#include <vector>

#ifndef GNSSPP_HAS_CLASLIB_BRIDGE
#define GNSSPP_HAS_CLASLIB_BRIDGE 0
#endif

#ifndef GNSSPP_CLASLIB_ROOT
#define GNSSPP_CLASLIB_ROOT ""
#endif

#ifndef GNSSPP_CLASLIB_DATA_DIR
#define GNSSPP_CLASLIB_DATA_DIR ""
#endif

#if GNSSPP_HAS_CLASLIB_BRIDGE
#include "rtklib.h"

extern "C" int showmsg(char* /*format*/, ...) {
    return 0;
}

extern "C" void settspan(gtime_t /*ts*/, gtime_t /*te*/) {}

extern "C" void settime(gtime_t /*time*/) {}
#endif

namespace libgnss::external::claslib {
namespace {

std::string pathJoin(const std::string& base, const std::string& leaf) {
    if (base.empty()) {
        return "";
    }
    return (std::filesystem::path(base) / leaf).string();
}

std::string firstNonEmpty(const std::string& value, const std::string& fallback) {
    return value.empty() ? fallback : value;
}

bool requireRegularFile(const std::string& label,
                        const std::string& path,
                        std::string* error_message) {
    if (!std::filesystem::is_regular_file(path)) {
        if (error_message != nullptr) {
            *error_message = label + " not found: " + path;
        }
        return false;
    }
    return true;
}

#if GNSSPP_HAS_CLASLIB_BRIDGE
void copyPath(char* destination, size_t size, const std::string& source) {
    if (size == 0) {
        return;
    }
    std::strncpy(destination, source.c_str(), size - 1);
    destination[size - 1] = '\0';
}
#endif

}  // namespace

bool isAvailable() {
    return GNSSPP_HAS_CLASLIB_BRIDGE != 0;
}

std::string defaultRootDir() {
    return GNSSPP_CLASLIB_ROOT;
}

std::string defaultDataDir() {
    const std::string configured = GNSSPP_CLASLIB_DATA_DIR;
    if (!configured.empty()) {
        return configured;
    }
    return pathJoin(defaultRootDir(), "data");
}

std::string defaultStaticConfigPath() {
    return pathJoin(defaultRootDir(), "util/rnx2rtkp/static.conf");
}

int runPostpos(const PostposOptions& options, std::string* error_message) {
#if !GNSSPP_HAS_CLASLIB_BRIDGE
    if (error_message != nullptr) {
        *error_message =
            "CLASLIB bridge is not linked; reconfigure with -DCLASLIB_PARITY_LINK=ON";
    }
    return -1;
#else
    if (!requireRegularFile("observation file", options.obs_path, error_message) ||
        !requireRegularFile("navigation file", options.nav_path, error_message) ||
        !requireRegularFile("SSR/L6 file", options.ssr_path, error_message)) {
        return -1;
    }
    if (options.out_path.empty()) {
        if (error_message != nullptr) {
            *error_message = "output path is empty";
        }
        return -1;
    }

    const std::string data_dir = options.data_dir.empty() ? defaultDataDir() : options.data_dir;
    const std::string config_path =
        options.config_path.empty() ? defaultStaticConfigPath() : options.config_path;
    if (!config_path.empty() &&
        !requireRegularFile("CLASLIB config", config_path, error_message)) {
        return -1;
    }

    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = {""};

    if (!config_path.empty()) {
        resetsysopts();
        if (!loadopts(config_path.c_str(), sysopts)) {
            if (error_message != nullptr) {
                *error_message = "failed to load CLASLIB config: " + config_path;
            }
            return -1;
        }
        getsysopts(&prcopt, &solopt, &filopt);
    }

    const std::string grid_path =
        firstNonEmpty(options.grid_path, pathJoin(data_dir, "clas_grid.def"));
    const std::string blq_path =
        firstNonEmpty(options.blq_path, pathJoin(data_dir, "clas_grid.blq"));
    const std::string eop_path =
        firstNonEmpty(options.eop_path, pathJoin(data_dir, "igu00p01.erp"));
    const std::string receiver_antenna_path =
        firstNonEmpty(options.receiver_antenna_path, pathJoin(data_dir, "igs14_L5copy.atx"));
    const std::string isb_path =
        firstNonEmpty(options.isb_path, pathJoin(data_dir, "isb.tbl"));
    const std::string phase_cycle_path =
        firstNonEmpty(options.phase_cycle_path, pathJoin(data_dir, "l2csft.tbl"));

    if (!requireRegularFile("CLAS grid file", grid_path, error_message) ||
        !requireRegularFile("CLAS grid BLQ file", blq_path, error_message) ||
        !requireRegularFile("CLAS EOP file", eop_path, error_message) ||
        !requireRegularFile("CLAS receiver ANTEX file", receiver_antenna_path, error_message) ||
        !requireRegularFile("CLAS ISB file", isb_path, error_message) ||
        !requireRegularFile("CLAS phase-cycle file", phase_cycle_path, error_message)) {
        return -1;
    }

    copyPath(filopt.grid, sizeof(filopt.grid), grid_path);
    copyPath(filopt.blq, sizeof(filopt.blq), blq_path);
    copyPath(filopt.eop, sizeof(filopt.eop), eop_path);
    copyPath(filopt.rcvantp, sizeof(filopt.rcvantp), receiver_antenna_path);
    copyPath(filopt.isb, sizeof(filopt.isb), isb_path);
    copyPath(filopt.ifpcs, sizeof(filopt.ifpcs), phase_cycle_path);

    prcopt.mode = PMODE_PPP_RTK;
    prcopt.nf = 3;
    prcopt.navsys = SYS_GPS | SYS_GAL | SYS_QZS;
    prcopt.sateph = EPHOPT_SSRAPC;
    prcopt.ionoopt = IONOOPT_EST_ADPT;
    prcopt.tropopt = TROPOPT_OFF;
    prcopt.tidecorr = 3;
    prcopt.posopt[1] = 1;
    prcopt.posopt[2] = 1;
    prcopt.posopt[3] = 1;
    prcopt.posopt[4] = 1;
    prcopt.posopt[5] = COMPOPT_MEAS;
    prcopt.posopt[6] = 1;
    prcopt.posopt[7] = 1;
    prcopt.posopt[8] = 1;
    prcopt.phasshft = 1;
    prcopt.isb = 0;
    if (!options.receiver_antenna_type.empty()) {
        copyPath(
            prcopt.anttype[0], sizeof(prcopt.anttype[0]), options.receiver_antenna_type);
    }
    if (options.l6_gps_week > 0) {
        prcopt.l6week = options.l6_gps_week;
    }

    if (options.output_ecef_xyz) {
        solopt.posf = SOLF_XYZ;
        solopt.times = TIMES_GPST;
        solopt.timef = 0;
        solopt.timeu = 3;
        solopt.outhead = 1;
        solopt.outopt = 1;
    }
    solopt.trace = options.trace_level;

    std::vector<std::string> input_storage = {
        options.obs_path,
        options.nav_path,
        options.ssr_path,
    };
    std::vector<char*> input_files;
    input_files.reserve(input_storage.size());
    for (std::string& input : input_storage) {
        input_files.push_back(input.data());
    }

    char output_path[1024] = "";
    copyPath(output_path, sizeof(output_path), options.out_path);
    gtime_t start_time = {0};
    gtime_t end_time = {0};

    const int ret = postpos(start_time,
                            end_time,
                            options.time_interval_seconds,
                            0.0,
                            &prcopt,
                            &solopt,
                            &filopt,
                            input_files.data(),
                            static_cast<int>(input_files.size()),
                            output_path,
                            "",
                            "");
    if (ret != 0 && error_message != nullptr) {
        std::ostringstream oss;
        oss << "CLASLIB postpos failed with status " << ret;
        *error_message = oss.str();
    }
    return ret;
#endif
}

}  // namespace libgnss::external::claslib
