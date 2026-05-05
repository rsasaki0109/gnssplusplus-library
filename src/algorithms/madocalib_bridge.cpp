#include <libgnss++/external/madocalib_bridge.hpp>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <vector>

#ifndef GNSSPP_HAS_MADOCALIB_BRIDGE
#define GNSSPP_HAS_MADOCALIB_BRIDGE 0
#endif

#ifndef GNSSPP_MADOCALIB_ROOT
#define GNSSPP_MADOCALIB_ROOT ""
#endif

#if GNSSPP_HAS_MADOCALIB_BRIDGE
extern "C" {
#include "rtklib.h"
}

#if defined(__GNUC__)
#define GNSSPP_MADOCALIB_WEAK __attribute__((weak))
#else
#define GNSSPP_MADOCALIB_WEAK
#endif

extern "C" int GNSSPP_MADOCALIB_WEAK showmsg(const char* format, ...) {
    if (format == nullptr || *format == '\0') {
        return 0;
    }
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    std::fprintf(stderr, "\n");
    va_end(args);
    return 0;
}

extern "C" void GNSSPP_MADOCALIB_WEAK settspan(gtime_t /*ts*/, gtime_t /*te*/) {}

extern "C" void GNSSPP_MADOCALIB_WEAK settime(gtime_t /*time*/) {}
#endif

namespace libgnss::external::madocalib {
namespace {

std::string pathJoin(const std::string& base, const std::string& leaf) {
    if (base.empty()) {
        return "";
    }
    return (std::filesystem::path(base) / leaf).string();
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

bool requireRegularFiles(const std::vector<std::string>& paths,
                         const std::string& label,
                         std::string* error_message) {
    for (const std::string& path : paths) {
        if (!requireRegularFile(label, path, error_message)) {
            return false;
        }
    }
    return true;
}

#if GNSSPP_HAS_MADOCALIB_BRIDGE
void copyPath(char* destination, size_t size, const std::string& source) {
    if (size == 0) {
        return;
    }
    std::strncpy(destination, source.c_str(), size - 1);
    destination[size - 1] = '\0';
}

bool parseMadocalibTime(const std::string& value,
                        gtime_t* out,
                        std::string* error_message) {
    *out = {};
    if (value.empty()) {
        return true;
    }

    double ep[6] = {};
    if (std::sscanf(value.c_str(),
                    "%lf/%lf/%lf %lf:%lf:%lf",
                    &ep[0],
                    &ep[1],
                    &ep[2],
                    &ep[3],
                    &ep[4],
                    &ep[5]) != 6 &&
        std::sscanf(value.c_str(),
                    "%lf-%lf-%lf %lf:%lf:%lf",
                    &ep[0],
                    &ep[1],
                    &ep[2],
                    &ep[3],
                    &ep[4],
                    &ep[5]) != 6 &&
        std::sscanf(value.c_str(),
                    "%lf %lf %lf %lf %lf %lf",
                    &ep[0],
                    &ep[1],
                    &ep[2],
                    &ep[3],
                    &ep[4],
                    &ep[5]) != 6) {
        if (error_message != nullptr) {
            *error_message =
                "invalid MADOCALIB time, expected 'YYYY/MM/DD HH:MM:SS': " + value;
        }
        return false;
    }
    *out = epoch2time(ep);
    return true;
}

void applySignalSelection(const prcopt_t& prcopt) {
    const int freq_nums_l1l2[MAXFREQ] = {1, 2, 0, 0, 0};
    const int freq_nums_l1l5[MAXFREQ] = {1, 5, 0, 0, 0};
    const int freq_nums_l1l2l5[MAXFREQ] = {1, 2, 5, 0, 0};

    const int freq_nums_e1e5a[MAXFREQ] = {1, 5, 0, 0, 0};
    const int freq_nums_e1e5b[MAXFREQ] = {1, 7, 0, 0, 0};
    const int freq_nums_e1e6[MAXFREQ] = {1, 6, 0, 0, 0};
    const int freq_nums_e1e5ae5be6[MAXFREQ] = {1, 5, 7, 6, 0};
    const int freq_nums_e1e5ae6e5b[MAXFREQ] = {1, 5, 6, 7, 0};

    const int freq_nums_b1b3[MAXFREQ] = {2, 6, 0, 0, 0};
    const int freq_nums_b1b2i[MAXFREQ] = {2, 7, 0, 0, 0};
    const int freq_nums_b1b2a[MAXFREQ] = {2, 5, 0, 0, 0};
    const int freq_nums_b1b3b2i[MAXFREQ] = {2, 6, 7, 0, 0};
    const int freq_nums_b1b3b2a[MAXFREQ] = {2, 6, 5, 0, 0};

    switch (prcopt.pppsig[0]) {
    case 0:
        set_obsdef(SYS_GPS, freq_nums_l1l2);
        break;
    case 1:
        set_obsdef(SYS_GPS, freq_nums_l1l5);
        break;
    default:
        set_obsdef(SYS_GPS, freq_nums_l1l2l5);
        break;
    }
    switch (prcopt.pppsig[1]) {
    case 0:
        set_obsdef(SYS_QZS, freq_nums_l1l5);
        break;
    case 1:
        set_obsdef(SYS_QZS, freq_nums_l1l2);
        break;
    default:
        set_obsdef(SYS_QZS, freq_nums_l1l2l5);
        break;
    }
    switch (prcopt.pppsig[2]) {
    case 0:
        set_obsdef(SYS_GAL, freq_nums_e1e5a);
        break;
    case 1:
        set_obsdef(SYS_GAL, freq_nums_e1e5b);
        break;
    case 2:
        set_obsdef(SYS_GAL, freq_nums_e1e6);
        break;
    case 4:
        set_obsdef(SYS_GAL, freq_nums_e1e5ae6e5b);
        break;
    default:
        set_obsdef(SYS_GAL, freq_nums_e1e5ae5be6);
        break;
    }
    switch (prcopt.pppsig[3]) {
    case 0:
        set_obsdef(SYS_BD2, freq_nums_b1b3);
        break;
    case 1:
        set_obsdef(SYS_BD2, freq_nums_b1b2i);
        break;
    default:
        set_obsdef(SYS_BD2, freq_nums_b1b3b2i);
        break;
    }
    switch (prcopt.pppsig[4]) {
    case 0:
        set_obsdef(SYS_CMP, freq_nums_b1b3);
        break;
    case 1:
        set_obsdef(SYS_CMP, freq_nums_b1b2a);
        break;
    default:
        set_obsdef(SYS_CMP, freq_nums_b1b3b2a);
        break;
    }
}
#endif

}  // namespace

bool isAvailable() {
    return GNSSPP_HAS_MADOCALIB_BRIDGE != 0;
}

std::string defaultRootDir() {
    return GNSSPP_MADOCALIB_ROOT;
}

std::string defaultSampleConfigPath() {
    return pathJoin(defaultRootDir(), "app/consapp/rnx2rtkp/gcc_mingw/sample.conf");
}

int runPostpos(const PostposOptions& options, std::string* error_message) {
#if !GNSSPP_HAS_MADOCALIB_BRIDGE
    if (error_message != nullptr) {
        *error_message =
            "MADOCALIB bridge is not linked; reconfigure with -DMADOCALIB_PARITY_LINK=ON";
    }
    return -1;
#else
    if (!requireRegularFile("observation file", options.obs_path, error_message)) {
        return -1;
    }
    if (!options.nav_path.empty() &&
        !requireRegularFile("navigation file", options.nav_path, error_message)) {
        return -1;
    }
    if (!requireRegularFiles(options.auxiliary_input_paths, "MADOCALIB input file", error_message) ||
        !requireRegularFiles(options.mdciono_paths, "MADOCALIB L6D file", error_message)) {
        return -1;
    }
    if (!options.antenna_path.empty() &&
        !requireRegularFile("receiver ANTEX file", options.antenna_path, error_message)) {
        return -1;
    }
    if (options.out_path.empty()) {
        if (error_message != nullptr) {
            *error_message = "output path is empty";
        }
        return -1;
    }

    const std::string config_path =
        options.config_path.empty() ? defaultSampleConfigPath() : options.config_path;
    if (!config_path.empty() &&
        !requireRegularFile("MADOCALIB config", config_path, error_message)) {
        return -1;
    }

    gtime_t start_time = {};
    gtime_t end_time = {};
    if (!parseMadocalibTime(options.start_time, &start_time, error_message) ||
        !parseMadocalibTime(options.end_time, &end_time, error_message)) {
        return -1;
    }

    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = {""};

    prcopt.mode = PMODE_KINEMA;
    prcopt.navsys = 0;
    prcopt.refpos = 1;
    prcopt.glomodear = 1;
    for (int i = 0; i < MIONO_MAX_PRN; ++i) {
        prcopt.l6dpath[i] = nullptr;
    }
    solopt.timef = 0;
    std::snprintf(solopt.prog, sizeof(solopt.prog), "gnss_ppp MADOCALIB bridge");

    if (!config_path.empty()) {
        resetsysopts();
        if (!loadopts(config_path.c_str(), sysopts)) {
            if (error_message != nullptr) {
                *error_message = "failed to load MADOCALIB config: " + config_path;
            }
            return -1;
        }
        getsysopts(&prcopt, &solopt, &filopt);
    }

    if (!prcopt.navsys) {
        prcopt.navsys = SYS_GPS | SYS_GLO | SYS_QZS | SYS_GAL;
    }
    if (prcopt.ionocorr || prcopt.modear >= ARMODE_CONT) {
        prcopt.ionoopt = IONOOPT_EST;
    }
    applySignalSelection(prcopt);

    for (size_t i = 0; i < options.mdciono_paths.size() && i < MIONO_MAX_PRN; ++i) {
        prcopt.l6dpath[i] = const_cast<char*>(options.mdciono_paths[i].c_str());
    }

    if (!options.antenna_path.empty()) {
        copyPath(filopt.rcvantp, sizeof(filopt.rcvantp), options.antenna_path);
        filopt.satantp[0] = '\0';
    }
    solopt.trace = options.trace_level;

    std::vector<std::string> input_storage;
    input_storage.push_back(options.obs_path);
    if (!options.nav_path.empty()) {
        input_storage.push_back(options.nav_path);
    }
    for (const std::string& input : options.auxiliary_input_paths) {
        if (!input.empty()) {
            input_storage.push_back(input);
        }
    }

    std::vector<char*> input_files;
    input_files.reserve(input_storage.size());
    for (std::string& input : input_storage) {
        input_files.push_back(input.data());
    }

    char output_path[1024] = "";
    copyPath(output_path, sizeof(output_path), options.out_path);

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
        oss << "MADOCALIB postpos failed with status " << ret;
        *error_message = oss.str();
    }
    return ret;
#endif
}

}  // namespace libgnss::external::madocalib
