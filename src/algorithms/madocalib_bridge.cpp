#include <libgnss++/external/madocalib_bridge.hpp>

#include <cstdarg>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
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

struct L6ePatternInfo {
    bool matched = false;
    std::string pattern;
};

L6ePatternInfo hourlyL6ePattern(const std::string& input_path) {
    const std::filesystem::path path(input_path);
    const std::string filename = path.filename().string();
    const auto dot_l6 = filename.rfind('.');
    if (dot_l6 == std::string::npos) {
        return {};
    }
    const std::string extension = filename.substr(dot_l6);
    if (extension != ".l6" && extension != ".L6") {
        return {};
    }
    const auto dot_prn = filename.rfind('.', dot_l6 - 1);
    if (dot_prn == std::string::npos || dot_l6 <= dot_prn + 1) {
        return {};
    }
    const std::string prn = filename.substr(dot_prn + 1, dot_l6 - dot_prn - 1);
    if (prn.size() != 3 || prn == "200" || prn == "201") {
        return {};
    }
    const std::string stem = filename.substr(0, dot_prn);
    if (stem.size() != 8) {
        return {};
    }
    for (size_t i = 0; i < 7; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(stem[i]))) {
            return {};
        }
    }
    const char hour_code = stem[7];
    if (!((hour_code >= 'A' && hour_code <= 'X') ||
          (hour_code >= 'a' && hour_code <= 'x'))) {
        return {};
    }

    const std::string year = stem.substr(0, 4);
    const std::string doy = stem.substr(4, 3);
    const std::filesystem::path day_dir = path.parent_path();
    const std::filesystem::path year_dir = day_dir.parent_path();
    const std::string pattern_name = "%Y%n%HU." + prn + extension;
    std::filesystem::path pattern_path;
    if (day_dir.filename() == doy && year_dir.filename() == year) {
        pattern_path = year_dir.parent_path() / "%Y" / "%n" / pattern_name;
    } else {
        pattern_path = day_dir / pattern_name;
    }
    return {true, pattern_path.string()};
}

std::vector<std::string> condenseHourlyL6eInputs(
    const std::vector<std::string>& inputs) {
    constexpr size_t kMadocalibL6eStreamSlots = 7;
    std::map<std::string, size_t> pattern_counts;
    std::vector<L6ePatternInfo> infos;
    infos.reserve(inputs.size());
    for (const std::string& input : inputs) {
        L6ePatternInfo info = hourlyL6ePattern(input);
        if (info.matched) {
            ++pattern_counts[info.pattern];
        }
        infos.push_back(std::move(info));
    }

    std::vector<std::string> condensed;
    condensed.reserve(inputs.size());
    std::set<std::string> emitted_patterns;
    for (size_t i = 0; i < inputs.size(); ++i) {
        const L6ePatternInfo& info = infos[i];
        if (!info.matched ||
            pattern_counts[info.pattern] <= kMadocalibL6eStreamSlots) {
            condensed.push_back(inputs[i]);
            continue;
        }
        if (emitted_patterns.insert(info.pattern).second) {
            condensed.push_back(info.pattern);
        }
    }
    return condensed;
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

    const std::vector<std::string> auxiliary_input_paths =
        condenseHourlyL6eInputs(options.auxiliary_input_paths);

    std::vector<std::string> input_storage;
    input_storage.push_back(options.obs_path);
    if (!options.nav_path.empty()) {
        input_storage.push_back(options.nav_path);
    }
    for (const std::string& input : auxiliary_input_paths) {
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
