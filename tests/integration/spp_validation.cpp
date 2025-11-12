#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "libgnss++/algorithms/spp.hpp"
#include "libgnss++/core/navigation.hpp"
#include "libgnss++/core/observation.hpp"
#include "libgnss++/core/solution.hpp"
#include "libgnss++/core/types.hpp"
#include "rtklib2/io/rinex_reader.hpp"
#include "rtklib2/solvers/spp/spp_solver.hpp"

namespace {

namespace fs = std::filesystem;

struct CliOptions {
    fs::path dataset_path;
    fs::path pos_output_path;
    fs::path kml_output_path;
    std::optional<fs::path> reference_path;
};

constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kRadPerDeg = M_PI / 180.0;

[[nodiscard]] fs::path ensure_extension(const fs::path& path, const std::string& extension) {
    if (path.has_extension()) {
        return path;
    }
    fs::path with_extension = path;
    with_extension.replace_extension(extension);
    return with_extension;
}

[[nodiscard]] CliOptions parse_arguments(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error(
            "Usage: spp_validation <dataset-path> [--reference <reference-pos>] [--output <pos-output>] [--kml <kml-output>]");
    }

    CliOptions options{};
    options.pos_output_path = ensure_extension(fs::current_path() / "output" / "spp_validation.pos", ".pos");
    options.kml_output_path = ensure_extension(fs::current_path() / "output" / "spp_validation.kml", ".kml");

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--reference") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --reference");
            }
            options.reference_path = fs::absolute(argv[++i]);
        } else if (arg == "--output") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --output");
            }
            options.pos_output_path = ensure_extension(fs::absolute(argv[++i]), ".pos");
        } else if (arg == "--kml") {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for --kml");
            }
            options.kml_output_path = ensure_extension(fs::absolute(argv[++i]), ".kml");
        } else if (options.dataset_path.empty()) {
            options.dataset_path = fs::absolute(arg);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.dataset_path.empty()) {
        throw std::runtime_error("Dataset path not specified");
    }

    fs::create_directories(options.pos_output_path.parent_path());
    fs::create_directories(options.kml_output_path.parent_path());

    return options;
}

[[nodiscard]] std::vector<libgnss::Vector3d> load_reference_positions(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Failed to open reference file: " + path.string());
    }

    std::vector<libgnss::Vector3d> positions;
    std::string line;

    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '%') {
            continue;
        }

        std::istringstream iss(line);
        std::string date, time;
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double height_m = 0.0;
        int quality = 0;
        int satellites = 0;
        double pdop = 0.0;
        if (!(iss >> date >> time >> lat_deg >> lon_deg >> height_m >> quality >> satellites >> pdop)) {
            continue;
        }

        libgnss::GeodeticCoord geodetic(lat_deg * kRadPerDeg, lon_deg * kRadPerDeg, height_m);
        positions.emplace_back(libgnss::spp_utils::geodeticToEcef(geodetic));
    }

    return positions;
}

[[nodiscard]] std::string quality_to_string(rtklib2::solvers::spp::SolutionQuality quality) {
    using rtklib2::solvers::spp::SolutionQuality;
    switch (quality) {
        case SolutionQuality::SPP:
            return "SPP";
        case SolutionQuality::DGPS:
            return "DGPS";
        case SolutionQuality::Float:
            return "FLOAT";
        case SolutionQuality::Fixed:
            return "FIXED";
        case SolutionQuality::PPPFloat:
            return "PPP-FLOAT";
        case SolutionQuality::PPPFixed:
            return "PPP-FIXED";
        case SolutionQuality::None:
        default:
            return "NONE";
    }
}

void write_position_file(const fs::path& path,
                         const std::vector<libgnss::ObservationData>& observations,
                         const std::vector<libgnss::PositionSolution>& solutions,
                         const std::vector<rtklib2::solvers::spp::SolutionSummary>& summaries) {
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("Failed to open output file: " + path.string());
    }

    stream << std::fixed << std::setprecision(3);
    stream << "% GPST_WEEK GPST_TOW   LAT(deg)   LON(deg)   HGT(m)  Q  NSAT  PDOP  HDOP  VDOP  GDOP  CLK_BIAS(m)\n";

    auto to_clock_meters = [](double seconds) {
        return seconds * libgnss::constants::SPEED_OF_LIGHT;
    };

    for (std::size_t i = 0; i < solutions.size(); ++i) {
        const auto& obs = observations[i];
        const auto& solution = solutions[i];
        const auto& summary = summaries[i];

        libgnss::GeodeticCoord geodetic = libgnss::spp_utils::ecefToGeodetic(solution.position_ecef);

        stream << std::setw(4) << obs.time.week << ' '
               << std::setw(10) << std::setprecision(3) << obs.time.tow << ' '
               << std::setw(11) << std::setprecision(9) << geodetic.latitude * kDegPerRad << ' '
               << std::setw(12) << std::setprecision(9) << geodetic.longitude * kDegPerRad << ' '
               << std::setw(10) << std::setprecision(4) << geodetic.height << ' '
               << std::setw(2) << static_cast<int>(summary.quality) << ' '
               << std::setw(4) << summary.satellites_used << ' '
               << std::setw(6) << std::setprecision(3) << solution.pdop << ' '
               << std::setw(6) << solution.hdop << ' '
               << std::setw(6) << solution.vdop << ' '
               << std::setw(6) << solution.gdop << ' '
               << std::setw(12) << to_clock_meters(summary.receiver_clock_bias) << '\n';
    }
}

void write_kml(const fs::path& path,
               const std::vector<libgnss::PositionSolution>& solutions) {
    std::ofstream stream(path);
    if (!stream) {
        throw std::runtime_error("Failed to open KML file: " + path.string());
    }

    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    stream << "  <Document>\n";
    stream << "    <Placemark>\n";
    stream << "      <name>SPP Trajectory</name>\n";
    stream << "      <Style><LineStyle><color>ff0000ff</color><width>3</width></LineStyle></Style>\n";
    stream << "      <LineString>\n";
    stream << "        <tessellate>1</tessellate>\n";
    stream << "        <coordinates>\n";

    for (const auto& solution : solutions) {
        libgnss::GeodeticCoord geodetic = libgnss::spp_utils::ecefToGeodetic(solution.position_ecef);
        const double lon_deg = geodetic.longitude * kDegPerRad;
        const double lat_deg = geodetic.latitude * kDegPerRad;
        stream << std::setprecision(9) << lon_deg << ',' << lat_deg << ',' << std::setprecision(4) << geodetic.height << '\n';
    }

    stream << "        </coordinates>\n";
    stream << "      </LineString>\n";
    stream << "    </Placemark>\n";
    stream << "  </Document>\n";
    stream << "</kml>\n";
}

struct ProcessingResult {
    std::vector<libgnss::ObservationData> observations;
    std::vector<libgnss::PositionSolution> solutions;
    std::vector<rtklib2::solvers::spp::SolutionSummary> summaries;
    std::vector<rtklib2::solvers::spp::SolutionMetrics> metrics;
};

[[nodiscard]] ProcessingResult run_solver(const std::vector<libgnss::ObservationData>& observations,
                                          const libgnss::NavigationData& navigation) {
    auto solver = rtklib2::solvers::spp::make_legacy_spp_solver();

    rtklib2::solvers::spp::SolverConfig config{};
    config.processor_config.elevation_mask = 15.0;
    config.processor_config.snr_mask = 30.0;
    config.processor_config.min_satellites = 4;
    config.processor_config.mode = libgnss::PositioningMode::SPP;
    config.spp_config.max_iterations = 15;
    config.spp_config.position_convergence_threshold = 1e-4;
    config.spp_config.enable_outlier_detection = true;

    solver->configure(config);

    ProcessingResult result{};
    result.observations = observations;
    result.solutions.reserve(observations.size());
    result.summaries.reserve(observations.size());
    result.metrics.reserve(observations.size());

    rtklib2::solvers::spp::SolverContext context{};
    context.navigation = navigation;

    for (const auto& obs : observations) {
        const auto solver_result = solver->solve_epoch(obs, context);
        result.solutions.push_back(solver_result.solution);
        result.summaries.push_back(solver_result.summary);
        result.metrics.push_back(solver_result.metrics);
    }

    return result;
}

[[nodiscard]] double compute_rms(const std::vector<libgnss::Vector3d>& reference,
                                 const std::vector<libgnss::PositionSolution>& solutions) {
    const std::size_t count = std::min(reference.size(), solutions.size());
    if (count == 0) {
        return 0.0;
    }

    double sum_sq = 0.0;
    std::size_t valid = 0;

    for (std::size_t i = 0; i < count; ++i) {
        if (!solutions[i].isValid()) {
            continue;
        }
        const libgnss::Vector3d diff = solutions[i].position_ecef - reference[i];
        sum_sq += diff.squaredNorm();
        ++valid;
    }

    if (valid == 0) {
        return 0.0;
    }

    return std::sqrt(sum_sq / static_cast<double>(valid));
}

[[nodiscard]] std::vector<libgnss::ObservationData> load_observations(const fs::path& observation_file) {
    std::vector<libgnss::ObservationData> observations;
    rtklib2::io::rinex::ObservationHeader header{};
    if (!rtklib2::io::rinex::read_observation_file(observation_file.string(), observations, &header)) {
        throw std::runtime_error("Failed to read observation file: " + observation_file.string());
    }

    if (observations.empty()) {
        throw std::runtime_error("No observation epochs loaded from: " + observation_file.string());
    }

    return observations;
}

[[nodiscard]] libgnss::NavigationData load_navigation(const fs::path& navigation_file) {
    libgnss::NavigationData navigation;
    if (!rtklib2::io::rinex::read_navigation_file(navigation_file.string(), navigation)) {
        throw std::runtime_error("Failed to read navigation file: " + navigation_file.string());
    }
    return navigation;
}

void print_summary(const CliOptions& options,
                   const ProcessingResult& processing,
                   const std::optional<double>& rms_error) {
    std::size_t valid_solutions = 0;
    double pdop_sum = 0.0;

    for (const auto& solution : processing.solutions) {
        if (solution.isValid()) {
            ++valid_solutions;
            pdop_sum += solution.pdop;
        }
    }

    const double average_pdop = valid_solutions > 0 ? pdop_sum / static_cast<double>(valid_solutions) : 0.0;

    std::cout << "SPP Validation Summary\n";
    std::cout << "---------------------\n";
    std::cout << "Dataset        : " << options.dataset_path << '\n';
    std::cout << "Observation epochs : " << processing.observations.size() << '\n';
    std::cout << "Valid solutions    : " << valid_solutions << '\n';
    std::cout << "Average PDOP       : " << std::setprecision(3) << average_pdop << '\n';
    if (rms_error.has_value()) {
        std::cout << "RMS position error : " << std::setprecision(4) << rms_error.value() << " m\n";
    }
    std::cout << "POS output         : " << options.pos_output_path << '\n';
    std::cout << "KML output         : " << options.kml_output_path << '\n';

    if (valid_solutions == 0) {
        std::cout << "WARNING: No valid solutions were produced. Check input data or solver configuration." << std::endl;
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse_arguments(argc, argv);

        const fs::path observation_file = options.dataset_path / "combined.mosaic.obs";
        const fs::path navigation_file = options.dataset_path / "combined.mosaic.nav";

        if (!fs::exists(observation_file)) {
            throw std::runtime_error("Observation file not found: " + observation_file.string());
        }
        if (!fs::exists(navigation_file)) {
            throw std::runtime_error("Navigation file not found: " + navigation_file.string());
        }

        const auto observations = load_observations(observation_file);
        const auto navigation = load_navigation(navigation_file);

        auto processing = run_solver(observations, navigation);

        write_position_file(options.pos_output_path, processing.observations, processing.solutions, processing.summaries);
        write_kml(options.kml_output_path, processing.solutions);

        std::optional<double> rms_error;
        if (options.reference_path.has_value()) {
            const auto reference_positions = load_reference_positions(options.reference_path.value());
            rms_error = compute_rms(reference_positions, processing.solutions);
        }

        print_summary(options, processing, rms_error);

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
