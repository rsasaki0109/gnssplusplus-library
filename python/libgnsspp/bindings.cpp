#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <libgnss++/gnss.hpp>

namespace py = pybind11;

namespace {

std::array<double, 3> vectorToArray(const libgnss::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}

std::string solutionStatusName(libgnss::SolutionStatus status) {
    switch (status) {
        case libgnss::SolutionStatus::NONE:
            return "NONE";
        case libgnss::SolutionStatus::SPP:
            return "SPP";
        case libgnss::SolutionStatus::DGPS:
            return "DGPS";
        case libgnss::SolutionStatus::FLOAT:
            return "FLOAT";
        case libgnss::SolutionStatus::FIXED:
            return "FIXED";
        case libgnss::SolutionStatus::PPP_FLOAT:
            return "PPP_FLOAT";
        case libgnss::SolutionStatus::PPP_FIXED:
            return "PPP_FIXED";
        default:
            return "UNKNOWN";
    }
}

std::string fileTypeName(libgnss::io::RINEXReader::FileType type) {
    switch (type) {
        case libgnss::io::RINEXReader::FileType::OBSERVATION:
            return "observation";
        case libgnss::io::RINEXReader::FileType::NAVIGATION:
            return "navigation";
        case libgnss::io::RINEXReader::FileType::METEOROLOGICAL:
            return "meteorological";
        case libgnss::io::RINEXReader::FileType::CLOCK:
            return "clock";
        case libgnss::io::RINEXReader::FileType::UNKNOWN:
        default:
            return "unknown";
    }
}

py::dict readRinexHeader(const std::string& path) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path)) {
        throw std::runtime_error("failed to open RINEX file: " + path);
    }

    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.readHeader(header)) {
        throw std::runtime_error("failed to read RINEX header: " + path);
    }

    py::dict system_obs_types;
    for (const auto& [system, obs_types] : header.system_obs_types) {
        system_obs_types[py::str(std::string(1, system))] = obs_types;
    }

    py::dict result;
    result["version"] = header.version;
    result["file_type"] = fileTypeName(header.file_type);
    result["satellite_system"] = header.satellite_system;
    result["marker_name"] = header.marker_name;
    result["receiver_type"] = header.receiver_type;
    result["antenna_type"] = header.antenna_type;
    result["approximate_position_ecef_m"] = vectorToArray(header.approximate_position);
    result["antenna_delta_hen_m"] = vectorToArray(header.antenna_delta);
    result["interval_s"] = header.interval;
    result["observation_types"] = header.observation_types;
    result["system_obs_types"] = system_obs_types;
    result["num_satellites"] = header.num_satellites;
    return result;
}

py::list readRinexObservationEpochs(const std::string& path, size_t max_epochs) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path)) {
        throw std::runtime_error("failed to open RINEX file: " + path);
    }

    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.readHeader(header)) {
        throw std::runtime_error("failed to read RINEX header: " + path);
    }
    if (header.file_type != libgnss::io::RINEXReader::FileType::OBSERVATION) {
        throw std::runtime_error("RINEX file is not an observation file: " + path);
    }

    py::list epochs;
    libgnss::ObservationData obs;
    while (reader.readObservationEpoch(obs)) {
        py::dict epoch;
        epoch["week"] = obs.time.week;
        epoch["tow"] = obs.time.tow;
        epoch["num_observations"] = obs.observations.size();
        epoch["num_satellites"] = obs.getNumSatellites();
        const auto stats = obs.getStats();
        epoch["num_systems"] = stats.num_systems;
        epoch["average_snr"] = stats.average_snr;

        py::list satellites;
        for (const auto& satellite : obs.getSatellites()) {
            satellites.append(satellite.toString());
        }
        epoch["satellites"] = satellites;
        epochs.append(epoch);

        if (max_epochs > 0 && py::len(epochs) >= static_cast<py::ssize_t>(max_epochs)) {
            break;
        }
    }

    return epochs;
}

libgnss::NavigationData loadNavigationData(const std::string& nav_path) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(nav_path)) {
        throw std::runtime_error("failed to open navigation file: " + nav_path);
    }
    libgnss::NavigationData nav;
    if (!reader.readNavigationData(nav)) {
        throw std::runtime_error("failed to read navigation data: " + nav_path);
    }
    return nav;
}

libgnss::Solution solveSppFile(const std::string& obs_path,
                               const std::string& nav_path,
                               size_t max_epochs) {
    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(obs_path)) {
        throw std::runtime_error("failed to open observation file: " + obs_path);
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        throw std::runtime_error("failed to read observation header: " + obs_path);
    }
    if (obs_header.file_type != libgnss::io::RINEXReader::FileType::OBSERVATION) {
        throw std::runtime_error("RINEX file is not an observation file: " + obs_path);
    }

    const auto nav = loadNavigationData(nav_path);

    libgnss::ProcessorConfig config;
    config.mode = libgnss::PositioningMode::SPP;
    libgnss::SPPProcessor processor;
    if (!processor.initialize(config)) {
        throw std::runtime_error("failed to initialize SPP processor");
    }

    libgnss::Solution solution;
    libgnss::ObservationData obs;
    size_t processed = 0;
    while (obs_reader.readObservationEpoch(obs)) {
        if (obs_header.approximate_position.norm() > 0.0) {
            obs.receiver_position = obs_header.approximate_position;
        }
        const auto epoch_solution = processor.processEpoch(obs, nav);
        if (epoch_solution.isValid()) {
            solution.addSolution(epoch_solution);
        }
        ++processed;
        if (max_epochs > 0 && processed >= max_epochs) {
            break;
        }
    }
    return solution;
}

libgnss::Solution solvePppFile(const std::string& obs_path,
                               const std::string& nav_path,
                               size_t max_epochs,
                               bool kinematic_mode,
                               bool enable_ar,
                               const std::string& sp3_path,
                               const std::string& clk_path) {
    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(obs_path)) {
        throw std::runtime_error("failed to open observation file: " + obs_path);
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        throw std::runtime_error("failed to read observation header: " + obs_path);
    }
    if (obs_header.file_type != libgnss::io::RINEXReader::FileType::OBSERVATION) {
        throw std::runtime_error("RINEX file is not an observation file: " + obs_path);
    }

    const auto nav = loadNavigationData(nav_path);

    libgnss::PPPProcessor::PPPConfig ppp_config;
    ppp_config.kinematic_mode = kinematic_mode;
    ppp_config.enable_ambiguity_resolution = enable_ar;
    ppp_config.use_precise_orbits = !sp3_path.empty();
    ppp_config.use_precise_clocks = !clk_path.empty();
    ppp_config.orbit_file_path = sp3_path;
    ppp_config.clock_file_path = clk_path;
    ppp_config.receiver_antenna_type = obs_header.antenna_type;
    ppp_config.receiver_antenna_delta_enu = obs_header.antenna_delta;

    libgnss::ProcessorConfig config;
    config.mode = libgnss::PositioningMode::PPP;
    config.use_precise_orbits = ppp_config.use_precise_orbits;
    config.use_precise_clocks = ppp_config.use_precise_clocks;
    config.orbit_file_path = sp3_path;
    config.clock_file_path = clk_path;

    libgnss::PPPProcessor processor(ppp_config);
    if (!processor.initialize(config)) {
        throw std::runtime_error("failed to initialize PPP processor");
    }

    libgnss::Solution solution;
    libgnss::ObservationData obs;
    size_t processed = 0;
    while (obs_reader.readObservationEpoch(obs)) {
        if (obs_header.approximate_position.norm() > 0.0) {
            obs.receiver_position = obs_header.approximate_position;
        }
        const auto epoch_solution = processor.processEpoch(obs, nav);
        if (epoch_solution.isValid()) {
            solution.addSolution(epoch_solution);
        }
        ++processed;
        if (max_epochs > 0 && processed >= max_epochs) {
            break;
        }
    }
    return solution;
}

libgnss::RTKProcessor::RTKConfig::IonoOpt parseRtkIonoMode(const std::string& iono_mode) {
    if (iono_mode == "off") {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
    }
    if (iono_mode == "iflc") {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    if (iono_mode == "est") {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::EST;
    }
    throw std::runtime_error("unsupported RTK ionosphere mode: " + iono_mode);
}

libgnss::Solution solveRtkFile(const std::string& rover_obs_path,
                               const std::string& base_obs_path,
                               const std::string& nav_path,
                               size_t max_epochs,
                               const std::string& iono_mode) {
    libgnss::io::RINEXReader rover_reader;
    if (!rover_reader.open(rover_obs_path)) {
        throw std::runtime_error("failed to open rover observation file: " + rover_obs_path);
    }
    libgnss::io::RINEXReader::RINEXHeader rover_header;
    if (!rover_reader.readHeader(rover_header)) {
        throw std::runtime_error("failed to read rover observation header: " + rover_obs_path);
    }

    libgnss::io::RINEXReader base_reader;
    if (!base_reader.open(base_obs_path)) {
        throw std::runtime_error("failed to open base observation file: " + base_obs_path);
    }
    libgnss::io::RINEXReader::RINEXHeader base_header;
    if (!base_reader.readHeader(base_header)) {
        throw std::runtime_error("failed to read base observation header: " + base_obs_path);
    }

    const auto nav = loadNavigationData(nav_path);

    libgnss::RTKProcessor::RTKConfig rtk_config;
    rtk_config.ionoopt = parseRtkIonoMode(iono_mode);
    libgnss::ProcessorConfig config;
    config.mode = libgnss::PositioningMode::RTK_FLOAT;
    libgnss::RTKProcessor processor(rtk_config);
    if (!processor.initialize(config)) {
        throw std::runtime_error("failed to initialize RTK processor");
    }
    if (base_header.approximate_position.norm() > 0.0) {
        processor.setBasePosition(base_header.approximate_position);
    }

    libgnss::Solution solution;
    libgnss::ObservationData rover_obs;
    libgnss::ObservationData base_obs;
    bool has_base_obs = base_reader.readObservationEpoch(base_obs);
    size_t processed = 0;
    while (rover_reader.readObservationEpoch(rover_obs)) {
        while (has_base_obs && (base_obs.time < rover_obs.time)) {
            has_base_obs = base_reader.readObservationEpoch(base_obs);
        }
        if (!has_base_obs) {
            break;
        }
        if (std::fabs(base_obs.time - rover_obs.time) > 1e-6) {
            continue;
        }
        const auto epoch_solution = processor.processRTKEpoch(rover_obs, base_obs, nav);
        if (epoch_solution.isValid()) {
            solution.addSolution(epoch_solution);
        }
        ++processed;
        if (max_epochs > 0 && processed >= max_epochs) {
            break;
        }
        has_base_obs = base_reader.readObservationEpoch(base_obs);
    }

    return solution;
}

libgnss::Solution loadSolution(const std::string& path) {
    libgnss::Solution solution;
    if (!solution.loadFromFile(path)) {
        throw std::runtime_error("failed to load solution file: " + path);
    }
    return solution;
}

std::array<double, 3> ecefToGeodeticDegrees(const std::array<double, 3>& ecef_m) {
    libgnss::Vector3d ecef;
    ecef << ecef_m[0], ecef_m[1], ecef_m[2];
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    libgnss::ecef2geodetic(ecef, latitude_rad, longitude_rad, height_m);
    return {latitude_rad * 180.0 / M_PI, longitude_rad * 180.0 / M_PI, height_m};
}

std::array<double, 3> geodeticDegreesToEcef(const std::array<double, 3>& llh_deg_m) {
    const auto ecef = libgnss::geodetic2ecef(
        llh_deg_m[0] * M_PI / 180.0,
        llh_deg_m[1] * M_PI / 180.0,
        llh_deg_m[2]);
    return vectorToArray(ecef);
}

}  // namespace

PYBIND11_MODULE(_libgnsspp, m) {
    m.doc() = "Minimal Python bindings for libgnss++ data inspection";

    py::enum_<libgnss::SolutionStatus>(m, "SolutionStatus")
        .value("NONE", libgnss::SolutionStatus::NONE)
        .value("SPP", libgnss::SolutionStatus::SPP)
        .value("DGPS", libgnss::SolutionStatus::DGPS)
        .value("FLOAT", libgnss::SolutionStatus::FLOAT)
        .value("FIXED", libgnss::SolutionStatus::FIXED)
        .value("PPP_FLOAT", libgnss::SolutionStatus::PPP_FLOAT)
        .value("PPP_FIXED", libgnss::SolutionStatus::PPP_FIXED);

    py::class_<libgnss::GNSSTime>(m, "GNSSTime")
        .def(py::init<int, double>(), py::arg("week"), py::arg("tow"))
        .def_readwrite("week", &libgnss::GNSSTime::week)
        .def_readwrite("tow", &libgnss::GNSSTime::tow);

    py::class_<libgnss::PositionSolution>(m, "PositionSolution")
        .def_property_readonly("time", [](const libgnss::PositionSolution& solution) {
            return solution.time;
        })
        .def_property_readonly("status", [](const libgnss::PositionSolution& solution) {
            return solution.status;
        })
        .def_property_readonly("status_name", [](const libgnss::PositionSolution& solution) {
            return solutionStatusName(solution.status);
        })
        .def_property_readonly("position_ecef_m", [](const libgnss::PositionSolution& solution) {
            return vectorToArray(solution.position_ecef);
        })
        .def_property_readonly("velocity_ecef_mps", [](const libgnss::PositionSolution& solution) {
            return vectorToArray(solution.velocity_ecef);
        })
        .def_property_readonly("velocity_ned_mps", [](const libgnss::PositionSolution& solution) {
            return vectorToArray(solution.velocity_ned);
        })
        .def_property_readonly("has_velocity", [](const libgnss::PositionSolution& solution) {
            return solution.has_velocity;
        })
        .def_property_readonly("position_geodetic_rad", [](const libgnss::PositionSolution& solution) {
            return std::array<double, 3>{
                solution.position_geodetic.latitude,
                solution.position_geodetic.longitude,
                solution.position_geodetic.height,
            };
        })
        .def_property_readonly("position_geodetic_deg", [](const libgnss::PositionSolution& solution) {
            return std::array<double, 3>{
                solution.position_geodetic.latitude * 180.0 / M_PI,
                solution.position_geodetic.longitude * 180.0 / M_PI,
                solution.position_geodetic.height,
            };
        })
        .def_property_readonly("num_satellites", [](const libgnss::PositionSolution& solution) {
            return solution.num_satellites;
        })
        .def_property_readonly("satellites_used", [](const libgnss::PositionSolution& solution) {
            std::vector<std::string> satellites;
            satellites.reserve(solution.satellites_used.size());
            for (const auto& satellite : solution.satellites_used) {
                satellites.push_back(satellite.toString());
            }
            return satellites;
        })
        .def_property_readonly("horizontal_accuracy_m", &libgnss::PositionSolution::getHorizontalAccuracy)
        .def_property_readonly("vertical_accuracy_m", &libgnss::PositionSolution::getVerticalAccuracy)
        .def_property_readonly("accuracy_3d_m", &libgnss::PositionSolution::get3DAccuracy)
        .def_property_readonly("ratio", [](const libgnss::PositionSolution& solution) {
            return solution.ratio;
        })
        .def_property_readonly("num_fixed_ambiguities", [](const libgnss::PositionSolution& solution) {
            return solution.num_fixed_ambiguities;
        })
        .def("is_valid", &libgnss::PositionSolution::isValid)
        .def("is_fixed", &libgnss::PositionSolution::isFixed)
        .def("to_nmea", &libgnss::PositionSolution::toNMEA)
        .def("to_dict", [](const libgnss::PositionSolution& solution) {
            py::dict result;
            result["week"] = solution.time.week;
            result["tow"] = solution.time.tow;
            result["status"] = solution.status;
            result["status_name"] = solutionStatusName(solution.status);
            result["position_ecef_m"] = vectorToArray(solution.position_ecef);
            result["num_satellites"] = solution.num_satellites;
            result["ratio"] = solution.ratio;
            result["num_fixed_ambiguities"] = solution.num_fixed_ambiguities;
            return result;
        });

    py::class_<libgnss::Solution::SolutionStatistics>(m, "SolutionStatistics")
        .def_readonly("total_epochs", &libgnss::Solution::SolutionStatistics::total_epochs)
        .def_readonly("valid_solutions", &libgnss::Solution::SolutionStatistics::valid_solutions)
        .def_readonly("fixed_solutions", &libgnss::Solution::SolutionStatistics::fixed_solutions)
        .def_readonly("float_solutions", &libgnss::Solution::SolutionStatistics::float_solutions)
        .def_readonly("availability_rate", &libgnss::Solution::SolutionStatistics::availability_rate)
        .def_readonly("fix_rate", &libgnss::Solution::SolutionStatistics::fix_rate)
        .def_readonly("mean_hdop", &libgnss::Solution::SolutionStatistics::mean_hdop)
        .def_readonly("mean_vdop", &libgnss::Solution::SolutionStatistics::mean_vdop)
        .def_readonly("mean_pdop", &libgnss::Solution::SolutionStatistics::mean_pdop)
        .def_readonly("rms_horizontal", &libgnss::Solution::SolutionStatistics::rms_horizontal)
        .def_readonly("rms_vertical", &libgnss::Solution::SolutionStatistics::rms_vertical)
        .def_readonly("rms_3d", &libgnss::Solution::SolutionStatistics::rms_3d)
        .def_readonly("mean_processing_time", &libgnss::Solution::SolutionStatistics::mean_processing_time)
        .def_readonly("mean_satellites", &libgnss::Solution::SolutionStatistics::mean_satellites)
        .def_readonly("convergence_time", &libgnss::Solution::SolutionStatistics::convergence_time);

    py::class_<libgnss::Solution>(m, "Solution")
        .def(py::init<>())
        .def("load_from_file", &libgnss::Solution::loadFromFile, py::arg("path"))
        .def("is_empty", &libgnss::Solution::isEmpty)
        .def("size", &libgnss::Solution::size)
        .def("records", [](const libgnss::Solution& solution) {
            return solution.solutions;
        })
        .def("filter_by_status", &libgnss::Solution::filterByStatus, py::arg("status"))
        .def("time_span", [](const libgnss::Solution& solution) {
            if (solution.isEmpty()) {
                throw std::runtime_error("solution set is empty");
            }
            return solution.getTimeSpan();
        })
        .def("last_solution", [](const libgnss::Solution& solution) {
            const auto* last = solution.getLastSolution();
            if (last == nullptr) {
                throw std::runtime_error("solution set is empty");
            }
            return *last;
        })
        .def("statistics", [](const libgnss::Solution& solution,
                               py::object reference_position) {
            libgnss::Vector3d reference = libgnss::Vector3d::Zero();
            if (!reference_position.is_none()) {
                const auto values = reference_position.cast<std::array<double, 3>>();
                reference << values[0], values[1], values[2];
            }
            return solution.calculateStatistics(reference);
        }, py::arg("reference_position_ecef_m") = py::none());

    m.def("read_rinex_header", &readRinexHeader, py::arg("path"));
    m.def("read_rinex_observation_epochs",
          &readRinexObservationEpochs,
          py::arg("path"),
          py::arg("max_epochs") = 0);
    m.def("solve_spp_file",
          &solveSppFile,
          py::arg("obs_path"),
          py::arg("nav_path"),
          py::arg("max_epochs") = 0);
    m.def("solve_ppp_file",
          &solvePppFile,
          py::arg("obs_path"),
          py::arg("nav_path"),
          py::arg("max_epochs") = 0,
          py::arg("kinematic_mode") = false,
          py::arg("enable_ar") = false,
          py::arg("sp3_path") = "",
          py::arg("clk_path") = "");
    m.def("solve_rtk_file",
          &solveRtkFile,
          py::arg("rover_obs_path"),
          py::arg("base_obs_path"),
          py::arg("nav_path"),
          py::arg("max_epochs") = 0,
          py::arg("iono_mode") = "off");
    m.def("load_solution", &loadSolution, py::arg("path"));
    m.def("solution_status_name", &solutionStatusName, py::arg("status"));
    m.def("ecef_to_geodetic_deg", &ecefToGeodeticDegrees, py::arg("ecef_m"));
    m.def("geodetic_deg_to_ecef", &geodeticDegreesToEcef, py::arg("llh_deg_m"));
}
