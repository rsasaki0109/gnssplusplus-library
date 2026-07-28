#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace libgnss_apps {

enum class TomlArrayStyle {
    COMMA_JOINED,
    SEPARATE_ARGUMENTS,
};

struct TomlCliSchema {
    std::string section_name;
    std::unordered_map<std::string, std::string> true_options;
    std::unordered_map<std::string, std::string> false_options;
    std::unordered_map<std::string, TomlArrayStyle> array_options;
};

/// Expands one optional --config TOML file into ordinary CLI arguments.
///
/// The selected table is converted first and the original CLI arguments are
/// appended afterward, so command-line values reliably override file defaults.
/// Keys may use snake_case or kebab-case and map to the corresponding --option.
std::vector<std::string> expandTomlConfigArguments(
    int argc, char* argv[], const TomlCliSchema& schema);

}  // namespace libgnss_apps
