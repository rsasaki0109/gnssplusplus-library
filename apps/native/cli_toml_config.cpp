#include "cli_toml_config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace libgnss_apps {
namespace {

struct TomlEntry {
    std::string key;
    std::string value;
    int line_number = 0;
};

std::string trim(std::string value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    return first >= last ? std::string{} : std::string(first, last);
}

std::string stripComment(const std::string& line) {
    bool single = false;
    bool doubled = false;
    bool escaped = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (doubled && ch == '\\' && !escaped) {
            escaped = true;
            continue;
        }
        if (ch == '\'' && !doubled) {
            single = !single;
        } else if (ch == '"' && !single && !escaped) {
            doubled = !doubled;
        } else if (ch == '#' && !single && !doubled) {
            return line.substr(0, i);
        }
        escaped = false;
    }
    return line;
}

std::size_t findEquals(const std::string& line) {
    bool single = false;
    bool doubled = false;
    bool escaped = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (doubled && ch == '\\' && !escaped) {
            escaped = true;
            continue;
        }
        if (ch == '\'' && !doubled) {
            single = !single;
        } else if (ch == '"' && !single && !escaped) {
            doubled = !doubled;
        } else if (ch == '=' && !single && !doubled) {
            return i;
        }
        escaped = false;
    }
    return std::string::npos;
}

std::string parseString(const std::string& raw, const std::string& context) {
    const std::string value = trim(raw);
    if (value.size() < 2 ||
        !((value.front() == '"' && value.back() == '"') ||
          (value.front() == '\'' && value.back() == '\''))) {
        return value;
    }
    if (value.front() == '\'') {
        return value.substr(1, value.size() - 2);
    }

    std::string parsed;
    parsed.reserve(value.size() - 2);
    for (std::size_t i = 1; i + 1 < value.size(); ++i) {
        if (value[i] != '\\') {
            parsed.push_back(value[i]);
            continue;
        }
        ++i;
        switch (value[i]) {
            case '\\': parsed.push_back('\\'); break;
            case '"': parsed.push_back('"'); break;
            case 'n': parsed.push_back('\n'); break;
            case 'r': parsed.push_back('\r'); break;
            case 't': parsed.push_back('\t'); break;
            default:
                throw std::invalid_argument(
                    "unsupported escape in " + context +
                    " (use TOML single quotes for Windows paths)");
        }
    }
    return parsed;
}

std::vector<std::string> parseArray(const std::string& raw,
                                    const std::string& context) {
    const std::string value = trim(raw);
    if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
        return {};
    }

    std::vector<std::string> values;
    std::string item;
    bool single = false;
    bool doubled = false;
    bool escaped = false;
    for (std::size_t i = 1; i + 1 < value.size(); ++i) {
        const char ch = value[i];
        if (doubled && ch == '\\' && !escaped) {
            escaped = true;
            item.push_back(ch);
            continue;
        }
        if (ch == '\'' && !doubled) {
            single = !single;
        } else if (ch == '"' && !single && !escaped) {
            doubled = !doubled;
        }
        if (ch == ',' && !single && !doubled) {
            if (trim(item).empty()) {
                throw std::invalid_argument("empty array item in " + context);
            }
            values.push_back(parseString(item, context));
            item.clear();
        } else {
            item.push_back(ch);
        }
        escaped = false;
    }
    if (single || doubled || trim(item).empty()) {
        throw std::invalid_argument("malformed array in " + context);
    }
    values.push_back(parseString(item, context));
    return values;
}

std::vector<TomlEntry> loadEntries(const std::string& path,
                                   const std::string& section_name) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::invalid_argument("cannot open --config file: " + path);
    }

    std::vector<TomlEntry> root;
    std::vector<TomlEntry> selected;
    std::string section;
    bool has_selected_section = false;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(stripComment(line));
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[') {
            if (line.size() < 3 || line.back() != ']') {
                throw std::invalid_argument(path + ":" + std::to_string(line_number) +
                                            ": malformed TOML table");
            }
            section = trim(line.substr(1, line.size() - 2));
            has_selected_section |= section == section_name;
            continue;
        }
        const std::size_t equals = findEquals(line);
        if (equals == std::string::npos) {
            throw std::invalid_argument(path + ":" + std::to_string(line_number) +
                                        ": expected key = value");
        }
        TomlEntry entry{trim(line.substr(0, equals)), trim(line.substr(equals + 1)),
                        line_number};
        if (entry.key.empty() || entry.value.empty()) {
            throw std::invalid_argument(path + ":" + std::to_string(line_number) +
                                        ": expected non-empty key and value");
        }
        if (section.empty()) {
            root.push_back(std::move(entry));
        } else if (section == section_name) {
            selected.push_back(std::move(entry));
        }
    }
    return has_selected_section ? selected : root;
}

std::string keyToOption(std::string key) {
    key = trim(std::move(key));
    if (key.size() >= 2 &&
        ((key.front() == '"' && key.back() == '"') ||
         (key.front() == '\'' && key.back() == '\''))) {
        key = key.substr(1, key.size() - 2);
    }
    std::replace(key.begin(), key.end(), '_', '-');
    if (key.empty() || key.front() == '-') {
        throw std::invalid_argument("invalid config key: " + key);
    }
    return "--" + key;
}

std::vector<std::string> entryArguments(const TomlEntry& entry,
                                        const std::string& path,
                                        const TomlCliSchema& schema) {
    const std::string context = path + ":" + std::to_string(entry.line_number);
    const std::string option = keyToOption(entry.key);
    const std::string value = trim(entry.value);
    std::string lowercase = value;
    std::transform(lowercase.begin(), lowercase.end(), lowercase.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    if (lowercase == "true") {
        const auto positive = schema.true_options.find(option);
        if (positive == schema.true_options.end()) {
            return {option};
        }
        return positive->second.empty()
            ? std::vector<std::string>{}
            : std::vector<std::string>{positive->second};
    }
    if (lowercase == "false") {
        const auto negative = schema.false_options.find(option);
        if (negative == schema.false_options.end()) {
            throw std::invalid_argument(
                context + ": boolean false is not supported for " + entry.key +
                "; omit default-off options instead");
        }
        return negative->second.empty()
            ? std::vector<std::string>{}
            : std::vector<std::string>{negative->second};
    }

    const auto array = parseArray(value, context);
    if (!array.empty()) {
        const auto style = schema.array_options.find(option);
        if (style == schema.array_options.end()) {
            throw std::invalid_argument(context + ": arrays are not supported for " +
                                        entry.key);
        }
        if (array.size() != 3) {
            throw std::invalid_argument(context + ": " + entry.key +
                                        " requires exactly 3 values");
        }
        if (style->second == TomlArrayStyle::COMMA_JOINED) {
            return {option, array[0] + "," + array[1] + "," + array[2]};
        }
        return {option, array[0], array[1], array[2]};
    }
    return {option, parseString(value, context)};
}

}  // namespace

std::vector<std::string> expandTomlConfigArguments(
    int argc, char* argv[], const TomlCliSchema& schema) {
    std::string config_path;
    std::vector<std::string> cli;
    cli.reserve(static_cast<std::size_t>(argc));
    cli.emplace_back(argv[0]);
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            if (!config_path.empty()) {
                throw std::invalid_argument("--config may only be specified once");
            }
            config_path = argv[++i];
        } else if (arg.rfind("--config=", 0) == 0) {
            if (!config_path.empty()) {
                throw std::invalid_argument("--config may only be specified once");
            }
            config_path = arg.substr(std::string("--config=").size());
            if (config_path.empty()) {
                throw std::invalid_argument("--config requires a path");
            }
        } else {
            cli.push_back(arg);
        }
    }
    if (config_path.empty()) {
        return cli;
    }

    std::vector<std::string> expanded{argv[0]};
    for (const auto& entry : loadEntries(config_path, schema.section_name)) {
        auto arguments = entryArguments(entry, config_path, schema);
        expanded.insert(expanded.end(),
                        std::make_move_iterator(arguments.begin()),
                        std::make_move_iterator(arguments.end()));
    }
    expanded.insert(expanded.end(),
                    std::make_move_iterator(cli.begin() + 1),
                    std::make_move_iterator(cli.end()));
    return expanded;
}

}  // namespace libgnss_apps
