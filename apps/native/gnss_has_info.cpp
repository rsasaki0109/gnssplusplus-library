// gnss_has_info: decode Galileo HAS signal-in-space pages (E6-B C/NAV) and
// summarise / dump the decoded MT1 messages and held corrections.

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <libgnss++/io/galileo_has.hpp>

namespace {

struct Options {
    std::string input_path;
    std::string format;
    std::string messages_csv;
    std::string corrections_csv;
    std::string updates_csv;
    std::string json_out;
    bool json = false;
    bool no_crc = false;
    bool operational_only = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " --input <pages> [options]\n"
        << "Decode Galileo HAS E6-B C/NAV pages (HAS SIS ICD 1.0) and summarise them.\n"
        << "  --input <file>             UBX (RXM-SFRBX), SBF (GALRawCNAV) or cssrlib page text\n"
        << "  --format <ubx|cssrlib|sbf> Input format (default: from the file extension;\n"
        << "                             anything other than .ubx / .sbf is cssrlib text)\n"
        << "  --messages-csv <file>      One row per decoded MT1 message\n"
        << "  --corrections-csv <file>   One row per decoded orbit / clock / bias entry\n"
        << "  --updates-csv <file>       Held per-satellite updates fed to PPP\n"
        << "  --operational-only         Ignore HASS = 0 (test mode) pages\n"
        << "  --no-crc                   Skip the C/NAV CRC-24Q page check\n"
        << "  --json                     Print the summary as JSON\n"
        << "  --json-out <file>          Also write the JSON summary to a file\n"
        << "  -h, --help                 Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

Options parseArguments(int argc, char* argv[]) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--input" && i + 1 < argc) {
            options.input_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            options.format = argv[++i];
        } else if (arg == "--messages-csv" && i + 1 < argc) {
            options.messages_csv = argv[++i];
        } else if (arg == "--corrections-csv" && i + 1 < argc) {
            options.corrections_csv = argv[++i];
        } else if (arg == "--updates-csv" && i + 1 < argc) {
            options.updates_csv = argv[++i];
        } else if (arg == "--operational-only") {
            options.operational_only = true;
        } else if (arg == "--no-crc") {
            options.no_crc = true;
        } else if (arg == "--json-out" && i + 1 < argc) {
            options.json_out = argv[++i];
        } else if (arg == "--json") {
            options.json = true;
        } else if (options.input_path.empty() && !arg.empty() && arg[0] != '-') {
            options.input_path = arg;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }
    if (options.input_path.empty()) {
        argumentError("--input is required", argv[0]);
    }
    return options;
}

std::string flagString(int flags) {
    std::string text;
    for (int bit = 5; bit >= 0; --bit) {
        text.push_back(((flags >> bit) & 1) != 0 ? '1' : '0');
    }
    return text;
}

std::string systemLetter(libgnss::GNSSSystem system) {
    switch (system) {
        case libgnss::GNSSSystem::GPS: return "G";
        case libgnss::GNSSSystem::Galileo: return "E";
        default: return "?";
    }
}

std::string satName(const libgnss::SatelliteId& satellite) {
    std::ostringstream oss;
    oss << systemLetter(satellite.system) << std::setw(2) << std::setfill('0')
        << static_cast<int>(satellite.prn);
    return oss.str();
}

std::string clockStatusName(libgnss::io::HasClockStatus status) {
    switch (status) {
        case libgnss::io::HasClockStatus::Ok: return "ok";
        case libgnss::io::HasClockStatus::NotAvailable: return "not_available";
        case libgnss::io::HasClockStatus::DoNotUse: return "do_not_use";
    }
    return "unknown";
}

bool writeMessagesCsv(const std::string& path,
                      const std::vector<libgnss::io::HasDecodedMessage>& messages) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "rx_week,rx_tow,ref_week,ref_tow,hass,mid,ms,status,toh,flags,mask_id,iod_set_id,"
           "orbit_vi,clock_full_vi,clock_subset_vi,code_bias_vi,phase_bias_vi,"
           "n_orbit,n_clock,n_code_bias,n_phase_bias,bits_used\n";
    out << std::fixed;
    for (const auto& message : messages) {
        const auto& mt1 = message.mt1;
        out << message.reception_time.week << "," << std::setprecision(3)
            << message.reception_time.tow << "," << message.reference_time.week << ","
            << message.reference_time.tow << "," << message.hass << "," << message.mid << ","
            << message.ms << "," << libgnss::io::hasMt1StatusName(message.status) << ","
            << mt1.toh << "," << flagString(mt1.flags) << "," << mt1.mask_id << ","
            << mt1.iod_set_id << "," << mt1.orbit_vi << "," << mt1.clock_full_vi << ","
            << mt1.clock_subset_vi << "," << mt1.code_bias_vi << "," << mt1.phase_bias_vi << ","
            << mt1.orbits.size() << "," << mt1.clocks.size() << "," << mt1.code_biases.size()
            << "," << mt1.phase_biases.size() << "," << mt1.bits_used << "\n";
    }
    return true;
}

bool writeCorrectionsCsv(const std::string& path,
                         const std::vector<libgnss::io::HasDecodedMessage>& messages) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "ref_week,ref_tow,toh,mask_id,iod_set_id,block,sat,iodref,signal,signal_name,"
           "radial_m,in_track_m,cross_track_m,clock_m,multiplier,dcc_raw,value,pdi,status\n";
    out << std::fixed;
    for (const auto& message : messages) {
        if (message.status != libgnss::io::HasMt1Status::Ok) {
            continue;
        }
        const auto& mt1 = message.mt1;
        std::ostringstream prefix;
        prefix << std::fixed << message.reference_time.week << "," << std::setprecision(3)
               << message.reference_time.tow << "," << mt1.toh << "," << mt1.mask_id << ","
               << mt1.iod_set_id << ",";
        out << std::setprecision(4);
        for (const auto& orbit : mt1.orbits) {
            out << prefix.str() << "orbit," << satName(orbit.satellite) << "," << orbit.iodref
                << ",,," << orbit.radial_m << "," << orbit.in_track_m << ","
                << orbit.cross_track_m << ",,,,,," << (orbit.available ? "ok" : "not_available")
                << "\n";
        }
        for (const auto& clock : mt1.clocks) {
            out << prefix.str() << (clock.subset ? "clock_subset," : "clock,")
                << satName(clock.satellite) << ",,,,,,," << clock.delta_clock_m << ","
                << clock.multiplier << "," << clock.dcc_raw << ",,," << clockStatusName(clock.status)
                << "\n";
        }
        for (const auto& bias : mt1.code_biases) {
            out << prefix.str() << "code_bias," << satName(bias.satellite) << ",,"
                << bias.signal_index << ","
                << libgnss::io::hasSignalName(bias.satellite.system, bias.signal_index)
                << ",,,,,,," << bias.value << ",," << (bias.available ? "ok" : "not_available")
                << "\n";
        }
        for (const auto& bias : mt1.phase_biases) {
            out << prefix.str() << "phase_bias," << satName(bias.satellite) << ",,"
                << bias.signal_index << ","
                << libgnss::io::hasSignalName(bias.satellite.system, bias.signal_index)
                << ",,,,,,," << bias.value << "," << bias.discontinuity << ","
                << (bias.available ? "ok" : "not_available") << "\n";
        }
    }
    return true;
}

bool writeUpdatesCsv(const std::string& path, const std::vector<libgnss::io::HasSsrUpdate>& updates) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "sat,week,tow,valid_until_tow,iode,mask_id,iod_set_id,orbit_tow,clock_tow,"
           "radial_m,in_track_m,cross_track_m,clock_m,code_biases_m,phase_biases_cycles\n";
    out << std::fixed;
    for (const auto& update : updates) {
        out << satName(update.satellite) << "," << update.time.week << "," << std::setprecision(3)
            << update.time.tow << "," << update.valid_until.tow << "," << update.iode << ","
            << update.mask_id << "," << update.iod_set_id << "," << update.orbit_time.tow << ","
            << update.clock_time.tow << "," << std::setprecision(4) << update.radial_m << ","
            << update.in_track_m << "," << update.cross_track_m << "," << update.clock_m << ",";
        bool first = true;
        for (const auto& [signal, value] : update.code_bias_m) {
            out << (first ? "" : ";")
                << libgnss::io::hasSignalName(update.satellite.system, signal) << "=" << value;
            first = false;
        }
        out << ",";
        first = true;
        for (const auto& [signal, value] : update.phase_bias_cycles) {
            out << (first ? "" : ";")
                << libgnss::io::hasSignalName(update.satellite.system, signal) << "=" << value;
            first = false;
        }
        out << "\n";
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    const Options options = parseArguments(argc, argv);
    libgnss::io::HasPageInputFormat format = libgnss::io::guessHasPageInputFormat(options.input_path);
    if (!options.format.empty() && !libgnss::io::parseHasPageInputFormat(options.format, format)) {
        argumentError("--format must be one of: ubx, cssrlib, sbf", argv[0]);
    }

    libgnss::io::GalileoHasDecoder::Options decoder_options;
    decoder_options.check_crc = !options.no_crc;
    decoder_options.accept_test_mode = !options.operational_only;
    libgnss::io::GalileoHasDecoder decoder(decoder_options);
    libgnss::io::HasPageReadStats read_stats;
    std::string error;
    if (!libgnss::io::decodeGalileoHasPages(options.input_path, format, decoder, &read_stats,
                                            &error)) {
        std::cerr << "Error: " << error << "\n";
        return 1;
    }

    const auto& stats = decoder.stats();
    const auto& messages = decoder.messages();
    std::map<std::string, size_t> flag_counts;
    std::map<std::string, size_t> vi_counts;
    std::map<std::string, std::set<std::string>> mask_signals;
    std::map<std::string, std::set<std::string>> mask_satellites;
    size_t ok_messages = 0;
    for (const auto& message : messages) {
        if (message.status != libgnss::io::HasMt1Status::Ok) {
            continue;
        }
        ++ok_messages;
        const auto& mt1 = message.mt1;
        ++flag_counts[flagString(mt1.flags)];
        const auto addVi = [&vi_counts](const char* name, int vi) {
            if (vi >= 0) {
                std::ostringstream key;
                key << name << "=" << libgnss::io::hasValidityIntervalSeconds(vi) << "s";
                ++vi_counts[key.str()];
            }
        };
        addVi("orbit", mt1.orbit_vi);
        addVi("clock", mt1.clock_full_vi);
        addVi("clock_subset", mt1.clock_subset_vi);
        addVi("code_bias", mt1.code_bias_vi);
        addVi("phase_bias", mt1.phase_bias_vi);
        if (mt1.has_mask) {
            for (const auto& system : mt1.mask.systems) {
                const std::string key = systemLetter(system.system);
                for (const int signal : system.signals) {
                    mask_signals[key].insert(libgnss::io::hasSignalName(system.system, signal));
                }
                for (const int prn : system.prns) {
                    mask_satellites[key].insert(
                        satName(libgnss::SatelliteId(system.system, static_cast<uint8_t>(prn))));
                }
            }
        }
    }

    if (!options.messages_csv.empty() && !writeMessagesCsv(options.messages_csv, messages)) {
        std::cerr << "Error: cannot write " << options.messages_csv << "\n";
        return 1;
    }
    if (!options.corrections_csv.empty() && !writeCorrectionsCsv(options.corrections_csv, messages)) {
        std::cerr << "Error: cannot write " << options.corrections_csv << "\n";
        return 1;
    }
    if (!options.updates_csv.empty() && !writeUpdatesCsv(options.updates_csv, decoder.updates())) {
        std::cerr << "Error: cannot write " << options.updates_csv << "\n";
        return 1;
    }

    const auto joinSet = [](const std::set<std::string>& values) {
        std::string text;
        for (const auto& value : values) {
            text += (text.empty() ? "" : " ") + value;
        }
        return text;
    };
    std::ostringstream json;
    json << "{\n"
         << "  \"input\": \"" << options.input_path << "\",\n"
         << "  \"format\": \"" << libgnss::io::hasPageInputFormatName(format) << "\",\n"
         << "  \"records\": " << read_stats.records << ",\n"
         << "  \"pages\": " << stats.pages << ",\n"
         << "  \"crc_failures\": " << stats.crc_failures << ",\n"
         << "  \"dummy_pages\": " << stats.dummy_pages << ",\n"
         << "  \"test_mode_pages\": " << stats.test_mode_pages << ",\n"
         << "  \"dont_use_pages\": " << stats.dont_use_pages << ",\n"
         << "  \"messages_decoded\": " << stats.messages_decoded << ",\n"
         << "  \"mt1_ok\": " << ok_messages << ",\n"
         << "  \"mt1_missing_mask\": " << stats.mt1_missing_mask << ",\n"
         << "  \"mt1_errors\": " << stats.mt1_errors << ",\n"
         << "  \"rs_failures\": " << stats.rs_failures << ",\n"
         << "  \"updates\": " << decoder.updates().size() << ",\n"
         << "  \"flag_patterns\": {";
    bool first_flag = true;
    for (const auto& [flags, count] : flag_counts) {
        json << (first_flag ? "" : ", ") << "\"" << flags << "\": " << count;
        first_flag = false;
    }
    json << "}\n}\n";
    if (!options.json_out.empty()) {
        std::ofstream out(options.json_out);
        if (!out) {
            std::cerr << "Error: cannot write " << options.json_out << "\n";
            return 1;
        }
        out << json.str();
    }
    if (options.json) {
        std::cout << json.str();
        return 0;
    }

    std::cout << "input: " << options.input_path << " ("
              << libgnss::io::hasPageInputFormatName(format) << ")\n"
              << "  records: " << read_stats.records << ", E6-B pages: " << read_stats.pages
              << " (skipped " << read_stats.skipped << ")\n"
              << "  pages: crc_failures=" << stats.crc_failures << " dummy=" << stats.dummy_pages
              << " test_mode=" << stats.test_mode_pages << " dont_use=" << stats.dont_use_pages
              << " reserved_status=" << stats.reserved_status_pages
              << " unsupported=" << stats.unsupported_type_pages
              << " redundant=" << stats.redundant_pages << "\n"
              << "  messages: decoded=" << stats.messages_decoded << " mt1_ok=" << ok_messages
              << " missing_mask=" << stats.mt1_missing_mask << " errors=" << stats.mt1_errors
              << " rs_failures=" << stats.rs_failures << " collection_resets="
              << stats.collection_resets << " flushes=" << stats.flushes << "\n";
    std::cout << "  flag patterns (mask|orbit|clock|clock subset|code bias|phase bias):";
    for (const auto& [flags, count] : flag_counts) {
        std::cout << " " << flags << "=" << count;
    }
    std::cout << "\n  validity intervals:";
    for (const auto& [key, count] : vi_counts) {
        std::cout << " " << key << "(" << count << ")";
    }
    std::cout << "\n";
    for (const auto& [system, signals] : mask_signals) {
        std::cout << "  mask " << system << ": satellites " << joinSet(mask_satellites[system])
                  << "; signals " << joinSet(signals) << "\n";
    }
    std::cout << "  held updates: " << decoder.updates().size()
              << " (unpaired clocks " << stats.unpaired_clocks << ")\n";
    return 0;
}
