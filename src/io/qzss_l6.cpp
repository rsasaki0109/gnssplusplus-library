// QZSS L6 / CLAS CSSR decoder.
// Decodes L6 binary frames into SSR corrections (orbit, clock, code/phase bias).

#include <libgnss++/io/qzss_l6.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/navigation.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>

namespace libgnss::qzss_l6 {

namespace {

// CSSR subtype IDs
constexpr int kSubtypeMask = 1;
constexpr int kSubtypeOrbit = 2;
constexpr int kSubtypeClock = 3;
constexpr int kSubtypeCodeBias = 4;
constexpr int kSubtypePhaseBias = 5;
constexpr int kSubtypeUra = 7;

// GNSS system IDs in CSSR
constexpr int kGnssGps = 0;
constexpr int kGnssGlonass = 1;
constexpr int kGnssGalileo = 2;
constexpr int kGnssBeidou = 3;
constexpr int kGnssQzssClas = 4;
constexpr int kGnssQzss = 5;

// SSR update interval table (IS-QZSS-L6-004, Table 4.2.2-9)
constexpr double kUdiTable[] = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800
};

constexpr int kVendorMadoca = 2;
constexpr int kVendorClas = 5;
constexpr int kServiceClockEphemeris = 0;
constexpr int kCssrType = 4073;

GNSSSystem cssrSystemToGnss(int sys_id, bool madoca_l6e_mode) {
    if (madoca_l6e_mode) {
        if (sys_id == 4) return GNSSSystem::QZSS;
        if (sys_id == 7) return GNSSSystem::BeiDou;
    }

    switch (sys_id) {
        case kGnssGps: return GNSSSystem::GPS;
        case kGnssGlonass: return GNSSSystem::GLONASS;
        case kGnssGalileo: return GNSSSystem::Galileo;
        case kGnssBeidou: return GNSSSystem::BeiDou;
        case kGnssQzssClas: return GNSSSystem::QZSS;
        case kGnssQzss: return GNSSSystem::QZSS;
        default: return GNSSSystem::UNKNOWN;
    }
}

int popcount(uint64_t v) {
    int c = 0;
    while (v) { c += (v & 1); v >>= 1; }
    return c;
}

std::vector<int> signalSlotsFromMask(uint16_t sigmask) {
    std::vector<int> slots;
    for (int i = 15; i >= 0; --i) {
        if ((sigmask >> i) & 1) slots.push_back(15 - i);
    }
    return slots;
}

bool isMadocaL6EPrn(int prn) {
    switch (prn) {
        case 204:
        case 205:
        case 206:
        case 207:
        case 209:
        case 210:
        case 211:
            return true;
        default:
            return false;
    }
}

SatelliteId normalizeQzssObservationId(const SatelliteId& satellite) {
    if (satellite.system == GNSSSystem::QZSS && satellite.prn >= 193) {
        return SatelliteId(GNSSSystem::QZSS,
                           static_cast<uint8_t>(satellite.prn - 192));
    }
    if (satellite.system == GNSSSystem::QZSS && satellite.prn >= 120 && satellite.prn <= 158) {
        return SatelliteId(GNSSSystem::QZSS,
                           static_cast<uint8_t>(satellite.prn - 119));
    }
    return satellite;
}

int qzssObservationPrn(uint8_t prn) {
    if (prn >= 120 && prn <= 158) {
        return static_cast<int>(prn) - 119;
    }
    return prn >= 193 ? static_cast<int>(prn) - 192 : static_cast<int>(prn);
}

int madocaFrameCountForSubtype(int subtype) {
    static constexpr int kFrameCountBySubtype[16] = {
        -1, 5, -1, 3, 5, 5, -1, 3,
        -1, -1, -1, -1, -1, -1, -1, -1,
    };
    return subtype >= 0 && subtype < 16 ? kFrameCountBySubtype[subtype] : -1;
}

double ssrUraSigmaMeters(int ura_index) {
    if (ura_index <= 0) {
        return 0.15;
    }
    if (ura_index >= 63) {
        return 5.4665;
    }
    return (std::pow(3.0, static_cast<double>((ura_index >> 3) & 0x07)) *
                (1.0 + static_cast<double>(ura_index & 0x07) / 4.0) -
            1.0) * 1e-3;
}

std::map<SatelliteId, std::map<int, double>> composeNetworkPhaseBiases(
    const std::map<SatelliteId, std::map<int, double>>& base_biases,
    const std::map<SatelliteId, std::map<int, double>>& network_biases) {
    auto composed = network_biases;
    for (const auto& [satellite, base_rows] : base_biases) {
        const auto network_sat_it = network_biases.find(satellite);
        if (network_sat_it == network_biases.end()) {
            continue;
        }
        auto& composed_rows = composed[satellite];
        for (const auto& [slot, base_bias] : base_rows) {
            auto row_it = composed_rows.find(slot);
            if (row_it != composed_rows.end()) {
                row_it->second += base_bias;
            } else {
                composed_rows[slot] = base_bias;
            }
        }
    }
    return composed;
}

std::map<SatelliteId, std::map<int, uint8_t>> composeNetworkPhaseDiscontinuity(
    const std::map<SatelliteId, std::map<int, uint8_t>>& base_pdi,
    const std::map<SatelliteId, std::map<int, uint8_t>>& network_pdi,
    const std::map<SatelliteId, std::map<int, double>>& network_biases) {
    auto composed = network_pdi;
    for (const auto& [satellite, base_rows] : base_pdi) {
        if (network_biases.find(satellite) == network_biases.end()) {
            continue;
        }
        auto& composed_rows = composed[satellite];
        for (const auto& [slot, base_value] : base_rows) {
            if (composed_rows.find(slot) == composed_rows.end()) {
                composed_rows[slot] = base_value;
            }
        }
    }
    return composed;
}

std::optional<double> readScaledSigned(BitReader& reader, int bits, double scale) {
    const int64_t raw = reader.readS(bits);
    const int64_t invalid_sentinel = -(int64_t{1} << (bits - 1));
    if (raw == invalid_sentinel) {
        return std::nullopt;
    }
    return static_cast<double>(raw) * scale;
}

std::string formatNullableDouble(const std::optional<double>& value) {
    return value ? std::to_string(*value) : std::string("nan");
}

void storeScaledTokenIfValid(std::map<std::string, std::string>& tokens,
                             const std::string& key,
                             const std::optional<double>& value) {
    if (value) {
        tokens[key] = std::to_string(*value);
    }
}

// Orbit correction scale factors (IS-QZSS-L6-004, Table 4.2.2-13)
constexpr double kOrbitRadialScale = 0.0016;  // radial meters per LSB
constexpr double kOrbitAlongCrossScale = 0.0064;  // along/cross meters per LSB
// Clock correction scale (Table 4.2.2-15)
constexpr double kClockScale = 0.0016;  // meters per LSB
// Code bias scale (Table 4.2.2-17)
constexpr double kCodeBiasScale = 0.02;  // meters per LSB
// Phase bias scale (Table 4.2.2-19)
constexpr double kPhaseBiasScale = 0.001;  // meters per LSB
// Phase discontinuity
constexpr int kPhaseDiscBits = 2;  // IS-QZSS-L6-004 Table 4.2.2-19

}  // namespace

void L6Decoder::processSubframe(const uint8_t* data, int bits, int gps_week) {
    gps_week_ = gps_week;
    BitReader reader(data, bits);

    while (reader.remaining() > 12) {
        const int msg_type = static_cast<int>(reader.readU(4));
        if (msg_type != 1) {  // CSSR type = 1
            break;
        }
        const int subtype = static_cast<int>(reader.readU(4));
        decodeCssrMessage(reader, gps_week);
        // Advance to byte boundary (if needed)
        break;  // One CSSR message per subframe (typical for CLAS)
    }
}

void L6Decoder::decodeCssrMessage(BitReader& reader, int gps_week) {
    // Already consumed msg_type(4) + subtype(4) → backtrack and re-read
    // Actually we need to re-read. Let me restructure.
    // The processSubframe already reads msg_type. We need subtype.
    // Let me fix: processSubframe reads msg_type=1, then we peek subtype.
}

double L6Decoder::computeTow(int epoch_time) const {
    int adjusted_epoch = epoch_time;
    if (mask_.ep0 > adjusted_epoch) {
        adjusted_epoch += 3600;
    }
    return mask_.tow0 + adjusted_epoch;
}

// Re-do: processSubframe should be the main parser
void L6Decoder::decodeSubtype1(BitReader& reader) {
    // Header for ST1: tow(20) + udi(4) + sync(1) + iod(4) + ngnss(4)
    const int tow = static_cast<int>(reader.readU(20));
    const int udi_idx = static_cast<int>(reader.readU(4));
    reader.readU(1);  // sync
    const int iod = static_cast<int>(reader.readU(4));
    const int ngnss = static_cast<int>(reader.readU(4));

    mask_.iod = iod;
    mask_.tow0 = (tow / 3600) * 3600;
    mask_.ep0 = tow % 3600;
    mask_.satellites.clear();

    for (int g = 0; g < ngnss; ++g) {
        const int sys_id = static_cast<int>(reader.readU(4));
        const uint64_t svmask = reader.readU(40);
        const uint16_t sigmask = static_cast<uint16_t>(reader.readU(16));
        const int cmi = static_cast<int>(reader.readU(1));
        const int nsat = popcount(svmask);
        const auto sys_signals = signalSlotsFromMask(sigmask);
        const int nsig = static_cast<int>(sys_signals.size());

        // PRN base per GNSS system (IS-QZSS-L6-004)
        int prn_base = 1;
        switch (sys_id) {
            case 4:
                prn_base = madoca_l6e_mode_ ? 193 : 1;
                break;
            case 5:
                prn_base = 193;
                break;
            case 7:
                prn_base = 19;
                break;
            default:
                prn_base = 1;
                break;
        }

        for (int s = 0; s < nsat; ++s) {
            // Find PRN: svmask bit index (0=MSB) maps to prn_base + index
            int prn = 0;
            int bit_count = 0;
            for (int b = 39; b >= 0; --b) {
                if ((svmask >> b) & 1) {
                    if (bit_count == s) {
                        prn = prn_base + (39 - b);
                        break;
                    }
                    ++bit_count;
                }
            }

            std::vector<int> sat_signals;
            if (cmi) {
                const uint64_t cellmask = reader.readU(nsig);
                for (int k = 0; k < nsig; ++k) {
                    if ((cellmask >> (nsig - 1 - k)) & 1) {
                        sat_signals.push_back(sys_signals[static_cast<size_t>(k)]);
                    }
                }
            } else {
                sat_signals = sys_signals;
            }

            CssrSatellite sat;
            sat.system = cssrSystemToGnss(sys_id, madoca_l6e_mode_);
            sat.prn = static_cast<uint8_t>(prn);
            sat.signal_slots = sat_signals;
            mask_.satellites.push_back(sat);
        }
    }

    current_epoch_.iod = iod;
    current_epoch_.mask = mask_;
    current_epoch_.tow = tow;
    current_epoch_.week = gps_week_;
}

void L6Decoder::decodeSubtype2(BitReader& reader) {
    // Header: tow_offset(12) + udi(4) + sync(1) + iod(4)
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);  // udi
    reader.readU(1);  // sync
    const int iod = static_cast<int>(reader.readU(4));

    const double tow = computeTow(tow_offset);

    std::map<SatelliteId, CssrOrbitCorrection> decoded_orbits;
    for (const auto& sat : mask_.satellites) {
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));
        // IODE: 10 bits for Galileo, 8 bits for others
        const int iode_bits = (sat.system == GNSSSystem::Galileo) ? 10 : 8;
        CssrOrbitCorrection corr;
        corr.iode = static_cast<int>(reader.readU(iode_bits));
        corr.dx = reader.readS(15) * kOrbitRadialScale;   // radial
        corr.dy = reader.readS(13) * kOrbitAlongCrossScale;  // along-track
        corr.dz = reader.readS(13) * kOrbitAlongCrossScale;  // cross-track
        decoded_orbits[sat_id] = corr;
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }
    current_epoch_.tow = tow;
    current_epoch_.orbits = decoded_orbits;
    current_epoch_.has_orbit = true;
}

void L6Decoder::decodeSubtype3(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));

    const double tow = computeTow(tow_offset);

    std::map<SatelliteId, CssrClockCorrection> decoded_clocks;
    for (const auto& sat : mask_.satellites) {
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));
        CssrClockCorrection corr;
        corr.dclock_m = reader.readS(15) * kClockScale;
        decoded_clocks[sat_id] = corr;
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }
    current_epoch_.tow = tow;
    current_epoch_.clocks = decoded_clocks;
    current_epoch_.has_clock = true;
}

void L6Decoder::decodeSubtype4(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));

    std::map<SatelliteId, std::map<int, double>> decoded_biases;
    for (const auto& sat : mask_.satellites) {
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));
        auto& biases = decoded_biases[sat_id];
        for (const int sig_id : sat.signal_slots) {
            biases[sig_id] = reader.readS(11) * kCodeBiasScale;
        }
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }

    for (const auto& [sat_id, biases] : decoded_biases) {
        base_code_biases_[sat_id] = biases;
    }
    current_epoch_.code_biases = base_code_biases_;
    for (const auto& [sat, nb] : network_code_biases_)
        for (const auto& [sig, val] : nb)
            current_epoch_.code_biases[sat][sig] = val;
    current_epoch_.has_code_bias = !current_epoch_.code_biases.empty();
}

void L6Decoder::decodeSubtype5(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));

    std::map<SatelliteId, std::map<int, double>> decoded_biases;
    std::map<SatelliteId, std::map<int, uint8_t>> decoded_pdi;
    for (const auto& sat : mask_.satellites) {
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));
        auto& biases = decoded_biases[sat_id];
        auto& pdi = decoded_pdi[sat_id];
        for (const int sig_id : sat.signal_slots) {
            biases[sig_id] = reader.readS(15) * kPhaseBiasScale;
            pdi[sig_id] = static_cast<uint8_t>(reader.readU(kPhaseDiscBits));
        }
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }

    for (const auto& [sat_id, biases] : decoded_biases) {
        base_phase_biases_[sat_id] = biases;
        base_phase_discontinuity_[sat_id] = decoded_pdi[sat_id];
    }
    current_epoch_.phase_biases = base_phase_biases_;
    current_epoch_.phase_discontinuity_indicators = base_phase_discontinuity_;
    for (const auto& [sat, nb] : network_phase_biases_)
        for (const auto& [sig, val] : nb)
            current_epoch_.phase_biases[sat][sig] = val;
    current_epoch_.has_phase_bias = !current_epoch_.phase_biases.empty();
}

void L6Decoder::decodeSubtype7(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));

    std::map<SatelliteId, int> decoded_uras;
    for (const auto& sat : mask_.satellites) {
        decoded_uras[normalizeQzssObservationId(SatelliteId(sat.system, sat.prn))] =
            static_cast<int>(reader.readU(6));
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }

    current_epoch_.tow = computeTow(tow_offset);
    for (const auto& [sat_id, ura_index] : decoded_uras) {
        base_ura_indices_[sat_id] = ura_index;
    }
    current_epoch_.ura_indices = base_ura_indices_;
    current_epoch_.has_ura = !current_epoch_.ura_indices.empty();
}

void L6Decoder::decodeSubtype11(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));

    const bool flg_orbit = reader.readU(1) != 0;
    const bool flg_clock = reader.readU(1) != 0;
    const bool flg_net = reader.readU(1) != 0;

    const int nsat = static_cast<int>(mask_.satellites.size());
    uint64_t selected_mask = 0;
    if (flg_net) {
        reader.readU(5);  // network_id
        selected_mask = reader.readU(nsat);
    }

    const double tow = computeTow(tow_offset);
    std::map<SatelliteId, CssrOrbitCorrection> decoded_orbits;
    std::map<SatelliteId, CssrClockCorrection> decoded_clocks;

    for (int i = 0; i < nsat; ++i) {
        if (flg_net && ((selected_mask >> (nsat - 1 - i)) & 1) == 0) continue;
        const auto& sat = mask_.satellites[static_cast<size_t>(i)];
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));

        if (flg_orbit) {
            const int iode_bits = (sat.system == GNSSSystem::Galileo) ? 10 : 8;
            CssrOrbitCorrection corr;
            corr.iode = static_cast<int>(reader.readU(iode_bits));
            corr.dx = reader.readS(15) * kOrbitRadialScale;
            corr.dy = reader.readS(13) * kOrbitAlongCrossScale;
            corr.dz = reader.readS(13) * kOrbitAlongCrossScale;
            decoded_orbits[sat_id] = corr;
        }
        if (flg_clock) {
            CssrClockCorrection corr;
            corr.dclock_m = reader.readS(15) * kClockScale;
            decoded_clocks[sat_id] = corr;
        }
    }

    if (mask_.tow0 < 0 || iod != mask_.iod) {
        return;
    }
    current_epoch_.tow = tow;
    if (flg_orbit) {
        current_epoch_.orbits = decoded_orbits;
        current_epoch_.has_orbit = !decoded_orbits.empty();
    }
    if (flg_clock) {
        current_epoch_.clocks = decoded_clocks;
        current_epoch_.has_clock = !decoded_clocks.empty();
    }
}

void L6Decoder::decodeSubtype6(BitReader& reader) {
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4);
    reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));
    // IOD check removed: always decode to maintain bit position

    const bool flg_code = reader.readU(1) != 0;
    const bool flg_phase = reader.readU(1) != 0;
    const bool flg_network = reader.readU(1) != 0;
    const int nsat = static_cast<int>(mask_.satellites.size());
    uint64_t selected_mask = (1ULL << nsat) - 1;  // all selected by default
    int network_id = 0;
    if (flg_network) {
        network_id = static_cast<int>(reader.readU(5));
        selected_mask = reader.readU(nsat);
    }

    for (int i = 0; i < nsat; ++i) {
        if (((selected_mask >> (nsat - 1 - i)) & 1) == 0) continue;
        const auto& sat = mask_.satellites[static_cast<size_t>(i)];
        const SatelliteId sat_id = normalizeQzssObservationId(SatelliteId(sat.system, sat.prn));
        const int nsig = static_cast<int>(sat.signal_slots.size());

        if (flg_code) {
            for (int s = 0; s < nsig; ++s) {
                reader.readS(11);  // code bias (handled via ST4 base)
            }
        }
        if (flg_phase) {
            // ST6 network-specific phase biases: store per-network
            auto& net_biases = current_epoch_.network_phase_biases[network_id][sat_id];
            auto& net_pdi =
                current_epoch_.network_phase_discontinuity_indicators[network_id][sat_id];
            for (int s = 0; s < nsig; ++s) {
                net_biases[sat.signal_slots[static_cast<size_t>(s)]] =
                    reader.readS(15) * kPhaseBiasScale;
                net_pdi[sat.signal_slots[static_cast<size_t>(s)]] =
                    static_cast<uint8_t>(reader.readU(kPhaseDiscBits));
            }
            // Don't set current_epoch_.phase_biases here.
            // ST6 updates will be picked up at next ST5 or clock emit.
        }
    }
}

void L6Decoder::decodeSubtype8(BitReader& reader) {
    // STEC polynomial coefficients
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4); reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));
    const bool iod_match = (iod == mask_.iod);

    const int stec_type = static_cast<int>(reader.readU(2));
    const int network_id = static_cast<int>(reader.readU(5));
    const int nsat = static_cast<int>(mask_.satellites.size());
    const uint64_t selected_mask = reader.readU(nsat);

    auto& tokens = current_epoch_.atmos_by_network[network_id];
    tokens["atmos_network_id"] = std::to_string(network_id);
    tokens["atmos_stec_avail"] = "1";

    auto satLabel = [](const CssrSatellite& s) {
        char buf[8];
        char sys = '?';
        switch (s.system) {
            case GNSSSystem::GPS: sys='G'; break;
            case GNSSSystem::Galileo: sys='E'; break;
            case GNSSSystem::QZSS: sys='J'; break;
            case GNSSSystem::GLONASS: sys='R'; break;
            case GNSSSystem::BeiDou: sys='C'; break;
            default: break;
        }
        const int prn = s.system == GNSSSystem::QZSS
                            ? qzssObservationPrn(s.prn)
                            : static_cast<int>(s.prn);
        std::snprintf(buf, sizeof(buf), "%c%02d", sys, prn);
        return std::string(buf);
    };

    int selected_count = 0;
    for (int i = 0; i < nsat; ++i) {
        if (((selected_mask >> (nsat - 1 - i)) & 1) == 0) continue;
        const auto& sat = mask_.satellites[static_cast<size_t>(i)];
        const std::string key = satLabel(sat);
        ++selected_count;

        reader.readU(6); // stec_quality
        storeScaledTokenIfValid(
            tokens, "atmos_stec_c00_tecu:" + key, readScaledSigned(reader, 14, 0.05));
        tokens["atmos_stec_type:" + key] = std::to_string(stec_type);

        if (stec_type > 0) {
            storeScaledTokenIfValid(
                tokens,
                "atmos_stec_c01_tecu_per_deg:" + key,
                readScaledSigned(reader, 12, 0.02));
            storeScaledTokenIfValid(
                tokens,
                "atmos_stec_c10_tecu_per_deg:" + key,
                readScaledSigned(reader, 12, 0.02));
        }
        if (stec_type > 1) {
            storeScaledTokenIfValid(
                tokens,
                "atmos_stec_c11_tecu_per_deg2:" + key,
                readScaledSigned(reader, 10, 0.02));
        }
        if (stec_type > 2) {
            storeScaledTokenIfValid(
                tokens,
                "atmos_stec_c02_tecu_per_deg2:" + key,
                readScaledSigned(reader, 8, 0.005));
            storeScaledTokenIfValid(
                tokens,
                "atmos_stec_c20_tecu_per_deg2:" + key,
                readScaledSigned(reader, 8, 0.005));
        }
    }
    tokens["atmos_selected_satellites"] = std::to_string(selected_count);
    current_epoch_.has_atmos = true;
    // Update persistent merged atmos (Python pending_atmos equivalent)
    for (const auto& [k, v] : tokens) merged_atmos_[k] = v;
}

void L6Decoder::decodeSubtype9(BitReader& reader) {
    // Gridded troposphere + STEC residuals
    const int tow_offset = static_cast<int>(reader.readU(12));
    reader.readU(4); reader.readU(1);
    const int iod = static_cast<int>(reader.readU(4));
    // IOD check removed: always decode to maintain bit position

    const int trop_type = static_cast<int>(reader.readU(2));
    const int stec_residual_range = static_cast<int>(reader.readU(1));
    const int network_id = static_cast<int>(reader.readU(5));
    const int nsat = static_cast<int>(mask_.satellites.size());
    const uint64_t selected_mask = reader.readU(nsat);
    reader.readU(6); // trop_quality
    const int grid_count = static_cast<int>(reader.readU(6));

    // Collect selected satellites
    std::vector<std::string> sel_sats;
    auto satLabel = [](const CssrSatellite& s) {
        char buf[8]; char sys='?';
        switch(s.system) {
            case GNSSSystem::GPS:sys='G';break; case GNSSSystem::Galileo:sys='E';break;
            case GNSSSystem::QZSS:sys='J';break; case GNSSSystem::GLONASS:sys='R';break;
            case GNSSSystem::BeiDou:sys='C';break; default:break;
        }
        const int prn = s.system == GNSSSystem::QZSS
                            ? qzssObservationPrn(s.prn)
                            : static_cast<int>(s.prn);
        std::snprintf(buf,sizeof(buf),"%c%02d",sys,prn);
        return std::string(buf);
    };
    for (int i = 0; i < nsat; ++i) {
        if (((selected_mask >> (nsat-1-i)) & 1) == 0) continue;
        sel_sats.push_back(satLabel(mask_.satellites[static_cast<size_t>(i)]));
    }

    const int stec_bits = stec_residual_range == 0 ? 7 : 16;

    // Read grid data
    std::string trop_hs, trop_wet;
    std::map<std::string, std::string> stec_residuals;
    for (int g = 0; g < grid_count; ++g) {
        const auto hs = readScaledSigned(reader, 9, 0.004);
        const auto wet = readScaledSigned(reader, 8, 0.004);
        if (g > 0) { trop_hs += ";"; trop_wet += ";"; }
        trop_hs += formatNullableDouble(hs);
        trop_wet += formatNullableDouble(wet);

        for (const auto& sat_key : sel_sats) {
            const auto res = readScaledSigned(reader, stec_bits, 0.04);
            if (g > 0) stec_residuals[sat_key] += ";";
            stec_residuals[sat_key] += formatNullableDouble(res);
        }
    }

    auto& tokens = current_epoch_.atmos_by_network[network_id];
    tokens["atmos_network_id"] = std::to_string(network_id);
    tokens["atmos_trop_avail"] = trop_type != 0 ? "1" : "0";
    tokens["atmos_stec_avail"] = sel_sats.empty() ? "0" : "2";
    tokens["atmos_grid_count"] = std::to_string(grid_count);
    tokens["atmos_trop_type"] = std::to_string(trop_type);
    tokens["atmos_trop_hs_residuals_m"] = trop_hs;
    tokens["atmos_trop_wet_residuals_m"] = trop_wet;
    tokens["atmos_stec_residual_range"] = std::to_string(stec_residual_range);
    tokens["atmos_selected_satellites"] = std::to_string(static_cast<int>(sel_sats.size()));
    for (const auto& sat_key : sel_sats) {
        tokens["atmos_stec_residual_size:" + sat_key] = std::to_string(stec_residual_range);
        tokens["atmos_stec_residuals_tecu:" + sat_key] = stec_residuals[sat_key];
    }
    current_epoch_.has_atmos = true;
    current_epoch_.last_gridded_network_id = network_id;
    // Update persistent merged atmos
    for (const auto& [k, v] : tokens) merged_atmos_[k] = v;
}

SatelliteId L6Decoder::maskIndexToSatId(int index) const {
    if (index >= 0 && index < static_cast<int>(mask_.satellites.size())) {
        const auto& s = mask_.satellites[static_cast<size_t>(index)];
        return SatelliteId(s.system, s.prn);
    }
    return {};
}

std::vector<CssrEpoch> L6Decoder::feedFrame(const uint8_t* frame_data, int gps_week) {
    completed_epochs_.clear();

    // Parse L6 frame header: preamble(32) + prn(8) + message_type(8) + alert(1) = 49 bits
    constexpr int kHeaderBits = 49;
    const uint32_t preamble =
        (static_cast<uint32_t>(frame_data[0]) << 24) |
        (static_cast<uint32_t>(frame_data[1]) << 16) |
        (static_cast<uint32_t>(frame_data[2]) << 8) |
        static_cast<uint32_t>(frame_data[3]);
    if (preamble != kL6Preamble) return {};

    // Header is byte-aligned: preamble(4 bytes) + prn(1 byte) + msg_type(1 byte) + alert(1 bit)
    const int prn = frame_data[4];
    const int msg_type = frame_data[5];

    // message_type encodes: vendor_id(3) | facility_id(2) | reserved(2) | subframe_start(1)
    int vendor_id = (msg_type >> 5) & 0x7;
    const int facility_id = (msg_type >> 3) & 0x3;
    const int service_id = (msg_type >> 2) & 0x1;
    const bool alert = (frame_data[6] & 0x80) == 0x80;
    const bool subframe_start = (msg_type & 0x1) != 0;

    // Extract data part (1695 bits starting at bit 49)
    std::vector<uint8_t> data_part((kDataPartBits + 7) / 8, 0);
    for (int i = 0; i < kDataPartBits; ++i) {
        const int src_bit = kHeaderBits + i;
        const int src_byte = src_bit / 8;
        const int src_pos = 7 - (src_bit % 8);
        if (src_byte >= kFrameBytes) break;
        const int bit_val = (frame_data[src_byte] >> src_pos) & 1;
        const int dst_byte = i / 8;
        const int dst_pos = 7 - (i % 8);
        data_part[static_cast<size_t>(dst_byte)] |= static_cast<uint8_t>(bit_val << dst_pos);
    }

    const bool madoca_l6e = vendor_id == kVendorMadoca &&
                            service_id == kServiceClockEphemeris &&
                            isMadocaL6EPrn(prn);
    if (alert) {
        return {};
    }
    madoca_l6e_mode_ = madoca_l6e;
    if (madoca_l6e) {
        vendor_id = kVendorClas;
    }
    // Decode CLAS directly; MADOCA L6E is normalized after header validation.
    ++frame_index_;
    if (vendor_id != 5) return {};

    const auto key = std::make_tuple(prn, vendor_id, facility_id);
    auto& pending = pending_subframes_[key];

    if (subframe_start || pending.frame_data_parts.empty()) {
        pending.frame_data_parts.clear();
        pending.expected_frames = kSubframeFrames;
        if (madoca_l6e) {
            BitReader first_message(data_part.data(), kDataPartBits);
            const int cssr_type = static_cast<int>(first_message.readU(12));
            const int subtype = static_cast<int>(first_message.readU(4));
            if (cssr_type == kCssrType) {
                pending.expected_frames = madocaFrameCountForSubtype(subtype);
            } else {
                return {};
            }
            if (pending.expected_frames < 1) {
                return {};
            }
        }
    }
    pending.frame_data_parts.push_back(data_part);

    if (static_cast<int>(pending.frame_data_parts.size()) < pending.expected_frames) {
        return {};
    }

    // Assemble subframe: concatenate all data parts (no header bits to skip,
    // vendor/facility/start are in the frame header, not the data part)
    std::vector<uint8_t> subframe_data((kSubframeBits + 7) / 8, 0);
    int dst_bit = 0;
    for (const auto& part : pending.frame_data_parts) {
        for (int src = 0; src < kDataPartBits; ++src) {
            const int byte_idx = src / 8;
            const int bit_idx = 7 - (src % 8);
            const int bit_val = (part[static_cast<size_t>(byte_idx)] >> bit_idx) & 1;
            const int d_byte = dst_bit / 8;
            const int d_bit = 7 - (dst_bit % 8);
            subframe_data[static_cast<size_t>(d_byte)] |= static_cast<uint8_t>(bit_val << d_bit);
            ++dst_bit;
        }
    }
    pending.frame_data_parts.clear();

    // Decode CSSR messages from subframe
    BitReader sfr(subframe_data.data(), dst_bit);

    while (sfr.remaining() > 16) {
        const int cssr_type = static_cast<int>(sfr.readU(12));
        if (cssr_type == 0) break;  // padding
        if (cssr_type != kCssrType) break;
        const int subtype = static_cast<int>(sfr.readU(4));

        const int pos_before = sfr.position();
        switch (subtype) {
            case kSubtypeMask: decodeSubtype1(sfr); break;
            case kSubtypeOrbit: decodeSubtype2(sfr); break;
            case kSubtypeClock: decodeSubtype3(sfr); break;
            case kSubtypeCodeBias: decodeSubtype4(sfr); break;
            case kSubtypePhaseBias: decodeSubtype5(sfr); break;
            case 6: decodeSubtype6(sfr); break;   // code+phase bias (network)
            case kSubtypeUra: decodeSubtype7(sfr); break;
            case 8: decodeSubtype8(sfr); break;   // STEC polynomial
            case 9: decodeSubtype9(sfr); break;   // gridded trop + STEC residuals
            case 11: decodeSubtype11(sfr); break;  // combined
            default:
                // Unknown subtype (10=service info, 12=atmos, etc.)
                // Cannot determine size without full decode — stop this subframe
                sfr.skip(sfr.remaining());
                break;
        }
        // Safety: if decoder didn't advance, stop to prevent infinite loop
        if (sfr.position() <= pos_before) break;

        // Emit epoch immediately after clock message (ST3 or ST11).
        // Later messages in the same subframe are patched into the emitted row below.
        if (current_epoch_.has_clock && !mask_.satellites.empty()) {
            current_epoch_.week = gps_week;
            current_epoch_.merged_atmos = merged_atmos_;
            completed_epochs_.push_back(current_epoch_);
            current_epoch_.has_clock = false;
        }
    }

    // Retroactive updates: MADOCA decodes every message in the assembled subframe
    // before exposing the resulting SSR row.
    if (!completed_epochs_.empty()) {
        auto& last = completed_epochs_.back();
        if (current_epoch_.has_code_bias) {
            last.code_biases = current_epoch_.code_biases;
            last.has_code_bias = !last.code_biases.empty();
        }
        if (current_epoch_.has_phase_bias) {
            last.phase_biases = current_epoch_.phase_biases;
            last.phase_discontinuity_indicators = current_epoch_.phase_discontinuity_indicators;
            last.has_phase_bias = !last.phase_biases.empty();
        }
        if (current_epoch_.has_ura) {
            last.ura_indices = current_epoch_.ura_indices;
            last.has_ura = !last.ura_indices.empty();
        }
        if (current_epoch_.has_atmos) {
            for (const auto& [net_id, tokens] : current_epoch_.atmos_by_network)
                last.atmos_by_network[net_id] = tokens;
            if (current_epoch_.last_gridded_network_id > 0)
                last.last_gridded_network_id = current_epoch_.last_gridded_network_id;
            last.has_atmos = true;
        }
    }

    return completed_epochs_;
}

std::vector<CssrEpoch> L6Decoder::decodeFile(const std::string& path, int gps_week) {
    std::vector<CssrEpoch> all_epochs;
    std::ifstream file(path, std::ios::binary);
    if (!file) return all_epochs;

    std::array<uint8_t, kFrameBytes> frame{};
    while (file.read(reinterpret_cast<char*>(frame.data()), kFrameBytes)) {
        auto epochs = feedFrame(frame.data(), gps_week);
        all_epochs.insert(all_epochs.end(), epochs.begin(), epochs.end());
    }

    // Post-process: carry-forward atmos networks across epochs.
    {
        std::map<int, std::map<std::string, std::string>> carried;
        for (auto& ep : all_epochs) {
            for (const auto& [net_id, tokens] : ep.atmos_by_network)
                carried[net_id] = tokens;
            ep.atmos_by_network = carried;
            ep.has_atmos = !carried.empty();
        }
    }

    // Carry-forward ST6 network phase biases across epochs.
    // Each network's ST6 arrives once per 30-second mask cycle.
    {
        std::map<int, std::map<SatelliteId, std::map<int, double>>> carried_biases;
        std::map<int, std::map<SatelliteId, std::map<int, uint8_t>>> carried_pdi;
        for (auto& ep : all_epochs) {
            for (const auto& [net_id, sat_biases] : ep.network_phase_biases)
                carried_biases[net_id] = sat_biases;
            for (const auto& [net_id, sat_pdi] : ep.network_phase_discontinuity_indicators)
                carried_pdi[net_id] = sat_pdi;
            ep.network_phase_biases = carried_biases;
            ep.network_phase_discontinuity_indicators = carried_pdi;
        }
    }

    // Post-process: build CSV-style merged corrections.
    // Emulate Python's pending_atmos accumulation while keeping ST5 base phase
    // biases separate from ST6 network-specific phase biases.  CLASLIB composes
    // network phase bias rows as ST5 base + ST6 network delta.
    {
        std::map<std::string, std::string> pending_atmos;
        std::map<SatelliteId, std::map<int, double>> base_pbias;
        std::map<SatelliteId, std::map<int, uint8_t>> base_pdi;
        for (auto& ep : all_epochs) {
            // Update pending atmos (message-order union, same as Python)
            for (const auto& [net_id, tokens] : ep.atmos_by_network) {
                for (const auto& [k, v] : tokens)
                    pending_atmos[k] = v;
            }
            ep.merged_atmos = pending_atmos;

            // Update pending phase bias base from ST5.  ST6 rows are composed
            // below per network and stay out of the base correction row.
            if (ep.has_phase_bias) {
                base_pbias = ep.phase_biases;
                base_pdi = ep.phase_discontinuity_indicators;
            }

            std::map<int, std::map<SatelliteId, std::map<int, double>>> composed_network_biases;
            std::map<int, std::map<SatelliteId, std::map<int, uint8_t>>> composed_network_pdi;
            for (const auto& [net_id, sat_biases] : ep.network_phase_biases) {
                const auto composed_biases =
                    composeNetworkPhaseBiases(base_pbias, sat_biases);
                if (!composed_biases.empty()) {
                    composed_network_biases[net_id] = composed_biases;
                }

                static const std::map<SatelliteId, std::map<int, uint8_t>> kEmptyPdi;
                const auto net_pdi_it =
                    ep.network_phase_discontinuity_indicators.find(net_id);
                const auto& network_pdi =
                    net_pdi_it != ep.network_phase_discontinuity_indicators.end()
                        ? net_pdi_it->second
                        : kEmptyPdi;
                const auto composed_pdi =
                    composeNetworkPhaseDiscontinuity(base_pdi, network_pdi, sat_biases);
                if (!composed_pdi.empty()) {
                    composed_network_pdi[net_id] = composed_pdi;
                }
            }

            ep.network_phase_biases = composed_network_biases;
            ep.network_phase_discontinuity_indicators = composed_network_pdi;
            ep.phase_biases = base_pbias;
            ep.phase_discontinuity_indicators = base_pdi;
            ep.has_phase_bias = !base_pbias.empty();
        }
    }

    return all_epochs;
}

namespace {

// CSSR signal slot → RTCM SSR signal ID (IS-QZSS-L6-004 Table 4.2.2-6)
uint8_t cssrSignalSlotToRtcmId(int gnss_id, int slot) {
    // Full mapping from IS-QZSS-L6-004 Table 4.2.2-6
    // gnss_id: 0=GPS, 1=GLO, 2=GAL, 3=BDS, 4/5=QZSS
    switch (gnss_id) {
        case 0: // GPS: 0-2→L1(2), 3-5→L1P(3), 6-8→L2CM(8), 9-10→L2P(9), 11-13→L5(22)
            if (slot <= 2) return 2;
            if (slot <= 5) return 3;
            if (slot <= 8) return 8;
            if (slot <= 10) return 9;
            if (slot <= 13) return 22;
            return 0;
        case 1: // GLONASS: 0→L1CA(2), 1→L1P(3), 2→L2CA(8), 3→L2P(9)
            if (slot == 0) return 2;
            if (slot == 1) return 3;
            if (slot == 2) return 8;
            if (slot == 3) return 9;
            return 0;
        case 2: // Galileo: 0-2→E1(2), 3-5→E5a(22), 6-8→E5b(14)
            if (slot <= 2) return 2;
            if (slot <= 5) return 22;
            if (slot <= 8) return 14;
            return 0;
        case 3: case 7: // BeiDou: 0-2→B1I(2), 3-5→B3I(8), 6-8→B2I(14)
            if (slot <= 2) return 2;
            if (slot <= 5) return 8;
            if (slot <= 8) return 14;
            return 0;
        case 4: case 5: // QZSS: 0-3→L1(2), 4-6→L2(8), 7-9→L5(22)
            if (slot <= 3) return 2;
            if (slot <= 6) return 8;
            if (slot <= 9) return 22;
            return 0;
        default: return 0;
    }
}

int gnssSystemToCssrId(GNSSSystem sys) {
    switch (sys) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::BeiDou: return 3;
        case GNSSSystem::QZSS: return 5;
        default: return -1;
    }
}

}  // namespace

void populateSSRProducts(
    const std::vector<CssrEpoch>& epochs,
    libgnss::SSRProducts& products,
    int preferred_network_id) {
    products.clear();
    products.setOrbitCorrectionsAreRac(true);

    for (const auto& epoch : epochs) {
        const GNSSTime time{epoch.week, epoch.tow};

        for (const auto& [sat_id, clock_corr] : epoch.clocks) {
            const SatelliteId product_sat_id = normalizeQzssObservationId(sat_id);

            libgnss::SSROrbitClockCorrection corr;
            corr.satellite = product_sat_id;
            corr.time = time;
            corr.clock_correction_m = clock_corr.dclock_m;
            corr.clock_valid = true;

            // Orbit (may not be present every epoch)
            auto orbit_it = epoch.orbits.find(sat_id);
            if (orbit_it != epoch.orbits.end()) {
                const auto& o = orbit_it->second;
                corr.orbit_correction_ecef = Vector3d(o.dx, o.dy, o.dz);
                corr.orbit_iode = o.iode;
                corr.orbit_valid = true;
            }

            // Code biases
            auto cb_it = epoch.code_biases.find(sat_id);
            if (cb_it != epoch.code_biases.end()) {
                const int gnss_id = gnssSystemToCssrId(sat_id.system);
                for (const auto& [slot, bias_m] : cb_it->second) {
                    const uint8_t rtcm_id = cssrSignalSlotToRtcmId(gnss_id, slot);
                    if (rtcm_id > 0) {
                        // PPP applies SSR code bias by subtraction.  MADOCALIB's
                        // MADOCA path applies decoded ST4 code bias by addition.
                        corr.code_bias_m[rtcm_id] = -bias_m;
                    }
                }
                corr.code_bias_valid = !corr.code_bias_m.empty();
            }

            // Phase biases from ST5 base.  ST6 network-specific composed rows
            // are emitted separately below with a non-zero bias_network_id.
            auto pb_it = epoch.phase_biases.find(sat_id);
            if (pb_it != epoch.phase_biases.end()) {
                const int gnss_id = gnssSystemToCssrId(sat_id.system);
                for (const auto& [slot, bias_m] : pb_it->second) {
                    const uint8_t rtcm_id = cssrSignalSlotToRtcmId(gnss_id, slot);
                    if (rtcm_id > 0) {
                        // PPP applies SSR phase bias by subtraction.  MADOCALIB's
                        // MADOCA path applies decoded ST5/ST6 phase bias by addition.
                        corr.phase_bias_m[rtcm_id] = -bias_m;
                    }
                }
                corr.phase_bias_valid = !corr.phase_bias_m.empty();

                // Parallel phase discontinuity indicators keyed on the same RTCM ids.
                auto pdi_it = epoch.phase_discontinuity_indicators.find(sat_id);
                if (pdi_it != epoch.phase_discontinuity_indicators.end()) {
                    for (const auto& [slot, pdi_value] : pdi_it->second) {
                        const uint8_t rtcm_id = cssrSignalSlotToRtcmId(gnss_id, slot);
                        if (rtcm_id > 0) {
                            corr.phase_discontinuity_indicators[rtcm_id] = pdi_value;
                        }
                    }
                }
            }

            auto ura_it = epoch.ura_indices.find(sat_id);
            if (ura_it != epoch.ura_indices.end()) {
                corr.ura_sigma_m = ssrUraSigmaMeters(ura_it->second);
                corr.ura_valid = std::isfinite(corr.ura_sigma_m);
            }

            // Use merged_atmos (Python pending_atmos equivalent):
            // flat dict accumulated in message order across all subframes.
            // network_id, trop, STEC residuals from the last ST9.
            // STEC polynomial from the last ST8.
            // Phase biases are already merged in epoch.phase_biases.
            if (!epoch.merged_atmos.empty()) {
                corr.atmos_tokens = epoch.merged_atmos;
                corr.atmos_valid = true;
                auto nit = corr.atmos_tokens.find("atmos_network_id");
                corr.atmos_network_id = (nit != corr.atmos_tokens.end()) ?
                    std::atoi(nit->second.c_str()) : 0;
            }
            products.addCorrection(corr);

            // Emit an extra atmos-only correction for the preferred network.
            // Keep the complete network atmosphere bank here: ST8 polynomial
            // terms plus the latest ST9 trop/STEC residual grid.  CLASLIB's
            // corrmea gate rejects satellites when the selected gridded STEC
            // residuals are unavailable; dropping residuals here turns those
            // rows into polynomial-only corrections and admits extra sats.
            if (preferred_network_id > 0 &&
                preferred_network_id != corr.atmos_network_id &&
                epoch.atmos_by_network.count(preferred_network_id)) {
                libgnss::SSROrbitClockCorrection pref_corr;
                pref_corr.satellite = product_sat_id;
                pref_corr.time = time;
                pref_corr.atmos_network_id = preferred_network_id;
                const auto& pt = epoch.atmos_by_network.at(preferred_network_id);
                pref_corr.atmos_tokens = pt;
                pref_corr.atmos_tokens["atmos_network_id"] =
                    std::to_string(preferred_network_id);
                pref_corr.atmos_valid = true;
                products.addCorrection(pref_corr);
            }

            // Add ST6 network-specific phase bias corrections.
            // Each network's biases go in a separate correction so that
            // interpolateCorrection can find the right bias_network_id.
            for (const auto& [net_id, sat_biases] : epoch.network_phase_biases) {
                auto bias_it = sat_biases.find(sat_id);
                if (bias_it == sat_biases.end()) continue;
                libgnss::SSROrbitClockCorrection bias_corr;
                bias_corr.satellite = product_sat_id;
                bias_corr.time = time;
                bias_corr.bias_network_id = net_id;
                const int gnss_id = gnssSystemToCssrId(sat_id.system);
                const std::map<int, uint8_t>* sat_pdi = nullptr;
                const auto net_pdi_it =
                    epoch.network_phase_discontinuity_indicators.find(net_id);
                if (net_pdi_it != epoch.network_phase_discontinuity_indicators.end()) {
                    const auto sat_pdi_it = net_pdi_it->second.find(sat_id);
                    if (sat_pdi_it != net_pdi_it->second.end()) {
                        sat_pdi = &sat_pdi_it->second;
                    }
                }
                for (const auto& [slot, bias_m] : bias_it->second) {
                    const uint8_t rtcm_id = cssrSignalSlotToRtcmId(gnss_id, slot);
                    if (rtcm_id > 0) {
                        bias_corr.phase_bias_m[rtcm_id] = -bias_m;
                        if (sat_pdi != nullptr) {
                            const auto pdi_it = sat_pdi->find(slot);
                            if (pdi_it != sat_pdi->end()) {
                                bias_corr.phase_discontinuity_indicators[rtcm_id] =
                                    pdi_it->second;
                            }
                        }
                    }
                }
                bias_corr.phase_bias_valid = !bias_corr.phase_bias_m.empty();
                if (bias_corr.phase_bias_valid)
                    products.addCorrection(bias_corr);
            }
        }
    }
}

}  // namespace libgnss::qzss_l6
