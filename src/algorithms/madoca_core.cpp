#include <libgnss++/algorithms/madoca_core.hpp>

#include <libgnss++/algorithms/madoca_parity.hpp>

#include <array>
#include <cmath>

namespace libgnss::algorithms::madoca_core {
namespace {

using SignalTable = std::array<int, 16>;

constexpr SignalTable kCssrSigGps = {
    1, 2, 3, 7, 8, 12, 16, 17, 18, 19, 20, 24, 25, 26, 0, 0,
};
constexpr SignalTable kCssrSigGlo = {
    1, 2, 14, 19, 66, 67, 68, 30, 31, 33, 44, 45, 46, 0, 0, 0,
};
constexpr SignalTable kCssrSigGal = {
    11, 1, 12, 24, 25, 26, 27, 28, 29, 37, 38, 39, 31, 32, 0, 0,
};
constexpr SignalTable kCssrSigBd2 = {
    40, 41, 18, 42, 43, 33, 27, 28, 29, 0, 0, 0, 0, 0, 0, 0,
};
constexpr SignalTable kCssrSigBd3 = {
    40, 41, 0, 42, 43, 0, 61, 62, 63, 56, 2, 12, 57, 58, 26, 0,
};
constexpr SignalTable kCssrSigQzs = {
    1, 7, 8, 12, 16, 17, 18, 24, 25, 26, 35, 36, 60, 9, 0, 0,
};
constexpr SignalTable kCssrSigNone = {};

constexpr std::array<double, 16> kSsrUpdateIntervals = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800,
};

const SignalTable& signalTableFor(int compact_ssr_system_id) {
    switch (compact_ssr_system_id) {
        case 0: return kCssrSigGps;
        case 1: return kCssrSigGlo;
        case 2: return kCssrSigGal;
        case 3: return kCssrSigBd2;
        case 4: return kCssrSigQzs;
        case 7: return kCssrSigBd3;
        default: return kCssrSigNone;
    }
}

}  // namespace

std::optional<CompactSsrSystemId> compactSsrSystemId(int id) {
    switch (id) {
        case 0: return CompactSsrSystemId::GPS;
        case 1: return CompactSsrSystemId::GLONASS;
        case 2: return CompactSsrSystemId::Galileo;
        case 3: return CompactSsrSystemId::BeiDou;
        case 4: return CompactSsrSystemId::QZSS;
        case 7: return CompactSsrSystemId::BeiDou3;
        default: return std::nullopt;
    }
}

GNSSSystem gnssSystemForCompactSsrId(int id) {
    switch (id) {
        case 0: return GNSSSystem::GPS;
        case 1: return GNSSSystem::GLONASS;
        case 2: return GNSSSystem::Galileo;
        case 3:
        case 7: return GNSSSystem::BeiDou;
        case 4: return GNSSSystem::QZSS;
        default: return GNSSSystem::UNKNOWN;
    }
}

std::optional<int> compactSsrIdForGnssSystem(GNSSSystem system, bool beidou3) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::BeiDou: return beidou3 ? 7 : 3;
        case GNSSSystem::QZSS: return 4;
        default: return std::nullopt;
    }
}

std::string_view compactSsrSystemName(int id) {
    switch (id) {
        case 0: return "gps";
        case 1: return "glonass";
        case 2: return "galileo";
        case 3: return "beidou-2";
        case 4: return "qzss";
        case 7: return "beidou-3";
        default: return "unknown";
    }
}

std::optional<CompactSsrSubtype> compactSsrSubtype(int subtype) {
    switch (subtype) {
        case 1: return CompactSsrSubtype::Mask;
        case 2: return CompactSsrSubtype::Orbit;
        case 3: return CompactSsrSubtype::Clock;
        case 4: return CompactSsrSubtype::CodeBias;
        case 5: return CompactSsrSubtype::PhaseBias;
        case 7: return CompactSsrSubtype::Ura;
        default: return std::nullopt;
    }
}

std::string_view compactSsrSubtypeName(int subtype) {
    switch (subtype) {
        case 1: return "mask";
        case 2: return "orbit";
        case 3: return "clock";
        case 4: return "code-bias";
        case 5: return "phase-bias";
        case 7: return "ura";
        default: return "unknown";
    }
}

std::optional<double> ssrUpdateIntervalSeconds(int index) {
    if (index < 0 || static_cast<int>(kSsrUpdateIntervals.size()) <= index) {
        return std::nullopt;
    }
    return kSsrUpdateIntervals[static_cast<std::size_t>(index)];
}

int compactSsrSignalCodeAt(int compact_ssr_system_id, int mask_slot) {
    if (mask_slot < 0 || 16 <= mask_slot) {
        return 0;
    }
    return signalTableFor(compact_ssr_system_id)[static_cast<std::size_t>(mask_slot)];
}

std::vector<int> compactSsrSignalCodes(int compact_ssr_system_id, std::uint16_t mask) {
    const auto& table = signalTableFor(compact_ssr_system_id);
    std::vector<int> codes;
    std::uint16_t bit = static_cast<std::uint16_t>(1u << 15);
    for (int slot = 0; slot < 16; ++slot) {
        if ((mask & bit) != 0) {
            codes.push_back(table[static_cast<std::size_t>(slot)]);
        }
        bit = static_cast<std::uint16_t>(bit >> 1);
    }
    return codes;
}

double ssrUraIndicatorToSigmaMeters(int ura_index) {
    if (ura_index == 0) {
        return 0.15;
    }
    if (ura_index < 0 || ura_index >= 63) {
        return 5.4665;
    }
    return (std::pow(3.0, static_cast<double>((ura_index >> 3) & 0x07)) *
                (1.0 + static_cast<double>(ura_index & 0x07) / 4.0) -
            1.0) *
           1e-3;
}

int selectBiasCode(int rtklib_system, int rtklib_code) {
    return madoca_parity::mcssrSelBiascode(rtklib_system, rtklib_code);
}

}  // namespace libgnss::algorithms::madoca_core
