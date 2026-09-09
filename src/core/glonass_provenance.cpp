#include <libgnss++/core/glonass_provenance.hpp>

#include <cmath>
#include <limits>

namespace libgnss {
namespace {

GlonassCanonicalRecordResult reject(
    GlonassCanonicalRecordRejectReason reason,
    const char* message) {
    GlonassCanonicalRecordResult result;
    result.reject_reason = reason;
    result.failure = message;
    return result;
}

}  // namespace

const char* glonassFrequencyChannelHeaderStatusName(
    GlonassFrequencyChannelHeaderStatus status) noexcept {
    switch (status) {
        case GlonassFrequencyChannelHeaderStatus::Absent:
            return "absent";
        case GlonassFrequencyChannelHeaderStatus::ValidEmpty:
            return "valid-empty";
        case GlonassFrequencyChannelHeaderStatus::Entries:
            return "entries";
        case GlonassFrequencyChannelHeaderStatus::Malformed:
            return "malformed";
        default:
            return "unknown";
    }
}

const char* glonassCanonicalRecordRejectReasonName(
    GlonassCanonicalRecordRejectReason reason) noexcept {
    switch (reason) {
        case GlonassCanonicalRecordRejectReason::None:
            return "none";
        case GlonassCanonicalRecordRejectReason::FieldCount:
            return "field-count";
        case GlonassCanonicalRecordRejectReason::NonFiniteField:
            return "nonfinite-field";
        case GlonassCanonicalRecordRejectReason::FcnNonFinite:
            return "fcn-nonfinite";
        case GlonassCanonicalRecordRejectReason::FcnNonIntegral:
            return "fcn-nonintegral";
        case GlonassCanonicalRecordRejectReason::FcnOutOfRange:
            return "fcn-out-of-range";
        default:
            return "unknown";
    }
}

GlonassCanonicalRecordResult decodeCanonicalGlonassGeph(
    const std::array<double, 15>& data) {
    if (!std::isfinite(data[10])) {
        return reject(GlonassCanonicalRecordRejectReason::FcnNonFinite,
                      "GLONASS geph data[10] FCN is non-finite");
    }
    for (const double value : data) {
        if (!std::isfinite(value)) {
            return reject(GlonassCanonicalRecordRejectReason::NonFiniteField,
                          "GLONASS geph data[0..14] contains a non-finite field");
        }
    }

    const double raw_channel = data[10];
    const double normalized_channel =
        raw_channel > 128.0 ? raw_channel - 256.0 : raw_channel;
    if (std::floor(normalized_channel) != normalized_channel) {
        return reject(GlonassCanonicalRecordRejectReason::FcnNonIntegral,
                      "GLONASS geph data[10] FCN is not integral");
    }
    if (normalized_channel < -7.0 || normalized_channel > 6.0) {
        return reject(GlonassCanonicalRecordRejectReason::FcnOutOfRange,
                      "GLONASS geph data[10] FCN is outside [-7,6]");
    }

    GlonassCanonicalRecordResult result;
    result.accepted = true;
    result.frequency_channel = static_cast<int>(normalized_channel);
    result.reject_reason = GlonassCanonicalRecordRejectReason::None;
    result.failure.clear();
    return result;
}

GlonassCanonicalRecordResult decodeCanonicalGlonassGeph(
    const std::vector<double>& data) {
    if (data.size() != 15U) {
        return reject(GlonassCanonicalRecordRejectReason::FieldCount,
                      "GLONASS geph record must contain exactly 15 fields");
    }
    std::array<double, 15> canonical{};
    for (std::size_t i = 0; i < canonical.size(); ++i) {
        canonical[i] = data[i];
    }
    return decodeCanonicalGlonassGeph(canonical);
}

}  // namespace libgnss
