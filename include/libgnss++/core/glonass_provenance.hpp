#pragma once

/**
 * @file glonass_provenance.hpp
 * @brief Source-preserving GLONASS navigation/header provenance primitives.
 *
 * These small value-only types live below the solver layer so the RINEX
 * reader and the opt-in Phase128 admission adapter share exactly the same
 * field-position and FCN rules.  They do not read files or alter factors.
 */

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace libgnss {

/** State of the fixed-label GLONASS FCN header ledger. */
enum class GlonassFrequencyChannelHeaderStatus {
    Absent,
    ValidEmpty,
    Entries,
    Malformed,
};

const char* glonassFrequencyChannelHeaderStatusName(
    GlonassFrequencyChannelHeaderStatus status) noexcept;

/** Per-record canonical geph-data rejection reason. */
enum class GlonassCanonicalRecordRejectReason {
    None,
    FieldCount,
    NonFiniteField,
    FcnNonFinite,
    FcnNonIntegral,
    FcnOutOfRange,
};

const char* glonassCanonicalRecordRejectReasonName(
    GlonassCanonicalRecordRejectReason reason) noexcept;

/** Result of validating one RTKLIB/RINEX GLONASS data[0..14] record. */
struct GlonassCanonicalRecordResult {
    bool accepted = false;
    int frequency_channel = 0;
    GlonassCanonicalRecordRejectReason reject_reason =
        GlonassCanonicalRecordRejectReason::FieldCount;
    std::string failure;
};

/**
 * Validate the canonical fifteen values and normalize data[10] exactly as
 * the native RTKLIB decoder does: an encoded unsigned byte greater than 128
 * is represented by subtracting 256, followed by the signed [-7,6] gate.
 * Field positions are never compacted, so a malformed value cannot move the
 * FCN from data[10] to another field.
 */
GlonassCanonicalRecordResult decodeCanonicalGlonassGeph(
    const std::array<double, 15>& data);

/** Vector convenience overload used by text decoders and focused tests. */
GlonassCanonicalRecordResult decodeCanonicalGlonassGeph(
    const std::vector<double>& data);

}  // namespace libgnss
