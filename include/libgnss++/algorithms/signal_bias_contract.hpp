#pragma once

// Truth-free receiver signal-bias contract for undifferenced code factors.
// A secondary Android code observable can carry a receiver/inter-frequency
// delay relative to the primary code.  This header only defines the stable
// eligibility and key identity; it does not estimate a bias or inspect a
// trajectory.

#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/types.hpp>

#include <cstdint>

namespace libgnss::signal_bias {

/// Secondary signal families that may receive a static receiver IFB state.
/// Primary signals intentionally map to zero (the epoch clock is their
/// reference), so the state has no position/clock gauge ambiguity.
inline bool isEligible(GNSSSystem system, SignalType signal) {
    return signal_policy::isSecondarySignal(system, signal);
}

/// Deterministic key ordinal for a (system, signal) state.  The ordinal is
/// deliberately sparse and independent of input order so GTSAM key identity
/// is stable across runs and backends.  A non-eligible signal returns -1.
inline int ordinal(GNSSSystem system, SignalType signal) {
    if (!isEligible(system, signal)) return -1;
    return 1 + (static_cast<int>(system) << 8) +
           static_cast<int>(static_cast<std::uint8_t>(signal));
}

}  // namespace libgnss::signal_bias
