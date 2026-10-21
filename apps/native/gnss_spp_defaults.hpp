#pragma once

#include <libgnss++/algorithms/spp.hpp>

namespace libgnss_apps {

// `gnss spp` weights measurements more steeply than the library defaults:
// a 10 dB SNR slope capped at 32x, GLONASS code variance x4 (its broadcast
// satellite-specific biases are several metres), and a 1 m temporal prior on
// the inter-system biases. The library defaults stay at the historical values
// so RTK, FGO, PPP and the bindings that share SPPProcessor are unchanged.
inline libgnss::SPPProcessor::SPPConfig gnssSppDefaultConfig() {
    libgnss::SPPProcessor::SPPConfig config;
    config.snr_weight_slope_db = 10.0;
    config.snr_weight_max_factor = 32.0;
    config.glonass_variance_factor = 4.0;
    config.isb_prior_sigma_m = 1.0;
    return config;
}

}  // namespace libgnss_apps
