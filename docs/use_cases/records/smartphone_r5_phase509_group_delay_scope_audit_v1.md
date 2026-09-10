# Phase509 — group-delay application is a distinct source-parity hypothesis

Current native fgo_internal.hpp groupDelayCorrectionMeters applies primary
TGD to GPS/QZSS regardless of signal, primary/secondary BeiDou by band,
and the Galileo helper (default primary; optional source-specific E1).
fgo_problems.cpp subtracts this from corrected P. SPP has a matching separate
implementation. No model changes made in this audit.

Cached .m inspection gives a distinct boundary to check: MatRTKLIB Gobs.m
residuals() computes resPc = P - (range - satellite_clock + ionosphere +
troposphere), without an explicit group-delay term. Gsat.m assigns satellite
clock output from its satellite-position call. Local +rtklib/satposs.m
documentation says satellite clock excludes TGD/BGD. gsdc2023 fgo_gnss_imu.m
uses resPc (and may first subtract base corrections). Gnav.getTGD exists and
selects the first satellite record, but no calls to getTGD were found inside
the inspected +gt .m class directory. Do not mistake an unused helper for
proof that the positioning path applies its correction.

This establishes a code-level explicit-term difference, NOT full runtime
source equivalence: compiled satellite wrapper implementation and preprocessing
must be traced, and base correction can change the comparison. Native raw
baseline is no-base. Do not simply disable TGD globally or call it a bug.
Next quantify actual selected-message TGD scale and confirm complete source
correction path before freezing a stage-isolated parity experiment.

Historical Phase22 source-specific Galileo E1 correction failed the primary
Pixel7 gate (~.076 m worse), despite mi8 improving ~.043 m; not a current H
result. Phase344 propagation oracle agreed at tiny numerical scale but did
not test code-bias application. No additional truth scoring, coordinates,
MAT or inference run used here. Overall objective remains unmet.
