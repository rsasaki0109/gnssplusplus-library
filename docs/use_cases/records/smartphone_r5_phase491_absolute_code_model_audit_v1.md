# Phase491 — absolute code model candidate triage

Inspected current fgo_problems.cpp, FGOConfig defaults and
observable_upstream_preprocessing.hpp. Code correction is P + satellite clock
- ionosphere - troposphere - group delay. Broadcast Klobuchar is conditional
on valid navigation ionosphere metadata and scaled by (GPS L1 / row frequency)^2;
Saastamoinen is enabled by the default model configuration. This is code-path
evidence, not proof that every route has valid ionosphere metadata.

Current upstream P quality uses band-global 85th-percentile C/N0, scale
10^(-(C/N0 - percentile)/20), with signal multipliers .8 for GPS L1/Galileo E1,
1.5 for GLONASS L1 and .5 for L5/E5a. In this branch the generic elevation
sigma power does not affect the P sigma. Do not change that generic knob and
expect a different upstream-quality likelihood.

Existing opt-in android_sv_time_uncertainty.hpp supplies a no-fitted-scale
floor max(existing sigma, c * ReceivedSvTimeUncertaintyNanos * 1e-9). Invalid
or absent uncertainty preserves existing sigma. Current H summary does not
expose the matching uncertainty-availability/floor fields, so the number of
affected retained factors is not yet established.

Historical Phase52 integrity-recovered results: uncertainty-floor candidate
macro 3.09369434071945 versus control 3.536446745840132 m, but failed frozen
promotion gates and kept experimental. That old recipe is not the current
Phase171/C7 baseline; do not quote its difference as a current improvement.
Phase12 residual ionosphere results concern a different Pixel7Pro route and
also do not establish current H benefit. These are not new scoring runs.

Next bounded action: aggregate raw timing-uncertainty availability and compare
with the actual retained-factor SNR sigma before selecting a frozen current
floor experiment. Avoid re-running old parameter ideas without confirming
their reach into this graph. No defaults changed, no truth read or submission.
