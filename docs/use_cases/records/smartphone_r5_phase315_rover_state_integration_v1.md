# Phase315: native rover epoch-state integration

`FGOConfig::use_source_rover_epoch_states` (default false) now connects the
source transmission-clock and broadcast-selection helpers to the production
pseudorange problem builder. One state is constructed per satellite from the
parser-masked first available frequency slot, before additional jump/residual
masks. Its position, forward-difference velocity, clock and drift are shared
by that satellite's eligible factor rows. Ephemeris metadata uses the same
receive-time-selected record. The historical two-pass path remains the default.

Missing navigation records or no admissible broadcast message drop only that
satellite's rows, with a satellite-epoch diagnostic counter. Malformed code,
ambiguous slots, and invalid state propagation still throw; there is no native
fallback when the option is on. The existing per-row health gate still applies.
State counters cover seeded input epochs, before final graph admission.

The input contract requires already tracking-selected, raw-quality-masked
observations. Native orbit equations, rotation, group-delay handling and
downstream GLONASS annotation are unchanged; this is not a claim of full
MALIB orbit or paired base/rover equation parity. No CLI recipe is switched on.

New builder test covers default-off configuration, 8 shared satellite states
across two epochs, equal L1/L5 satellite positions despite different code
ranges, a local missing-navigation event, and malformed provenance rejection.
Build of `gnss_run_tests` succeeded. The focused filter
`FGOSourceRoverStateTest.*:FGORemaskingPoolTest.*:AndroidRawGnssTest.*:BasePseudorangeCompensationTest.*`
passed all 55 tests. This is not full CTest or a real-data byte-identity check.

No raw route inference, ground-truth access, MAT input/intermediate,
saved-positioning input, submission, or accuracy evaluation is performed here.
The next integration step is a raw-only phone state/admission diagnostic and
an explicit paired base/rover CLI recipe before a frozen accuracy evaluation.
The operational H score and unachieved 0.782/LB objective are unchanged.
