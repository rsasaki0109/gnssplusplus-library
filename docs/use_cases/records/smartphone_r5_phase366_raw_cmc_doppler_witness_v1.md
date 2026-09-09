# Phase366 — raw code-minus-carrier / Doppler witness

Primary-agent raw-only extension of native_carrier_jump_audit.cpp. Built
successfully against current native libraries; executed once per H/U raw
GNSS input after this extension. No truth, MAT, saved positioning, native
solver run or new accuracy evaluation.

Among finite parser-admitted adjacent P/L pairs with 0 < dt <= 1.5 s, count
abs(delta P - lambda * delta L) > 10 m. For each, independently compute the
source trapezoidal Doppler-versus-carrier expression from raw observables.
The fixed 1.5 m residual threshold is the existing source L-D threshold,
not a threshold chosen by scoring these routes.

| Raw aggregate | H | U |
| --- | ---: | ---: |
| Raw CMC difference > 10 m | 2254 | 617 |
| Finite L-D witnesses | 2254 | 617 |
| Absolute L-D difference <= 1.5 m | 2254 | 617 |
| Maximum absolute L-D difference (m, rounded) | 0.569602 | 0.421929 |

Every raw CMC-triggering pair has carrier/Doppler agreement within the
existing source tolerance. This supports testing whether noisy code removes
useful phase constraints. It does not prove absence of cycle slips or
statistical independence: ADR and Doppler come from the same device.

These are **not exact final FGO rejection counts**. The final builder uses
corrected P/L and additional model/quality gates: H's 2254 raw pairs differ
from its 2229 recorded final code-phase rejections. U's equal count is not
proof of identity. No inference input was made from previous trajectories.

Next add a default-off admission experiment that retains every existing
gate except the code-phase-jump rejection, while retaining the existing
upstream L-D mask and robust TDCP loss. Synthetic tests must separate noisy
code / smooth carrier from a true carrier jump, and demonstrate unchanged
default behavior. Before scoring, verify added factor support in same-run
builder telemetry; do not silently accept nonfinite, slip-marked, clock-jump
or gap pairs. Freeze the same rule for both H and U development comparisons,
without threshold sweeps or claims of fresh validation.
