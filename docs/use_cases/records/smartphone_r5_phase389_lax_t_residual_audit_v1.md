# Phase389 — convergence and GNSS residual aggregates

Inspected existing Phase382 LAX-T / Phase234 H summary aggregates and native
residual/serialization code. No truth or candidate coordinate reads; no new
inference, score, or parameter tuning.

Both stages in both runs terminate by `outer_convergence_tolerance`, not the
1000-iteration budget. Main iterations are LAX-T 95 and H 41; GNSS-first 84
and 57. All report finite decreasing costs and no fallback. This establishes
reported numerical termination, not accuracy or a global optimum. Do not
increase the iteration limit based on the 3.1 m development score.

| Main postfit aggregate | LAX-T | H |
| --- | ---: | ---: |
| P residual RMS, m | 5.501474581475073 | 4.893186209869611 |
| P normalized RMS | 2.834277043062923 | 2.5862446573738933 |
| TDCP residual RMS, m | 0.021492656902611634 | 0.01163556640252874 |
| TDCP normalized RMS | 6.491013574376732 | 4.729542594541887 |
| TDCP max absolute residual, m | 2.2056019896530756 | 0.45897223935116926 |
| TDCP median arc length, epochs | 4 | 5 |
| Upstream stop epochs | 69 | 988 |

Normalized RMS is pre-robust residual/sigma, not reduced chi-square, a
robust-weight summary, or ground-truth error. Raw measurement populations and
route duration differ; total costs cannot isolate the added IMU contribution.
The legacy generic-Doppler count being zero does not mean Phase213 main
Doppler factors are absent. Consult stage-specific counters.

Found a serialization inconsistency: `upstream_observable_quality` advertised
fixed 0.03 m TDCP sigma despite `tdcp_contract` correctly reporting enabled
source metre sigma. Changed only that summary's label and fixed-sigma field:
source metre mode is identified explicitly, and nonfixed modes emit null.
No factor construction, weighting, optimization, or existing artifact edits.

Next useful diagnostic is same-run factor-family robust cost/downweighting
support, particularly the LAX-T TDCP tail. This evidence does not authorize
threshold sweeps or establish TDCP as the positioning error's cause.
