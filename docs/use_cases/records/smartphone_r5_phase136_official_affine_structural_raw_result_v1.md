# Phase136/135 official-affine structural raw result

This is the sealed structural-only result for the independent Phase136 wrapper
boundary authorization.  The authorized recipe is Phase135 official affine
measurement factors, Phase118 TDCP Huber-k selection, and Phase107 native raw
base compensation.  Phase117, Phase120, Phase126--134 compound selectors,
and additional-frequency-band processing were off.  No truth, accuracy,
MAT, PDC input, precomputed coordinate, or Kaggle operation was authorized.

The run order was exactly one native invocation for MTV-A followed by exactly
one for LAX-T.  Both invocations returned zero.  The output solution files
are sealed only by opaque byte/newline/SHA-256 metadata; their rows were not
opened or interpreted.  The native summary was likewise consumed only as
structural metadata.  Its wrapper-normalized top-level `main` field was null;
main telemetry below is explicitly recovered from the native `graph` and
`native_source_clock_c0d_factor` objects, without interpreting coordinate
fields.

| route | epochs | affine P / D / TDCP | Pose3-X bridge | GNSS-first cost (initial → final) | main cost (initial → final) | main accepted | C/D finite handoff | raw-base | output epochs | disposition |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |
| MTV-A | 2159 | 43259 / 20748 / 31269 | 2159 | 165214679.50699747 → 136827359.04150486 | 1765606150453.9958 → 214191269.26530388 | 12 | 2159 / 2159, exact | once (43259; 14022 interpolation misses) | 2159 | structural GO |
| LAX-T | 1466 | 30664 / 8942 / 14012 | 1466 | 66070529.934910096 → 46937212.955303565 | 1660928418678.5596 → 88416528.19976115 | 12 | 1466 / 1466, exact | once (30664; 134 interpolation misses) | 1466 | structural GO |

Both routes had 536/581 accepted GNSS-first outer iterations and 12 accepted
main outer iterations, with finite strict cost decrease in both stages.  The
main solver branch was `MULTIFRONTAL_QR`/`EliminateQR`.  C was seven-dimensional
meter state and D was meter-per-second clock drift; every retained epoch had
finite C and D values and exact same-run epoch-key handoff.  The CCDD sigma
was 0.1 m.  The affine-family counts were all positive and equal to the
native inserted/admitted family counters: no legacy DD carrier/pseudorange,
receiver signal-bias, standalone ambiguity, base-DD TDCP, or PDC state bridge
families were present.  The Pose3-X bridge count and key order matched the
retained epoch count.

The ordinary TDCP family had fixed sigma 0.03 m, official Highway Huber
`k=0.5`, finite residuals for every inserted factor, and no DD/base or
ambiguity state.  Raw-base compensation was built and applied exactly once;
all adopted rows were finite and in-domain, with no extrapolation or endpoint
hold.  Phase135 reported one Sagnac representation and finite source
geometry/Jacobians.  Pixel5 final-output offset application was exactly once,
and expected output coverage was finite and complete.

The native C0D telemetry recorded 8 indeterminate linear-solve attempts for
MTV-A and 6 for LAX-T (20 and 23 total inner lambda attempts respectively).
These are retained as a structural risk signal; no retry, tuning, fallback,
or rerun was performed.  They do not alter the recorded accepted-iteration
and strict-cost gates.

## Provenance and accounting

The independent authorization is commit
`445f53b44b186c111a926f4e9b5daf88ef2ffd07`, authorization SHA-256
`22f4ed4d7c18222bf78f56d7fb06958d36a9a7fe77a14f30eb4ce107ed437586`.
The Phase136 boundary pins are audit `5ef049c927dab2a67ba66a97a2a4e44ce98de4f8`,
freeze `a24f7d1ff4fc53b8e270767c0b32f25874e2fc94`, implementation/final wrapper
fix `3aa8bc08e7b7747fad8e127691160314eed592b2`, qualification manifest
`d592e43b4796f851a87ed200c4724e087f3d16d9`, and pre-raw repin
`efba5e896b841d1b7e26761baef2168f1bd6498f`.  The authorized wrapper SHA is
`5ff1c932bb685ca9a052b0d50c2df84fa93e8132a79dff8b06a469e42da10a21`.

The wrapper sealed the native structural result at SHA-256
`6975dc338ef278ed95abb1e5ef2313c345fa06811629e2ce6f9ab3e22ecc092c`
(52185 bytes).  Raw input read counts were GNSS 2, IMU 2, broadcast navigation
2, and base RINEX 2 (one of each per route).  Native solver invocations were
2.  Truth reads, solution-coordinate reads, accuracy calculations,
MAT/PDC/precomputed-coordinate reads, Kaggle/token access, and reruns or
fallbacks were all zero.  The per-route raw input hashes and opaque solution
seals are recorded in the JSON artifact next to this note.

This is a structural GO only.  No truth-only authorization exists in this
artifact, so no accuracy evaluation or Kaggle submission is permitted.
