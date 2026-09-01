# Moving CLAS held-DD row continuation experiment

Status: **promoted after six-run hard sign-off**. This document records the
root cause, bounded-window evidence, rejected alternatives, and final result.

## Problem

The fixed baseline for PPC Tokyo run2 produces 1,955 FIX epochs after the
60-epoch article warm-up, or 21.507151%. MRTKLIB v0.4.2 produces 21.7%, so
native needs at least 18 net FIX epochs while retaining:

- FIX RMS2D at or below 0.514 m;
- zero FIX epochs above 3 m horizontal error;
- full interval and epoch coverage.

## Rejected hypothesis: position-variance gate

Two complete Tokyo run2 candidates changed the direct state-DD position
variance gate from the MRTKLIB-equivalent 0.9999 m² threshold to 1.10 and
1.25 m². Both outputs were byte-identical to the baseline:

- SHA-256: `53E0930DC19E5ABE0F4C73850294FB1B4A2157936F7C23EB612DDE1248BD0762`
- fixed epochs: 1,955
- FIX rate: 21.507151%
- FIX RMS2D: 0.322380 m
- gained/lost FIX epochs: 0/0

The position-variance gate is therefore not the Tokyo run2 FIX deficit.

## Root cause

The baseline contains accurate, bounded FLOAT gaps between FIX epochs. At
GPS TOW 177048.4, the direct state-DD system has only four accepted DD rows
and exits before LAMBDA because the MRTKLIB `minamb=6` floor is not met. The
surrounding evidence is:

| TOW | Baseline status | Accepted DD rows / `nb` | Ratio | Horizontal error |
|---:|---|---:|---:|---:|
| 177048.2 | FIX | 6 | 3072.130391 | — |
| 177048.4 | FLOAT | 4 rows before the floor | 0 | 0.204342 m |
| 177048.6 | FIX | 6 | 3242.188666 | — |

The gap is caused by transient satellite/row loss, not by a divergent
position or an unsafe integer candidate.

## Candidate policy

The experiment keeps the ordinary direct state-DD floor at six rows. A
four-row attempt is permitted only when all of the following are true:

1. the moving CLAS MRTKLIB-parity direct state-DD path is active;
2. the previous published epoch was FIX;
3. an ambiguity hold created by an earlier low-chi-square fix is active and
   remains valid after current-epoch slip detection;
4. the previous FIX was either the full-row FIX that authorized the bridge or
   a publication-only reduced-row FIX in the same uninterrupted bridge;
5. the reduced-row publication streak has not reached its configured bound.

The evaluation candidate bounds the complete bridge at five reduced-row
publications, including the first continuation from a full-row FIX. A sixth
reduced-row publication is therefore impossible without a new full-row FIX.
The bound prevents an unlimited reduced-geometry recovery chain while covering
the short five-epoch geometry dropouts observed on Tokyo run2. During
evaluation the production held floor remained six; after the completed
six-run sign-off it was promoted to four while the ordinary floor remained six.

The implementation records whether the preceding published solution was a full
(at least six-row) CLAS direct state-DD FIX or a reduced-row publication, plus
the consecutive reduced-publication count. Standard-PPP fallback and every
CLAS early return that publishes FLOAT, SINGLE, continuity output, or an
invalid solution explicitly clear that eligibility and the streak. A
historical ambiguity hold therefore cannot authorize a reduced-row attempt
after an intervening non-FIX publication.

A validated reduced-row FIX is publication-only. Internally it follows the
same lifecycle as the FLOAT produced by the six-row baseline: `nfix` is reset
on every reduced publication,
the existing full-row hold constraints are neither replaced nor applied to the
float filter, rejected speculative publications do not tear the historical
hold down or seed post-reset quarantine, accepted-participant ambiguity flags
are restored to their pre-AR values, and the 15-epoch FLOAT counter advances.
This separation is required to preserve later full-row FIX opportunities.

The evaluated policy is:

```text
GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS=4
GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK=5
```

Unset, malformed, or out-of-range environment values retain the validated
defaults of four rows and five publications. Explicit `6` and `1` recover the
previous behavior. Direct typed configuration outside the supported streak
range still fails closed to the one-epoch bridge.

## Bounded-window A/B

The evaluation window contains the first 320 Tokyo run2 observation epochs,
GPS TOW 177000.0 through 177063.8. The first 60 epochs are excluded by the
article scoring convention, leaving 260 scored epochs. The default-six
window is identical to the corresponding full baseline at all 320 epochs.

| Candidate | FIX epochs | FIX rate | FIX RMS2D | p68 | Max FIX error | >1 m | >3 m | Gained / lost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Baseline / default 6 | 33 | 12.692308% | 0.248057 m | 0.243643 m | 0.618552 m | 0 | 0 | — |
| Held 4, chaining allowed (rejected) | 40 | 15.384615% | 0.503116 m | 0.250133 m | 1.369049 m | 4 | 0 | +7 / 0 |
| Held 4, no reduced-row chain | 35 | 13.461538% | 0.280111 m | 0.244454 m | 0.820890 m | 0 | 0 | +2 / 0 |
| Held 4, publication-only lifecycle | 35 | 13.461538% | 0.280110 m | 0.244449 m | 0.820891 m | 0 | 0 | +2 / 0 |
| Held 4, publication-only, max streak 5 | 39 | 15.000000% | 0.459958 m | 0.246257 m | 1.333418 m | 3 | 0 | +6 / 0 |

The one-epoch bridge gains TOW 177048.4 and 177052.8. The bounded streak-5
candidate retains those gains and adds 177053.0 through 177053.6. It loses no
baseline FIX epochs; its maximum gained error is 1.333418 m and it introduces
no FIX error above 3 m.

After hardening all early-return paths, the same 320-epoch run is byte-identical
to the selected no-chain output (SHA-256
`1314AA9A8F7B99EF71161E9B82C03759E735A9CB561D0AE2A5456569420D8AF7`).
It retains the same two gained FIX epochs and zero lost FIX epochs, proving
that the safety hardening does not perturb the intended continuation case.

The final publication-only implementation retains those same two gains and
zero losses. More importantly, all other 318 output epochs are byte-identical
to the default-six baseline after matching by TOW. Its all-solution RMS2D is
1.001784 m versus 1.001793 m for the baseline; the earlier internally mutating
implementation produced 1.226140 m. This verifies on the bounded window that
the reduced publication itself is isolated from later float/filter output.

The streak-1 replay produced by the streak-aware binary is byte-identical to
the earlier publication-only output (SHA-256
`77A324A6DD148A8CEAAE001F3FE8536556ABC06A6FF89A0B75A34F17FF9DB0F7`).
This proves that the streak implementation is inactive when its bound remains
at one.

## State-continuous 2820-epoch A/B

The second probe replays Tokyo run2 from its first observation through TOW
177563.8, preserving 2,820 epochs of filter history and 2,760 scored epochs.
It includes both the difficult 177053.x bridge and the 177541.x cluster that
supplies the remaining FIX-rate margin:

| Candidate | FIX epochs | FIX rate | FIX RMS2D | p68 | Max FIX error | >1 m | >3 m | Gained / lost vs baseline |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Baseline / default 6 | 317 | 11.485507% | 0.482474 m | 0.207193 m | 1.013394 m | 17 | 0 | — |
| Publication-only streak 1 | 322 | 11.666667% | 0.487193 m | 0.207668 m | 1.013394 m | 17 | 0 | +5 / 0 |
| Publication-only streak 5 | 331 | 11.992754% | 0.509025 m | 0.215656 m | 1.333418 m | 20 | 0 | +14 / 0 |

Streak 5 gains nine FIX epochs beyond streak 1 and loses none. The four newly
published epochs at TOW 177541.6 through 177542.2 have horizontal errors from
0.977910 m to 0.985465 m. Across the full probe, all 14 gains remain below
1.334 m and none exceeds 3 m. The candidate artifact SHA-256 is
`15E441AC72B442FED6A11A14ED5C0C6F8DA7A5A34C1266E3C2B93522573F6BB7`.

## Test evidence

- streak-5 evaluation binary SHA-256:
  `E6717E4FA7B25BCBC9EA93D806D6FF902B6DC8EA893DB231399F0D687D92F196`;
- targeted held-floor, direct state-DD, and environment tests: 3/3 passed;
- PPP AR and CLAS regression set: 45 passed, 1 dedicated-process skip;
- dedicated CLAS outage-reset kill-switch CTest: passed;
- complete native test executable: 871 tests, 818 passed, 53 existing
  fixture-dependent skips, 0 failures. The saved GoogleTest JSON SHA-256 is
  `5793D4F71B798EBE0D929A694758CDB9EED6A11F30FB9DB5691D15EEFFFC5066`.

Final promoted-build verification uses `gnss_ppp.exe` SHA-256
`E77B83ADC487DF90E0EF293EB7C2B57AFDA0C159F60EB134380173E408956F3F`.
The complete native suite again ran 871 tests: 818 passed, 53 existing
fixture-dependent skips, and 0 failed. Its final GoogleTest JSON SHA-256 is
`CFA23BF8080E8D53F6F676A6C2395C5A367C2E6905E6FFEF9FFDE3F525108B1F`;
the dedicated outage-reset legacy kill-switch CTest also passed. Python
scorecard and README regression coverage passed 34 tests and 26 subtests.

On the 320-epoch Tokyo run2 slice, the unset production defaults and explicit
row 4 / streak 5 outputs are byte-identical, including the earlier evaluation
artifact (SHA-256
`F8D2B855EC68A43BC62B5AF4340E9365B743B99493C51367C8059A7F9474B3E2`).
Explicit row 6 / streak 1 is byte-identical to the fixed baseline slice
(SHA-256
`FC3526078C06B876579AF47D6151D29DD3A0CD91E36536BC28BF9EABAEC4E5A8`).

## Completed full-run evidence

The hardened one-epoch-bridge Nagoya run3 evaluation is complete and passes
every hard gate:

| Metric | Hardened held 4 | MRTKLIB v0.4.2 | Result |
|---|---:|---:|---:|
| FIX rate | 8.912240% | 6.300% | PASS |
| FIX RMS2D | 0.304038 m | 0.318 m | PASS |
| FIX p68 | 0.295628 m | 0.339 m | PASS |
| FIX epochs above 3 m | 0 | 0 required | PASS |
| TTFF (30 consecutive FIX) | 27 s | 9 s | soft miss |

It gains seven FIX epochs and loses none versus the fixed baseline, retaining
100% interval coverage and 99.961546% epoch coverage. The hardened and earlier
no-chain exploratory full outputs are byte-identical (SHA-256
`84A670E701C64DB2FA9F4AB5E6815E170A58CD3418F9F5D260405836FA64BC2F`).

The selected maximum-streak-5 candidate has also completed Nagoya run3. It
uses binary SHA-256
`E6717E4FA7B25BCBC9EA93D806D6FF902B6DC8EA893DB231399F0D687D92F196`,
explicit held floor 4, and explicit maximum publication streak 5. Its output
SHA-256 is
`8EDC2840EF5DA6022B0B4290CC25AB0CFF9E18FC0123815E797EA47108DE7536`.

| Metric | Fixed baseline | Streak-5 candidate | MRTKLIB v0.4.2 | Result |
|---|---:|---:|---:|---:|
| FIX epochs | 451 | 463 | — | +12 / -0 |
| FIX rate | 8.776026% | 9.009535% | 6.300% | PASS |
| FIX RMS2D | 0.304093 m | 0.304348 m | 0.318 m | PASS |
| FIX p68 | 0.295563 m | 0.295944 m | 0.339 m | PASS |
| Maximum FIX error | 0.587467 m | 0.587467 m | — | unchanged |
| FIX epochs above 3 m | 0 | 0 | 0 required | PASS |
| TTFF (30 consecutive FIX) | 27 s | 27 s | 9 s | soft miss |

All twelve added FIX epochs are below 0.366 m horizontal error. The candidate
loses no baseline FIX epochs, retains 100% interval coverage and 99.961546%
epoch coverage, and therefore passes the first of six final-drive hard
sign-offs without relying on an aggregate average.

Nagoya run1 is the second completed maximum-streak-5 drive. Its output
SHA-256 is
`C1CC7E5CF40A344A3B323272B16B60291A503B9352D47084A5E506E3009D0F5D`.

| Metric | Fixed baseline | Streak-5 candidate | MRTKLIB v0.4.2 | Result |
|---|---:|---:|---:|---:|
| FIX epochs | 2,770 | 2,784 | — | +14 / -0 |
| FIX rate | 36.737401% | 36.923077% | 17.000% | PASS |
| FIX RMS2D | 0.449769 m | 0.451044 m | 1.105 m | PASS |
| FIX p68 | 0.433544 m | 0.433567 m | 0.402 m | soft miss |
| Maximum FIX error | 1.042970 m | 1.042970 m | — | unchanged |
| FIX epochs above 3 m | 0 | 0 | 0 required | PASS |
| TTFF (30 consecutive FIX) | 0 s | 0 s | 0 s | PASS |

The fourteen added FIX epochs have a maximum horizontal error of 1.025292 m;
none exceeds 3 m. The candidate retains 100% interval coverage and
99.973691% epoch coverage and therefore passes the second of six per-drive
hard sign-offs.

Nagoya run2 is the third completed drive. Its output SHA-256 is
`6390D199FCB0BAAA8D7BB239E93F73A62C1E46447F0A8F93E4F5A089BB7542C2`.

| Metric | Fixed baseline | Streak-5 candidate | MRTKLIB v0.4.2 | Result |
|---|---:|---:|---:|---:|
| FIX epochs | 2,230 | 2,250 | — | +20 / -0 |
| FIX rate | 23.756259% | 23.969319% | 23.400% | PASS |
| FIX RMS2D | 0.554480 m | 0.553906 m | 1.119 m | PASS |
| FIX p68 | 0.612004 m | 0.611945 m | 0.461 m | soft miss |
| Maximum FIX error | 0.767281 m | 0.767281 m | — | unchanged |
| FIX epochs above 3 m | 0 | 0 | 0 required | PASS |
| TTFF (30 consecutive FIX) | 0 s | 0 s | 0 s | PASS |

All twenty added FIX epochs are below 0.636 m. The candidate loses no baseline
FIX epochs and passes the third per-drive hard sign-off.

Tokyo run1 is the fourth completed drive. Its output SHA-256 is
`44868E6DC2470DD5985813F1B54B2AA08C21C42954F618B7721D2BD1E97BA908`.

| Metric | Fixed baseline | Streak-5 candidate | MRTKLIB v0.4.2 | Result |
|---|---:|---:|---:|---:|
| FIX epochs | 1,270 | 1,303 | — | +33 / -0 |
| FIX rate | 10.703751% | 10.981879% | 4.900% | PASS |
| FIX RMS2D | 0.351834 m | 0.350715 m | 0.747 m | PASS |
| FIX p68 | 0.126047 m | 0.126383 m | 0.244 m | PASS |
| Maximum FIX error | 1.960639 m | 1.960639 m | — | unchanged |
| FIX epochs above 3 m | 0 | 0 | 0 required | PASS |
| TTFF (30 consecutive FIX) | 0 s | 0 s | 0 s | PASS |

All 33 added FIX epochs are below 0.583 m. The candidate loses no baseline FIX
epochs and passes both the hard and soft Tokyo run1 sign-offs.

## Rejected full-run implementation

The first hardened six-run implementation correctly prevented reduced-row
chains, but treated an accepted reduced-row publication as an ordinary
internal FIX. It replaced the active hold with the four-row constraint set,
applied that hold to the float filter, incremented `nfix`, and reset the FLOAT
counter. Tokyo run2 exposed the resulting downstream regression:

| Metric | Baseline | First held-4 implementation | MRTKLIB v0.4.2 |
|---|---:|---:|---:|
| FIX epochs | 1,955 | 1,924 | — |
| FIX rate | 21.507151% | 21.166117% | 21.700% |
| FIX RMS2D | 0.322380 m | 0.424295 m | 0.514 m |
| FIX p68 | 0.204812 m | 0.198555 m | 0.120 m |
| Max FIX error | 1.013394 m | 2.612103 m | — |
| FIX epochs above 3 m | 0 | 0 | 0 required |
| Gained / lost FIX epochs | — | +22 / -53 | — |

The 53 lost epochs cluster after reduced-row gains, including a long loss
sequence after the 177541.4--177542.6 gain cluster. This is direct evidence
that publishing the reduced solution was useful while mutating the internal
hold/filter lifecycle was harmful. That implementation is rejected; the
publication-only lifecycle above is the accepted implementation.

## Final six-run sign-off

The maximum-streak-5 candidate completed all six Tokyo/Nagoya drives. It gains
176 FIX epochs and loses none against the fixed baseline. Across 58,259 scored
epochs it produces 14,635 FIX epochs (25.120582%), 0.359365 m FIX RMS2D,
0.347847 m FIX p68, and zero FIX epochs above 3 m.

| Run | Candidate FIX | MRTKLIB FIX | Candidate RMS2D | MRTKLIB RMS2D | >3 m FIX | Hard gate |
|---|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 10.981879% | 4.900% | 0.350715 m | 0.747 m | 0 | PASS |
| Tokyo 2 | 21.859186% | 21.700% | 0.329495 m | 0.514 m | 0 | PASS |
| Tokyo 3 | 38.377740% | 7.400% | 0.191483 m | 0.801 m | 0 | PASS |
| Nagoya 1 | 36.923077% | 17.000% | 0.451044 m | 1.105 m | 0 | PASS |
| Nagoya 2 | 23.969319% | 23.400% | 0.553906 m | 1.119 m | 0 | PASS |
| Nagoya 3 | 9.009535% | 6.300% | 0.304348 m | 0.318 m | 0 | PASS |

p68 passes on Tokyo 1 and Nagoya 3; TTFF passes on every run except Nagoya 3.
The machine-readable sign-off is `docs/ppc_clas_full_metrics.json`; its SHA-256
is `5D0E313602A1064CB10B5049EDDF27D789BA62540A71C7892298A28BCED63766`.
The generated Markdown table and all three figures were regenerated from the
complete-coverage outputs. The validated defaults are held row floor 4 and
maximum publication streak 5, with explicit row 6 / streak 1 retained as the
compatibility kill switch.
