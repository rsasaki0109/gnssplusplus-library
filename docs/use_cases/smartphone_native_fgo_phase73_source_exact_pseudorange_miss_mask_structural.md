# Phase73 source-exact pseudorange miss-mask structural result

The Phase73 source-exact finite-`pc` implementation passed its frozen,
truth-free structural matrix on Luna Max. The matrix used the exact
Phase64/65-pinned raw Android GNSS, IMU, broadcast-navigation, and base RINEX
inputs, with one flag-off control and two candidate repetitions per route.
There were 12 native invocations and zero truth, MATLAB, validation, holdout,
Kaggle, token, archive-reopen, or post-truth reads.

The candidate flag was
`--native-base-pseudorange-source-miss-mask`, together with the existing base
compensation flags. It retains an already adopted undifferenced pseudorange
factor only when the same-satellite/same-signal base `pc` is finite and
in-domain, then applies the source sign
`P_rover_corrected_m=P_rover_raw_m-pc_m`. Missing stream, interpolation/domain,
and nonfinite correction reasons are reported separately. TDCP, Doppler, IMU,
SPP, epoch indices, and the flag-off path are unchanged.

| route | original adopted | retained finite `pc` | missing stream | out of domain | nonfinite | retained/original | `pc` abs p50 / p95 / max (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MTV-a | 61,754 | 46,556 | 13,559 | 1,639 | 0 | 0.7538944845678013 | 7.093666204009779 / 14.90088104925494 / 15.637516129413319 |
| MTV-h | 73,286 | 51,890 | 20,172 | 1,224 | 0 | 0.7080479218404606 | 3.9124480425125676 / 10.620111099031618 / 49.64431970213562 |
| LAX-t | 34,319 | 33,552 | 0 | 767 | 0 | 0.9776508639529123 | 6.1548258637075115 / 11.184682536881832 / 49.55040764532433 |
| MTV-u | 24,395 | 22,369 | 412 | 1,614 | 0 | 0.9169501947120312 | 2.7449832184598115 / 16.10826814840464 / 18.314306376782138 |

All four routes passed exact raw/base hashes and byte sizes, finite-output and
earth-domain checks, exact prediction-key domain coverage, candidate repeat
identity, Phase43 control identity, factor accounting, and the non-pseudorange
population invariants. In every candidate summary, retained finite-`pc`
fraction is exactly `1.0`, and retained factor count equals inserted
pseudorange factor count. The retained/original values above are descriptive;
they are not truth-row coverage and do not relax the earlier Phase62/65/72
coverage gates.

The machine-readable structural freeze is
[`smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_freeze_v1.json`](records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_freeze_v1.json),
the evaluator manifest is
[`smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_manifest_v1.json`](records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_manifest_v1.json),
and the sealed output record is
[`smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json`](records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json).
The output result SHA-256 is
`ab1c93d3ce5125bd10fd76b515e96f99f7de59a373b44a203832800c027b9d7e` and its
output manifest SHA-256 is
`bd623852f9d7ef33abba26edc2c2a6f604ed68a8c7809ab1a8ab84f7747f35ab`.

This stage makes no accuracy or `0.782` claim. A separate accuracy freeze is
required before reading the four existing Phase44 development truth files;
there is no validation or Kaggle action in this structural stage.
