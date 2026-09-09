# Phase137 Phase136/135 isolated truth-only accuracy result

The independent authorization was commit
`2c7bba817e9bcf2d389099e375e97f27212f0781` (authorization SHA-256
`b97dbfbef7a9039c1dcd072cceb574136e8e8c15eb03a544f6a6a7b11b95f161`).  The
evaluator ran exactly once in the authorized order MTV-A then LAX-T.  It read
one immutable candidate and one official truth payload per route and performed
one score calculation per route.  No native solver, raw/base input, MAT/PDC,
precomputed coordinate, Kaggle, repair, fallback, or rerun occurred.

## Sealed scores

| route | candidate rows | finite/Earth-valid | domain coverage | over-70 m/s | P50 (m) | P95 (m) | route score (m) | route gate |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| MTV-A | 2158 | true | 1.0 | 0 | 136.6047762154 | 533.5962515944 | 335.1005139049 | NO-GO |
| LAX-T | 1465 | true | 1.0 | 0 | 71.1856534834 | 324.4762396850 | 197.8309465842 | NO-GO |

The unweighted macro in the fixed route order is
`266.46573024454483 m`.  The strict promotion comparator is
`candidate_macro_score_m < 0.782`; it failed.  The route `<=3 m` and no
regression-vs-Phase112 gates also failed.  Schema/exact alignment, finite and
Earth-valid values, exact prediction-domain coverage, and zero over-70-m/s
transitions passed.  The result is therefore sealed as NO-GO without changing
or re-running either output.

The candidate opaque seals were unchanged from the Phase136 structural result:
MTV-A SHA-256
`dad4b5ee82942e11b051155f76fd28fe59c21e2928caba0d48a8955a6024005a` (172694
bytes, 2158 rows) and LAX-T SHA-256
`df022cad3fdd68a3de2a6b2eceac0c0520a5a22fda61c76edf997efde964b519` (117254
bytes, 1465 rows).  Candidate and truth coordinate rows are omitted from this
note and the JSON result; only parser/metric aggregates and opaque metadata are
sealed.

## Accounting and baselines

Read accounting is candidate paths 2, candidate payload reads 2, candidate
coordinate interpretations 2, truth paths 2, truth reads 2, and accuracy
calculations 2, all in one Phase137 truth-only evaluator process.  Native
solver invocations, raw GNSS/IMU/navigation reads, raw base reads,
MAT/PDC/precomputed-coordinate reads, and Kaggle/token access are all zero.
Reruns, repairs, and fallbacks are zero.  Solution publication is false.

The informational baselines remain Phase112 macro `0.8318381724` and Phase118
champion macro `0.8141981503`; they were not re-read or re-run.  Pixel5 offset
was not reapplied.  Because the strict gate failed, accuracy promotion and
Kaggle submission remain unauthorized.

The machine-readable sealed result is
`smartphone_r5_phase137_phase136_accuracy_result_v1.json`, SHA-256
`cd3d57f445a9f5bc07206600edc5e27fd1638f857723f9fa3b2aa7d490caaf5e`.
