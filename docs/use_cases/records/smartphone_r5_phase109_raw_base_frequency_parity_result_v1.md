# Phase109 raw-base frequency-parity structural result

- status: `go-phase109-raw-base-frequency-parity-structural`
- candidate: `phase109-phase101-raw-base-existing-additional-frequency-band-guard-v1`
- routes: `2021-03-16-18-59-us-ca-mtv-a/pixel5, 2022-04-01-18-22-us-ca-lax-t/pixel5`
- truth/MAT/phone-result-coordinate/PDC/Kaggle/accuracy lanes: not read
- solution CSV: withheld and not opened

## Route gates

| Route | Return | Structural gates |
|---|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `11/11` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `11/11` |

The JSON artifact contains compact per-signal/band conservation, exactly-once correction, GNSS-first/main progress, C7/D handoff, QR, and output-coverage telemetry.  A failed gate remains sealed as fail-closed; no retry or fallback is available.


## Result-seal note

The two native runs completed before the wrapper encountered a post-run taxonomy-key exception. The sealed JSON preserves that exception; artifact recovery corrected only the evaluator key alias in memory, with zero reruns/fallbacks and no solution/truth/accuracy reads.
