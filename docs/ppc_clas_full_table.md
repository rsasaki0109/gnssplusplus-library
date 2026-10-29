| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | libgnss++ max FIX* | >3 m FIX* | libgnss++ TTFF | MRTKLIB TTFF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 100.000% / 99.983% | 11.916% | 4.900% | 0.345 m | 0.747 m | 0.125 m | 0.244 m | 41.674 m | 16.725 m | 80.772 m | 1.961 m | 0 | 0.000 s | 15.000 s |
| Tokyo 2 | 100.000% / 99.989% | 25.358% | 21.700% | 0.333 m | 0.514 m | 0.178 m | 0.120 m | 22.587 m | 12.182 m | 42.532 m | 1.333 m | 0 | 34.000 s | 368.000 s |
| Tokyo 3 | 100.000% / 99.980% | 38.765% | 7.400% | 0.193 m | 0.801 m | 0.082 m | 0.075 m | 35.471 m | 19.996 m | 89.603 m | 2.986 m | 0 | 4.000 s | 28.000 s |
| Nagoya 1 | 100.000% / 99.974% | 44.854% | 17.000% | 0.447 m | 1.105 m | 0.426 m | 0.402 m | 57.770 m | 4.706 m | 123.770 m | 1.043 m | 0 | 0.000 s | 0.000 s |
| Nagoya 2 | 100.000% / 99.947% | 24.941% | 23.400% | 0.547 m | 1.119 m | 0.611 m | 0.461 m | 27.959 m | 17.031 m | 43.082 m | 0.767 m | 0 | 0.000 s | 0.000 s |
| Nagoya 3 | 100.000% / 99.962% | 10.041% | 6.300% | 0.304 m | 0.318 m | 0.299 m | 0.339 m | 15.775 m | 15.784 m | 17.691 m | 0.580 m | 0 | 27.000 s | 9.000 s |
| **Six-run aggregate** | — | **27.232%** | — | **0.361 m** | — | **0.348 m** | — | **36.637 m** | **16.278 m** | **70.927 m** | **2.986 m** | **0** | — | — |

\* libgnss++ precision uses the raw PPC reference point (already antenna-positioned; no lever-arm transform applied).
† Published MRTKLIB v0.4.2 precision also uses the raw PPC reference point; precision columns use the same reference convention and are directly comparable.

## MRTKLIB v0.4.2 sign-off

The hard gate requires every run to retain at least 99% time and epoch coverage, meet or exceed MRTKLIB FIX rate, meet or beat MRTKLIB FIX RMS2D, and contain zero FIX epochs above 3 m. p68 and TTFF are tracked as soft gates.

| Run | Coverage | FIX rate | FIX RMS2D | >3 m FIX | Hard gate | p68 | TTFF | Soft gate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | PASS | PASS | PASS | PASS | **PASS** | PASS | PASS | PASS |
| Tokyo 2 | PASS | PASS | PASS | PASS | **PASS** | MISS | PASS | PARTIAL |
| Tokyo 3 | PASS | PASS | PASS | PASS | **PASS** | MISS | PASS | PARTIAL |
| Nagoya 1 | PASS | PASS | PASS | PASS | **PASS** | MISS | PASS | PARTIAL |
| Nagoya 2 | PASS | PASS | PASS | PASS | **PASS** | MISS | PASS | PARTIAL |
| Nagoya 3 | PASS | PASS | PASS | PASS | **PASS** | PASS | MISS | PARTIAL |

**Overall hard gate: PASS**
Overall soft gate: PARTIAL
