| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | libgnss++ max FIX* | >3 m FIX* | libgnss++ TTFF | MRTKLIB TTFF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 100.000% / 99.975% | 10.982% | 4.900% | 0.351 m | 0.747 m | 0.126 m | 0.244 m | 41.861 m | 16.824 m | 80.507 m | 1.961 m | 0 | 0.000 s | 15.000 s |
| Tokyo 2 | 100.000% / 99.989% | 21.859% | 21.700% | 0.329 m | 0.514 m | 0.206 m | 0.120 m | 25.880 m | 18.346 m | 45.128 m | 1.333 m | 0 | 34.000 s | 368.000 s |
| Tokyo 3 | 100.000% / 99.980% | 38.378% | 7.400% | 0.191 m | 0.801 m | 0.082 m | 0.075 m | 35.276 m | 19.617 m | 88.519 m | 2.986 m | 0 | 4.000 s | 28.000 s |
| Nagoya 1 | 100.000% / 99.974% | 36.923% | 17.000% | 0.451 m | 1.105 m | 0.434 m | 0.402 m | 57.163 m | 7.966 m | 119.111 m | 1.043 m | 0 | 0.000 s | 0.000 s |
| Nagoya 2 | 100.000% / 99.958% | 23.969% | 23.400% | 0.554 m | 1.119 m | 0.612 m | 0.461 m | 25.829 m | 16.233 m | 40.405 m | 0.767 m | 0 | 0.000 s | 0.000 s |
| Nagoya 3 | 100.000% / 99.962% | 9.010% | 6.300% | 0.304 m | 0.318 m | 0.296 m | 0.339 m | 13.724 m | 14.392 m | 14.380 m | 0.587 m | 0 | 27.000 s | 9.000 s |
| **Six-run aggregate** | — | **25.121%** | — | **0.359 m** | — | **0.348 m** | — | **36.522 m** | **16.885 m** | **70.361 m** | **2.986 m** | **0** | — | — |

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
