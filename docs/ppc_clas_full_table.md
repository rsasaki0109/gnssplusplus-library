| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | libgnss++ max FIX* | >3 m FIX* | libgnss++ TTFF | MRTKLIB TTFF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 100.000% / 99.966% | 9.735% | 4.900% | 0.306 m | 0.747 m | 0.127 m | 0.244 m | 41.661 m | 16.645 m | 79.925 m | 1.545 m | 0 | 0.000 s | 15.000 s |
| Tokyo 2 | 100.000% / 99.989% | 20.891% | 21.700% | 0.326 m | 0.514 m | 0.209 m | 0.120 m | 25.877 m | 18.159 m | 45.239 m | 1.013 m | 0 | 34.000 s | 368.000 s |
| Tokyo 3 | 100.000% / 99.980% | 37.623% | 7.400% | 0.185 m | 0.801 m | 0.082 m | 0.075 m | 35.246 m | 19.747 m | 88.489 m | 2.986 m | 0 | 4.000 s | 28.000 s |
| Nagoya 1 | 100.000% / 99.974% | 36.724% | 17.000% | 0.444 m | 1.105 m | 0.420 m | 0.402 m | 57.154 m | 7.885 m | 119.031 m | 1.049 m | 0 | 0.000 s | 0.000 s |
| Nagoya 2 | 100.000% / 99.958% | 23.863% | 23.400% | 0.614 m | 1.119 m | 0.612 m | 0.461 m | 25.706 m | 16.352 m | 40.178 m | 3.200 m | 19 | 0.000 s | 0.000 s |
| Nagoya 3 | 100.000% / 99.962% | 4.865% | 6.300% | 0.317 m | 0.318 m | 0.291 m | 0.339 m | 13.550 m | 13.748 m | 14.102 m | 0.591 m | 0 | 29.200 s | 9.000 s |
| **Six-run aggregate** | — | **24.110%** | — | **0.370 m** | — | **0.349 m** | — | **36.446 m** | **16.800 m** | **70.182 m** | **3.200 m** | **19** | — | — |

\* libgnss++ precision uses the raw PPC reference point (already antenna-positioned; no lever-arm transform applied).
† Published MRTKLIB v0.4.2 precision also uses the raw PPC reference point; precision columns use the same reference convention and are directly comparable.
