| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | libgnss++ max FIX* | >3 m FIX* | libgnss++ TTFF | MRTKLIB TTFF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 100.000% / 99.975% | 10.704% | 4.900% | 0.352 m | 0.747 m | 0.126 m | 0.244 m | 41.862 m | 16.800 m | 80.343 m | 1.961 m | 0 | 0.000 s | 15.000 s |
| Tokyo 2 | 100.000% / 99.989% | 21.507% | 21.700% | 0.322 m | 0.514 m | 0.205 m | 0.120 m | 25.882 m | 18.287 m | 45.148 m | 1.013 m | 0 | 66.000 s | 368.000 s |
| Tokyo 3 | 100.000% / 99.980% | 37.951% | 7.400% | 0.192 m | 0.801 m | 0.082 m | 0.075 m | 35.276 m | 19.531 m | 88.519 m | 2.986 m | 0 | 4.000 s | 28.000 s |
| Nagoya 1 | 100.000% / 99.974% | 36.737% | 17.000% | 0.450 m | 1.105 m | 0.434 m | 0.402 m | 57.163 m | 7.948 m | 119.111 m | 1.043 m | 0 | 0.000 s | 0.000 s |
| Nagoya 2 | 100.000% / 99.958% | 23.959% | 23.400% | 0.625 m | 1.119 m | 0.612 m | 0.461 m | 25.829 m | 16.230 m | 40.405 m | 3.200 m | 19 | 0.000 s | 0.000 s |
| Nagoya 3 | 100.000% / 99.962% | 8.776% | 6.300% | 0.304 m | 0.318 m | 0.296 m | 0.339 m | 13.724 m | 14.360 m | 14.380 m | 0.587 m | 0 | 27.000 s | 9.000 s |
| **Six-run aggregate** | — | **24.851%** | — | **0.377 m** | — | **0.349 m** | — | **36.523 m** | **16.843 m** | **70.337 m** | **3.200 m** | **19** | — | — |

\* libgnss++ precision uses the raw PPC reference point (already antenna-positioned; no lever-arm transform applied).
† Published MRTKLIB v0.4.2 precision also uses the raw PPC reference point; precision columns use the same reference convention and are directly comparable.
