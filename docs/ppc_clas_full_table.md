| Run | Coverage (time / epochs) | libgnss++ FIX | MRTKLIB FIX | libgnss++ FIX RMS2D* | MRTKLIB RMS2D† | libgnss++ FIX p68* | MRTKLIB p68† | All RMS2D* | FLOAT RMS2D* | SINGLE RMS2D* | libgnss++ max FIX* | >3 m FIX* | libgnss++ TTFF | MRTKLIB TTFF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Tokyo 1 | 100.000% / 99.966% | 9.541% | 4.900% | 0.766 m | 0.747 m | 0.306 m | 0.244 m | 41.866 m | 17.161 m | 80.321 m | 3.136 m | 36 | 0.000 s | 15.000 s |
| Tokyo 2 | 100.000% / 99.989% | 16.942% | 21.700% | 0.521 m | 0.514 m | 0.387 m | 0.120 m | 25.769 m | 17.067 m | 45.405 m | 2.484 m | 0 | 7.000 s | 368.000 s |
| Tokyo 3 | 100.000% / 99.980% | 37.236% | 7.400% | 0.327 m | 0.801 m | 0.309 m | 0.075 m | 35.292 m | 19.675 m | 88.663 m | 1.845 m | 0 | 4.000 s | 28.000 s |
| Nagoya 1 | 100.000% / 99.974% | 33.448% | 17.000% | 1.046 m | 1.105 m | 1.186 m | 0.402 m | 57.253 m | 7.676 m | 119.256 m | 1.638 m | 0 | 0.000 s | 0.000 s |
| Nagoya 2 | 100.000% / 99.958% | 17.588% | 23.400% | 0.745 m | 1.119 m | 0.776 m | 0.461 m | 25.792 m | 15.812 m | 39.933 m | 1.304 m | 0 | 0.000 s | 0.000 s |
| Nagoya 3 | 100.000% / 99.962% | 6.421% | 6.300% | 0.929 m | 0.318 m | 0.938 m | 0.339 m | 13.609 m | 13.986 m | 14.180 m | 1.499 m | 0 | 28.200 s | 9.000 s |
| **Six-run aggregate** | — | **22.055%** | — | **0.663 m** | — | **0.508 m** | — | **36.525 m** | **16.615 m** | **70.291 m** | **3.136 m** | **36** | — | — |

\* libgnss++ precision uses PPC vehicle truth transformed to the antenna phase center.
† Published MRTKLIB v0.4.2 precision uses the unmodified PPC reference point; precision columns are contextual rather than reference-identical.
