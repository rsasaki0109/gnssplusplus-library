# Phase127 inventory-first structural raw result

- Status: `no-go-phase127-inventory-first-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.
- The native solution file was kept opaque and never opened. Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.

| Route | Inventory | Solver | Return | Rover coverage | Base coverage | Phase127 | Main QR/progress | Failure |
|---|---|---|---:|---|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `0/11832` | `0/17199` | `False` | `False/False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `0/8626` | `0/920` | `False` | `False/False` | `pre-solver inventory failed closed` |

Stage 1 inventory failure is fail-closed and prevents a native launch for that route. The sealed result contains structural telemetry only.
