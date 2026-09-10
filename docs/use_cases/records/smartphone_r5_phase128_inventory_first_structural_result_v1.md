# Phase128 inventory-first structural raw result

- Status: `no-go-phase128-inventory-first-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.
- Native solution content was kept opaque and never opened. Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.

| Route | Inventory | Solver | Return | Rover FCN coverage | Base FCN coverage | Nav accepted/rejected | Phase128 | Main QR/progress | Failure |
|---|---|---|---:|---:|---:|---:|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `6858/11832` | `9642/17199` | `671/480` | `False` | `False/False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `4058/8626` | `396/920` | `686/442` | `False` | `False/False` | `pre-solver inventory failed closed` |

Inventory failure is fail-closed and prevents a native launch for that route.
