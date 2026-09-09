# Phase129 GLONASS local-miss structural raw result

- Status: `no-go-phase129-glonass-local-miss-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized inventory and at most one solver attempt per route.
- Solution rows were never opened or interpreted; only opaque output hashes/expected row metadata were sealed.
- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.

| Route | Inventory | Solver | Return | Rover certified/miss | Base certified/miss | Main accepted | Main cost | GO | Failure |
|---|---|---|---:|---:|---:|---:|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `6858/5210` | `9642/7557` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `4058/4815` | `396/524` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |

Inventory failure is fail-closed and prevents a native launch for that route.
