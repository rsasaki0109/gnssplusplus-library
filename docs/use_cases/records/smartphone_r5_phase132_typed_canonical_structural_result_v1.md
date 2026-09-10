# Phase132 typed canonical preflight structural raw result

- Status: `no-go-phase132-typed-canonical-preflight-structural`
- Matrix: MTV-A then LAX-T, exactly one inventory pass and at most one native solver attempt per route.
- Typed preflight key: `(GNSSSystem, PRN, physical-frequency-family[, certified GLONASS FCN])`; literal text is provenance only.
- Phase130 literal preflight calls are sealed as `0`; Phase130 counters are comparison-only.
- Solution rows were not opened or interpreted; only opaque hashes and row counts were sealed.
- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.

| Route | Inventory | Solver attempt | Return | Typed calls | Old literal calls | Native resolver calls | Main accepted | Main cost | GO | Failure |
|---|---|---|---:|---:|---:|---:|---:|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `True` | `True` | `2` | `1` | `0` | `0` | `0` | `None->None` | `False` | `Unknown argument: --native-phase130-shared-ledger-key-local-support` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `True` | `True` | `2` | `1` | `0` | `0` | `0` | `None->None` | `False` | `Unknown argument: --native-phase130-shared-ledger-key-local-support` |

Inventory telemetry (certified / explicit miss):

| Route | Rover | Base | Canonical support keys | Factor input / retained / missing | Base streams used / unused |
|---|---:|---:|---:|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `6797 / 5271` | `9642 / 7557` | `6 (4 retained)` | `11832 / 6797 / 5035` | `6797 / 2845` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `4058 / 4815` | `396 / 524` | `4 (3 retained)` | `8626 / 4058 / 4568` | `396 / 0` |

Both inventories passed and constructed/forwarded the native command once.
The native binary rejected the unsupported Phase130 runner-only flag before
native resolver/graph entry; no rerun is permitted and all structural gates
remain fail-closed.

Qualification note: the existing CTest baseline was 166/192 passed and 26
failed.  The two `run_tests` loader failures lacked `libmetis-gtsam.so`; the
remaining failures were historical sealed source/binary pin expectations or
the existing strict-doc warning.  The Phase132 focused Python set was 29/29
passed.  These qualification results are retained as baseline metadata and
were not rerun or hidden.
