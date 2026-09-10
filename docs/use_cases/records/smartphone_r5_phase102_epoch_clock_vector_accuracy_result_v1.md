# Phase102 epoch-clock-vector accuracy result

- Status: `no-go-phase102-epoch-clock-vector-accuracy-gates`
- Decision: accuracy gate failed closed; preserve artifacts and do not release
- Scope: MTV-A and LAX-T, exactly one fresh Phase101 raw-only run each
- Native inputs: raw device_gnss.csv, device_imu.csv, and broadcast brdc.nav only
- Truth: evaluator subprocess only; solution rows are not published

## Macro

- Candidate macro: `None` m
- Phase82 same-route macro: `1.026451431707709` m
- Phase100 QR scalar-clock macro: `2.2339371202271145` m
- Phase43 control macro: `None` m
- Strict `0.782 m` gate: `False`

## Routes

| Route | Return | Candidate | Phase82 | Phase100 | Control | Truth read | Gates |
|---|---:|---:|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `None` | `None` | `None` | `None` | `False` | `False` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `None` | `None` | `None` | `None` | `False` | `False` |

No solution coordinate rows are included in this result. GO does not authorize release, validation, or Kaggle submission.
