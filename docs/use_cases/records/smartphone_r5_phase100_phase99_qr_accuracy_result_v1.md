# Phase100 Phase99 QR accuracy result

- Status: `no-go-phase100-qr-accuracy-gates`
- Decision: accuracy gate failed closed; preserve artifacts and do not release
- Scope: MTV-A and LAX-T, exactly one fresh native Phase99 QR run each
- Native inputs: raw device_gnss.csv, device_imu.csv, and broadcast brdc.nav only
- Truth: evaluator subprocess only; solution rows are not published

## Macro

- Candidate macro: `2.2339371202271145` m
- Phase43 control macro: `3.3158312396716876` m
- Improvement: `1.081894119444573` m

## Routes

| Route | Return | Candidate score | Control score | Improvement | Truth read | Gates |
|---|---:|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `1.147436714982201` | `2.1200062768576293` | `0.9725695618754284` | `True` | `True` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `3.3204375254720286` | `4.511656202485746` | `1.1912186770137172` | `True` | `False` |

No solution coordinate rows are included in this result. A GO does not authorize release, validation, or Kaggle submission.
