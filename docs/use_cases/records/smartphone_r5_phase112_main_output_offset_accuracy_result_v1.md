# Phase112 main output offset truth-only accuracy result

- status: `no-go-phase112-main-output-offset-truth-only-accuracy`
- Native solver/raw/base: not rerun; Phase112 sealed solutions reused
- Truth: one official file read per route by the authorized evaluator subprocess
- Solution/truth coordinate rows: omitted from this result

- Candidate macro: `0.8318381724000121` m
- Phase82 same-route macro: `1.026451431707709` m
- Phase108 offsetless QR macro: `0.9819120288701966` m
- Strict `0.782 m` gate: `False`

| Route | Candidate (m) | Phase82 (m) | Phase108 QR (m) | Truth read | Gates |
|---|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0.997685253035948` | `1.1139384500152307` | `1.1396560717187856` | `True` | `True` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0.6659910917640763` | `0.9389644134001871` | `0.8241679860216076` | `True` | `True` |

Failure is fail-closed; accuracy GO does not authorize release, publication, or Kaggle submission.
