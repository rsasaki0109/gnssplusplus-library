# Phase108 Phase107 withheld-solution truth-only accuracy result

- Status: `no-go-phase108-phase107-truth-only-accuracy-gates`
- Decision: truth-only accuracy gate failed closed; preserve artifacts and do not rerun
- Native/raw/base processing: not rerun; existing Phase107 outputs reused
- Truth: one official file read per route by this evaluator subprocess
- Candidate/truth coordinate rows: omitted from this result

## Aggregate

- Candidate macro: `0.9819120288701966` m
- Phase82 same-route macro: `1.026451431707709` m
- Phase103 no-base C7 macro: `2.225306568701026` m
- Strict `0.782 m` gate: `False`

## Routes

| Route | Candidate (m) | Phase82 (m) | Phase103 C7 (m) | Truth read | Gates |
|---|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `1.1396560717187856` | `1.1139384500152307` | `1.139793072101309` | `True` | `False` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0.8241679860216076` | `0.9389644134001871` | `3.310820065300743` | `True` | `True` |

Failure is fail-closed. GO, if any, does not authorize release, validation, or Kaggle submission.
