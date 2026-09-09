# Phase103 Phase102 truth-only evaluator correction result

- Status: `no-go-phase103-phase102-truth-only-correction-accuracy-gates`
- Decision: truth-only correction accuracy gate failed closed; preserve artifacts and do not release
- Native solver: not rerun; immutable Phase102 MTV-A/LAX-T outputs reused
- Truth: one read per route in this evaluator subprocess only
- Solution rows: not included in this result

## Macro

- Candidate macro: `2.225306568701026` m
- Phase82 same-route macro: `1.026451431707709` m
- Phase100 QR scalar-clock macro: `2.2339371202271145` m
- Strict `0.782 m` gate: `False`

## Routes

| Route | Native reused | Candidate | Phase82 | Phase100 | Truth read | Gates |
|---|---:|---:|---:|---:|---:|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `true` | `1.139793072101309` | `1.1139384500152307` | `1.147436714982201` | `True` | `False` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `true` | `3.310820065300743` | `0.9389644134001871` | `3.3204375254720286` | `True` | `False` |

The only correction was presence-sensitive handling of optional native summary field `mat_used`; no false field was synthesized. GO does not authorize release, validation, or Kaggle submission.
