# Phase70 base-matching taxonomy SBAS recovery

Phase70 is a separately frozen, truth-free recovery of the Phase69
evaluator-integrity failure. The native RINEX source recognizes system
characters `G/R/E/C/J/S/I`; the Phase69 helper omitted `S` (SBAS), although the
LAX base member contains SBAS rows such as `S31`/`S33`.

The recovery consumes every physical RINEX3 record using the Phase69 long-line
and standard-continuation framing. It recognizes SBAS and NavIC rows so they
cannot be mistaken for malformed input, then omits them from the selected
matching index because the pinned source signal policy has no selected rover
pseudorange signal for those systems. A focused fixture asserts that an SBAS
row is consumed, contributes no matching stream, and cannot alter the Android
raw proxy population. GPS long records and epoch boundaries remain covered.

The Phase68/69 taxonomy and Phase67 native coverage gates remain unchanged.
Each pinned raw CSV and base RINEX member is read exactly once in one process
after the Phase70 evaluator manifest is sealed. No truth, MAT, navigation,
IMU, solver, validation, archive, or prior partial output is read. Raw adopted
and base selected populations remain diagnostic proxies and are not claimed
equal to native adopted FGO factors.

See the [Phase70 freeze](records/smartphone_r5_phase70_base_matching_taxonomy_sbas_recovery_freeze_v1.json)
for the source mapping, read contract, and prior failure pins.

## Sealed result

The one-shot audit completed with raw/base reads **4/4**, truth/MAT/nav/IMU/
solver/archive reads **0**, and all classification sum checks true. The
diagnostic raw adopted proxy contained **278,431** rows: exact SignalType
in-domain **201,660 (72.4273%)**, same-frequency variant **0**, out-of-domain
**7,896 (2.8359%)**, missing frequency **34,592 (12.4239%)**, and missing
satellite **34,283 (12.3129%)**. There were **50,417** duplicate canonical
frequency events/extra rows. The base parser consumed and omitted **492 SBAS**
records (all in LAX-t) under the source policy; they did not enter matching.

This is a taxonomy finding, not an accuracy result: proxy populations are not
claimed equal to native adopted FGO factors, no Phase67 gate was relaxed, and
`0.782` was not evaluated. No native correction is authorized. The strongest
actionable structure is the absence of same-frequency variants (so key
canonicalization has no evidence) plus GPS L5 missing-frequency rows caused by
the base reader's default primary/secondary band selection. A separately
frozen follow-up may audit enabling
`preserve_additional_frequency_bands=true` for the base compensation reader
only; it must preserve exact keys and independently verify coverage.

The immutable result summary is in the
[Phase70 result record](records/smartphone_r5_phase70_base_matching_taxonomy_sbas_recovery_result_v1.json),
with generated result SHA-256
`d2b9ec262fae3b3705a4c2debb844ac5270452c267e1c682a277d328a6270b17`.
