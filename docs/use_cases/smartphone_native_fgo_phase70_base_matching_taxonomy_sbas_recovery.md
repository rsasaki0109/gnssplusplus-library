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
