# Phase239: raw SNR admission boundary

Code-only investigation plus aggregate Phase234 loader telemetry. No truth,
MAT payload, saved position input, or native rerun.

`output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m`
removes zero TimeNanos, unknown GLONASS SVIDs, QZSS/SBAS/IRNSS,
BiasUncertaintyNanos > 1e4, and ReceivedSvTimeNanos < 1e10 before
building per-band matrices. Its S matrix receives Cn0DbHz directly at
line 185. P invalidity masks happen later and do not remove this S.

`src/io/android_raw_gnss.cpp:744` implements corresponding timing and
clock-uncertainty gates before signal construction. Supported rows retain
SNR even when P/D/L are status-masked. However, at line 842 the loader
drops the whole row if rawPseudorange fails. The code comment explicitly
notes the source retains such rows until its later P mask. This is a
potential SNR population difference, not yet a demonstrated H discrepancy.
Do not change this safety boundary without measuring affected rows and
checking independent D/L usability.

Native Pixel5 duplicate supported satellite/signal keys fail the load
(line 997); first-row deduplication is limited to two Samsung models.
The source also has an explicit half-block operation for those Samsung
models. Its generic intersect/indexed assignment is not a documented
Pixel5 duplicate-selection policy. Successful Phase234 is evidence no
duplicate *admitted native* Pixel5 key triggered failure, not proof every
raw/source key is unique.

Phase234 aggregate telemetry reports 112833 raw rows and 109747 selected
rows over 3140 epochs, with enriched pseudoranges ignored and no device-WLS
seed. The 3086-row difference is not attributed by this summary. It must
not be assumed to consist of invalid-pseudorange rows.

Next: quantify raw-only removal reasons and band SNR population on H using
the loader's existing row diagnostics or a bounded raw-only audit. Include
frequency mapping and missing/zero SNR treatment. Preserve the best recipe
until a concrete mismatch and its scope are demonstrated.
