# Phase573: H/A adjacent residual diagnostics completed

Both Phase571 native jobs are terminal (exit 0):

| Route | Session | Runtime s | Linked residual pairs | Pooled post-fit correlation |
|---|---:|---:|---:|---:|
| H | 67031 | 215.5737909299787 | 64049 | -0.200635 |
| MTV-A | 67335 | 174.6761694320012 | 41616 | 0.00491398 |

Both runners verified frozen source/binary/raw pins, byte-identical positioning
output, and exact epochs/graph/GNSS-first/TDCP summary equality. H output SHA
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e;
A output SHA 76322d25a5399d5a1324ad8bc4dd9d8a36b2a9148d4cbf13b33f2445c9f209b5.
The values are printed at the existing stream precision, not high-precision
covariance estimates. No truth reads, rescoring, inference changes or promotion.

H has negative pooled adjacent post-fit association, A is near zero. These
do not provide a shared empirical noise parameter. Phase572's exact controls
show endpoint noise correlation can be changed by pooling offsets or fitting;
therefore neither value proves or disproves independent endpoint noise.
Do not use H's -0.20 or A's 0.005 as a fitted whitening coefficient, and do
not assume the theoretical -0.5 applies to these reconstructed residuals.
Any temporal covariance candidate needs an explicit endpoint-noise model
and compatible robust-loss treatment, not tuning from these aggregates.

After both completions, registered the three passing Phase572 tests in
tests/CMakeLists.txt. This deliberately changes that source hash AFTER the
successful run validations; historical manifests remain unchanged. Full
CTest unrun. Measurement and solver defaults remain as before; .782-class
and leaderboard goal remain unmet. No live Phase571 jobs to restart.
