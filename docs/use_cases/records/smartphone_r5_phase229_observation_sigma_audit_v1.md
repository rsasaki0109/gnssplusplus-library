# Phase229: source observation-sigma audit

Primary agent. Read cached .m algorithm text and native source only; no MAT,
raw solve, truth/candidate payload reads or evaluation.

obserrmodel.m computes a separate band-wide SNR reference with
prctile(obs.(f).S, prm.sn_ptile, "all"). Its SNR base is
10^(-(S-reference)/sn_den). parameters.m selects the SNR model, percentile
85, denominator 20, carrier coefficient 1/400 and signal-type factors
[0.8,1.5,0.8,0.8,0.5,0.5,0.5,NaN]. fgo_gnss_imu.m passes the earlier
epoch's resulting L sigma directly to noise_sigmas for TDCP. There is no
wavelength conversion in these inspected sigma-construction/call sites.

The current Phase227 summary instead reports a fixed TDCP sigma of 0.03 m;
the dynamic Phase117 selector is off. Existing Phase117-related native
telemetry describes source L sigma as cycles converted by wavelength. This
is a unit-parity question requiring upstream resL and factor-residual tracing,
not proof from this audit alone that the native conversion is wrong.

Next trace resL construction and the original TDCP factor's residual units,
then locate the exact native dynamic-sigma conversion. A controlled unit test
should establish whether the source 1/400 coefficient already represents
metres before enabling any dynamic-sigma branch. Also establish which
retained/masked SNR population feeds the percentile; identical percentile
numbers do not imply identical weights when populations differ.

Do not simply combine Phase117 and Phase184: current guards prohibit it,
and the source/native unit and population contracts must be resolved first.
No production edits made. H best remains 1.2680574850266653 m; broader-route
validation and the 0.782/leaderboard objective remain outstanding.
