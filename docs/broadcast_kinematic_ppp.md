# Broadcast-Only Kinematic PPP in Urban Driving

What limits `gnss_ppp --kinematic` with broadcast ephemerides only (no precise
or SSR products) on the PPC 2024 urban drives, what was fixed, and what is left.

## Result

PPC 2024 Tokyo run1 and Nagoya run1 (`rover.obs` + `base.nav`, 5 Hz, float
PPP, default white-noise position re-seeded from SPP every epoch), scored per
epoch against `reference.csv`. H is the horizontal and U the absolute up error;
availability is the share of reference epochs with an output solution.

| Solver | Tokyo H50 / H95 | Tokyo U50 / U95 | Tokyo avail. | Nagoya H50 / H95 | Nagoya U50 / U95 | Nagoya avail. |
|---|---|---|---:|---|---|---:|
| `gnss_ppp --kinematic`, develop after PR #537 | 7.58 / 42.43 m | 19.68 / 56.01 m | 99.29 % | 5.36 / 20.26 m | 4.69 / 57.03 m | 98.80 % |
| `gnss_ppp --kinematic`, this change | **0.80 / 4.88 m** | **2.06 / 13.47 m** | 99.12 % | **3.27 / 11.60 m** | **2.37 / 22.26 m** | 98.68 % |
| `gnss_ppp --kinematic`, after the solid-earth-tide frame fix (2026-09-30) | 0.79 / 4.87 m | 2.07 / 13.48 m | 99.12 % | 3.56 / 13.31 m | 2.21 / 32.24 m | 98.68 % |
| `gnss_ppp --kinematic`, BeiDou receiver clocks (2026-10-01) | 0.91 / 5.29 m | 3.26 / 15.26 m | 99.12 % | **1.99 / 10.31 m** | 3.18 / 31.28 m | 98.68 % |
| `gnss_spp` (same data) | 1.78 / 20.44 m | 2.09 / 47.94 m | 99.12 % | 2.80 / 11.05 m | 4.11 / 20.85 m | 98.65 % |
| RTKLIB demo5 b34k PPP-kinematic, broadcast (config below) | 3.62 / 18.16 m | 4.32 / 48.96 m | 68.91 % | 5.59 / 11.77 m | 27.82 / 72.67 m | 11.99 % |
| same, innovation gates opened (`pos2-rejionno=30`, `pos2-rejcode=100`) | 3.90 / 31.68 m | 6.57 / 103.89 m | 96.75 % | 4.57 / 17.76 m | 24.05 / 66.73 m | 96.68 % |

The solid-earth-tide frame fix (the IERS 2010 tide was computed with the
Sun and Moon in the ICRS instead of the Earth-fixed frame; see
[Galileo HAS](galileo_has.md)) changes the modelled tide by up to a few
decimetres. Tokyo run1 is unchanged; Nagoya run1 moves to H95 13.3 m and U95
32.2 m, the same as with the legacy Step-1 tide (`--no-iers-solid-tide`:
3.56 / 13.31 m, U95 32.23 m) or with no solid tide (3.55 / 13.29 m, U95
32.06 m), so its previous U95 of 22 m was a side effect of the mis-framed
tide in a run whose up tail follows metre-level code errors. On runs 2 and 3
H50 moves by -0.06 to +0.13 m and the up statistics move both ways (Nagoya
run2 U50 5.39 -> 6.02 m and U95 31.0 -> 36.0 m, run3 U50 7.44 -> 5.69 m).

The BeiDou receiver clocks (one for BDS-3, one for BDS-2, with the
Galileo / QZSS / BeiDou inter-system biases kept across the per-epoch SPP
re-seeding of the GPS clock; see
[the BeiDou note](reproduce.md#beidou-with-broadcast-ephemerides-2026-10-01))
remove most of the Nagoya east bias (median east error +2.79 -> +1.41 m on
run1). The BeiDou code was the cause: the estimated biases are steady at about
+7.7 m (BDS-3) and +3.6 m (BDS-2) on all PPC drives, and with the shared GPS
clock that code offset leaned the solution east. H50 on the other runs:
Tokyo run2 1.33 -> 0.81 m, run3 0.79 -> 0.74 m, Nagoya run2 3.67 -> 2.98 m,
run3 3.89 -> 1.85 m (H95 run2 24.8 -> 15.1 m, run3 28.3 -> 18.9 m). The up
error grows on most runs (Tokyo run1 U50 2.07 -> 3.26 m, Nagoya run1
2.21 -> 3.18 m, run3 5.69 -> 9.20 m; Tokyo run2 improves 1.91 -> 0.74 m):
the biased BeiDou code had been offsetting an up error of the other
constellations. Without BeiDou (GPS + Galileo + QZSS) Tokyo run1 has a median
up error of +4.5 m; with BeiDou and its clocks, GPS + Galileo + QZSS + BeiDou
gives Tokyo run1 H50 / H95 0.95 / 4.27 m, U50 3.12 m and Nagoya run3 H50 1.64 m,
U50 2.20 m (U95 14.8 m); the Nagoya run3 up tail of the all-system run comes
from GLONASS (GPS + Galileo + QZSS + GLONASS: U50 6.63 m).

Before PR #537 (one measurement update per epoch) Tokyo run1 was at
18.1 / 117 m horizontal. The 29 epochs (20 Tokyo, 9 Nagoya) no longer output are epochs where the SPP
seed failed and the old filter coasted on a stale position with 4-6
satellites; their errors were 9 m to 5 km. For `--use-dynamics-model` see
[Dynamics model](#dynamics-model---use-dynamics-model-2026-10-01).

The Galileo HAS kinematic lane (`gnss reproduce has-idd-ppp`, OBE4 static
antenna processed kinematically) also benefits from the residual screening:
over the hour after the first 10 minutes the horizontal RMS drops from
0.156 m to 0.086 m and the up RMS from 0.206 m to 0.099 m; see
[Galileo HAS support](galileo_has.md).

On open sky the change is neutral: broadcast-only kinematic PPP on the OBE4
hour stays at H / U RMS 0.136 / 0.230 m after the first 10 minutes (0.139 /
0.220 m before). CLAS and MADOCA output is byte-identical (CLAS never reaches
this filter, coherent MADOCA is excluded from the screening and carries SSR
code biases), as are the HAS / legacy SSR static runs. The BeiDou and
single-frequency fixes also apply to broadcast-only static runs that contain
BeiDou or single-frequency satellites (the OBE4 hour has neither, so its
broadcast static output is unchanged).

## Root causes

The PPP solution was 2-4 times worse than the SPP seed it starts from every
epoch, so the extra error came from the filter, not from the data.

1. **BeiDou group delays missing (bug).** Broadcast (D1/D2) BeiDou clocks
   are referenced to B3I; B1I and B2I codes carry TGD1 / TGD2, which reach
   -45 ns (-13.5 m) on BDS-3 satellites. The PPP ionosphere-free code ignored
   them and paired B1I with B2a (whose group delay is only broadcast in
   B-CNAV), so BDS-3 rows were biased by 15-45 m (C33 prefit residual
   -50 m, C42 -36 m, against about -10 m for GPS in the first two minutes of
   Tokyo run1). The SPP applies TGD1 on its single-frequency B1I rows, which
   is why the SPP was better. Fix: without precise, SSR or DCB products,
   BeiDou uses B1I with B2I / B3I and removes TGD1 / TGD2. Tokyo H50 7.58 ->
   4.01 m, Nagoya 5.36 -> 3.84 m. (BDS-3 transmits no B2I; its band-7 code
   is B2b, so BDS-3 pairs B1I with B3I only since 2026-10-01. The PPC rover
   logs no B2b, so the PPC numbers are not affected by that part.)
2. **No post-fit residual screening in kinematic PPP (missing safeguard).**
   The only code gate rejected residuals above 20 km with broadcast
   ephemerides. On Tokyo run1 6.9 % of the code rows had prefit residuals
   above 30 m and a quarter of the epochs carried at least one row above
   100 m (NLOS). They were committed, and the persistent zenith troposphere
   state absorbed their positive, low-elevation-heavy excess delay: it ran
   from 2.3 m to 20-24 m (8-11 m after the BeiDou fix), which pulled the up
   error to tens of metres and fed the float ambiguities, from which the
   position recovered only over minutes. RTKLIB / MADOCALIB `ppp_res()`
   excludes the satellite with the largest post-fit residual beyond four
   sigmas and redoes the epoch from the predicted state. The kinematic filter
   now does the same, with the residuals standardized by their own
   covariance (Baarda w-test, `w_i = (S^-1 r)_i / sqrt((S^-1)_ii)`), until no
   row exceeds 4 or fewer than `min_satellites` satellites would remain (then
   no update is committed and the SPP seed is output). Normalizing by the
   measurement sigma alone, as RTKLIB does, singles out the phase rows of
   healthy satellites here (the native phase sigma is about 1 cm and has no
   broadcast signal-in-space term): Nagoya U95 39 m instead of 22 m. The
   troposphere now stays between 2.2 and 2.9 m on Tokyo run1. Screening
   runs for kinematic motion (not `--low-dynamics`) except coherent MADOCA,
   which stays on its MADOCALIB bridge semantics; CLAS never reaches it.
3. **Single-frequency rows without ionosphere correction (bug).** When a
   satellite's second frequency is missing (32 % of the GLONASS and about
   3 % of the GPS / BeiDou satellite-epochs on Tokyo run1), the
   ionosphere-free filter used its raw L1 code, with the full ionospheric
   delay (10 m at L1 on this solar-maximum afternoon), and tied its raw L1
   phase to the satellite's ionosphere-free ambiguity. With broadcast
   ephemerides such rows now remove the broadcast group delay (as the SPP
   does) and the Klobuchar ionosphere with RTKLIB's 50 % error variance, and
   use code only. Tokyo H95 11.8 -> 4.9 m.

Other suspects checked and ruled out on this data:

- **Cycle slips / outages.** A RTKLIB-style ambiguity reset after an outage
  (2 or 10 s) changed Tokyo / Nagoya H50 by less than 0.2 m; loss-of-lock
  flags and the geometry-free / Melbourne-Wubbena tests already reset the
  ambiguities after blockages.
- **Broadcast signal-in-space variance.** Adding the broadcast URA to the code
  variance was neutral to negative (Tokyo H50 0.81 -> 1.36 m, Nagoya
  3.25 -> 2.98 m); adding it to the phase variance as RTKLIB does made both
  runs worse (Tokyo H50 2.52 m, Nagoya U50 7.9 m).
- **Estimating Galileo / BeiDou / QZSS inter-system biases**
  (`GNSS_PPP_ESTIMATE_ISB`): mixed (BeiDou ISB: Nagoya H50 3.22 -> 1.89 m but
  H95 12.3 -> 15.3 m, Tokyo H50 0.77 -> 1.56 m), left off. Those runs
  re-initialized the system clocks every epoch, so the bias was re-estimated
  from scratch; since 2026-10-01 BeiDou has its own clocks by default with
  broadcast ephemerides and the biases are kept across epochs (see above).

## Remaining limitations

- **Metre-level floor at Nagoya.** Before the BeiDou receiver clocks Nagoya
  run1 had H50 2.5-3.6 m (Tokyo 0.6-0.9 m), a slowly varying east bias of
  1-3 m that tracked BeiDou and GLONASS (GPS + Galileo + QZSS only: H50
  1.78 m; with BeiDou 2.89 m, with GLONASS 2.09 m, with both 3.27 m). The
  BeiDou part was the missing BeiDou receiver clock (now H50 1.99 m, median
  east +1.41 m; GPS + Galileo + QZSS + BeiDou 1.70 m). GLONASS remains (one
  GLONASS clock re-initialized every epoch, no inter-frequency code biases),
  as does the SPP, which still shares one clock across GPS / Galileo / QZSS /
  BeiDou (all systems H50 2.80 m, GPS + Galileo + QZSS 0.82 m).
- **Up error.** The kinematic up error is metre-level on every run (median
  +3 m on Tokyo run1 and Nagoya run1, +9 m on Nagoya run3 with GLONASS) and
  is not explained by BeiDou: GPS + Galileo + QZSS alone has a median of
  +4.5 m on Tokyo run1.
- **Urban canyons with few satellites.** Below 15 satellites the Tokyo error
  is 2-8 m median, above 20 satellites 0.6-0.9 m (H95 about 2 m): the
  remaining tail is geometry and NLOS the screening cannot identify.
- **`--use-dynamics-model` relies on the SPP seed check.** The guard that
  re-seeds the motion states when the predicted position disagrees with SPP
  (below) fires on 4.4 % of the Tokyo run1 epochs. Where the SPP itself is
  tens of metres off for a long stretch, the dynamics output follows the
  re-seeded SPP on those epochs; Tokyo run3 H95 (5.99 m against 4.03 m with
  the default white-noise position) and Nagoya run1 H95 (11.4 m against
  10.3 m) are the two tails it does not improve.

## Dynamics model (`--use-dynamics-model`, 2026-10-01)

Outside CLAS, `--use-dynamics-model` diverged with broadcast ephemerides
(hundreds of metres to kilometres before PR #541; since the screening it
rejected the inconsistent updates and printed the SPP seed, Tokyo run1 H50
1.78 m). It now follows RTKLIB's ppp-kine with `pos1-dynamics=on`
(`udpos_ppp()` / `udclk_ppp()`, items 1 and 2) and adds two safeguards
(items 3 and 4):

1. **Receiver clock re-seeded from SPP every epoch.** The mode switched the
   clock re-seed off and the broadcast / SSR clock had no process noise
   (`process_noise_clock = 0`), so the filter held the receiver clock while
   the PPC rover clock drifts by about 150 m/s. RTKLIB re-initializes the
   clock every epoch (`udclk_ppp()`, `VAR_CLK = 60^2`); the dynamics mode now
   does the same, keeping the Galileo / QZSS / BeiDou inter-system biases as
   the default mode does.
2. **Position / velocity / acceleration states.** The filter had position and
   velocity only, with random walks of 0.04 m^2/s on the position and
   0.01 m^2/s^3 on the velocity. It now carries three acceleration states
   (`x = [r v a]`, `r += v dt + a dt^2/2`, `v += a dt`) with process noise on
   the acceleration only, 1 m/s^2/sqrt(s) horizontal and 0.1 m/s^2/sqrt(s)
   vertical in the local frame (`--process-noise-accel-h / -v`; RTKLIB
   `stats-prnaccelh / v`). Velocity is initialized from the SPP Doppler
   velocity when the SPP has one. As in RTKLIB the motion states are
   re-seeded from SPP when the mean position variance exceeds the initial
   60^2 m^2, tested on the predicted covariance so that an outage of several
   seconds restarts from SPP, and no PPP update is made on an epoch without
   an SPP seed (RTKLIB skips PPP when `pntpos()` fails). Without that,
   15 s without a seed on Tokyo run1 ended in a 2.4 km update from five
   satellites.
3. **Doppler velocity update.** Every epoch the velocity states are updated
   with the SPP Doppler velocity and its covariance before the GNSS
   measurement update. The velocity is otherwise observed only through the
   carrier-phase rows; after a few rejected or code-only epochs a velocity
   error of a few m/s was integrated into the position (Nagoya run1: a
   4 m/s drift that ended 55 m east). RTKLIB does not use Doppler in PPP.
4. **SPP disagreement guard.** The motion states are re-seeded from SPP when
   the predicted position and the SPP seed disagree beyond the 99.9 % bound
   of a chi-square with three degrees of freedom,
   `d' (P + C_spp)^-1 d > 16.27`. With a small position covariance the
   post-fit screening otherwise rejects the code rows that disagree with a
   wrong prediction, so the filter cannot recover (Tokyo run1 under NLOS at
   tow 189460: the position oscillated by up to 300 m with 11-27 of about 30
   satellites rejected per epoch).

Scope: kinematic `gnss_ppp` with broadcast, SP3 / CLK or SSR (HAS, RTCM)
products. `--clas-osr` keeps its MRTKLIB dynamics and clock handling
unchanged (CLAS tokyo_run1 output byte-identical), MADOCA L6 runs keep the
previous settings until they are compared with the MADOCALIB bridge, and
`--low-dynamics` still disables the dynamics. Runs without
`--use-dynamics-model` are byte-identical.

PPC 2024 run1 (scored as above):

| Solver | Tokyo H50 / H95 | Tokyo U50 / U95 | Tokyo avail. | Nagoya H50 / H95 | Nagoya U50 / U95 | Nagoya avail. |
|---|---|---|---:|---|---|---:|
| `gnss_ppp --kinematic` (default white-noise position) | 0.91 / 5.29 m | 3.26 / 15.26 m | 99.12 % | 1.99 / 10.31 m | 3.18 / 31.28 m | 98.68 % |
| `--use-dynamics-model`, develop (outputs the SPP seed) | 1.78 / 20.61 m | 2.10 / 48.63 m | 99.12 % | 2.81 / 11.57 m | 4.13 / 22.02 m | 98.65 % |
| `--use-dynamics-model`, items 1-2 only | 0.89 / 11.11 m | 3.67 / 22.36 m | 99.12 % | 1.59 / 30.65 m | 2.36 / 79.64 m | 98.65 % |
| + Doppler velocity update | 0.85 / 5.40 m | 3.50 / 13.95 m | 99.12 % | 1.43 / 41.03 m | 2.90 / 16.31 m | 98.65 % |
| + SPP disagreement guard (this change) | **0.74 / 4.01 m** | 3.35 / **9.48 m** | 99.12 % | **1.71** / 11.41 m | **2.86 / 26.72 m** | 98.65 % |
| RTKLIB demo5 b34k ppp-kine, `pos1-dynamics=on` | 2.77 / 19.34 m | 2.00 / 36.01 m | 34.01 % | 4.67 / 11.04 m | 16.59 / 30.98 m | 14.73 % |
| same, gates opened (`rejionno=30`, `rejcode=100`) | 3.47 / 19.60 m | 3.81 / 69.86 m | 75.29 % | 3.37 / 15.34 m | 17.49 / 57.81 m | 74.92 % |
| same, gates opened, `prnaccelh=1`, `prnaccelv=0.1` | 3.23 / 17.82 m | 4.29 / 69.05 m | 76.25 % | 3.51 / 16.01 m | 18.20 / 56.18 m | 76.70 % |

All dynamics rows use the default acceleration noise (1 / 0.1). The RTKLIB
rows use the configuration in [Reproduce](#reproduce) with
`pos1-dynamics=on` (demo5 default acceleration noise 0.1 / 0.01
m/s^2/sqrt(s)).

All six drives, default white-noise position against the dynamics model:

| Run | default H50 / H95 | dynamics H50 / H95 | default U50 / U95 | dynamics U50 / U95 |
|---|---|---|---|---|
| Tokyo run1 | 0.91 / 5.29 m | 0.74 / 4.01 m | 3.26 / 15.26 m | 3.35 / 9.48 m |
| Tokyo run2 | 0.81 / 4.48 m | 0.63 / 3.25 m | 0.74 / 10.30 m | 0.83 / 4.19 m |
| Tokyo run3 | 0.74 / 4.03 m | 0.64 / 5.99 m | 3.08 / 6.90 m | 2.97 / 6.38 m |
| Nagoya run1 | 1.99 / 10.31 m | 1.71 / 11.41 m | 3.18 / 31.28 m | 2.86 / 26.72 m |
| Nagoya run2 | 2.98 / 15.05 m | 2.60 / 9.19 m | 6.81 / 38.23 m | 6.73 / 27.90 m |
| Nagoya run3 | 1.85 / 18.86 m | 1.85 / 6.65 m | 9.20 / 56.45 m | 8.59 / 24.36 m |

Availability is unchanged except Nagoya run1 and run2 (-0.03 %, epochs
without an SPP seed). Acceleration noise 3 / 1 m/s^2/sqrt(s) gives a
similar picture (mean H50 1.26 m against 1.36 m over the six runs, mean H95
6.73 m against 6.75 m, mean U95 15.9 m against 16.5 m); 1 / 0.1 is the
default because it is also neutral on a static antenna.

OBE4 hour processed kinematically (RMS after the first 10 minutes,
H / U): HAS IDD 0.085 / 0.102 m by default, 0.196 / 0.442 m with the
develop dynamics mode and 0.084 / 0.100 m now; broadcast only 0.138 /
0.237 m by default, 0.238 / 1.168 m on develop and 0.138 / 0.236 m now; IGS
final SP3 / CLK 0.265 / 0.317 m in all three (that path already had a clock
process noise).

## Reproduce

```bash
gnss_ppp --obs PPC-Dataset/tokyo/run1/rover.obs --nav PPC-Dataset/tokyo/run1/base.nav \
  --kinematic --out tokyo_run1_ppp.pos
# dynamics model (add --process-noise-accel-h 3 --process-noise-accel-v 1 for
# the 3 / 1 comparison)
gnss_ppp --obs PPC-Dataset/tokyo/run1/rover.obs --nav PPC-Dataset/tokyo/run1/base.nav \
  --kinematic --use-dynamics-model --out tokyo_run1_ppp_dyn.pos
```

RTKLIB demo5 b34k `rnx2rtkp -k conf rover.obs base.nav` with:

```text
pos1-posmode       =ppp-kine
pos1-frequency     =l1+l2
pos1-soltype       =forward
pos1-elmask        =15
pos1-dynamics      =off
pos1-tidecorr      =off
pos1-ionoopt       =dual-freq
pos1-tropopt       =est-ztd
pos1-sateph        =brdc
pos1-navsys        =61
pos2-armode        =off
out-solformat      =xyz
```

(demo5 defaults otherwise, including `pos2-rejionno=5` / `pos2-rejcode=30`;
the second RTKLIB row adds `pos2-rejionno=30` and `pos2-rejcode=100`. On the
Windows build the input paths must use backslashes.)
