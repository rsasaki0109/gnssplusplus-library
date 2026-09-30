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

Before PR #537 (one measurement update per epoch) Tokyo run1 was at
18.1 / 117 m horizontal. The 29 epochs (20 Tokyo, 9 Nagoya) no longer output are epochs where the SPP
seed failed and the old filter coasted on a stale position with 4-6
satellites; their errors were 9 m to 5 km. `--use-dynamics-model` is not
covered by this change; see [Remaining limitations](#remaining-limitations).

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
   4.01 m, Nagoya 5.36 -> 3.84 m.
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
  H95 12.3 -> 15.3 m, Tokyo H50 0.77 -> 1.56 m), left off.

## Remaining limitations

- **Metre-level floor at Nagoya.** With 20-30 satellites Nagoya run1 still
  has H50 2.5-3.2 m (Tokyo 0.6-0.9 m), a slowly varying east bias of 1-3 m.
  It tracks BeiDou and GLONASS: GPS + Galileo + QZSS only gives H50 1.78 m
  (with BeiDou 2.89 m, with GLONASS 2.09 m, with both 3.27 m; subsets
  measured by removing systems from `rover.obs`), and the SPP
  shows the same pattern (GPS + Galileo + QZSS H50 0.82 m, all systems
  2.80 m). Candidate causes are BeiDou / GLONASS broadcast clock errors, the
  GLONASS inter-frequency code biases (one GLONASS clock, no per-channel
  bias) and the BeiDou receiver inter-system bias; none of them is a
  one-line fix. Static broadcast PPP shows the same BeiDou-3 effect on the
  Kamakura 2025-08-21 hour (GPS + BDS-3 ends 5 m low with the former
  multi-pass static filter and 13 m low with one update per epoch, GPS +
  BDS-2 or GLONASS / QZSS / Galileo within 0.52 m); see
  [the static filter note](reproduce.md#static-filter-one-measurement-update-per-epoch-2026-10-01).
- **Urban canyons with few satellites.** Below 15 satellites the Tokyo error
  is 2-8 m median, above 20 satellites 0.6-0.9 m (H95 about 2 m): the
  remaining tail is geometry and NLOS the screening cannot identify.
- **`--use-dynamics-model` with broadcast ephemerides** diverges (hundreds of
  metres to kilometres on develop) because that mode neither re-seeds the
  receiver clock from SPP nor gives it process noise (`process_noise_clock`
  is 0 on the broadcast path), and its velocity random walk (0.01 m^2/s^3)
  is far below car dynamics. With the screening it no longer diverges: it
  rejects the inconsistent updates and outputs the SPP seed (Tokyo H50
  1.79 m). Re-seeding the clock every epoch makes it work (Tokyo 0.95 /
  8.73 m, Nagoya 2.75 / 57.13 m) but not better than the default white-noise
  position; the mode shares its CLI defaults with the CLAS lane, so it is
  left for a separate change.

## Reproduce

```bash
gnss_ppp --obs PPC-Dataset/tokyo/run1/rover.obs --nav PPC-Dataset/tokyo/run1/base.nav \
  --kinematic --out tokyo_run1_ppp.pos
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
