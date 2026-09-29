# Galileo HAS Support

Status of Galileo High Accuracy Service (HAS) support in libgnss++.

| Capability | Status |
|---|---|
| Float PPP with HAS corrections from the HAS Internet Data Distribution (IDD, RTCM 3 SSR) | **Supported** (`gnss_ppp --ssr-rtcm <idd.rtc> --ssr-rtcm-profile has-idd`) |
| Static float PPP validated against a known coordinate | **Validated** on the public OBE4 / 2023-08-17 IDD sample, lane `gnss reproduce has-idd-ppp` |
| Kinematic float PPP | **Validated** on the same sample (white-noise position, no motion prior): 0.02 m horizontal / -0.39 m up after one hour, gated by the lane |
| HAS signal-in-space (Galileo E6-B page) decoder | **Planned** (stage B): Reed-Solomon page recovery, MT1 mask / orbit / clock / bias decoding into the same SSR products |
| HAS phase biases and PPP-AR | **Not available**: the HAS Service Level 1 corrections used here carry no phase biases, so ambiguities stay float |

## Using HAS IDD corrections

The HAS IDD service distributes the HAS corrections as an RTCM 3 SSR stream
(GPS 1060 / 1059, Galileo 1243 / 1242) together with the GPS 1019 and Galileo
1046 broadcast ephemerides the corrections refer to.

```bash
gnss_ppp --obs OBE42023229c.obs --nav OBE42023229c.nav \
  --ssr-rtcm idd2023229c.rtc --ssr-rtcm-profile has-idd \
  --static --out has_static.pos
```

`--ssr-rtcm-profile has-idd` switches the RTCM SSR ingestion to RTCM 10403.3
semantics, which the `legacy` default (kept byte-identical for existing users)
does not apply:

- **IODE-matched broadcast orbits.** Every correction sample carries the SSR
  IODE / IODnav and PPP evaluates exactly that broadcast record. A satellite
  whose referenced ephemeris is missing is skipped instead of being corrected
  against a different record.
- **Ephemerides from the stream.** GPS 1019 and Galileo 1045 / 1046 messages
  in the SSR file are merged into the navigation data. A RINEX navigation file
  converted from the same stream usually keeps only the last record per
  satellite; on the OBE4 sample, for example, the HAS corrections for G11 and
  G12 reference IODEs that are not in the RINEX file.
- **Galileo I/NAV only.** HAS clocks refer to the I/NAV clock, so F/NAV
  records with the same IODnav are never selected
  (`NavigationData::setGalileoEphemerisSource(INavOnly)`).
- **Updates held until the next one.** Each orbit / clock update is applied
  with its rates and clock polynomial from its epoch until the next update of
  the same satellite (at most 90 s), and never interpolated across an IODE
  change.
- **RTCM code biases.** Bias signal IDs follow the RTCM SSR tables (GPS 0 =
  L1 C/A, 10 = L2 P, 8 = L2C(L); Galileo 2 = E1-C, 6 = E5a-Q, 9 = E5b-Q,
  16 = E6-C) and are added to the pseudorange. HAS code-bias messages carry
  a fixed, stale epoch, so the latest bias set per satellite is held by
  stream order. The HAS biases replace broadcast TGD / BGD.

Orbit corrections are applied with the RTCM convention
(`x = x_brdc - R_rac->ecef * dRAC`, clock `dt = dt_brdc + dC / c`) and refer
to the antenna phase centre of the broadcast orbit, so no satellite antenna
offset is applied.

## Validation (OBE4, 2023-08-17 02:00-03:00 GPST)

Data: the public HAS IDD sample in
[hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
`data/doy2023-229/` (`OBE42023229c.obs`, `OBE42023229c.nav`,
`idd2023229c.rtc`). Its license is not stated, so the files are not
redistributed with libgnss++; download them and point
`--has-data-root` / `GNSSPP_HAS_DATA_ROOT` at that directory. The reference
coordinate is the cssrlib sample value
`(4186704.2262, 834903.7677, 4723664.9337)` m (OBE4, Septentrio AsteRx4 with
a SEPCHOKE_B3E6 antenna).

```bash
python3 apps/gnss.py reproduce has-idd-ppp --has-data-root /data/cssrlib-data/data/doy2023-229 --check
```

Lane result (2026-09-29, MSVC Release, elapsed time from the first epoch
01:59:12 GPST):

| Run | H / U at 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| `has-idd`, static | 0.016 / -0.490 m | 0.058 / -0.421 m | 0.073 / -0.082 m | **0.095 / -0.135 m** | **4.2 min** | **21.0 min** |
| `legacy` conversion of the same stream, static | 0.374 / -0.537 m | 0.261 / -0.219 m | 0.267 / +0.282 m | 0.160 / +0.088 m | 57.5 min | 53.1 min |
| `has-idd`, kinematic | 0.035 / -0.195 m | 0.064 / -0.127 m | 0.136 / +0.058 m | **0.018 / -0.390 m** | 45.0 min | never (-0.44 m at the last epoch) |
| `has-idd`, kinematic, before the one-update fix | 3.03 / -5.95 m | 2.47 / -5.13 m | 2.20 / -5.71 m | 1.87 / -5.61 m | never | never |

The lane gates the static run (H at 60 min <= 0.20 m, |U| at 60 min <= 0.40 m,
horizontal convergence <= 10 min, vertical convergence <= 30 min) and the
kinematic run (H <= 0.30 m and |U| <= 0.60 m at 30 and 60 min), and reports
the other rows. The kinematic horizontal error peaks at 0.27 m between 30 and
45 min and the up error drifts from +0.3 m to -0.44 m over the last five
minutes; the static run follows the same excursions with less amplitude, so
they come from the corrections / observations of this hour rather than from
the kinematic model.

Comparison with [cssrlib](https://github.com/hirokawa/cssrlib) (main,
`samples/test_ppprtcm.py` case 1, which processes the same files from
02:00:00 GPST with `igs20.atx`). libgnss++ was run on the observation file cut
to start at 02:00:00 as well:

| Solver (static float PPP, from 02:00:00) | 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| libgnss++ `has-idd` | 0.036 / -0.460 m | 0.076 / -0.407 m | 0.081 / -0.061 m | 0.105 / -0.137 m | 3.8 min | 20.3 min |
| libgnss++ `has-idd` + `--antex igs20.atx` | 0.037 / -0.568 m | 0.077 / -0.515 m | 0.081 / -0.169 m | 0.105 / -0.244 m | 3.8 min | 27.9 min |
| cssrlib (static) | 0.214 / -0.487 m | 0.162 / -0.215 m | 0.140 / -0.100 m | 0.152 / -0.300 m | 11.0 min | 12.9 min |
| libgnss++ `legacy` (develop behaviour) | 0.306 / -0.606 m | 0.225 / -0.248 m | 0.262 / +0.253 m | 0.148 / +0.060 m | 55.1 min | 15.4 min |

Both solvers end the hour at 0.10-0.15 m horizontal and 0.14-0.30 m vertical
error. cssrlib's kinematic mode in that sample uses a 0.01 m/sqrt(s) position
random walk and gives the same numbers as its static mode. The libgnss++
kinematic filter re-seeds the position from SPP every epoch (white-noise
position, no motion prior); from 02:00:00 it gives 0.051 / -0.183, 0.072 /
-0.143, 0.178 / +0.099 and 0.024 / -0.456 m at 10 / 20 / 30 / 60 min (with
`--antex igs20.atx`: 0.052 / -0.290, 0.072 / -0.251, 0.178 / -0.008 and
0.023 / -0.563 m).

H is the horizontal error and U the up error at the given time after the
first epoch; convergence is the first time after which H stays below 0.20 m
(resp. |U| below 0.40 m) until the end of the hour. No receiver ANTEX is
applied; with `--antex igs20.atx` the up error shifts by about -0.1 m.

## Known limitations

- **Kinematic PPP (fixed).** Until the one-update fix, `--kinematic` settled
  about 2 m horizontal / 5.6 m vertical off on this hour with HAS, legacy SSR
  and broadcast-only input alike. The PPP filter re-applied the same epoch's
  measurement update up to eight times while the observation geometry stayed
  at the prior (SPP-seeded) position, so every extra pass pushed the position
  again by the innovation it had already absorbed and the troposphere and
  float ambiguities soaked up the difference. Kinematic PPP now commits one
  update per epoch, as RTKLIB / MADOCALIB do. Static and `--low-dynamics`
  runs keep the historical pass count (their prior is the previous solution,
  so the stale-geometry push is millimetre-level).
- **Galileo inter-system bias.** Galileo shares the GPS receiver clock unless
  `GNSS_PPP_ESTIMATE_ISB=gal` is set (as on the other non-MADOCA PPP paths).
  On this sample Galileo code residuals sit about 2 m below GPS; estimating
  the ISB did not improve the one-hour static result.
- **Satellite antenna frequency dependency.** Corrections are applied at the
  broadcast antenna phase centre without the per-signal satellite PCO / PCV
  differences cssrlib applies (centimetre level).
- **One-hour, one-station validation.** The public IDD sample is a single hour
  at one station; there is no multi-day validation yet.
