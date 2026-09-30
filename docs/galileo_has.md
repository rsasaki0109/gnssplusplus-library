# Galileo HAS Support

Status of Galileo High Accuracy Service (HAS) support in libgnss++.

| Capability | Status |
|---|---|
| Float PPP with HAS corrections from the HAS Internet Data Distribution (IDD, RTCM 3 SSR) | **Supported** (`gnss_ppp --ssr-rtcm <idd.rtc> --ssr-rtcm-profile has-idd`) |
| Static float PPP validated against a known coordinate | **Validated** on the public OBE4 / 2023-08-17 IDD sample, lane `gnss reproduce has-idd-ppp` |
| Kinematic float PPP | **Validated** on the same sample (white-noise position, no motion prior): 0.12 m horizontal / +0.06 m up after one hour (hour RMS 0.09 m / 0.10 m), gated by the lane |
| HAS signal-in-space (Galileo E6-B C/NAV page) decoder | **Supported** (`gnss_ppp --has-pages <log>`, `gnss has-info`): u-blox RXM-SFRBX, Septentrio SBF GALRawCNAV and cssrlib page text; CRC-24Q, Reed-Solomon (HPVRS) page recovery, MT1 mask / orbit / clock full-set / clock subset / code bias / phase bias decoding |
| SIS decoder parity with cssrlib | **Exact**: every decoded orbit, clock and code-bias value of three public recordings (1 x 9 min u-blox X20, 2 x 1 h Septentrio mosaic-X5; 95 k values) equals cssrlib's `cssr_has` decoder |
| Static float PPP with SIS corrections | **Validated, indicative** on the Kamakura 2025-02-15 hour (Japan, outside the HAS service area), lane `gnss reproduce has-sis-ppp`: 0.07 m horizontal / +0.11 m up after one hour |
| HAS phase biases and PPP-AR | **Not available**: phase biases are decoded (`gnss has-info`) but not applied; the HAS Service Level 1 streams used here carry none, so ambiguities stay float |

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

## Using HAS signal-in-space pages

HAS corrections are broadcast on the Galileo E6-B signal as 448-bit HAS pages
inside the C/NAV pages. `gnss_ppp --has-pages` decodes a receiver log of those
pages and feeds the corrections to float PPP with the same conventions as the
`has-idd` profile (IODE-matched broadcast orbits, Galileo I/NAV only, held
updates, code biases instead of TGD / BGD):

```bash
gnss_ppp --obs 046r_rnx.obs --nav BRDC00WRD_S_20250460000_01D_MN.rnx \
  --has-pages 046r_gale6.txt --static --out has_sis_static.pos
```

| Page log | `--has-pages-format` | Notes |
|---|---|---|
| u-blox UBX (`.ubx`) | `ubx` | RXM-SFRBX with gnssId 2 / sigId 8 (E6-B, 16 words), e.g. X20 / F9 E6 firmware; page time = latest RXM-RAWX epoch |
| Septentrio SBF (`.sbf`, `*.yy_`) | `sbf` | GALRawCNAV (block 4024); pages the receiver flags as CRC-failed are dropped |
| cssrlib page text | `cssrlib` (default for other extensions) | `wn tow prn type len hex` lines as in [cssrlib-data](https://github.com/hirokawa/cssrlib-data) `*_gale6.txt` |

`gnss has-info --input <log>` prints the page / message statistics, the masks
(satellites and signals), the flag patterns and validity intervals, and dumps
`--messages-csv`, `--corrections-csv` (every decoded orbit / clock / bias
value with its HAS sign) and `--updates-csv` (the held per-satellite updates
PPP uses).

Decoding follows the HAS SIS ICD Issue 1.0:

- **Pages.** CRC-24Q over the 462 reserved + HAS page bits; dummy pages
  (`0xAF3BC3`) are skipped. HASS 0 (test) and 1 (operational) pages are used,
  HASS 3 ("don't use") discards every received message and truncates the held
  corrections at that time.
- **High Parity Vertical Reed-Solomon.** Pages are collected per message ID;
  any MS distinct page IDs recover the message by inverting the k x k
  sub-matrix of the RS(255,32) generator matrix over GF(256) (primitive
  polynomial 0x11D). The generator matrix of ICD Annex B is derived from the
  generator polynomial (checked against the Annex B file and the Annex C
  decoding example in the unit tests). Later pages of a decoded message are
  checked by re-encoding, so a reused message ID starts a new collection.
- **MT1.** Mask, orbit, clock full-set, clock subset, code-bias and phase-bias
  blocks, with Mask ID / IOD Set ID caches: a clock-only message is paired with
  the orbit block of the same Mask ID and IOD Set ID. DCC "not available" and
  "do not use" values end the satellite's held correction.
- **Time and validity.** TOH is resolved to GST (= GPST seconds of week) from
  the page reception time (ICD Eq. 28 / 29). Each orbit + clock pair becomes a
  held update from the later of the two reference times until the next update
  of the satellite, at most until the end of the shorter validity interval
  (the recorded streams use 300 s for orbits and biases, 60 s for clocks).
- **Conventions.** HAS orbit corrections are added to the broadcast position
  (ICD Eq. 22), so they are stored negated in the RTCM-convention SSR
  container; the clock correction is DCC x DCM added to the broadcast clock
  (Eq. 23); code biases are added to the pseudoranges (Eq. 25) and are mapped
  from the HAS signal index (ICD Table 20) to the RTCM SSR signal IDs. GPS L2
  biases follow the tracked RINEX code: C2W / C2P / C2Y use the HAS L2 P bias,
  C2L / C2S / C2X the L2 CL bias.
- **Coverage.** HAS corrects GPS and Galileo only; with `--has-pages` any
  satellite without a valid HAS orbit / clock at the epoch (other
  constellations, expired validity, missing IODref ephemeris) is excluded
  instead of falling back to its broadcast orbit.

## Validation (HAS SIS, Kamakura, 2025)

Data: the public HAS SIS samples of
[hirokawa/cssrlib-data](https://github.com/hirokawa/cssrlib-data)
(`data/doy2025-046`: 2025-02-15 17:00-18:00 GPST, `data/doy2025-233`:
2025-08-21 07:00-08:00 GPST; Septentrio mosaic-X5 with a JAVRINGANT_DM, E6-B
pages `*_gale6.txt`, RINEX observations) and the IGS merged broadcast
navigation files of those days (`BRDC00WRD_S_2025{046,233}0000_01D_MN.rnx`
from `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2025/`). The reference
coordinate is the cssrlib sample value `(-3962108.6836, 3381309.5672,
3668678.6720)` m. Kamakura is **outside the HAS service area** (the HAS SDD
excludes 60S-60N / 90E-180E), so these numbers are indicative only.

```bash
python3 apps/gnss.py reproduce has-sis-ppp --has-sis-data-root /data/cssrlib-data/data --check
```

Decoder parity with cssrlib (`cssr_has`, hirokawa/cssrlib main, 2026-09):
every message of the three recordings was decoded by both, and all values
agree exactly.

| Recording | Pages (dummy) | MT1 messages | Orbit / clock / code-bias values compared | Max difference |
|---|---:|---:|---:|---:|
| u-blox X20, Boulder CO, 2025-07-08 19:34-19:43 GPST (rtklibexplorer/GNSS_IMU `drive_0708`) | 3214 (1772) | 67 (66 usable; the first clock message precedes any mask) | 2288 / 2564 / 1949 | 0 |
| mosaic-X5, Kamakura, 2025-08-21 07h | 34026 (14151) | 432 | 14896 / 18620 / 12721 | 0 |
| mosaic-X5, Kamakura, 2025-02-15 17h | 33451 (17839) | 432 | 13596 / 16995 / 11669 | 0 |

The HAS-corrected satellite positions and clocks at 2025-08-21 07:30:00 also
agree with cssrlib's `satposs()` to 0.1 mm in clock and to about 2 cm in position
(transmission-time differences). The MRTKLIB mosaic-G5 SBF sample
(`tests/data/has/has_testdata.tar.gz`, Tokyo 2026-06-10, 15 min) decodes to
109 MT1 messages (3 receiver-flagged CRC failures); it carries no RINEX
observations, so it is used for decoding only.

Static and kinematic float PPP (`--has-pages`, IGS BRDC navigation, default
15 degree elevation mask), H / U error at the given time after the first
epoch; convergence as in the IDD table below:

| Run | 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| 2025-02-15 17h, libgnss++ SIS, static | 0.452 / +0.479 m | 0.174 / +0.245 m | **0.045 / +0.268 m** | **0.074 / +0.113 m** | **19.4 min** | **23.7 min** |
| 2025-02-15 17h, libgnss++ SIS, kinematic | 0.560 / +0.343 m | 0.177 / +0.171 m | 0.084 / +0.292 m | 0.245 / -0.235 m | never | 5.3 min |
| 2025-02-15 17h, cssrlib SIS, static (10 deg, E29 excluded, igs20.atx) | 0.531 / +0.450 m | 0.220 / -0.033 m | 0.077 / +0.193 m | 0.153 / -0.065 m | 48.4 min | 45.1 min |
| 2025-02-15 17h, libgnss++ broadcast only, GPS + Galileo | 0.520 / +0.423 m | 0.275 / +0.472 m | 0.228 / +0.330 m | 0.260 / +0.327 m | never | 50.7 min |
| 2025-08-21 07h, libgnss++ SIS, static | 0.764 / +0.246 m | 0.607 / -0.258 m | 0.605 / -0.958 m | 0.516 / -1.165 m | never | never |
| 2025-08-21 07h, libgnss++ SIS, kinematic | 0.407 / -0.070 m | 0.358 / +0.111 m | 0.366 / -0.079 m | 0.378 / -0.959 m | never | never |
| 2025-08-21 07h, cssrlib SIS, static (10 deg, L1 C/A + L2 CL) | 0.303 / -0.663 m | 0.112 / -0.501 m | 0.079 / -0.186 m | 0.079 / +0.212 m | 18.1 min | 22.2 min |
| 2025-08-21 07h, cssrlib SIS, static (15 deg, L1 C/A + L2 W) | 0.381 / -1.214 m | 0.117 / -0.629 m | 0.139 / -0.607 m | 0.084 / +0.563 m | 18.4 min | never |
| 2025-08-21 07h, libgnss++ JPL GDGPS RTCM SSR, GPS + Galileo | 0.623 / -0.670 m | 0.536 / -0.699 m | 0.499 / -0.797 m | 0.445 / -0.892 m | never | never |
| 2025-08-21 07h, libgnss++ broadcast only, GPS + Galileo | 0.690 / -0.866 m | 0.656 / -0.797 m | 0.583 / -0.838 m | 0.477 / -1.081 m | never | never |
| For reference: OBE4 (Germany) HAS IDD, static (table below) | 0.016 / -0.490 m | 0.058 / -0.421 m | 0.073 / -0.082 m | 0.095 / -0.135 m | 4.2 min | 21.0 min |

The lane gates the 2025-02-15 static run (H <= 0.20 m and |U| <= 0.40 m at
30 and 60 min) and the decoder (all 432 MT1 messages of each hour, no CRC
failure), and reports the other rows. On the 2025-08-21 hour libgnss++ sits
about 1 m low in every configuration, whatever the correction source (HAS SIS,
JPL GDGPS, broadcast only), while cssrlib on the same corrections ends the
hour within 0.6 m; only 11-13 GPS / Galileo satellites carry HAS corrections
above 15 degrees there, and the offset is a libgnss++ PPP-model issue on that
hour (it follows the troposphere estimation: with a 10 degree mask the static
run ends at -1.36 m, and at -0.54 m with the troposphere fixed to its model),
not a decoding one.

Smoke test on the u-blox X20 drive (rtklibexplorer/GNSS_IMU `drive_0708`,
Boulder CO, inside the service area, 9 min, RXM-RAWX converted to RINEX,
IGS BRDC navigation): kinematic float PPP runs on all 549 1-Hz epochs with
the 2850 HAS updates decoded from the same UBX file. The RTK track shipped
with the data set is relative to a base of unknown absolute coordinate
(constant offset E +6.0 / N +4.7 / U -23.0 m for both HAS and broadcast PPP),
so it cannot validate absolute accuracy; after removing that offset the HAS
kinematic track scatters 0.46 m (H RMS) / 1.25 m (U RMS) after 5 min against
0.31 m / 1.33 m broadcast only, as expected for an unconverged 9-minute run.

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

Lane result (2026-09-29, kinematic rows 2026-09-30, MSVC Release, elapsed time from the first epoch
01:59:12 GPST):

| Run | H / U at 10 min | 20 min | 30 min | 60 min | Converged H < 0.20 m | Converged \|U\| < 0.40 m |
|---|---|---|---|---|---:|---:|
| `has-idd`, static | 0.016 / -0.490 m | 0.058 / -0.421 m | 0.073 / -0.082 m | **0.095 / -0.135 m** | **4.2 min** | **21.0 min** |
| `legacy` conversion of the same stream, static | 0.374 / -0.537 m | 0.261 / -0.219 m | 0.267 / +0.282 m | 0.160 / +0.088 m | 57.5 min | 53.1 min |
| `has-idd`, kinematic | 0.035 / -0.195 m | 0.064 / -0.127 m | 0.059 / +0.015 m | **0.115 / +0.059 m** | 59.6 min | 6.0 min |
| `has-idd`, kinematic, before the residual screening | 0.035 / -0.195 m | 0.064 / -0.127 m | 0.136 / +0.058 m | 0.018 / -0.390 m | 45.0 min | never (-0.44 m at the last epoch) |
| `has-idd`, kinematic, before the one-update fix | 3.03 / -5.95 m | 2.47 / -5.13 m | 2.20 / -5.71 m | 1.87 / -5.61 m | never | never |

The lane gates the static run (H at 60 min <= 0.20 m, |U| at 60 min <= 0.40 m,
horizontal convergence <= 10 min, vertical convergence <= 30 min) and the
kinematic run (H <= 0.30 m and |U| <= 0.60 m at 30 and 60 min), and reports
the other rows. With the kinematic post-fit residual screening (see
[Broadcast kinematic PPP](broadcast_kinematic_ppp.md)) the kinematic run stays
within 0.21 m horizontally and 0.24 m vertically after the first 10 minutes
(RMS 0.086 m / 0.099 m, against 0.156 m / 0.206 m and peaks of 0.27 m /
0.48 m before); it crosses 0.20 m horizontally once more just before the end
of the hour, hence the late horizontal convergence time.

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
-0.143, 0.064 / +0.009 and 0.099 / +0.114 m at 10 / 20 / 30 / 60 min (with
`--antex igs20.atx`: 0.052 / -0.290, 0.072 / -0.251, 0.063 / -0.098 and
0.099 / +0.007 m).

H is the horizontal error and U the up error at the given time after the
first epoch; convergence is the first time after which H stays below 0.20 m
(resp. |U| below 0.40 m) until the end of the hour. No receiver ANTEX is
applied; with `--antex igs20.atx` the up error shifts by about -0.1 m.

## Known limitations

- **SIS: service area.** Both public SIS hours are from Japan, outside the HAS
  service area; the in-area X20 recording is 9 minutes of driving with an
  RTK track of unknown absolute datum. There is no in-area static SIS
  validation yet.
- **SIS: navigation data.** The RINEX 4 navigation file shipped with the
  2025-02-15 cssrlib-data hour is not usable by the libgnss++ RINEX reader
  (every satellite is rejected with ~100 km residuals, also without HAS), and
  the RINEX navigation files of both hours hold only the ephemerides received
  during the hour, so earlier IODrefs cannot be matched. The lane uses the IGS
  merged BRDC files instead.
- **SIS: phase biases** are decoded and dumped but not applied (float PPP).
- **SIS: SBF observations.** SBF input is used for the HAS pages only; the
  observations must be converted to RINEX separately.
- **IDD profile unchanged.** The `has-idd` profile keeps its stage-A
  behaviour: GPS L2 code biases are chosen by the coarse signal (L2C) even for
  C2W observations, and satellites without corrections keep their broadcast
  orbit. `--has-pages` does both the tracked-code selection and the exclusion.

- **Kinematic PPP (fixed).** Until the one-update fix, `--kinematic` settled
  about 2 m horizontal / 5.6 m vertical off on this hour with HAS, legacy SSR
  and broadcast-only input alike. The PPP filter re-applied the same epoch's
  measurement update up to eight times while the observation geometry stayed
  at the prior (SPP-seeded) position, so every extra pass pushed the position
  again by the innovation it had already absorbed and the troposphere and
  float ambiguities soaked up the difference. Kinematic PPP now commits one
  update per epoch, as RTKLIB / MADOCALIB do. Static and `--low-dynamics`
  runs keep the historical pass count (their prior is the previous solution,
  so the stale-geometry push is millimetre-level). Kinematic PPP also screens
  its post-fit residuals (w-test, 4 sigma); see
  [Broadcast kinematic PPP](broadcast_kinematic_ppp.md).
- **Galileo inter-system bias.** Galileo shares the GPS receiver clock unless
  `GNSS_PPP_ESTIMATE_ISB=gal` is set (as on the other non-MADOCA PPP paths).
  On this sample Galileo code residuals sit about 2 m below GPS; estimating
  the ISB did not improve the one-hour static result.
- **Satellite antenna frequency dependency.** Corrections are applied at the
  broadcast antenna phase centre without the per-signal satellite PCO / PCV
  differences cssrlib applies (centimetre level).
- **One-hour, one-station validation.** The public IDD sample is a single hour
  at one station; there is no multi-day validation yet.
