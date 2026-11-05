# Browser GUI (WebAssembly)

**Open it: <https://rsasaki0109.github.io/gnssplusplus-library/app/>**

An RTKLIB-style GUI that runs entirely in the browser. The libgnss++
command-line tools are compiled to WebAssembly unchanged, so a run in the
browser executes the same `main()` as the native CLI, with the same
options and output formats. Files stay in the browser tab; nothing is
uploaded to a server.

| GUI tool | RTKLIB counterpart | Runs | Typical command it shows |
| --- | --- | --- | --- |
| Post-process | RTKPOST | `gnss_spp`, `gnss_solve`, `gnss_fuse`, `gnss_ppp` | `gnss solve --rover rover.obs --base base.obs --nav base.nav --out rover_rtk.pos --mode kinematic` |
| Convert | RTKCONV | `gnss_convert` | `gnss convert --input base.rtcm --format rtcm --gps-week 2324 --obs-out base.obs --nav-out base.nav` |
| Plot | RTKPLOT | (in the page) | ground track, E/N/U or error vs reference, satellites / ratio / PDOP, statistics |
| Observations | RTKPLOT (observation data, skyplot) | `gnss_visibility` | `gnss visibility --obs rover.obs --nav base.nav --csv rover_vis.csv --summary-json rover_vis_summary.json` |
| RINEX info | — | `gnss_rinex_info` | `gnss rinex-info --count-records rover.obs base.nav` |

Every run prints the equivalent `gnss ...` command, so a browser session can
be repeated natively, for example on a larger dataset or in a script.

## Quick tour

1. Press **Run demo pipeline** in the Files panel. It loads a 40 s excerpt of
   the PPC Tokyo run1 drive (RTCM3 rover and base streams plus the reference
   trajectory), converts both streams to RINEX, runs kinematic RTK and opens
   the result in Plot with the errors against the reference.
2. Or add your own files with **Add files** or drag and drop: RINEX OBS/NAV,
   SP3, CLK, RTCM3, u-blox UBX, `.pos` (libgnss++ or RTKLIB) and reference CSV
   files are recognised from their content.
3. Outputs (`.pos`, `.kml`, RINEX, summary JSON) appear in Files; download
   them or open `.pos` files in Plot. Solution 2 and a reference trajectory
   can be overlaid, as in RTKPLOT.
4. **Observations** shows the data before you solve: a skyplot, one tracking
   row per satellite coloured by SNR (or system) with loss-of-lock (LLI)
   slip ticks, SNR against elevation, the satellite count and a per-satellite
   table. Hovering a time highlights the satellites of that epoch on the
   skyplot. The receiver position for azimuth/elevation comes from the RINEX
   header, else the first SPP fix (rover files converted from RTCM have no
   header position), or an ECEF position you enter.

Post-process exposes the everyday options (positioning mode, elevation mask,
RTK preset, ionosphere option, ratio threshold, base position, KML) and an
**Extra arguments** field for everything else in `gnss solve --help-advanced`,
`gnss spp --help` or `gnss ppp --help`. A token in that field that equals a
file name in Files (an ANTEX or BLQ file, a TOML config) is mounted as an
input automatically.

**Kinematic RTK + IMU** runs `gnss fuse` with an IMU CSV (PPC format, or
rtklibexplorer with *IMU format*), the IMU-to-antenna lever arm in body FLU
and, by default, the tight-coupling preset (`--navi776-tc`). It writes the
fused trajectory and the RTK solution the fusion ran on (`*_rtk.pos`); open
both in Plot (solution 1 / solution 2) to compare. The sample `imu.csv`
covers the 40 s RTCM3 excerpt and fills in the PPC Tokyo lever arm
`0.31, 0, 0.55`. The benefit depends on the data and tuning: see
[RTK only vs RTK + GNSS/IMU on PPC](ppc_rtk_vs_gnss_imu_fusion.md) for the
full-run comparison and its caveats.

**Save options** writes the RTK, RTK + IMU or PPP settings as the flat TOML
that `gnss solve --config`, `gnss fuse --config` and `gnss ppp --config`
read (input and output files are left out, as in RTKPOST's options file); **Load options**, or *load* on a
`.toml` in Files, reads one back, including the repository examples in
`configs/examples/`. Options the form has no field for become extra
arguments. For example, a saved static RTK setup runs natively as:

```bash
gnss solve --config rover_rtk_options.toml --rover rover.obs --base base.obs --nav base.nav --out rtk.pos
```

`gnss spp` has no `--config`, so SPP settings are not saved.

## What runs where

```text
page (index.html, js/app.js)          one module worker per run (js/worker.js)
  Files (in-memory Blobs)   ───────▶   fresh Emscripten instance of <tool>.wasm
  forms -> argv (js/commands.js)       inputs mounted read-only at /in (WORKERFS)
  Plot (js/plot.js, canvas)  ◀───────   main(argc, argv) -> files in /out
```

- **Abort** terminates the worker; the page stays responsive during a run.
- Each run starts a new module instance, so no solver state carries over.
- Memory is limited to 4 GB per run (wasm32). Input files are read in place
  (not copied into the page's memory). Full-size data works: PPC Tokyo run1
  (90 MB rover + 36 MB base RINEX, 40 min at 5 Hz) solves in about 75 s in
  Chromium against 64 s for the native `gnss solve`, with the same FIX/FLOAT
  epochs and positions within 0.1 mm of the native run.
- Live inputs (NTRIP, serial ports, TCP) are not available in a browser; use
  `gnss live` natively for real-time work.
- Browser support: current Chrome / Edge, Firefox and Safari (module workers
  and WebAssembly exception handling: Chrome/Edge 95+, Firefox 114+,
  Safari 15.2+).

## Build and run locally

The GitHub Pages workflow (`.github/workflows/docs.yml`) builds the tools with
the official Emscripten image, runs a Node smoke test of the demo pipeline and
publishes the app under `/app/`. To do the same locally with an
[Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html):

```bash
bash wasm/build.sh                                # emcmake cmake + build
node wasm/test/smoke.mjs build-wasm/wasm/dist     # same pipeline as the demo
python3 wasm/assemble_site.py --dist build-wasm/wasm/dist --out build-wasm/site
python3 -m http.server -d build-wasm/site 8000    # open http://localhost:8000/
```

`wasm/README.md` lists the Emscripten settings and how a tool is added.
