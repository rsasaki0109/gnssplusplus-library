# Browser GUI (WebAssembly)

**Open it: <https://rsasaki0109.github.io/gnssplusplus-library/app/>**

An RTKLIB-style GUI that runs entirely in the browser. The libgnss++
command-line tools are compiled to WebAssembly unchanged, so a run in the
browser executes the same `main()` as the native CLI, with the same
options and output formats. Files stay in the browser tab; nothing is
uploaded to a server.

| GUI tool | RTKLIB counterpart | Runs | Typical command it shows |
| --- | --- | --- | --- |
| Post-process | RTKPOST | `gnss_spp`, `gnss_solve`, `gnss_ppp` | `gnss solve --rover rover.obs --base base.obs --nav base.nav --out rover_rtk.pos --mode kinematic` |
| Convert | RTKCONV | `gnss_convert` | `gnss convert --input base.rtcm --format rtcm --gps-week 2324 --obs-out base.obs --nav-out base.nav` |
| Plot | RTKPLOT | (in the page) | ground track, E/N/U or error vs reference, satellites / ratio / PDOP, statistics |
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

Post-process exposes the everyday options (positioning mode, elevation mask,
RTK preset, ionosphere option, ratio threshold, base position, KML) and an
**Extra arguments** field for everything else in `gnss solve --help-advanced`,
`gnss spp --help` or `gnss ppp --help`. A token in that field that equals a
file name in Files (an ANTEX or BLQ file, a TOML config) is mounted as an
input automatically.

## What runs where

```text
page (index.html, js/app.js)          one module worker per run (js/worker.js)
  Files (in-memory Blobs)   ───────▶   fresh Emscripten instance of <tool>.wasm
  forms -> argv (js/commands.js)       inputs mounted read-only at /in (WORKERFS)
  Plot (js/plot.js, canvas)  ◀───────   main(argc, argv) -> files in /out
```

- **Abort** terminates the worker; the page stays responsive during a run.
- Each run starts a new module instance, so no solver state carries over.
- Memory is limited to 4 GB per run (wasm32). Multi-hour, high-rate RINEX
  files work, but a native build is faster (typically 1.5-3x).
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
