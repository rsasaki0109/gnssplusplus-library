# RTKLIB migration: `rnx2rtkp` to `gnss solve`

Use this route when an existing workflow is organized around
`rnx2rtkp`: rover/base/navigation RINEX in, a text position file out, and a
repeatable configuration in between. The first useful result is a non-empty
libgnss++ `.pos` file from the same observation window.

## First run

`apps/gnss.py solve` dispatches to the native `gnss_solve` executable. Build
that executable before asking the dispatcher for help. The existing
[Robotics quick start](../robotics_quickstart.md) shows the broader native
build and data flow; this is the smallest copy/paste prerequisite for this
route when run from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_solve --parallel 2
test -x build/apps/gnss_solve
```

Check the command surface and create an output directory:

```bash
python3 apps/gnss.py solve --help
mkdir -p output/use_cases
```

With a PPC-Dataset run (or another directory containing the three named
files), run a bounded 200-epoch sample:

```bash
RUN_DIR=data/PPC-Dataset/tokyo/run1
test -r "$RUN_DIR/rover.obs" -a -r "$RUN_DIR/base.obs" -a -r "$RUN_DIR/base.nav"

python3 apps/gnss.py solve \
  --config configs/examples/solve.example.toml \
  --rover "$RUN_DIR/rover.obs" \
  --base "$RUN_DIR/base.obs" \
  --nav "$RUN_DIR/base.nav" \
  --mode kinematic \
  --preset low-cost \
  --ratio 2.4 \
  --max-epochs 200 \
  --out output/use_cases/rtklib_migration.pos \
  --kml output/use_cases/rtklib_migration.kml
```

The `--config` file is an in-tree TOML example with a `[gnss_solve]` table;
explicit command-line values take precedence. Use `python3 apps/gnss.py
solve --help-advanced` to inspect additional controls before copying an
RTKLIB tuning value into a new profile.

## Mapping the `rnx2rtkp` mindset

| `rnx2rtkp` concept | libgnss++ boundary |
|---|---|
| Positional rover/base/nav RINEX inputs | `--rover <file>`, `--base <file>`, and `--nav <file>`; `--data-dir <dir>` is the shortcut for `rover.obs`, `base.obs`, and `navigation.nav` |
| `-k <rtklib.conf>` option file | `--config <path>` with flat TOML defaults under `[gnss_solve]`; command-line flags override the file |
| `-m`/position mode and receiver tuning | `--mode auto\|kinematic\|static\|moving-base`, plus a named `--preset survey\|low-cost\|moving-base\|odaiba` |
| `-o <output.pos>` | `--out <file>`; `--kml <file>` is an optional second artifact |
| RTKLIB status/Q/ns fields | libgnss++ status, satellite count, PDOP, ratio, baseline, and residual telemetry in its own `.pos` schema |

There is no promise that every RTKLIB configuration key has a one-to-one
`gnss solve` option. Start with the inputs, mode, preset, ratio, and epoch
window; record any remaining tuning as an explicit experiment.

## `.pos` compatibility boundary

Both tools commonly use the `.pos` suffix, but the file schemas differ. A
libgnss++ output starts with `% LibGNSS++ Position Solution` and writes
`GPS_Week GPS_TOW X Y Z Lat Lon Height Status NumSat PDOP Ratio Baseline`
followed by RTK update telemetry. It is directly consumable by repository
tools such as `gnss stats`, `gnss compare`, the web UI, and the Python artifact
helpers. Do not treat the suffix alone as a drop-in RTKLIB interchange
contract.

To bring an RTKLIB text solution into the libgnss++ artifact surface, use the
existing adapter:

```bash
python3 scripts/convert_rtklib_pos.py \
  output/rtklib_solution.pos \
  output/use_cases/rtklib_solution_libgnss.pos
```

The adapter reads RTKLIB date/time, latitude, longitude, height, quality, and
satellite columns and writes the libgnss++ GPS-week/TOW/ECEF/LLH/status form.
Inspect the converted header before using it in a comparison.

## Fair comparison checklist

Compare solver behavior only after fixing the experimental inputs:

- use the same rover, base, navigation, antenna coordinates, and signal
  selection;
- use the same start/end epoch or the same `--skip-epochs` and `--max-epochs`
  window;
- use the same base coordinates (RINEX header or an explicit `--base-ecef`);
- compare matched `(GPS week, TOW)` rows and keep missing rows visible;
- keep presets, ratio thresholds, interpolation, and output-status filters
  recorded beside each artifact;
- score against an independent reference only after both files are written.

The repository's [validation guide](../validation.md) defines sign-off
terminology and [benchmarks](../benchmarks.md) documents the RTKLIB `demo5`
comparison inputs. [Interfaces](../interfaces.md) lists the CLI, file, Python,
and web consumers of the output.

## First output and exit criteria

The bounded run is wired correctly when it returns zero, creates both files,
and the position file is non-empty with the libgnss++ header:

```bash
test -s output/use_cases/rtklib_migration.pos
test -s output/use_cases/rtklib_migration.kml
grep -q '^% LibGNSS++ Position Solution' output/use_cases/rtklib_migration.pos
python3 apps/gnss.py stats output/use_cases/rtklib_migration.pos
```

For a migration decision, also archive the exact command, input hashes, time
window, and a matched-row comparison. A produced file by itself is not an
accuracy or compatibility sign-off.

## Boundary and next step

This guide covers batch RTK post-processing. It does not establish one-to-one
RTKLIB option parity, a universal accuracy ranking, or suitability for every
receiver and baseline. For a measured migration, run the relevant [validation
gates](../validation.md), then use the [benchmark reproduction
notes](../benchmarks.md) with the same data and time window.

Next step: convert or normalize the reference solution if needed, compare
matched epochs, and promote a named TOML configuration only after the result
is reproducible.
