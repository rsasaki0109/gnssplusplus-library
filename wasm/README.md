# WebAssembly build

Compiles selected batch CLI tools from `apps/native/` to WebAssembly for the
browser GUI in `web/` (user guide: `docs/web_app.md`).

```bash
bash wasm/build.sh                                   # emcmake + cmake --build
node wasm/test/smoke.mjs build-wasm/wasm/dist        # demo pipeline in Node
python3 wasm/assemble_site.py --dist build-wasm/wasm/dist --out build-wasm/site
python3 -m http.server -d build-wasm/site 8000       # http://localhost:8000/
```

Without a local Emscripten SDK, the image CI uses works too:

```bash
docker run --rm -v "$PWD:/src" -w /src emscripten/emsdk:4.0.10 bash -c \
    "apt-get update -qq && apt-get install -y -qq libeigen3-dev && bash wasm/build.sh"
```

`wasm/build.sh` runs `emcmake cmake -S . -B build-wasm
-DCMAKE_BUILD_TYPE=Release -DEigen3_DIR=/usr/share/eigen3/cmake` and builds
the `gnsspp_wasm` target. `-DEigen3_DIR` is needed because the Emscripten
toolchain only searches its own sysroot for packages; Eigen is header-only,
so the host copy is used as-is. The root `CMakeLists.txt` detects `EMSCRIPTEN`, defaults the Python
bindings off, compiles every C++ object with `-fwasm-exceptions` and, after
the library targets, adds only this directory (no examples, apps, ROS 2 or
tests).

## Outputs

`build-wasm/wasm/dist/<tool>.js` + `<tool>.wasm` for `gnss_convert`,
`gnss_rinex_info`, `gnss_visibility`, `gnss_spp`, `gnss_solve` and
`gnss_ppp`. Each `.js` is an
ES module whose default export is an Emscripten module factory:

```js
const { default: create } = await import('./gnss_solve.js');
const module = await create({ print, printErr });   // main() not run yet
module.FS.writeFile('/in/rover.obs', bytes);          // ... inputs
const exitCode = module.callMain(['--rover', '/in/rover.obs', /* ... */]);
const pos = module.FS.readFile('/out/rover_rtk.pos');
```

`web/js/engine.js` implements exactly this (plus read-only WORKERFS mounts for
browser `File`s) and is shared by the GUI worker and the smoke test. A module
instance runs `main()` once; create a new one per run.

Link settings (`wasm/CMakeLists.txt`): `MODULARIZE` + `EXPORT_ES6`,
`INVOKE_RUN=0`, `EXIT_RUNTIME=1`, `ENVIRONMENT=web,worker,node`,
`ALLOW_MEMORY_GROWTH` up to 4 GB, an 8 MB stack (the native default; the
solvers keep Eigen fixed-size matrices on the stack), `FORCE_FILESYSTEM` and
`-lworkerfs.js`. No pthreads, so the page needs no cross-origin isolation
headers and works on plain static hosting.

## Adding a tool

1. Add a `gnsspp_add_wasm_tool(...)` line in `wasm/CMakeLists.txt` with the
   same sources and libraries as in `apps/CMakeLists.txt`.
2. Add it to `TOOLS` in `wasm/assemble_site.py`.
3. Map GUI settings to its argv in `web/js/commands.js` (unit-tested by
   `web/test/web_app.test.mjs`) and, if it is part of the demo flow, extend
   `wasm/test/smoke.mjs`.

Only file-based batch tools make sense here: sockets, serial ports and
threads are unavailable in the browser build.
