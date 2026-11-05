// Runs one native CLI tool, compiled to WebAssembly (wasm/CMakeLists.txt), in
// a fresh Emscripten module instance. Shared by the browser worker
// (worker.js) and the Node smoke test (wasm/test/smoke.mjs).

import { IN_DIR, OUT_DIR } from './commands.js';

// Emscripten's WORKERFS answers every read() with its own blob.slice() and
// FileReaderSync call. The C++ streams read a few KB at a time, so a 90 MB
// RINEX file costs tens of thousands of synchronous reads (35 s vs 6 s from
// MEMFS in Chromium). Serve small reads from a per-file read-ahead chunk
// instead; large reads, and anything unexpected, keep the original path.
export const READ_AHEAD_BYTES = 4 * 1024 * 1024;

export function addWorkerfsReadAhead(workerfs, chunkBytes = READ_AHEAD_BYTES) {
  const ops = workerfs && workerfs.stream_ops;
  if (!ops || typeof ops.read !== 'function' || ops.readAheadBytes) return false;
  const direct = ops.read;
  ops.read = function readAhead(stream, buffer, offset, length, position) {
    const node = stream.node;
    const blob = node && node.contents;
    if (!workerfs.reader || typeof Blob === 'undefined' || !(blob instanceof Blob)
        || !Number.isFinite(position) || length >= chunkBytes) {
      return direct.call(this, stream, buffer, offset, length, position);
    }
    const size = blob.size;
    if (position >= size) return 0;
    const end = Math.min(size, position + length);
    let cache = node.readAheadCache;
    if (!cache || position < cache.start || end > cache.start + cache.bytes.length) {
      const stop = Math.min(size, position + chunkBytes);
      cache = { start: position, bytes: new Uint8Array(workerfs.reader.readAsArrayBuffer(blob.slice(position, stop))) };
      node.readAheadCache = cache;
    }
    const from = position - cache.start;
    buffer.set(cache.bytes.subarray(from, from + (end - position)), offset);
    return end - position;
  };
  ops.readAheadBytes = chunkBytes;
  return true;
}

function listFiles(FS, dir, prefix = '') {
  const files = [];
  for (const entry of FS.readdir(dir)) {
    if (entry === '.' || entry === '..') continue;
    const path = `${dir}/${entry}`;
    const mode = FS.stat(path).mode;
    if (FS.isDir(mode)) files.push(...listFiles(FS, path, `${prefix}${entry}/`));
    else if (FS.isFile(mode)) files.push({ name: `${prefix}${entry}`, data: FS.readFile(path) });
  }
  return files;
}

/**
 * @param factory   default export of <tool>.js (Emscripten MODULARIZE factory)
 * @param options.args     argv after the program name (paths under /in and /out)
 * @param options.inputs   [{ name, data: Blob | Uint8Array }] mounted at /in/<name>
 * @param options.onStdout / onStderr  called once per output line
 * @param options.mountBlobs  mount Blob inputs read-only with WORKERFS instead
 *                            of copying them (needs FileReaderSync: workers only)
 * @param options.moduleOptions  extra Emscripten module settings
 * @returns { exitCode, outputs: [{ name, data: Uint8Array }], elapsedMs }
 */
export async function runTool(factory, options) {
  const { args, inputs = [], onStdout, onStderr, mountBlobs = false, moduleOptions = {} } = options;
  const module = await factory({
    print: (line) => onStdout && onStdout(line),
    printErr: (line) => onStderr && onStderr(line),
    ...moduleOptions,
  });
  const { FS } = module;
  FS.mkdir(IN_DIR);
  FS.mkdir(OUT_DIR);

  const workerfs = FS.filesystems && FS.filesystems.WORKERFS;
  const blobs = inputs.filter((f) => typeof Blob !== 'undefined' && f.data instanceof Blob);
  let mounted = false;
  if (mountBlobs && workerfs && blobs.length === inputs.length && blobs.length > 0) {
    try {
      addWorkerfsReadAhead(workerfs);
      FS.mount(workerfs, { blobs: inputs.map((f) => ({ name: f.name, data: f.data })) }, IN_DIR);
      mounted = true;
    } catch {
      mounted = false; // fall back to copying into MEMFS below
    }
  }
  if (!mounted) {
    for (const file of inputs) {
      const bytes = file.data instanceof Uint8Array
        ? file.data
        : new Uint8Array(await file.data.arrayBuffer());
      FS.writeFile(`${IN_DIR}/${file.name}`, bytes);
    }
  }

  // Relative default paths a tool may still use land next to the outputs.
  FS.chdir(OUT_DIR);
  const started = (typeof performance !== 'undefined' ? performance : Date).now();
  let exitCode;
  try {
    exitCode = module.callMain(args);
  } catch (error) {
    // exit() / an uncaught C++ exception surfaces as ExitStatus or an abort.
    if (error && error.name === 'ExitStatus') exitCode = error.status;
    else throw error;
  }
  const elapsedMs = (typeof performance !== 'undefined' ? performance : Date).now() - started;
  return { exitCode, outputs: listFiles(FS, OUT_DIR), elapsedMs };
}
