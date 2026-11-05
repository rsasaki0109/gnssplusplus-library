// Main-thread side of a tool run: one module worker per job.

const WASM_BASE = new URL('../wasm/', import.meta.url).href;

/**
 * Start a job. Returns { done: Promise<result>, abort() }.
 * job: { tool, args, inputs: [{ name, data: Blob }] }
 * hooks: { onLog(stream, line), onStatus(text) }
 * result: { exitCode, outputs: [{ name, data: Uint8Array }], elapsedMs, aborted }
 */
export function startJob(job, hooks = {}) {
  const worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
  let settle;
  const done = new Promise((resolve, reject) => {
    settle = { resolve, reject };
    worker.onmessage = ({ data }) => {
      if (data.type === 'log') {
        for (const [stream, line] of data.lines) hooks.onLog && hooks.onLog(stream, line);
      } else if (data.type === 'status') {
        hooks.onStatus && hooks.onStatus(data.text);
      } else if (data.type === 'done') {
        worker.terminate();
        resolve({ ...data, aborted: false });
      } else if (data.type === 'error') {
        worker.terminate();
        reject(new Error(data.message));
      }
    };
    worker.onerror = (event) => {
      worker.terminate();
      reject(new Error(event.message || 'worker failed to start (module workers need a current browser)'));
    };
  });
  worker.postMessage({ tool: job.tool, args: job.args, inputs: job.inputs, wasmBase: WASM_BASE });
  return {
    done,
    abort() {
      worker.terminate();
      settle.resolve({ exitCode: null, outputs: [], elapsedMs: 0, aborted: true });
    },
  };
}

// Smallest module that uses a 128-bit SIMD instruction (i8x16.splat +
// i8x16.popcnt); the tools are built with -msimd128.
const SIMD_PROBE = new Uint8Array([
  0, 97, 115, 109, 1, 0, 0, 0, 1, 5, 1, 96, 0, 1, 123, 3, 2, 1, 0, 10, 10, 1, 8, 0, 65, 0, 253, 15, 253, 98, 11,
]);

/** True when this browser's WebAssembly engine supports 128-bit SIMD. */
export function wasmSimdSupported(probe = SIMD_PROBE) {
  try {
    return typeof WebAssembly === 'object' && WebAssembly.validate(probe);
  } catch {
    return false;
  }
}

/** True when the wasm build was deployed next to the app. */
export async function wasmAvailable(tool = 'gnss_solve') {
  try {
    const response = await fetch(new URL(`${tool}.wasm`, WASM_BASE), { method: 'HEAD' });
    return response.ok;
  } catch {
    return false;
  }
}
