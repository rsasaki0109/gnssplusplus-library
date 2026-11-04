// Module worker: runs one CLI tool per worker so the UI stays responsive and
// Abort can simply terminate the worker.

import { runTool } from './engine.js';

const FLUSH_LINES = 200;
const FLUSH_MS = 100;

self.onmessage = async ({ data }) => {
  const { tool, args, inputs, wasmBase } = data;
  // callMain() blocks this worker, so log lines are batched by count/time
  // inside the print callbacks rather than with timers.
  let pending = [];
  let lastFlush = performance.now();
  const flush = () => {
    if (pending.length) self.postMessage({ type: 'log', lines: pending });
    pending = [];
    lastFlush = performance.now();
  };
  const sink = (stream) => (line) => {
    pending.push([stream, line]);
    if (pending.length >= FLUSH_LINES || performance.now() - lastFlush > FLUSH_MS) flush();
  };

  try {
    self.postMessage({ type: 'status', text: `loading ${tool}.wasm` });
    const { default: factory } = await import(new URL(`${tool}.js`, wasmBase).href);
    self.postMessage({ type: 'status', text: `running ${tool}` });
    const result = await runTool(factory, {
      args,
      inputs,
      onStdout: sink('out'),
      onStderr: sink('err'),
      mountBlobs: typeof FileReaderSync !== 'undefined',
    });
    flush();
    const transfer = [...new Set(result.outputs.map((f) => f.data.buffer))];
    self.postMessage({ type: 'done', ...result }, transfer);
  } catch (error) {
    flush();
    self.postMessage({ type: 'error', message: (error && (error.stack || error.message)) || String(error) });
  }
};
