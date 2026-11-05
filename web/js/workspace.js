// In-browser "working directory": uploaded, sample and generated files.
// Everything stays in this tab's memory (and, when the user opts in, in this
// browser's IndexedDB via store.js); nothing is uploaded anywhere.

import { sniffKind } from './formats.js';

export class Workspace extends EventTarget {
  constructor() {
    super();
    this.files = new Map();
  }

  /** Add or replace a file. data: Blob | Uint8Array | string. */
  async add(name, data, origin = 'upload', { added = Date.now() } = {}) {
    const blob = data instanceof Blob ? data : new Blob([data]);
    const head = new Uint8Array(await blob.slice(0, 4096).arrayBuffer());
    const entry = { name, blob, size: blob.size, kind: sniffKind(name, head), origin, added };
    this.files.set(name, entry);
    this.dispatchEvent(new CustomEvent('change', { detail: { added: name } }));
    return entry;
  }

  remove(name) {
    if (this.files.delete(name)) this.dispatchEvent(new CustomEvent('change', { detail: { removed: name } }));
  }

  clear() {
    this.files.clear();
    this.dispatchEvent(new CustomEvent('change', { detail: {} }));
  }

  get(name) {
    return this.files.get(name);
  }

  names() {
    return [...this.files.keys()];
  }

  /** Files of the given kind(s), newest first. */
  list(...kinds) {
    return [...this.files.values()]
      .filter((f) => kinds.length === 0 || kinds.includes(f.kind))
      .sort((a, b) => b.added - a.added);
  }

  async text(name) {
    const entry = this.files.get(name);
    if (!entry) throw new Error(`no such file: ${name}`);
    return entry.blob.text();
  }
}
