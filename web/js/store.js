// Optional persistence of the Files workspace in this browser (IndexedDB),
// so a reload or a new visit starts with the same files. Opt-in from the
// Files panel; nothing leaves the browser.

const DB_NAME = 'gnsspp-workspace';
const STORE = 'files';

function request(req) {
  return new Promise((resolve, reject) => {
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error || new Error('IndexedDB request failed'));
  });
}

export class FileStore {
  constructor(indexedDb = globalThis.indexedDB) {
    this.idb = indexedDb;
    this.db = null;
  }

  /** True when IndexedDB can be opened here (false in some private modes). */
  async available() {
    try {
      await this.open();
      return true;
    } catch {
      return false;
    }
  }

  async open() {
    if (this.db) return this.db;
    if (!this.idb) throw new Error('IndexedDB is not available');
    const req = this.idb.open(DB_NAME, 1);
    req.onupgradeneeded = () => {
      if (!req.result.objectStoreNames.contains(STORE)) req.result.createObjectStore(STORE, { keyPath: 'name' });
    };
    this.db = await request(req);
    return this.db;
  }

  /** Run fn(objectStore) in a transaction; resolves when it commits. */
  async tx(mode, fn) {
    const db = await this.open();
    const tx = db.transaction(STORE, mode);
    fn(tx.objectStore(STORE));
    await new Promise((resolve, reject) => {
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error || new Error('IndexedDB transaction failed'));
      tx.onabort = () => reject(tx.error || new Error('IndexedDB transaction aborted (storage full?)'));
    });
  }

  /** record: { name, blob, origin, added, meta? } */
  put(record) {
    return this.tx('readwrite', (store) => store.put(record));
  }

  delete(name) {
    return this.tx('readwrite', (store) => store.delete(name));
  }

  clear() {
    return this.tx('readwrite', (store) => store.clear());
  }

  /** All stored records, oldest first. */
  async all() {
    const db = await this.open();
    const records = await request(db.transaction(STORE, 'readonly').objectStore(STORE).getAll());
    return records.sort((a, b) => a.added - b.added);
  }
}

/** "12 files, 130.2 MB" style summary of stored records. */
export function describeStored(records) {
  const bytes = records.reduce((sum, r) => sum + (r.blob ? r.blob.size : 0), 0);
  const size = bytes >= 1e9 ? `${(bytes / 1e9).toFixed(2)} GB` : bytes >= 1e6 ? `${(bytes / 1e6).toFixed(1)} MB` : `${Math.round(bytes / 1e3)} kB`;
  return `${records.length} file${records.length === 1 ? '' : 's'}, ${size}`;
}
