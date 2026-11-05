// libgnss++ Web: RTKLIB-style GUI over the libgnss++ CLI tools compiled to
// WebAssembly. Each run executes the unchanged native main() in a worker.

import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, buildVisibilityCommand, defaultConvertNames, defaultPosName, DISPATCH_NAMES, POST_MODES,
} from './commands.js';
import { parseSolution, parseVisibilityCsv, Q_LABELS, rinexObsTimeSpan, SYSTEMS } from './formats.js';
import {
  chooseOrigin, errorClasses, errorClassLabels, errorSummary, isFixQuality, nearestIndex, referenceErrors, solutionSummary, toEnu,
} from './analysis.js';
import { formatGpst, splitGpsSeconds, SECONDS_PER_WEEK } from './geo.js';
import { exportOptionsToml, importOptionsToml } from './options.js';
import { canvasesToPng, cssVar, PlotPanel, SkyPlot } from './plot.js';
import { startJob, wasmAvailable } from './runner.js';
import { describeStored, FileStore } from './store.js';
import { enuPlane, MAX_SPAN_M, TILE_SOURCES, TileLayer } from './tiles.js';
import { Workspace } from './workspace.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const workspace = new Workspace();
const fileMeta = new Map(); // name -> { gpsWeek } hints for sample files

const SAMPLES = [
  { path: 'samples/rover.rtcm', meta: { gpsWeek: 2324 } },
  { path: 'samples/base.rtcm', meta: { gpsWeek: 2324 } },
  { path: 'samples/reference.csv' },
  // PPC Tokyo: antenna 0.31 m forward, 0.55 m above the IMU (body FLU).
  { path: 'samples/imu.csv', meta: { leverArm: ['0.31', '0', '0.55'] } },
  { path: 'samples/synthetic_ppp.obs' },
  { path: 'samples/synthetic_ppp.sp3' },
  { path: 'samples/synthetic_ppp.clk' },
];

function setStatus(text) {
  $('#status').textContent = text;
}

function formatSize(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

function download(name, blob) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = name.split('/').pop();
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}

const store = {
  load(key) {
    try { return JSON.parse(localStorage.getItem(`gnsspp-web:${key}`) || 'null'); } catch { return null; }
  },
  save(key, value) {
    try { localStorage.setItem(`gnsspp-web:${key}`, JSON.stringify(value)); } catch { /* private mode */ }
  },
};

// ---------------------------------------------------------------- tabs
function showTool(tool) {
  for (const button of $$('.tools button')) button.setAttribute('aria-selected', String(button.dataset.tool === tool));
  for (const section of $$('section.tool')) section.hidden = section.dataset.tool !== tool;
  if (location.hash !== `#${tool}`) history.replaceState(null, '', `#${tool}`);
  if (tool === 'plot') plotView.render();
  if (tool === 'obs') obsView.render();
}

// ---------------------------------------------------------------- files
function renderFiles(fresh) {
  const list = $('#file-list');
  list.replaceChildren();
  const files = workspace.list();
  if (files.length === 0) {
    const li = document.createElement('li');
    li.className = 'empty';
    li.textContent = 'No files yet';
    list.append(li);
  }
  for (const f of files) {
    const li = document.createElement('li');
    if (f.name === fresh) li.classList.add('fresh');
    const kind = document.createElement('span');
    kind.className = 'kind';
    kind.textContent = f.kind;
    const name = document.createElement('span');
    name.className = 'name';
    name.title = `${f.name} (${f.origin})`;
    name.innerHTML = '';
    name.append(document.createTextNode(f.name), document.createElement('br'));
    const size = document.createElement('span');
    size.className = 'size';
    size.textContent = `${formatSize(f.size)} · ${f.origin}`;
    name.append(size);
    const acts = document.createElement('span');
    acts.className = 'acts';
    if (f.kind === 'pos' || f.kind === 'csv') {
      const plot = document.createElement('button');
      plot.textContent = 'plot';
      plot.title = 'Open in Plot';
      plot.onclick = () => plotView.open(f.name);
      acts.append(plot);
    }
    if (f.kind === 'toml') {
      const load = document.createElement('button');
      load.textContent = 'load';
      load.title = 'Load into Post-process options';
      load.onclick = () => f.blob.text().then((text) => applyOptionsToml(f.name, text)).catch((e) => setStatus(`Options: ${e.message}`));
      acts.append(load);
    }
    if (f.kind === 'vis') {
      const view = document.createElement('button');
      view.textContent = 'view';
      view.title = 'Open in Observations';
      view.onclick = () => obsView.open(f.name);
      acts.append(view);
    }
    const save = document.createElement('button');
    save.textContent = '⤓';
    save.title = 'Download';
    save.setAttribute('aria-label', `Download ${f.name}`);
    save.onclick = () => download(f.name, f.blob);
    const remove = document.createElement('button');
    remove.textContent = '✕';
    remove.title = 'Remove';
    remove.setAttribute('aria-label', `Remove ${f.name}`);
    remove.onclick = () => workspace.remove(f.name);
    acts.append(save, remove);
    li.append(kind, name, acts);
    list.append(li);
  }
}

async function addFiles(fileList) {
  for (const file of fileList) await workspace.add(file.name, file, 'upload');
  setStatus(`Added ${fileList.length} file(s)`);
}

// ---- optional persistence of Files in this browser (IndexedDB)
const KEEP_SETTING = 'gnsspp.keepFiles';
const fileStore = new FileStore();
const keepBox = $('#keep-files');
let keepFiles = false;
let restoringFiles = false;

function storedRecord(name) {
  const entry = workspace.get(name);
  return entry && { name, blob: entry.blob, origin: entry.origin, added: entry.added, meta: fileMeta.get(name) || null };
}

async function updateKeepNote() {
  const note = $('#keep-files-note');
  if (!keepFiles) {
    note.textContent = '(stored in this browser only)';
    return;
  }
  try {
    note.textContent = `(kept: ${describeStored(await fileStore.all())} in this browser)`;
  } catch (error) {
    note.textContent = `(could not read the stored files: ${error.message})`;
  }
}

/** Mirror one workspace change into the store (when keeping files). */
function persistChange(detail) {
  if (!keepFiles || restoringFiles) return;
  let job;
  if (detail && detail.added) {
    const record = storedRecord(detail.added);
    if (!record) return;
    job = fileStore.put(record).catch((error) => {
      setStatus(`Could not keep ${record.name} in this browser (${error.message}); it stays for this session only`);
    });
  } else if (detail && detail.removed) {
    job = fileStore.delete(detail.removed);
  } else {
    job = fileStore.clear();
  }
  job.catch((error) => setStatus(`Browser storage: ${error.message}`)).finally(updateKeepNote);
}

async function setKeepFiles(on) {
  keepFiles = on;
  writeSetting(KEEP_SETTING, on ? '1' : '0');
  if (on) {
    if (navigator.storage && navigator.storage.persist) navigator.storage.persist().catch(() => {});
    let kept = 0;
    for (const name of workspace.names()) {
      try {
        await fileStore.put(storedRecord(name));
        kept += 1;
      } catch (error) {
        setStatus(`Could not keep ${name} in this browser (${error.message})`);
      }
    }
    if (kept) setStatus(`Keeping ${kept} file(s) in this browser`);
  } else {
    await fileStore.clear();
    setStatus('Files are no longer kept after reload; the stored copies were deleted');
  }
  await updateKeepNote();
}

async function restoreKeptFiles() {
  if (!(await fileStore.available())) {
    keepBox.disabled = true;
    keepBox.closest('label').title = 'Browser storage (IndexedDB) is not available here, e.g. in a private window';
    return;
  }
  keepFiles = readSetting(KEEP_SETTING) === '1';
  keepBox.checked = keepFiles;
  keepBox.addEventListener('change', () => setKeepFiles(keepBox.checked).catch((error) => setStatus(`Browser storage: ${error.message}`)));
  if (!keepFiles) return;
  const records = await fileStore.all();
  restoringFiles = true;
  try {
    for (const record of records) {
      if (record.meta) fileMeta.set(record.name, record.meta);
      await workspace.add(record.name, record.blob, record.origin, { added: record.added });
    }
  } finally {
    restoringFiles = false;
  }
  if (records.length) setStatus(`Restored ${describeStored(records)} kept in this browser`);
  await updateKeepNote();
}

function preferredOption(names, prefer) {
  if (!prefer) return names[0];
  return names.find((n) => n.toLowerCase().includes(prefer)) || names[0];
}

/** Refill every file <select> and checklist from the workspace, keeping choices. */
function refreshFileSelectors() {
  const group = postGroup();
  for (const select of $$('select[data-kinds]')) {
    const kinds = select.dataset.kinds.split(/\s+/);
    const names = workspace.list(...kinds).map((f) => f.name).sort();
    const optional = select.hasAttribute('data-optional')
      || (select.dataset.optionalModes || '').split(/\s+/).includes(group);
    const previous = select.value;
    select.replaceChildren();
    if (optional || names.length === 0) {
      const none = document.createElement('option');
      none.value = '';
      none.textContent = names.length === 0 && !optional ? '(no matching files)' : '(none)';
      select.append(none);
    }
    for (const name of names) {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      select.append(option);
    }
    if (names.includes(previous) || (previous === '' && optional)) select.value = previous;
    else if (!optional && names.length) {
      let pick = preferredOption(names, select.dataset.prefer);
      // Base defaults to a different file than the rover.
      if (select.name === 'base') {
        const rover = select.form.elements.rover.value;
        if (pick === rover) pick = names.find((n) => n !== rover) || pick;
      }
      select.value = pick;
    }
  }
  const checklist = $('#info-files');
  const checked = new Set($$('input:checked', checklist).map((i) => i.value));
  checklist.replaceChildren();
  const infoNames = workspace.list(...checklist.dataset.kinds.split(/\s+/)).map((f) => f.name).sort();
  if (infoNames.length === 0) checklist.textContent = 'No RINEX files in Files yet.';
  for (const name of infoNames) {
    const label = document.createElement('label');
    label.className = 'check';
    const box = document.createElement('input');
    box.type = 'checkbox';
    box.value = name;
    box.checked = checked.size === 0 || checked.has(name);
    label.append(box, document.createTextNode(` ${name}`));
    checklist.append(label);
  }
}

// ---------------------------------------------------------------- run view
class RunView {
  constructor(root, idleText) {
    this.root = root;
    root.innerHTML = `
      <div class="cmd"><pre data-cmd></pre><button type="button" class="btn small" data-copy>Copy</button></div>
      <div class="progress" data-progress hidden><div></div></div>
      <div class="meta" data-meta></div>
      <pre class="log" data-log></pre>
      <ul class="outputs" data-outputs></ul>`;
    this.cmd = $('[data-cmd]', root);
    this.bar = $('[data-progress]', root);
    this.meta = $('[data-meta]', root);
    this.logEl = $('[data-log]', root);
    this.outputs = $('[data-outputs]', root);
    this.lines = [];
    this.frame = 0;
    this.meta.textContent = idleText;
    $('[data-copy]', root).onclick = () => navigator.clipboard && navigator.clipboard.writeText(this.cmd.textContent);
  }

  preview(commandLine) {
    if (!this.running) this.cmd.textContent = commandLine;
  }

  start(commandLine) {
    this.running = true;
    this.started = performance.now();
    this.cmd.textContent = commandLine;
    this.lines = [];
    this.outputs.replaceChildren();
    this.bar.hidden = false;
    this.progress(null);
    this.meta.textContent = 'Running…';
    this.renderLog();
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      this.meta.textContent = `Running… ${((performance.now() - this.started) / 1000).toFixed(1)} s`;
    }, 250);
  }

  log(stream, line) {
    this.lines.push([stream, line]);
    if (!this.frame) this.frame = requestAnimationFrame(() => { this.frame = 0; this.renderLog(); });
  }

  renderLog() {
    const tail = this.lines.slice(-2000);
    const esc = (s) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    const skipped = this.lines.length - tail.length;
    this.logEl.innerHTML = (skipped ? `<span class="err">… ${skipped} earlier lines (download the full log)</span>\n` : '')
      + tail.map(([s, l]) => (s === 'err' ? `<span class="err">${esc(l)}</span>` : esc(l))).join('\n');
    this.logEl.scrollTop = this.logEl.scrollHeight;
  }

  progress(fraction) {
    this.bar.classList.toggle('busy', fraction === null);
    $('div', this.bar).style.width = fraction === null ? '' : `${Math.round(Math.min(1, Math.max(0, fraction)) * 100)}%`;
  }

  finish(result) {
    this.running = false;
    clearInterval(this.timer);
    this.renderLog();
    this.bar.hidden = true;
    this.meta.replaceChildren();
    const status = document.createElement('span');
    if (result.aborted) {
      status.className = 'bad';
      status.textContent = 'Aborted';
    } else {
      status.className = result.exitCode === 0 ? 'good' : 'bad';
      status.textContent = result.exitCode === 0 ? 'Done' : `Exit code ${result.exitCode}`;
    }
    const time = document.createElement('span');
    time.textContent = result.aborted ? '' : `${(result.elapsedMs / 1000).toFixed(2)} s`;
    const logLink = document.createElement('button');
    logLink.className = 'btn small';
    logLink.textContent = 'Download log';
    logLink.onclick = () => download('run.log', new Blob([this.lines.map((l) => l[1]).join('\n') + '\n']));
    this.meta.append(status, time, logLink);
    for (const out of result.outputs || []) {
      const li = document.createElement('li');
      const name = document.createElement('span');
      name.className = 'name';
      name.textContent = out.name;
      const size = document.createElement('span');
      size.className = 'size';
      size.textContent = formatSize(out.data.byteLength);
      const save = document.createElement('button');
      save.textContent = 'download';
      save.onclick = () => download(out.name, workspace.get(out.name).blob);
      li.append(name, size, save);
      if (/\.pos$/i.test(out.name)) {
        const plot = document.createElement('button');
        plot.textContent = 'plot';
        plot.onclick = () => plotView.open(out.name);
        li.append(plot);
      }
      this.outputs.append(li);
    }
  }

  fail(error) {
    this.running = false;
    clearInterval(this.timer);
    this.bar.hidden = true;
    this.log('err', String(error.message || error));
    this.renderLog();
    this.meta.innerHTML = '<span class="bad">Failed</span>';
  }
}

let engineReady = false;
let activeJob = null;

/** Submit handler helper: show errors that runCommand() did not already show. */
function reportTo(view) {
  return (error) => {
    if (!error.shown) view.fail(error);
    setStatus(`Error: ${error.message}`);
  };
}

/** Run a built command against workspace files; outputs are added to Files. */
async function runCommand(cmd, view, form, hooks = {}) {
  if (!engineReady) throw new Error('the WebAssembly tools are not available on this page');
  if (activeJob) throw new Error('another run is in progress');
  const inputs = cmd.inputs.map((name) => {
    const entry = workspace.get(name);
    if (!entry) throw new Error(`input file not in Files: ${name}`);
    return { name, data: entry.blob };
  });
  const submit = $('button[type="submit"]', form);
  const abort = $('[data-abort]', form);
  view.start(cmd.commandLine);
  setStatus(`Running gnss ${cmd.commandLine.split(' ')[1]}…`);
  activeJob = startJob({ tool: cmd.tool, args: cmd.args, inputs }, {
    onLog: (stream, line) => { view.log(stream, line); if (hooks.onLine) hooks.onLine(line); },
    onStatus: (text) => setStatus(text),
  });
  submit.disabled = true;
  abort.disabled = false;
  abort.onclick = () => activeJob && activeJob.abort();
  try {
    const result = await activeJob.done;
    for (const out of result.outputs) await workspace.add(out.name, out.data, 'output');
    view.finish(result);
    setStatus(result.aborted ? 'Aborted' : `Finished with exit code ${result.exitCode}`);
    return result;
  } catch (error) {
    view.fail(error);
    setStatus('Run failed');
    error.shown = true;
    throw error;
  } finally {
    activeJob = null;
    submit.disabled = !engineReady;
    abort.disabled = true;
  }
}

function formValues(form) {
  const values = {};
  for (const el of form.elements) {
    if (!el.name) continue;
    values[el.name] = el.type === 'checkbox' ? el.checked : el.value;
  }
  return values;
}

function persistForm(form, key) {
  const saved = store.load(key);
  if (saved) {
    for (const el of form.elements) {
      if (!el.name || el.tagName === 'SELECT' && el.dataset.kinds) continue;
      if (!(el.name in saved)) continue;
      if (el.type === 'checkbox') el.checked = Boolean(saved[el.name]);
      else el.value = saved[el.name];
    }
  }
  form.addEventListener('change', () => {
    const values = formValues(form);
    for (const el of form.elements) if (el.dataset && el.dataset.kinds) delete values[el.name];
    store.save(key, values);
  });
}

// ---------------------------------------------------------------- post-processing
const postForm = $('#post-form');
const postView = new RunView($('#tool-post [data-run]'), 'Choose input files and press Execute.');

function postGroup() {
  const mode = postForm.elements.mode.value;
  if (mode === 'single') return 'spp';
  if (mode === 'rtk-imu') return 'imu';
  if (mode === 'fgo') return 'fgo';
  return mode.startsWith('ppp') ? 'ppp' : 'rtk';
}

function updatePostVisibility() {
  const group = postGroup();
  for (const el of $$('[data-show]', postForm)) el.hidden = !el.dataset.show.split(/\s+/).includes(group);
  const ecef = postForm.elements.basePos.value === 'ecef';
  for (const el of $$('[data-show-base]', postForm)) el.hidden = !ecef || !['rtk', 'imu'].includes(group);
}

function postOptions() {
  const v = formValues(postForm);
  const group = postGroup();
  const withBase = group === 'rtk' || group === 'imu' || group === 'fgo';
  return {
    mode: v.mode,
    rover: v.rover,
    base: withBase ? v.base : '',
    nav: v.nav,
    sp3: withBase ? '' : v.sp3,
    clk: withBase ? '' : v.clk,
    imu: group === 'imu' ? v.imu : '',
    leverArm: group === 'imu' ? [v.leverX, v.leverY, v.leverZ] : null,
    tightCoupling: v.tightCoupling,
    imuFormat: v.imuFormat,
    out: v.out.trim(),
    elevationMask: v.elevationMask,
    maxEpochs: v.maxEpochs,
    preset: v.preset,
    iono: v.iono,
    ratio: v.ratio,
    baseEcef: (group === 'rtk' || group === 'imu') && v.basePos === 'ecef' ? [v.baseX, v.baseY, v.baseZ] : null,
    fgoPreset: v.fgoPreset,
    kml: v.kml,
    verbose: v.verbose,
    ionoFree: v.ionoFree,
    enableAr: v.enableAr,
    extra: v.extra,
    workspaceNames: workspace.names(),
  };
}

function updatePostPreview() {
  postForm.elements.out.placeholder = defaultPosName(postForm.elements.rover.value, postForm.elements.mode.value);
  try {
    postView.preview(buildPostCommand(postOptions()).commandLine);
  } catch (error) {
    postView.preview(`# ${error.message}`);
  }
}

async function roverTimeSpan(name) {
  const entry = workspace.get(name);
  if (!entry) return null;
  const head = await entry.blob.slice(0, 65536).text();
  const span = rinexObsTimeSpan(head);
  return span.first !== null && span.last !== null && span.last > span.first ? span : null;
}

async function runPost() {
  const cmd = buildPostCommand(postOptions());
  const hooks = {};
  if (cmd.tool === 'gnss_solve' && cmd.args.includes('--verbose')) {
    const span = await roverTimeSpan(postForm.elements.rover.value);
    if (span) {
      const tow0 = splitGpsSeconds(span.first).tow;
      hooks.onLine = (line) => {
        const m = /^epoch \d+ tow=([\d.]+)/.exec(line);
        if (!m) return;
        let dt = Number(m[1]) - tow0;
        if (dt < -SECONDS_PER_WEEK / 2) dt += SECONDS_PER_WEEK;
        postView.progress(dt / (span.last - span.first));
      };
    }
  }
  return runCommand(cmd, postView, postForm, hooks);
}

// ---- options file (gnss solve / gnss ppp --config TOML)
function applyOptionsToml(name, text) {
  const imported = importOptionsToml(text);
  const el = postForm.elements;
  el.mode.value = imported.mode;
  updatePostVisibility();
  refreshFileSelectors();
  for (const [field, value] of Object.entries(imported.fields)) {
    if (!el[field]) continue;
    if (el[field].type === 'checkbox') el[field].checked = Boolean(value);
    else el[field].value = value;
  }
  for (const field of ['preset', 'ratio', 'elevationMask', 'maxEpochs']) {
    if (!(field in imported.fields) && el[field]) el[field].value = '';
  }
  if (!('iono' in imported.fields)) el.iono.value = 'auto';
  const matched = [];
  for (const [role, base] of Object.entries(imported.files)) {
    if (role === 'out') el.out.value = base;
    else if (['rover', 'base', 'nav', 'sp3', 'clk'].includes(role) && workspace.get(base)) {
      el[role].value = base;
      matched.push(base);
    }
  }
  el.extra.value = imported.extra;
  updatePostVisibility();
  updatePostPreview();
  postForm.dispatchEvent(new Event('change'));
  const notes = [...imported.notes];
  if (matched.length) notes.push(`inputs from Files: ${matched.join(', ')}`);
  setStatus(`Loaded options from ${name} (${POST_MODES[imported.mode].label})${notes.length ? ` — ${notes.join('; ')}` : ''}`);
}

$('#options-input').addEventListener('change', async (event) => {
  const file = event.target.files[0];
  event.target.value = '';
  if (!file) return;
  try {
    await workspace.add(file.name, file, 'upload');
    applyOptionsToml(file.name, await file.text());
  } catch (error) {
    setStatus(`Options: ${error.message}`);
  }
});

$('#options-save').addEventListener('click', async () => {
  try {
    const opts = postOptions();
    let hint = '';
    try { hint = buildPostCommand(opts).commandLine; } catch { /* inputs not chosen yet */ }
    const text = exportOptionsToml(opts, hint);
    const stem = (opts.out || defaultPosName(opts.rover, opts.mode)).replace(/\.[^./]*$/, '');
    const name = `${stem}_options.toml`;
    await workspace.add(name, new Blob([text], { type: 'text/plain' }), 'output');
    download(name, workspace.get(name).blob);
    setStatus(`Saved ${name}: use it as gnss ${DISPATCH_NAMES[POST_MODES[opts.mode].tool]} --config ${name}`);
  } catch (error) {
    setStatus(`Options: ${error.message}`);
  }
});

postForm.addEventListener('submit', (event) => {
  event.preventDefault();
  runPost().catch(reportTo(postView));
});
postForm.addEventListener('input', updatePostPreview);
postForm.addEventListener('change', () => {
  updatePostVisibility();
  refreshFileSelectors();
  applyImuMeta();
  updatePostPreview();
});

// A sample IMU file carries its lever arm; fill it in while the fields are empty.
function applyImuMeta() {
  const el = postForm.elements;
  const meta = postGroup() === 'imu' && fileMeta.get(el.imu.value);
  if (!meta || !meta.leverArm || [el.leverX, el.leverY, el.leverZ].some((input) => input.value !== '')) return;
  [el.leverX.value, el.leverY.value, el.leverZ.value] = meta.leverArm;
}

// ---------------------------------------------------------------- conversion
const convForm = $('#conv-form');
const convView = new RunView($('#tool-conv [data-run]'), 'Choose an RTCM3 or UBX log and press Convert.');
let convInputSeen = '';

function convFormat() {
  const v = convForm.elements.format.value;
  if (v !== 'auto') return v;
  const entry = workspace.get(convForm.elements.input.value);
  return entry && entry.kind === 'ubx' ? 'ubx' : 'rtcm';
}

function convOptions() {
  const v = formValues(convForm);
  return {
    input: v.input,
    format: convFormat(),
    gpsWeek: v.gpsWeek,
    obsOut: v.obsEnabled ? v.obsOut.trim() : '',
    navOut: v.navEnabled ? v.navOut.trim() : '',
    limit: v.limit,
    quiet: v.quiet,
    extra: v.extra,
    workspaceNames: workspace.names(),
  };
}

function updateConv() {
  const input = convForm.elements.input.value;
  if (input && input !== convInputSeen) {
    convInputSeen = input;
    const names = defaultConvertNames(input);
    convForm.elements.obsOut.value = names.obsOut;
    convForm.elements.navOut.value = names.navOut;
    const meta = fileMeta.get(input);
    if (meta && meta.gpsWeek) convForm.elements.gpsWeek.value = meta.gpsWeek;
  }
  const rtcm = convFormat() === 'rtcm';
  for (const el of $$('[data-show-format]', convForm)) el.hidden = !rtcm;
  try {
    convView.preview(buildConvertCommand(convOptions()).commandLine);
  } catch (error) {
    convView.preview(`# ${error.message}`);
  }
}

convForm.addEventListener('submit', (event) => {
  event.preventDefault();
  Promise.resolve().then(() => runCommand(buildConvertCommand(convOptions()), convView, convForm))
    .catch(reportTo(convView));
});
convForm.addEventListener('input', updateConv);
convForm.addEventListener('change', updateConv);

// ---------------------------------------------------------------- RINEX info
const infoForm = $('#info-form');
const infoView = new RunView($('#tool-info [data-run]'), 'Select RINEX files and press Inspect.');

function infoOptions() {
  return {
    files: $$('#info-files input:checked').map((i) => i.value),
    countRecords: infoForm.elements.countRecords.checked,
  };
}

function updateInfo() {
  try {
    infoView.preview(buildInfoCommand(infoOptions()).commandLine);
  } catch (error) {
    infoView.preview(`# ${error.message}`);
  }
}

infoForm.addEventListener('submit', (event) => {
  event.preventDefault();
  Promise.resolve().then(() => runCommand(buildInfoCommand(infoOptions()), infoView, infoForm))
    .catch(reportTo(infoView));
});
infoForm.addEventListener('change', updateInfo);

// ---------------------------------------------------------------- plot
const Q_COLOR_VARS = {
  0: '--q-dr', 1: '--q-fix', 2: '--q-float', 3: '--q-sbas', 4: '--q-dgps', 5: '--q-single', 6: '--q-ppp', 7: '--q-pppfix', 8: '--q-dr',
};
const MAP_SETTING = 'gnsspp.plotMap';
const ERROR_COLOR_VARS = ['--err-0', '--err-1', '--err-2', '--err-3', '--err-4', '--err-none'];

function readSetting(key) {
  try { return localStorage.getItem(key); } catch { return null; }
}

function writeSetting(key, value) {
  try { localStorage.setItem(key, value); } catch { /* private mode: keep it for this page only */ }
}

class PlotView {
  constructor() {
    this.cache = new Map();
    this.hidden = new Set();
    this.selects = { sol1: $('#plot-sol1'), sol2: $('#plot-sol2'), ref: $('#plot-ref') };
    this.originSelect = $('#plot-origin');
    this.errorsBox = $('#plot-errors');
    this.colorSelect = $('#plot-color');
    this.auxSelect = $('#plot-aux');
    this.mapSelect = $('#plot-map');
    const savedMap = readSetting(MAP_SETTING);
    if (savedMap && TILE_SOURCES[savedMap]) this.mapSelect.value = savedMap;
    this.mapEpoch = 0;
    this.tiles = new TileLayer({ onChange: () => this.gt.draw() });
    const linkTime = (source) => (view) => {
      for (const p of this.timePanels) if (p !== source) p.setXRange(view.x0, view.x1);
    };
    const hover = (x) => this.hoverTime(x);
    const leave = () => this.clearCursor();
    this.gt = new PlotPanel($('#cv-gt'), {
      equalAspect: true, xLabel: 'E (m)', yLabel: 'N (m)', onHover: (x, y) => this.hoverGround(x, y), onLeave: leave,
      background: {
        active: () => Boolean(this.mapSource() && this.plane),
        key: () => `${this.mapSelect.value}:${this.tiles.version}:${this.mapEpoch}`,
        paint: (ctx, v, r, t) => this.paintMap(ctx, v, r, t),
      },
    });
    this.pe = new PlotPanel($('#cv-e'), { timeAxis: true, yLabel: 'E (m)', onHover: hover, onLeave: leave });
    this.pn = new PlotPanel($('#cv-n'), { timeAxis: true, yLabel: 'N (m)', onHover: hover, onLeave: leave });
    this.pu = new PlotPanel($('#cv-u'), { timeAxis: true, yLabel: 'U (m)', onHover: hover, onLeave: leave });
    this.pa = new PlotPanel($('#cv-aux'), { timeAxis: true, yLabel: 'Nsat', onHover: hover, onLeave: leave });
    this.timePanels = [this.pe, this.pn, this.pu, this.pa];
    for (const p of this.timePanels) p.opts.onViewChange = linkTime(p);
    for (const el of [...Object.values(this.selects), this.originSelect, this.errorsBox]) {
      el.addEventListener('change', () => this.reload());
    }
    this.auxSelect.addEventListener('change', () => this.render());
    this.colorSelect.addEventListener('change', () => { this.keepView = true; this.render(); });
    for (const button of $$('#tool-plot [data-png]')) {
      button.addEventListener('click', () => this.savePng(button.dataset.png).catch((error) => setStatus(`PNG: ${error.message}`)));
    }
    this.mapSelect.addEventListener('change', () => {
      writeSetting(MAP_SETTING, this.mapSelect.value);
      this.keepView = true; // the track itself does not move
      this.render();
    });
    new MutationObserver(() => this.render()).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this.render());
  }

  open(name) {
    const entry = workspace.get(name);
    if (!entry) return;
    if (entry.kind === 'csv') this.selects.ref.value = name;
    else this.selects.sol1.value = name;
    showTool('plot');
    this.reload();
  }

  async parsed(name) {
    if (!name) return null;
    const entry = workspace.get(name);
    if (!entry) return null;
    const key = `${name}:${entry.added}`;
    if (!this.cache.has(key)) this.cache.set(key, parseSolution(name, await entry.blob.text()));
    return this.cache.get(key);
  }

  async reload() {
    $('#plot-cursor').textContent = '';
    try {
      this.sol1 = await this.parsed(this.selects.sol1.value);
      this.sol2 = await this.parsed(this.selects.sol2.value);
      this.ref = await this.parsed(this.selects.ref.value);
      this.error = null;
    } catch (error) {
      this.error = error;
    }
    this.render();
  }

  render() {
    const grid = $('#plot-grid');
    const empty = $('#plot-empty');
    if (this.error || !this.sol1) {
      grid.hidden = true;
      empty.hidden = false;
      empty.textContent = this.error ? String(this.error.message) : 'Load or compute a .pos solution (libgnss++ or RTKLIB format) to plot it.';
      $('#plot-qfilter').replaceChildren();
      return;
    }
    if ($('#tool-plot').hidden) return;
    grid.hidden = false;
    empty.hidden = true;
    const originMode = this.originSelect.value === 'reference' && !this.ref ? 'mean' : this.originSelect.value;
    this.origin = chooseOrigin(originMode, this.sol1, this.ref);
    this.enu1 = toEnu(this.sol1, this.origin);
    this.enu2 = this.sol2 ? toEnu(this.sol2, this.origin) : null;
    this.enuRef = this.ref ? toEnu(this.ref, this.origin) : null;
    this.err1 = this.ref ? referenceErrors(this.sol1, this.ref, this.origin) : null;
    this.err2 = this.ref && this.sol2 ? referenceErrors(this.sol2, this.ref, this.origin) : null;
    this.plane = enuPlane(this.origin); // map tiles are placed in this ENU frame
    this.mapEpoch += 1;
    const showErrors = this.errorsBox.checked && this.ref;
    $('#ts-title').textContent = showErrors ? 'Error vs reference E / N / U (m)' : 'Position E / N / U (m)';

    const qColors = {};
    for (const [q, name] of Object.entries(Q_COLOR_VARS)) {
      if (!this.hidden.has(Number(q))) qColors[q] = cssVar(name, '#888');
    }
    const sol2Color = cssVar('--sol2', '#7c3aed');
    const refColor = cssVar('--ref', '#111');
    const sol2Colors = Object.fromEntries(Object.keys(qColors).map((q) => [q, sol2Color]));
    // Solution 1 by quality, or by horizontal error class (hidden qualities
    // stay hidden: their epochs get a class with no colour).
    this.colorByError = this.colorSelect.value === 'error' && Boolean(this.err1);
    let sol1Cls = this.sol1.q;
    let sol1Colors = qColors;
    if (this.colorByError) {
      sol1Cls = errorClasses(this.err1);
      for (let i = 0; i < sol1Cls.length; i += 1) if (this.hidden.has(this.sol1.q[i])) sol1Cls[i] = 255;
      sol1Colors = Object.fromEntries(ERROR_COLOR_VARS.map((name, k) => [k, cssVar(name, '#888')]));
    }
    this.sol1Cls = sol1Cls;
    const sol1Layer = (x, y) => ({ type: 'points', x, y, cls: sol1Cls, colors: sol1Colors, size: 3, order: 2 });
    const sol2Layer = (x, y) => ({ type: 'points', x, y, cls: this.sol2.q, colors: sol2Colors, size: 3, order: 1 });
    const refLayer = (x, y) => ({ type: 'line', x, y, colors: refColor, width: 1, order: 0 });

    const gtLayers = [sol1Layer(this.enu1.e, this.enu1.n)];
    if (this.sol2) gtLayers.push(sol2Layer(this.enu2.e, this.enu2.n));
    if (this.ref) gtLayers.push(refLayer(this.enuRef.e, this.enuRef.n));
    this.gt.setLayers(gtLayers, { keepView: this.keepView });
    this.updateMapNote();

    for (const [panel, key] of [[this.pe, 'e'], [this.pn, 'n'], [this.pu, 'u']]) {
      const layers = [];
      if (showErrors) {
        layers.push(sol1Layer(this.sol1.t, this.err1[key]));
        if (this.sol2) layers.push(sol2Layer(this.sol2.t, this.err2[key]));
      } else {
        layers.push(sol1Layer(this.sol1.t, this.enu1[key]));
        if (this.sol2) layers.push(sol2Layer(this.sol2.t, this.enu2[key]));
        if (this.ref) layers.push(refLayer(this.ref.t, this.enuRef[key]));
      }
      panel.setLayers(layers, { keepView: this.keepView });
    }
    const aux = this.auxSelect.value;
    this.pa.opts.yLabel = { ns: 'Nsat', ratio: 'Ratio', pdop: 'PDOP' }[aux];
    this.pa.opts.integerY = aux === 'ns';
    const auxLayers = [sol1Layer(this.sol1.t, this.sol1[aux])];
    if (this.sol2) auxLayers.push(sol2Layer(this.sol2.t, this.sol2[aux]));
    this.pa.setLayers(auxLayers, { keepView: this.keepView });
    this.keepView = false;

    this.renderFilter();
    this.renderLegend(qColors, sol2Color, refColor, sol1Colors);
    this.renderStats();
  }

  mapSource() {
    return TILE_SOURCES[this.mapSelect.value] || null;
  }

  paintMap(ctx, v, r, t) {
    const source = this.mapSource();
    if (!source || !this.plane) return;
    this.mapStatus = this.tiles.paint(ctx, source, this.plane, v, r, t);
    if (this.mapStatus.tooWide) {
      this.updateMapNote();
      return;
    }
    ctx.fillStyle = cssVar('--map-veil', 'rgba(255, 255, 255, 0.18)');
    ctx.fillRect(r.left, r.top, r.width, r.height);
    this.updateMapNote();
  }

  updateMapNote() {
    const note = $('#plot-map-note');
    const source = this.mapSource();
    note.hidden = !source;
    if (!source) return;
    const status = document.createElement('span');
    if (this.tiles.failed > 0 && this.tiles.loaded === 0) {
      status.className = 'warn';
      status.textContent = 'Map tiles could not be loaded (offline or blocked); the track is drawn without a map.';
    } else if (this.mapStatus && this.mapStatus.tooWide) {
      status.textContent = `Zoom in to see the map (shown for views up to ${MAX_SPAN_M / 1000} km across).`;
    } else if (this.mapStatus) {
      status.textContent = `Map zoom ${this.mapStatus.zoom}`
        + (this.mapStatus.missing ? ` · loading ${this.mapStatus.missing} of ${this.mapStatus.requested} tiles` : '');
    }
    const credit = document.createElement('span');
    const link = document.createElement('a');
    link.href = source.link;
    link.target = '_blank';
    link.rel = 'noopener';
    link.textContent = source.attribution;
    credit.append('Map: ', link);
    note.replaceChildren(status, credit);
  }

  renderFilter() {
    const box = $('#plot-qfilter');
    box.replaceChildren();
    const counts = {};
    for (const sol of [this.sol1, this.sol2]) {
      if (!sol) continue;
      for (let i = 0; i < sol.n; i += 1) counts[sol.q[i]] = (counts[sol.q[i]] || 0) + 1;
    }
    for (const q of Object.keys(counts).map(Number).sort((a, b) => a - b)) {
      const button = document.createElement('button');
      button.type = 'button';
      const on = !this.hidden.has(q);
      button.className = on ? 'on' : '';
      if (on) button.style.background = cssVar(Q_COLOR_VARS[q], '#888');
      button.textContent = `${Q_LABELS[q]} ${counts[q]}`;
      button.setAttribute('aria-pressed', String(on));
      button.onclick = () => {
        if (this.hidden.has(q)) this.hidden.delete(q); else this.hidden.add(q);
        this.keepView = true;
        this.render();
      };
      box.append(button);
    }
  }

  renderLegend(qColors, sol2Color, refColor, sol1Colors) {
    const legend = $('#plot-legend');
    legend.replaceChildren();
    this.legendItems = [];
    const item = (color, text) => {
      this.legendItems.push({ color, text });
      const span = document.createElement('span');
      if (color) {
        const sw = document.createElement('span');
        sw.className = 'sw';
        sw.style.background = color;
        span.append(sw);
      }
      span.append(document.createTextNode(text));
      legend.append(span);
    };
    if (this.colorByError) {
      item(null, `${this.sol1.name} horizontal error:`);
      const present = new Set(this.sol1Cls);
      errorClassLabels().forEach((label, k) => {
        if (present.has(k)) item(sol1Colors[k], label);
      });
    } else {
      const present = new Set(this.sol1.q);
      for (const q of [...present].sort()) if (qColors[q]) item(qColors[q], `${Q_LABELS[q]} (${this.sol1.name})`);
    }
    if (this.sol2) item(sol2Color, this.sol2.name);
    if (this.ref) item(refColor, `${this.ref.name} (reference)`);
    this.colorSelect.closest('label').title = this.ref ? '' : 'Choose a reference to colour by error';
  }

  /** Save one plot card ('gt' | 'ts' | 'aux') as PNG, legend and map credit included. */
  async savePng(which) {
    if (!this.sol1) return;
    const cards = {
      gt: { canvases: [this.gt.canvas], title: 'Ground track E/N (m)', suffix: 'ground_track' },
      ts: { canvases: [this.pe.canvas, this.pn.canvas, this.pu.canvas], title: $('#ts-title').textContent, suffix: 'enu' },
      aux: { canvases: [this.pa.canvas], title: this.pa.opts.yLabel, suffix: this.auxSelect.value },
    };
    const card = cards[which];
    const names = [this.sol1, this.sol2].filter(Boolean).map((sol) => sol.name);
    if (this.ref) names.push(`reference ${this.ref.name}`);
    const notes = [];
    const source = this.mapSource();
    if (which === 'gt' && source) notes.push(`Map: ${source.attribution}`);
    const legend = which === 'aux' ? this.legendItems.filter((it) => !it.text.endsWith('(reference)')) : this.legendItems;
    const blob = await canvasesToPng(card.canvases, {
      title: `${card.title} · ${names.join(' · ')}`,
      legend,
      notes,
      background: cssVar('--panel', '#fff'),
      ink: cssVar('--ink', '#1f2937'),
      muted: cssVar('--muted', '#6b7280'),
    });
    const name = `${this.sol1.name.replace(/\.[^./]*$/, '')}_${card.suffix}.png`;
    download(name, blob);
    setStatus(`Saved ${name}`);
  }

  renderStats() {
    const fmt = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : '–');
    const sols = [this.sol1, this.sol2].filter(Boolean);
    const enus = [this.enu1, this.enu2];
    const errs = [this.err1, this.err2];
    const summaries = sols.map((s, i) => solutionSummary(s, enus[i]));
    const rows = [];
    const row = (label, values) => rows.push(`<tr><td>${label}</td>${values.map((v) => `<td>${v}</td>`).join('')}</tr>`);
    const esc = (s) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
    const head = `<tr><th></th>${sols.map((s) => `<th>${esc(s.name)}</th>`).join('')}</tr>`;
    row('Format', sols.map((s) => esc(s.format)));
    row('Epochs', summaries.map((s) => s.epochs));
    row('Start (GPST)', summaries.map((s) => formatGpst(s.start)));
    row('Span (s)', summaries.map((s) => fmt(s.end - s.start, 1)));
    const labels = new Set(summaries.flatMap((s) => Object.keys(s.counts)));
    for (const label of ['FIX', 'PPP-FIX', 'FLOAT', 'PPP', 'DGPS', 'SBAS', 'SINGLE', 'DR', 'NONE']) {
      if (!labels.has(label)) continue;
      row(label, summaries.map((s) => {
        const c = s.counts[label] || 0;
        return `${c} (${fmt((100 * c) / s.epochs, 1)}%)`;
      }));
    }
    for (const axis of ['e', 'n', 'u']) {
      row(`${axis.toUpperCase()} mean / std (m)`, summaries.map((s) => `${fmt(s[axis].mean)} / ${fmt(s[axis].std)}`));
    }
    if (this.ref) {
      rows.push(`<tr><th colspan="${sols.length + 1}">Error vs ${esc(this.ref.name)}</th></tr>`);
      const all = sols.map((s, i) => errorSummary(errs[i]));
      const fix = sols.map((s, i) => errorSummary(errs[i], (k) => isFixQuality(s.q[k])));
      row('Matched epochs', all.map((a) => a.matched));
      row('Horizontal RMS (m)', all.map((a) => fmt(a.horizontalRms)));
      row('Horizontal 50% / 95% (m)', all.map((a) => `${fmt(a.horizontal50)} / ${fmt(a.horizontal95)}`));
      row('Horizontal max (m)', all.map((a) => fmt(a.horizontalMax)));
      row('Vertical RMS / 95% (m)', all.map((a) => `${fmt(a.u.rms)} / ${fmt(a.vertical95)}`));
      row('FIX epochs: H RMS / V RMS (m)', fix.map((a) => (a.matched ? `${fmt(a.horizontalRms)} / ${fmt(a.u.rms)}` : '–')));
    }
    const [lat, lon, h] = this.origin;
    $('#plot-stats').innerHTML = `<table>${head}${rows.join('')}</table>`
      + `<p class="hint">Origin: ${lat.toFixed(9)}°, ${lon.toFixed(9)}°, ${h.toFixed(3)} m. Times are GPST.</p>`;
  }

  showCursor(i) {
    const sol = this.sol1;
    if (!sol || i < 0) return;
    const t = sol.t[i];
    for (const p of this.timePanels) p.setCursor({ x: t });
    this.gt.setCursor({ x: this.enu1.e[i], y: this.enu1.n[i] });
    const { week, tow } = splitGpsSeconds(t);
    const parts = [
      `${formatGpst(t, 2)} GPST (week ${week} tow ${tow.toFixed(3)})`,
      Q_LABELS[sol.q[i]],
      `lat ${sol.lat[i].toFixed(9)} lon ${sol.lon[i].toFixed(9)} h ${sol.h[i].toFixed(4)}`,
      `E ${this.enu1.e[i].toFixed(3)} N ${this.enu1.n[i].toFixed(3)} U ${this.enu1.u[i].toFixed(3)}`,
    ];
    if (Number.isFinite(sol.ns[i])) parts.push(`ns ${sol.ns[i]}`);
    if (Number.isFinite(sol.ratio[i])) parts.push(`ratio ${sol.ratio[i].toFixed(1)}`);
    if (this.err1 && Number.isFinite(this.err1.e[i])) {
      parts.push(`err H ${Math.hypot(this.err1.e[i], this.err1.n[i]).toFixed(3)} U ${this.err1.u[i].toFixed(3)}`);
    }
    $('#plot-cursor').textContent = parts.join('  |  ');
  }

  hoverTime(x) {
    if (this.sol1) this.showCursor(nearestIndex(this.sol1.t, x));
  }

  hoverGround(x, y) {
    if (!this.sol1) return;
    const { kx, ky } = this.gt.pixelScale();
    let best = -1;
    let bestD = 400; // 20 px radius
    for (let i = 0; i < this.sol1.n; i += 1) {
      if (this.hidden.has(this.sol1.q[i])) continue;
      const d = ((this.enu1.e[i] - x) * kx) ** 2 + ((this.enu1.n[i] - y) * ky) ** 2;
      if (d < bestD) { bestD = d; best = i; }
    }
    if (best >= 0) this.showCursor(best);
  }

  clearCursor() {
    for (const p of [...this.timePanels, this.gt]) p.setCursor(null);
  }
}

const plotView = new PlotView();

// ---------------------------------------------------------------- observations
const obsForm = $('#obs-form');
const obsRunView = new RunView($('#tool-obs [data-run]'), 'Choose observation and navigation files and press Analyze.');

function updateObsVisibility() {
  const ecef = obsForm.elements.receiverPos.value === 'ecef';
  for (const el of $$('[data-show-receiver]', obsForm)) el.hidden = !ecef;
}

function obsOptions() {
  const v = formValues(obsForm);
  return {
    obs: v.obs,
    nav: v.nav,
    minElevation: v.minElevation,
    maxEpochs: v.maxEpochs,
    receiverEcef: v.receiverPos === 'ecef' ? [v.receiverX, v.receiverY, v.receiverZ] : null,
    extra: v.extra,
    workspaceNames: workspace.names(),
  };
}

function updateObsPreview() {
  try {
    obsRunView.preview(buildVisibilityCommand(obsOptions()).commandLine);
  } catch (error) {
    obsRunView.preview(`# ${error.message}`);
  }
}

async function runObs() {
  const cmd = buildVisibilityCommand(obsOptions());
  const result = await runCommand(cmd, obsRunView, obsForm);
  const csv = cmd.outputs.find((name) => name.endsWith('_vis.csv'));
  if (result.exitCode === 0 && workspace.get(csv)) obsView.open(csv);
  return result;
}

obsForm.addEventListener('submit', (event) => {
  event.preventDefault();
  Promise.resolve().then(runObs).catch(reportTo(obsRunView));
});
obsForm.addEventListener('input', updateObsPreview);
obsForm.addEventListener('change', () => { updateObsVisibility(); updateObsPreview(); });

const SNR_BINS = [25, 30, 35, 40, 45];
const SNR_LABELS = ['< 25', '25–30', '30–35', '35–40', '40–45', '≥ 45'];
const SYS_VARS = ['--sys-gps', '--sys-glo', '--sys-gal', '--sys-bds', '--sys-qzs', '--sys-sbas', '--sys-irn'];

function snrClass(snr) {
  if (!Number.isFinite(snr)) return 0;
  let k = 0;
  while (k < SNR_BINS.length && snr >= SNR_BINS[k]) k += 1;
  return k;
}

class ObsView {
  constructor() {
    this.cache = new Map();
    this.hiddenSys = new Set();
    this.dataSelect = $('#obs-data');
    this.colorSelect = $('#obs-color');
    const leave = () => this.frame(() => this.clearCursor());
    this.sky = new SkyPlot($('#cv-sky'), { onHover: (i) => (i >= 0 ? this.frame(() => this.showSample(i)) : null) });
    this.track = new PlotPanel($('#cv-track'), {
      timeAxis: true, integerY: true, yTickFormat: (v) => this.rowLabel(v),
      onHover: (x, y) => this.frame(() => this.hoverTrack(x, y)), onLeave: leave,
    });
    this.snrEl = new PlotPanel($('#cv-snr-el'), { xLabel: 'elevation (deg)', yLabel: 'SNR (dB-Hz)', onLeave: leave });
    this.nsat = new PlotPanel($('#cv-obs-nsat'), {
      timeAxis: true, integerY: true, yLabel: 'Nsat', onHover: (x) => this.frame(() => this.hoverEpoch(x)), onLeave: leave,
    });
    this.track.opts.onViewChange = (view) => this.nsat.setXRange(view.x0, view.x1);
    this.nsat.opts.onViewChange = (view) => this.track.setXRange(view.x0, view.x1);
    this.dataSelect.addEventListener('change', () => this.reload());
    this.colorSelect.addEventListener('change', () => { this.keepView = true; this.render(); });
    new MutationObserver(() => this.render()).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this.render());
  }

  open(name) {
    this.dataSelect.value = name;
    showTool('obs');
    this.reload();
  }

  // Run at most one hover update per animation frame (the latest one).
  frame(update) {
    this.pendingUpdate = update;
    if (this.frameId) return;
    this.frameId = requestAnimationFrame(() => {
      this.frameId = 0;
      const next = this.pendingUpdate;
      this.pendingUpdate = null;
      if (next && this.data) next();
    });
  }

  // Row indices per satellite (time order) and per epoch, for hover lookups.
  buildIndex(d) {
    this.satRows = d.sats.map(() => []);
    this.epochRows = new Map();
    for (let i = 0; i < d.n; i += 1) {
      this.satRows[d.sat[i]].push(i);
      const rows = this.epochRows.get(d.t[i]);
      if (rows) rows.push(i); else this.epochRows.set(d.t[i], [i]);
    }
    for (const rows of this.satRows) rows.sort((a, b) => d.t[a] - d.t[b]);
  }

  async reload() {
    $('#obs-cursor').textContent = '';
    const name = this.dataSelect.value;
    const entry = name && workspace.get(name);
    this.error = null;
    this.data = null;
    if (entry) {
      try {
        const key = `${name}:${entry.added}`;
        if (!this.cache.has(key)) this.cache.set(key, parseVisibilityCsv(name, await entry.blob.text()));
        this.data = this.cache.get(key);
        const summaryName = name.replace(/\.csv$/, '_summary.json');
        const summary = workspace.get(summaryName);
        this.summary = summary ? JSON.parse(await summary.blob.text()) : null;
      } catch (error) {
        this.error = error;
      }
    }
    this.render();
  }

  rowLabel(v) {
    if (!this.data) return '';
    const i = this.data.sats.length - 1 - Math.round(v);
    return Math.abs(v - Math.round(v)) < 1e-6 && this.data.sats[i] ? this.data.sats[i] : '';
  }

  colors() {
    if (this.colorSelect.value === 'sys') {
      return { cls: this.data.sys, colors: Object.fromEntries(SYS_VARS.map((v, k) => [k, cssVar(v, '#888')])) };
    }
    if (!this.snrCls || this.snrCls.length !== this.data.n) {
      this.snrCls = Uint8Array.from(this.data.snr, snrClass);
    }
    return { cls: this.snrCls, colors: Object.fromEntries(SNR_LABELS.map((_, k) => [k, cssVar(`--snr-${k}`, '#888')])) };
  }

  render() {
    const grid = $('#obs-grid');
    const empty = $('#obs-empty');
    if (this.error || !this.data) {
      grid.hidden = true;
      empty.hidden = false;
      if (this.error) empty.textContent = String(this.error.message);
      $('#obs-sysfilter').replaceChildren();
      return;
    }
    if ($('#tool-obs').hidden) return;
    grid.hidden = false;
    empty.hidden = true;
    const d = this.data;
    if (this.rowY?.length !== d.n || this.rowsFor !== d) {
      this.rowsFor = d;
      this.rowY = Float64Array.from(d.sat, (s) => d.sats.length - 1 - s);
      this.snrCls = null;
      this.buildIndex(d);
    }
    const { cls, colors } = this.colors();
    // System filter: drop hidden systems from the class -> colour map via a mask class.
    const visible = new Uint8Array(d.n);
    for (let i = 0; i < d.n; i += 1) visible[i] = this.hiddenSys.has(d.sys[i]) ? 0 : 1;
    const shownCls = Uint8Array.from(cls, (c, i) => (visible[i] ? c : 255));

    // One labelled row per satellite where it fits; at least as tall as the skyplot.
    const trackHeight = Math.max($('#cv-sky').clientHeight || 0, Math.min(900, d.sats.length * 14 + 40), 220);
    $('#cv-track').style.height = `${trackHeight}px`;
    this.track.opts.yStep = Math.max(1, Math.ceil((14 * d.sats.length) / Math.max(1, trackHeight - 40)));
    const slipX = [];
    const slipY = [];
    for (let i = 0; i < d.n; i += 1) {
      if (visible[i] && (d.lli[i] & 1)) { slipX.push(d.t[i]); slipY.push(this.rowY[i]); }
    }
    this.track.setLayers([
      { type: 'points', x: d.t, y: this.rowY, cls: shownCls, colors, size: 3, order: 1 },
      { type: 'vticks', x: Float64Array.from(slipX), y: Float64Array.from(slipY), colors: cssVar('--slip', '#111'), size: 11, width: 2, order: 2 },
    ], { keepView: this.keepView });

    this.sky.setData({
      az: d.az, el: d.el, cls: shownCls, colors,
      labels: this.lastPositions().filter((p) => visible[p.index]).map((p) => ({ az: d.az[p.index], el: d.el[p.index], text: p.sat })),
    });

    this.snrEl.setLayers([{ type: 'points', x: d.el, y: d.snr, cls: shownCls, colors, size: 2 }], { keepView: this.keepView });

    // Satellites per epoch (visible systems).
    const counts = new Map();
    for (let i = 0; i < d.n; i += 1) if (visible[i]) counts.set(d.t[i], (counts.get(d.t[i]) || 0) + 1);
    const et = Float64Array.from(counts.keys()).sort();
    const ec = Float64Array.from(et, (t) => counts.get(t));
    this.nsat.setLayers([{ type: 'points', x: et, y: ec, colors: cssVar('--accent', '#0f766e'), size: 3 }], { keepView: this.keepView });
    this.epochTimes = et;
    this.keepView = false;

    this.renderFilter();
    this.renderLegend(colors);
    this.renderStats(visible);
  }

  lastPositions() {
    const d = this.data;
    const last = new Map();
    for (let i = 0; i < d.n; i += 1) last.set(d.sat[i], i);
    return [...last.entries()].map(([s, index]) => ({ sat: d.sats[s], index }));
  }

  renderFilter() {
    const box = $('#obs-sysfilter');
    box.replaceChildren();
    const counts = {};
    for (const s of this.data.sys) counts[s] = (counts[s] || 0) + 1;
    for (const s of Object.keys(counts).map(Number).sort((a, b) => a - b)) {
      const button = document.createElement('button');
      button.type = 'button';
      const on = !this.hiddenSys.has(s);
      button.className = on ? 'on' : '';
      if (on) button.style.background = cssVar(SYS_VARS[s], '#888');
      const sats = new Set();
      for (let i = 0; i < this.data.n; i += 1) if (this.data.sys[i] === s) sats.add(this.data.sat[i]);
      button.textContent = `${SYSTEMS[s]} ${sats.size}`;
      button.setAttribute('aria-pressed', String(on));
      button.onclick = () => {
        if (this.hiddenSys.has(s)) this.hiddenSys.delete(s); else this.hiddenSys.add(s);
        this.keepView = true;
        this.render();
      };
      box.append(button);
    }
  }

  renderLegend(colors) {
    const legend = $('#obs-legend');
    legend.replaceChildren();
    const labels = this.colorSelect.value === 'sys' ? SYSTEMS : SNR_LABELS.map((l) => `${l} dB-Hz`);
    const present = new Set(this.colorSelect.value === 'sys' ? this.data.sys : this.snrCls);
    labels.forEach((label, k) => {
      if (!present.has(k)) return;
      const span = document.createElement('span');
      const sw = document.createElement('span');
      sw.className = 'sw';
      sw.style.background = colors[k];
      span.append(sw, document.createTextNode(label));
      legend.append(span);
    });
    const slip = document.createElement('span');
    const sw = document.createElement('span');
    sw.className = 'sw';
    sw.style.background = cssVar('--slip', '#111');
    slip.append(sw, document.createTextNode(this.data.hasLli ? 'LLI slip' : 'no LLI column in this CSV'));
    legend.append(slip);
  }

  renderStats(visible) {
    const d = this.data;
    const perSat = new Map();
    for (let i = 0; i < d.n; i += 1) {
      if (!visible[i]) continue;
      const s = perSat.get(d.sat[i]) || { rows: 0, slips: 0, snr: 0, snrN: 0, maxEl: -90 };
      s.rows += 1;
      if (d.lli[i] & 1) s.slips += 1;
      if (Number.isFinite(d.snr[i]) && d.snr[i] > 0) { s.snr += d.snr[i]; s.snrN += 1; }
      s.maxEl = Math.max(s.maxEl, d.el[i]);
      perSat.set(d.sat[i], s);
    }
    const fmt = (v, k = 1) => (Number.isFinite(v) ? v.toFixed(k) : '–');
    const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
    const sum = this.summary;
    const head = [
      ['Data', esc(d.name)],
      ['Epochs', d.epochs.length],
      ['Span (GPST)', `${formatGpst(d.epochs[0])} – ${formatGpst(d.epochs[d.epochs.length - 1])}`],
      ['Satellites', perSat.size],
      ['Rows', d.n],
      ['LLI slips', [...perSat.values()].reduce((a, s) => a + s.slips, 0)],
    ];
    if (sum) {
      head.push(['Receiver position', `${esc(sum.receiver_position_source || '')} (${(sum.receiver_position_ecef_m || []).map((v) => v.toFixed(1)).join(', ')})`]);
      head.push(['Mean SNR (dB-Hz)', fmt(sum.mean_snr_dbhz)]);
      head.push(['Skipped (no ephemeris)', sum.skipped_without_nav]);
    }
    const sats = [...perSat.entries()].sort((a, b) => a[0] - b[0]);
    const satRows = sats.map(([s, v]) => `<tr><td>${esc(d.sats[s])}</td><td>${v.rows}</td><td>${fmt(v.snrN ? v.snr / v.snrN : NaN)}</td><td>${fmt(v.maxEl)}</td><td>${v.slips}</td></tr>`).join('');
    $('#obs-stats').innerHTML = `<table>${head.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('')}</table>`
      + `<table><tr><th>Sat</th><th>Epochs</th><th>Mean SNR</th><th>Max el (deg)</th><th>LLI slips</th></tr>${satRows}</table>`;
  }

  // Satellites at the epoch nearest to t, highlighted on the skyplot.
  cursorAt(t) {
    const d = this.data;
    const j = nearestIndex(d.epochs, t);
    const te = d.epochs[j];
    const marks = [];
    const sysColors = SYS_VARS.map((v) => cssVar(v, '#888'));
    for (const i of this.epochRows.get(te) || []) {
      if (!this.hiddenSys.has(d.sys[i])) marks.push({ az: d.az[i], el: d.el[i], text: d.sats[d.sat[i]].slice(1), color: sysColors[d.sys[i]] });
    }
    this.sky.setCursor(marks);
    this.track.setCursor({ x: te });
    this.nsat.setCursor({ x: te });
    return { te, count: marks.length };
  }

  hoverEpoch(x) {
    if (!this.data) return;
    const { te, count } = this.cursorAt(x);
    const { week, tow } = splitGpsSeconds(te);
    $('#obs-cursor').textContent = `${formatGpst(te, 2)} GPST (week ${week} tow ${tow.toFixed(3)})  |  ${count} satellites`;
  }

  hoverTrack(x, y) {
    const d = this.data;
    if (!d) return;
    const row = Math.round(y);
    const s = d.sats.length - 1 - row;
    const rows = this.satRows[s] || [];
    let best = -1;
    if (rows.length) {
      // Binary search for the satellite's sample nearest in time.
      let lo = 0;
      let hi = rows.length - 1;
      while (hi - lo > 1) {
        const mid = (lo + hi) >> 1;
        if (d.t[rows[mid]] < x) lo = mid; else hi = mid;
      }
      best = Math.abs(d.t[rows[lo]] - x) <= Math.abs(d.t[rows[hi]] - x) ? rows[lo] : rows[hi];
    }
    if (best < 0) { this.hoverEpoch(x); return; }
    this.cursorAt(d.t[best]);
    this.describe(best);
  }

  showSample(i) {
    this.cursorAt(this.data.t[i]);
    this.describe(i);
  }

  describe(i) {
    const d = this.data;
    const { week, tow } = splitGpsSeconds(d.t[i]);
    $('#obs-cursor').textContent = [
      `${formatGpst(d.t[i], 2)} GPST (week ${week} tow ${tow.toFixed(3)})`,
      `${d.sats[d.sat[i]]} ${d.signal[i]}`,
      `az ${d.az[i].toFixed(1)}° el ${d.el[i].toFixed(1)}°`,
      `SNR ${fmtNum(d.snr[i])} dB-Hz`,
      `LLI ${d.lli[i]}${d.lli[i] & 1 ? ' (slip)' : ''}`,
    ].join('  |  ');
  }

  clearCursor() {
    this.sky.setCursor([]);
    this.track.setCursor(null);
    this.nsat.setCursor(null);
  }
}

function fmtNum(v) {
  return Number.isFinite(v) ? v.toFixed(1) : '–';
}

const obsView = new ObsView();

// ---------------------------------------------------------------- samples
async function loadSamples() {
  setStatus('Loading sample data…');
  for (const sample of SAMPLES) {
    const response = await fetch(sample.path);
    if (!response.ok) throw new Error(`could not fetch ${sample.path} (${response.status})`);
    const name = sample.path.split('/').pop();
    if (sample.meta) fileMeta.set(name, sample.meta);
    await workspace.add(name, await response.blob(), 'sample');
  }
  setStatus('Sample data loaded');
}

async function runSamplePipeline() {
  const button = $('#run-sample');
  button.disabled = true;
  try {
    await loadSamples();
    showTool('conv');
    // Drive the forms as a user would, so each view shows what ran.
    for (const [input, withNav] of [['rover.rtcm', false], ['base.rtcm', true]]) {
      const el = convForm.elements;
      el.input.value = input;
      el.format.value = 'rtcm';
      updateConv();
      el.gpsWeek.value = 2324;
      el.obsEnabled.checked = true;
      el.navEnabled.checked = withNav;
      el.quiet.checked = true;
      el.limit.value = '';
      el.extra.value = '';
      updateConv();
      const cmd = buildConvertCommand(convOptions());
      const result = await runCommand(cmd, convView, convForm);
      if (result.exitCode !== 0) throw new Error(`${cmd.commandLine} failed`);
    }
    // Observation viewer on the converted rover (position from SPP: no 1005 in the rover stream).
    showTool('obs');
    refreshFileSelectors();
    obsForm.elements.obs.value = 'rover.obs';
    obsForm.elements.nav.value = 'base.nav';
    obsForm.elements.receiverPos.value = 'auto';
    obsForm.elements.extra.value = '';
    updateObsVisibility();
    const vis = await runObs();
    if (vis.exitCode !== 0) throw new Error('gnss visibility failed');
    showTool('post');
    postForm.elements.mode.value = 'kinematic';
    updatePostVisibility();
    refreshFileSelectors();
    postForm.elements.rover.value = 'rover.obs';
    postForm.elements.base.value = 'base.obs';
    postForm.elements.nav.value = 'base.nav';
    postForm.elements.basePos.value = 'header';
    postForm.elements.out.value = '';
    postForm.elements.extra.value = '';
    updatePostVisibility();
    updatePostPreview();
    const result = await runPost();
    if (result.exitCode !== 0) throw new Error('RTK post-processing failed');
    plotView.selects.sol1.value = 'rover_rtk.pos';
    plotView.selects.ref.value = 'reference.csv';
    plotView.originSelect.value = 'reference';
    plotView.errorsBox.checked = true;
    showTool('plot');
    await plotView.reload();
    setStatus('Demo pipeline finished: RTCM3 → RINEX → observation view → kinematic RTK → plot vs reference');
  } catch (error) {
    setStatus(`Demo pipeline stopped: ${error.message}`);
  } finally {
    button.disabled = !engineReady;
  }
}

// ---------------------------------------------------------------- boot
workspace.addEventListener('change', (event) => {
  persistChange(event.detail);
  renderFiles(event.detail && event.detail.added);
  refreshFileSelectors();
  updatePostPreview();
  updateConv();
  updateInfo();
  if (!$('#tool-plot').hidden) plotView.reload();
  updateObsPreview();
  if (!$('#tool-obs').hidden) obsView.reload();
});

for (const button of $$('.tools button')) button.onclick = () => showTool(button.dataset.tool);
$('#file-input').addEventListener('change', (event) => {
  addFiles([...event.target.files]);
  event.target.value = '';
});
$('#load-sample').onclick = () => loadSamples().catch((error) => setStatus(error.message));
$('#clear-files').onclick = () => {
  const n = workspace.names().length;
  if (n && window.confirm(`Remove all ${n} file(s)${keepFiles ? ', including the copies kept in this browser' : ''}?`)) workspace.clear();
};
restoreKeptFiles().catch((error) => setStatus(`Browser storage: ${error.message}`));
$('#run-sample').onclick = () => runSamplePipeline();

let dragDepth = 0;
window.addEventListener('dragenter', (event) => {
  if (![...event.dataTransfer.types].includes('Files')) return;
  dragDepth += 1;
  $('#drop-overlay').hidden = false;
});
window.addEventListener('dragleave', () => {
  dragDepth = Math.max(0, dragDepth - 1);
  if (dragDepth === 0) $('#drop-overlay').hidden = true;
});
window.addEventListener('dragover', (event) => event.preventDefault());
window.addEventListener('drop', (event) => {
  event.preventDefault();
  dragDepth = 0;
  $('#drop-overlay').hidden = true;
  if (event.dataTransfer.files.length) addFiles([...event.dataTransfer.files]);
});

persistForm(postForm, 'post');
persistForm(convForm, 'conv');
persistForm(obsForm, 'obs');
renderFiles();
refreshFileSelectors();
updatePostVisibility();
updatePostPreview();
updateConv();
updateInfo();
updateObsVisibility();
updateObsPreview();
const initialTool = location.hash.slice(1);
showTool(['post', 'conv', 'plot', 'obs', 'info'].includes(initialTool) ? initialTool : 'post');

(async () => {
  engineReady = await wasmAvailable();
  for (const button of $$('form button[type="submit"], #run-sample')) button.disabled = !engineReady;
  if (!engineReady) {
    const banner = $('#banner');
    banner.hidden = false;
    banner.textContent = 'The WebAssembly tools were not found next to this page (wasm/*.wasm). '
      + 'Build them with Emscripten (see wasm/README.md). Plot still works for local .pos files.';
    $('#engine').textContent = 'engine: unavailable';
    return;
  }
  try {
    const info = await (await fetch('wasm/build-info.json')).json();
    $('#engine').textContent = `engine: libgnss++ ${info.version} (${info.commit}) · WebAssembly`;
  } catch {
    $('#engine').textContent = 'engine: libgnss++ · WebAssembly';
  }
})();
