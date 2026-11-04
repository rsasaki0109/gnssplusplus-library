// libgnss++ Web: RTKLIB-style GUI over the libgnss++ CLI tools compiled to
// WebAssembly. Each run executes the unchanged native main() in a worker.

import {
  buildConvertCommand, buildInfoCommand, buildPostCommand, defaultConvertNames, defaultPosName, POST_MODES,
} from './commands.js';
import { parseSolution, Q_LABELS, rinexObsTimeSpan } from './formats.js';
import { chooseOrigin, errorSummary, isFixQuality, nearestIndex, referenceErrors, solutionSummary, toEnu } from './analysis.js';
import { formatGpst, splitGpsSeconds, SECONDS_PER_WEEK } from './geo.js';
import { exportOptionsToml, importOptionsToml } from './options.js';
import { cssVar, PlotPanel } from './plot.js';
import { startJob, wasmAvailable } from './runner.js';
import { Workspace } from './workspace.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const workspace = new Workspace();
const fileMeta = new Map(); // name -> { gpsWeek } hints for sample files

const SAMPLES = [
  { path: 'samples/rover.rtcm', meta: { gpsWeek: 2324 } },
  { path: 'samples/base.rtcm', meta: { gpsWeek: 2324 } },
  { path: 'samples/reference.csv' },
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
  return mode.startsWith('ppp') ? 'ppp' : 'rtk';
}

function updatePostVisibility() {
  const group = postGroup();
  for (const el of $$('[data-show]', postForm)) el.hidden = !el.dataset.show.split(/\s+/).includes(group);
  const ecef = postForm.elements.basePos.value === 'ecef';
  for (const el of $$('[data-show-base]', postForm)) el.hidden = !ecef || group !== 'rtk';
}

function postOptions() {
  const v = formValues(postForm);
  const group = postGroup();
  return {
    mode: v.mode,
    rover: v.rover,
    base: group === 'rtk' ? v.base : '',
    nav: v.nav,
    sp3: group === 'rtk' ? '' : v.sp3,
    clk: group === 'rtk' ? '' : v.clk,
    out: v.out.trim(),
    elevationMask: v.elevationMask,
    maxEpochs: v.maxEpochs,
    preset: v.preset,
    iono: v.iono,
    ratio: v.ratio,
    baseEcef: group === 'rtk' && v.basePos === 'ecef' ? [v.baseX, v.baseY, v.baseZ] : null,
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
    setStatus(`Saved ${name}: use it as gnss ${POST_MODES[opts.mode].tool === 'gnss_ppp' ? 'ppp' : 'solve'} --config ${name}`);
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
  updatePostPreview();
});

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

class PlotView {
  constructor() {
    this.cache = new Map();
    this.hidden = new Set();
    this.selects = { sol1: $('#plot-sol1'), sol2: $('#plot-sol2'), ref: $('#plot-ref') };
    this.originSelect = $('#plot-origin');
    this.errorsBox = $('#plot-errors');
    this.auxSelect = $('#plot-aux');
    const linkTime = (source) => (view) => {
      for (const p of this.timePanels) if (p !== source) p.setXRange(view.x0, view.x1);
    };
    const hover = (x) => this.hoverTime(x);
    const leave = () => this.clearCursor();
    this.gt = new PlotPanel($('#cv-gt'), { equalAspect: true, xLabel: 'E (m)', yLabel: 'N (m)', onHover: (x, y) => this.hoverGround(x, y), onLeave: leave });
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
    const showErrors = this.errorsBox.checked && this.ref;
    $('#ts-title').textContent = showErrors ? 'Error vs reference E / N / U (m)' : 'Position E / N / U (m)';

    const qColors = {};
    for (const [q, name] of Object.entries(Q_COLOR_VARS)) {
      if (!this.hidden.has(Number(q))) qColors[q] = cssVar(name, '#888');
    }
    const sol2Color = cssVar('--sol2', '#7c3aed');
    const refColor = cssVar('--ref', '#111');
    const sol2Colors = Object.fromEntries(Object.keys(qColors).map((q) => [q, sol2Color]));
    const sol1Layer = (x, y) => ({ type: 'points', x, y, cls: this.sol1.q, colors: qColors, size: 3, order: 2 });
    const sol2Layer = (x, y) => ({ type: 'points', x, y, cls: this.sol2.q, colors: sol2Colors, size: 3, order: 1 });
    const refLayer = (x, y) => ({ type: 'line', x, y, colors: refColor, width: 1, order: 0 });

    const gtLayers = [sol1Layer(this.enu1.e, this.enu1.n)];
    if (this.sol2) gtLayers.push(sol2Layer(this.enu2.e, this.enu2.n));
    if (this.ref) gtLayers.push(refLayer(this.enuRef.e, this.enuRef.n));
    this.gt.setLayers(gtLayers, { keepView: this.keepView });

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
    this.renderLegend(qColors, sol2Color, refColor);
    this.renderStats();
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

  renderLegend(qColors, sol2Color, refColor) {
    const legend = $('#plot-legend');
    legend.replaceChildren();
    const item = (color, text) => {
      const span = document.createElement('span');
      const sw = document.createElement('span');
      sw.className = 'sw';
      sw.style.background = color;
      span.append(sw, document.createTextNode(text));
      legend.append(span);
    };
    const present = new Set(this.sol1.q);
    for (const q of [...present].sort()) if (qColors[q]) item(qColors[q], `${Q_LABELS[q]} (${this.sol1.name})`);
    if (this.sol2) item(sol2Color, this.sol2.name);
    if (this.ref) item(refColor, `${this.ref.name} (reference)`);
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
    setStatus('Demo pipeline finished: RTCM3 → RINEX → kinematic RTK → plot vs reference');
  } catch (error) {
    setStatus(`Demo pipeline stopped: ${error.message}`);
  } finally {
    button.disabled = !engineReady;
  }
}

// ---------------------------------------------------------------- boot
workspace.addEventListener('change', (event) => {
  renderFiles(event.detail && event.detail.added);
  refreshFileSelectors();
  updatePostPreview();
  updateConv();
  updateInfo();
  if (!$('#tool-plot').hidden) plotView.reload();
});

for (const button of $$('.tools button')) button.onclick = () => showTool(button.dataset.tool);
$('#file-input').addEventListener('change', (event) => {
  addFiles([...event.target.files]);
  event.target.value = '';
});
$('#load-sample').onclick = () => loadSamples().catch((error) => setStatus(error.message));
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
renderFiles();
refreshFileSelectors();
updatePostVisibility();
updatePostPreview();
updateConv();
updateInfo();
const initialTool = location.hash.slice(1);
showTool(['post', 'conv', 'plot', 'info'].includes(initialTool) ? initialTool : 'post');

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
