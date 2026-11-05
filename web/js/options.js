// Post-process options <-> flat TOML that `gnss solve --config` and
// `gnss ppp --config` read (apps/native/cli_toml_config.cpp): one table per
// tool, key = option name in snake_case, `true` for a bare flag, `false` for
// the tool's --no-* form, arrays only where the tool accepts them. Input and
// output files are not written, as RTKPOST's options file omits them.

import { IMU_FORMATS, POST_MODES, RTK_IONO, RTK_PRESETS, splitArgs } from './commands.js';

// GUI option -> CLI key per tool. Keys follow the CLI flag names.
const SOLVE_KEYS = { preset: 'preset', iono: 'iono', ratio: 'ratio', elevationMask: 'elevation_mask_deg', maxEpochs: 'max_epochs' };
const PPP_KEYS = { elevationMask: 'elevation_mask', maxEpochs: 'max_epochs' };
const FUSE_KEYS = { preset: 'preset', ratio: 'ratio', elevationMask: 'elevation_mask_deg', maxEpochs: 'max_epochs' };
const TOOL_KEYS = { gnss_solve: SOLVE_KEYS, gnss_ppp: PPP_KEYS, gnss_fuse: FUSE_KEYS };
const DISPATCH = { gnss_solve: 'solve', gnss_ppp: 'ppp', gnss_fuse: 'fuse' };
// `false` values each tool's TOML schema turns into a negative flag.
const NEGATIVE_FLAG = { enable_ar: 'no-ar', enable_wide_lane_ar: 'no-wide-lane-ar' };
// Keys naming input/output files: skipped on export, matched to Files on import.
const FILE_KEYS = ['rover', 'base', 'nav', 'obs', 'sp3', 'clk', 'imu', 'out', 'rtk_pos_out', 'data_dir', 'kml'];
const NUMERIC_FIELDS = ['ratio', 'elevationMask', 'maxEpochs'];

// Whether the form can hold this value (else it goes to Extra arguments).
function fitsField(field, value) {
  if (typeof value === 'boolean' || Array.isArray(value)) return false;
  if (NUMERIC_FIELDS.includes(field)) return Number.isFinite(Number(value)) && String(value).trim() !== '';
  if (field === 'preset') return RTK_PRESETS.includes(value);
  if (field === 'iono') return RTK_IONO.includes(value);
  return true;
}

function tomlValue(value) {
  if (Array.isArray(value)) return `[${value.map(tomlValue).join(', ')}]`;
  if (typeof value === 'boolean') return String(value);
  const text = String(value);
  if (/^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(text)) return text;
  return `"${text.replace(/\\/g, '\\\\').replace(/"/g, '\\"')}"`;
}

function hasValue(v) {
  return v !== undefined && v !== null && String(v).trim() !== '';
}

/** `--a 1 --b --c x y z` -> [['a', '1'], ['b', true], ['c', ['x', 'y', 'z']]]. */
export function extraArgsToEntries(text) {
  const tokens = splitArgs(text || '');
  const entries = [];
  for (let i = 0; i < tokens.length; i += 1) {
    const token = tokens[i];
    if (!token.startsWith('--') || token.length < 3) {
      throw new Error(`extra argument "${token}" does not follow an --option`);
    }
    const values = [];
    while (i + 1 < tokens.length && !tokens[i + 1].startsWith('--')) values.push(tokens[(i += 1)]);
    const key = token.slice(2).replace(/-/g, '_');
    entries.push([key, values.length === 0 ? true : values.length === 1 ? values[0] : values]);
  }
  return entries;
}

/**
 * TOML text for the current Post-process options (see postOptions() in
 * app.js), or throws for a mode whose tool has no --config (SPP).
 */
export function exportOptionsToml(opts, commandHint = '') {
  const mode = POST_MODES[opts.mode];
  if (!mode) throw new Error(`unknown positioning mode: ${opts.mode}`);
  if (mode.tool === 'gnss_spp') throw new Error('gnss spp has no --config; options files cover the RTK, RTK + IMU and PPP modes');
  const entries = [];
  if (mode.tool === 'gnss_fuse') {
    const arm = (opts.leverArm || []).map((v) => (hasValue(v) ? String(v) : '0'));
    if (arm.length === 3 && arm.some((v) => Number(v) !== 0)) entries.push(['lever_arm', arm]);
    if (opts.tightCoupling) entries.push(['navi776_tc', true]);
    if (opts.imuFormat && opts.imuFormat !== 'ppc') entries.push(['imu_format', opts.imuFormat]);
    for (const [field, key] of Object.entries(FUSE_KEYS)) {
      if (hasValue(opts[field])) entries.push([key, opts[field]]);
    }
    if (opts.baseEcef) entries.push(['base_ecef', opts.baseEcef.map(String)]);
  } else if (mode.tool === 'gnss_solve') {
    entries.push(['mode', mode.rtkMode]);
    for (const [field, key] of Object.entries(SOLVE_KEYS)) {
      if (field === 'iono' && opts.iono === 'auto') continue;
      if (hasValue(opts[field])) entries.push([key, opts[field]]);
    }
    if (opts.baseEcef) entries.push(['base_ecef', opts.baseEcef.map(String)]);
    if (!opts.kml) entries.push(['kml', false]);
    if (opts.verbose) entries.push(['verbose', true]);
  } else {
    entries.push(['kinematic', mode.pppMotion === '--kinematic']);
    for (const [field, key] of Object.entries(PPP_KEYS)) {
      if (hasValue(opts[field])) entries.push([key, opts[field]]);
    }
    if (opts.enableAr) entries.push(['enable_ar', true]);
  }
  entries.push(...extraArgsToEntries(opts.extra));
  const lines = [
    `# libgnss++ options (${mode.label}), written by the libgnss++ browser GUI.`,
    `# Use: gnss ${DISPATCH[mode.tool]} --config <this file> <input/output options>`,
  ];
  if (commandHint) lines.push(`# Run it came from: ${commandHint}`);
  lines.push('', `[${mode.tool}]`);
  for (const [key, value] of entries) lines.push(`${key} = ${tomlValue(value)}`);
  return `${lines.join('\n')}\n`;
}

function stripComment(line) {
  let quote = null;
  for (let i = 0; i < line.length; i += 1) {
    const ch = line[i];
    if (quote) {
      if (ch === '\\' && quote === '"') i += 1;
      else if (ch === quote) quote = null;
    } else if (ch === '"' || ch === "'") quote = ch;
    else if (ch === '#') return line.slice(0, i);
  }
  return line;
}

function parseScalar(raw) {
  const v = raw.trim();
  if (v === 'true' || v === 'false') return v === 'true';
  if (v.startsWith("'") && v.endsWith("'") && v.length >= 2) return v.slice(1, -1);
  if (v.startsWith('"') && v.endsWith('"') && v.length >= 2) {
    return v.slice(1, -1).replace(/\\(["\\nrt])/g, (_, c) => ({ n: '\n', r: '\r', t: '\t' }[c] || c));
  }
  return v;
}

function parseArray(raw) {
  const inner = raw.trim().slice(1, -1);
  const items = [];
  let cur = '';
  let quote = null;
  for (const ch of inner) {
    if (quote) { if (ch === quote) quote = null; cur += ch; continue; }
    if (ch === '"' || ch === "'") { quote = ch; cur += ch; continue; }
    if (ch === ',') { items.push(parseScalar(cur)); cur = ''; continue; }
    cur += ch;
  }
  if (cur.trim()) items.push(parseScalar(cur));
  return items;
}

/** Flat TOML -> { sections: { name: [[key, value], ...] } } ('' = root). */
export function parseFlatToml(text) {
  const sections = { '': [] };
  let section = '';
  text.split(/\r?\n/).forEach((rawLine, index) => {
    const line = stripComment(rawLine).trim();
    if (!line) return;
    if (line.startsWith('[')) {
      if (!line.endsWith(']')) throw new Error(`line ${index + 1}: malformed table header`);
      section = line.slice(1, -1).trim();
      sections[section] = sections[section] || [];
      return;
    }
    const eq = line.indexOf('=');
    if (eq <= 0) throw new Error(`line ${index + 1}: expected key = value`);
    const key = line.slice(0, eq).trim().replace(/^["']|["']$/g, '').replace(/-/g, '_');
    const value = line.slice(eq + 1).trim();
    sections[section].push([key, value.startsWith('[') && value.endsWith(']') ? parseArray(value) : parseScalar(value)]);
  });
  return { sections };
}

function entryToArgs(key, value) {
  const flag = `--${key.replace(/_/g, '-')}`;
  if (value === true) return [flag];
  if (value === false) return [`--${NEGATIVE_FLAG[key] || `no-${key.replace(/_/g, '-')}`}`];
  if (Array.isArray(value)) return [flag, ...value.map(String)];
  return [flag, String(value)];
}

function quoteArg(arg) {
  return /^[\w@%+=:,./-]+$/.test(arg) ? arg : `"${arg.replace(/["\\]/g, '\\$&')}"`;
}

/**
 * Options file text -> form settings.
 * Returns { mode, fields: { name: value }, files: { role: basename }, extra, notes }.
 * fields use the Post-process form's input names; files are basenames of
 * rover/base/nav/... keys for the caller to match against Files.
 */
export function importOptionsToml(text) {
  const { sections } = parseFlatToml(text);
  const tool = ['gnss_solve', 'gnss_ppp', 'gnss_fuse'].find((name) => sections[name]) || null;
  const entries = tool ? sections[tool] : sections[''];
  const isPpp = tool === 'gnss_ppp' || (!tool && entries.some(([k]) => ['kinematic', 'enable_ar', 'sp3', 'clk'].includes(k)));
  const isFuse = tool === 'gnss_fuse' || (!tool && entries.some(([k]) => ['imu', 'lever_arm', 'navi776_tc'].includes(k)));
  const fields = {};
  const files = {};
  const extra = [];
  const notes = [];
  let mode = isFuse ? 'rtk-imu' : isPpp ? 'ppp-static' : 'kinematic';
  const keys = isFuse ? FUSE_KEYS : isPpp ? PPP_KEYS : SOLVE_KEYS;
  const keyToField = Object.fromEntries(Object.entries(keys).map(([f, k]) => [k, f]));
  fields.kml = true;
  fields.verbose = false;
  fields.enableAr = false;
  fields.basePos = 'header';
  if (isFuse) Object.assign(fields, { tightCoupling: false, imuFormat: 'ppc', leverX: '', leverY: '', leverZ: '' });
  for (const [key, value] of entries) {
    if (FILE_KEYS.includes(key) && typeof value === 'string') {
      files[key === 'obs' ? 'rover' : key] = value.split(/[\\/]/).pop();
      continue;
    }
    if (isFuse && key === 'lever_arm') {
      const arm = Array.isArray(value) ? value : String(value).split(',');
      if (arm.length === 3) [fields.leverX, fields.leverY, fields.leverZ] = arm.map((v) => String(v).trim());
      else extra.push(...entryToArgs(key, value));
    } else if (isFuse && key === 'navi776_tc' && typeof value === 'boolean') {
      fields.tightCoupling = value;
    } else if (isFuse && key === 'imu_format' && IMU_FORMATS.includes(value)) {
      fields.imuFormat = value;
    } else if (!isPpp && !isFuse && key === 'mode') {
      if (['kinematic', 'static', 'moving-base'].includes(value)) mode = value;
      else notes.push(`mode = "${value}" has no GUI equivalent; using kinematic`);
    } else if (isPpp && key === 'kinematic') {
      mode = value === true ? 'ppp-kinematic' : 'ppp-static';
    } else if (isPpp && key === 'static' && value === true) {
      mode = 'ppp-static';
    } else if (key in keyToField && fitsField(keyToField[key], value)) {
      fields[keyToField[key]] = String(value);
    } else if (!isPpp && key === 'base_ecef' && Array.isArray(value) && value.length === 3) {
      fields.basePos = 'ecef';
      [fields.baseX, fields.baseY, fields.baseZ] = value.map(String);
    } else if (key === 'kml' && typeof value === 'boolean') {
      fields.kml = value;
    } else if (!isPpp && key === 'verbose' && value === true) {
      fields.verbose = true;
    } else if (isPpp && key === 'enable_ar' && typeof value === 'boolean') {
      fields.enableAr = value;
    } else {
      extra.push(...entryToArgs(key, value));
    }
  }
  return { tool: POST_MODES[mode].tool, mode, fields, files, extra: extra.map(quoteArg).join(' '), notes };
}
