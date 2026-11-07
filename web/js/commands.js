// Maps GUI settings to the argv of the native CLI tools. Pure functions,
// shared by the browser app and the Node tests, so a browser run can always
// be repeated with the printed `gnss ...` command.

export const IN_DIR = '/in';
export const OUT_DIR = '/out';

// Native executable -> `gnss` dispatcher sub-command (apps/gnss.py).
export const DISPATCH_NAMES = {
  gnss_spp: 'spp',
  gnss_solve: 'solve',
  gnss_ppp: 'ppp',
  gnss_convert: 'convert',
  gnss_rinex_info: 'rinex-info',
  gnss_visibility: 'visibility',
  gnss_fuse: 'fuse',
  gnss_fgo: 'fgo',
};

export const POST_MODES = {
  single: { tool: 'gnss_spp', label: 'Single (SPP)', needs: ['rover', 'nav'] },
  kinematic: { tool: 'gnss_solve', label: 'Kinematic RTK', rtkMode: 'kinematic', needs: ['rover', 'base', 'nav'] },
  static: { tool: 'gnss_solve', label: 'Static RTK', rtkMode: 'static', needs: ['rover', 'base', 'nav'] },
  'moving-base': { tool: 'gnss_solve', label: 'Moving-base RTK', rtkMode: 'moving-base', needs: ['rover', 'base', 'nav'] },
  'rtk-imu': { tool: 'gnss_fuse', label: 'Kinematic RTK + IMU', needs: ['rover', 'base', 'nav', 'imu'] },
  fgo: { tool: 'gnss_fgo', label: 'Kinematic FGO (batch, experimental)', needs: ['rover', 'base', 'nav'] },
  'ppp-kinematic': { tool: 'gnss_ppp', label: 'PPP Kinematic', pppMotion: '--kinematic', needs: ['rover'] },
  'ppp-static': { tool: 'gnss_ppp', label: 'PPP Static', pppMotion: '--static', needs: ['rover'] },
};

export const RTK_PRESETS = ['survey', 'low-cost', 'moving-base', 'odaiba', 'urban'];
export const RTK_IONO = ['auto', 'off', 'iflc', 'est'];
export const IMU_FORMATS = ['ppc', 'rtklibexplorer-sf'];
export const FGO_PRESETS = ['real-data-float', 'real-data-fixed', 'default'];

// PPP correction sources: broadcast / precise products (NAV, SP3, CLK),
// Galileo HAS from the internet distribution (IDD, RTCM 3 SSR) or from E6-B
// pages logged by a receiver (docs/galileo_has.md), BeiDou PPP-B2b from the
// B2b messages a receiver logged (docs/bds_ppp_b2b.md), QZSS MADOCA-PPP from
// raw L6E (+ L6D ionosphere) logs, or QZSS CLAS from a raw L6 log
// (experimental, docs/use_cases/clas_submeter.md).
export const PPP_CORRECTIONS = ['products', 'has-idd', 'has-sis', 'b2b', 'madoca', 'clas'];
const CORRECTION_NAMES = {
  'has-idd': 'Galileo HAS', 'has-sis': 'Galileo HAS', b2b: 'BeiDou PPP-B2b', madoca: 'MADOCA-PPP', clas: 'CLAS',
};
// gnss ppp --has-pages-format for each page log kind (formats.js sniffKind).
export const HAS_PAGE_FORMATS = { has: 'cssrlib', ubx: 'ubx', sbf: 'sbf' };
// MADOCA PPP-AR as MADOCALIB's pppar sample and the CI MADOCA release matrix
// (scripts/ci/run_madoca_release_matrix.py) run it.
export const MADOCA_AR = {
  ratio: 1.5,
  convergence: ['--convergence-policy', 'local-enu',
    '--convergence-threshold-horizontal', '0.75', '--convergence-threshold-vertical', '1.25'],
};

/** Shell-like split: whitespace separated, '...' and "..." group, \ escapes. */
export function splitArgs(text) {
  const out = [];
  let cur = '';
  let quote = null;
  let any = false;
  for (let i = 0; i < (text || '').length; i += 1) {
    const ch = text[i];
    if (quote) {
      if (ch === quote) quote = null;
      else if (ch === '\\' && quote === '"' && i + 1 < text.length) cur += text[++i];
      else cur += ch;
    } else if (ch === '"' || ch === "'") {
      quote = ch; any = true;
    } else if (ch === '\\' && i + 1 < text.length) {
      cur += text[++i]; any = true;
    } else if (/\s/.test(ch)) {
      if (any || cur) out.push(cur);
      cur = ''; any = false;
    } else {
      cur += ch; any = true;
    }
  }
  if (quote) throw new Error('unterminated quote in extra arguments');
  if (any || cur) out.push(cur);
  return out;
}

function quoteForShell(arg) {
  return /^[\w@%+=:,./-]+$/.test(arg) ? arg : `'${arg.replace(/'/g, "'\\''")}'`;
}

function finite(value) {
  if (value === undefined || value === null || String(value).trim() === '') return null;
  const n = Number(value);
  if (!Number.isFinite(n)) throw new Error(`not a number: ${value}`);
  return n;
}

/**
 * Small builder that keeps the wasm argv (absolute /in, /out paths) and the
 * display argv (bare file names) in step.
 */
class Command {
  constructor(tool) {
    this.tool = tool;
    this.args = [];
    this.display = [];
    this.inputs = [];   // workspace file names to mount under IN_DIR
    this.outputs = [];  // file names the run is expected to write under OUT_DIR
  }

  flag(...tokens) {
    this.args.push(...tokens.map(String));
    this.display.push(...tokens.map(String));
    return this;
  }

  input(option, name) {
    if (!this.inputs.includes(name)) this.inputs.push(name);
    if (option) this.flag(option);
    this.args.push(`${IN_DIR}/${name}`);
    this.display.push(name);
    return this;
  }

  output(option, name) {
    this.outputs.push(name);
    if (option) this.flag(option);
    this.args.push(`${OUT_DIR}/${name}`);
    this.display.push(name);
    return this;
  }

  /** Free-form extra arguments; a token equal to a workspace file name becomes an input. */
  extra(text, workspaceNames = []) {
    for (const token of splitArgs(text)) {
      if (workspaceNames.includes(token)) this.input(null, token);
      else this.flag(token);
    }
    return this;
  }

  result() {
    const dispatch = DISPATCH_NAMES[this.tool] || this.tool;
    return {
      tool: this.tool,
      args: this.args,
      inputs: this.inputs,
      outputs: this.outputs,
      commandLine: ['gnss', dispatch, ...this.display].map(quoteForShell).join(' '),
    };
  }
}

function stem(name) {
  return name.replace(/\.[^./]*$/, '') || name;
}

/** Default solution file name for a rover observation file. */
export function defaultPosName(rover, mode, corrections) {
  let suffix = { single: 'spp', 'ppp-kinematic': 'ppp', 'ppp-static': 'ppp', 'rtk-imu': 'fused', fgo: 'fgo' }[mode] || 'rtk';
  if (suffix === 'ppp' && (corrections === 'madoca' || corrections === 'clas')) suffix = corrections;
  if (suffix === 'ppp' && (corrections === 'has-idd' || corrections === 'has-sis')) suffix = 'has';
  if (suffix === 'ppp' && corrections === 'b2b') suffix = 'b2b';
  return `${stem(rover || 'solution')}_${suffix}.pos`;
}

/**
 * Post-processing (RTKPOST equivalent).
 * opts: { mode, rover, base, nav, sp3, clk, out, elevationMask, maxEpochs,
 *         preset, iono, ratio, baseEcef: [x,y,z] | null, kml, verbose,
 *         ionoFree, enableAr, imu, leverArm: [x,y,z] | null, tightCoupling,
 *         imuFormat, fgoPreset, corrections, l6e: [names], l6d: [names],
 *         clasL6, hasIdd, hasPages, hasPagesFormat, b2bLog, antex, extra, workspaceNames }
 */
export function buildPostCommand(opts) {
  const mode = POST_MODES[opts.mode];
  if (!mode) throw new Error(`unknown positioning mode: ${opts.mode}`);
  for (const role of mode.needs) {
    const what = { nav: 'a navigation', imu: 'an IMU (CSV)' }[role] || `a ${role}`;
    if (!opts[role]) throw new Error(`${mode.label} needs ${what} file`);
  }
  const corrections = mode.tool === 'gnss_ppp' ? opts.corrections || 'products' : 'products';
  if (!PPP_CORRECTIONS.includes(corrections)) throw new Error(`unknown PPP corrections: ${corrections}`);
  const l6e = opts.l6e || [];
  const l6d = opts.l6d || [];
  if (mode.tool === 'gnss_ppp') {
    if (corrections === 'products' && !opts.nav && !(opts.sp3 && opts.clk)) {
      throw new Error('PPP needs a navigation file or both SP3 and CLK products');
    }
    if (corrections !== 'products' && !opts.nav) {
      throw new Error(`${CORRECTION_NAMES[corrections]} needs a navigation file (broadcast ephemeris)`);
    }
    if (corrections === 'madoca' && l6e.length === 0) throw new Error('MADOCA-PPP needs at least one L6E file');
    if (corrections === 'clas' && !opts.clasL6) throw new Error('CLAS needs an L6 file');
    if (corrections === 'has-idd' && !opts.hasIdd) throw new Error('Galileo HAS (IDD) needs an RTCM 3 SSR file');
    if (corrections === 'has-sis' && !opts.hasPages) throw new Error('Galileo HAS (E6-B) needs a page log');
    if (corrections === 'has-sis' && !Object.values(HAS_PAGE_FORMATS).includes(opts.hasPagesFormat || 'cssrlib')) {
      throw new Error(`unknown HAS page log format: ${opts.hasPagesFormat}`);
    }
    if (corrections === 'b2b' && !opts.b2bLog) throw new Error('BeiDou PPP-B2b needs a B2b message log');
  }
  const out = opts.out || defaultPosName(opts.rover, opts.mode, corrections);
  const elevation = finite(opts.elevationMask);
  const maxEpochs = finite(opts.maxEpochs);
  const cmd = new Command(mode.tool);

  if (mode.tool === 'gnss_spp') {
    cmd.input('--obs', opts.rover).input('--nav', opts.nav);
    if (opts.sp3) cmd.input('--sp3', opts.sp3);
    if (opts.clk) cmd.input('--clk', opts.clk);
    cmd.output('--out', out);
    if (elevation !== null) cmd.flag('--elevation-mask-deg', elevation);
    if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
    if (opts.ionoFree) cmd.flag('--ionosphere-free');
    cmd.output('--summary-json', `${stem(out)}_summary.json`);
  } else if (mode.tool === 'gnss_solve') {
    cmd.input('--rover', opts.rover).input('--base', opts.base).input('--nav', opts.nav);
    cmd.output('--out', out);
    cmd.flag('--mode', mode.rtkMode);
    if (opts.preset) cmd.flag('--preset', opts.preset);
    if (opts.iono && opts.iono !== 'auto') cmd.flag('--iono', opts.iono);
    const ratio = finite(opts.ratio);
    if (ratio !== null) cmd.flag('--ratio', ratio);
    if (elevation !== null) cmd.flag('--elevation-mask-deg', elevation);
    if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
    if (opts.baseEcef) {
      const xyz = opts.baseEcef.map(finite);
      if (xyz.length !== 3 || xyz.some((v) => v === null)) throw new Error('base ECEF needs X, Y and Z');
      cmd.flag('--base-ecef', ...xyz);
    }
    if (opts.kml) cmd.output('--kml', `${stem(out)}.kml`);
    else cmd.flag('--no-kml');
    if (opts.verbose) cmd.flag('--verbose');
  } else if (mode.tool === 'gnss_fuse') {
    // GNSS/IMU fusion: the fused trajectory (--out) plus the RTK solution the
    // fusion ran on (--rtk-pos-out), which --navi776-tc refines with the IMU.
    cmd.input('--rover', opts.rover).input('--base', opts.base).input('--nav', opts.nav).input('--imu', opts.imu);
    cmd.output('--out', out);
    cmd.output('--rtk-pos-out', `${stem(out)}_rtk.pos`);
    if (opts.imuFormat && opts.imuFormat !== 'ppc') cmd.flag('--imu-format', opts.imuFormat);
    if (opts.leverArm) {
      const arm = opts.leverArm.map((v) => finite(v) ?? 0);
      if (arm.some((v) => v !== 0)) cmd.flag('--lever-arm', arm.join(','));
    }
    if (opts.tightCoupling) cmd.flag('--navi776-tc');
    if (opts.preset) cmd.flag('--preset', opts.preset);
    const ratio = finite(opts.ratio);
    if (ratio !== null) cmd.flag('--ratio', ratio);
    if (elevation !== null) cmd.flag('--elevation-mask-deg', elevation);
    if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
    if (opts.baseEcef) {
      const xyz = opts.baseEcef.map(finite);
      if (xyz.length !== 3 || xyz.some((v) => v === null)) throw new Error('base ECEF needs X, Y and Z');
      cmd.flag('--base-ecef', ...xyz);
    }
    if (opts.kml) cmd.output('--kml', `${stem(out)}.kml`);
  } else if (mode.tool === 'gnss_fgo') {
    // Batch factor graph over the whole (or --max-epochs) span: code, TDCP
    // and DD carrier factors; real-data-fixed adds LAMBDA ambiguity fixing.
    cmd.input('--obs', opts.rover).input('--base', opts.base).input('--nav', opts.nav);
    cmd.output('--out', out);
    const preset = opts.fgoPreset || 'real-data-float';
    cmd.flag('--preset', preset);
    if (preset === 'real-data-fixed') {
      // The batch fix alone labels every epoch FIXED while the positions stay
      // float-level (docs/ppc_rtk_vs_gnss_imu_fusion.md, carrier-only Eigen
      // caveat): take the per-epoch conditional LAMBDA output instead, and
      // never report the batch label.
      cmd.flag('--epoch-lambda-fixed-output').flag('--report-batch-fix-as-float');
    }
    const ratio = finite(opts.ratio);
    if (ratio !== null) cmd.flag('--lambda-ratio-threshold', ratio);
    if (elevation !== null) cmd.flag('--min-elevation', elevation);
    if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
    cmd.output('--summary-json', `${stem(out)}_summary.json`);
  } else {
    cmd.input('--obs', opts.rover);
    if (opts.nav) cmd.input('--nav', opts.nav);
    const ratio = finite(opts.ratio);
    if (corrections === 'products') {
      if (opts.sp3) cmd.input('--sp3', opts.sp3);
      if (opts.clk) cmd.input('--clk', opts.clk);
    } else if (corrections === 'madoca') {
      for (const name of l6e) cmd.input('--madoca-l6', name);
      for (const name of l6d) cmd.input('--madoca-l6d', name);
    } else if (corrections === 'has-idd') {
      // RTCM 10403.3 semantics: IODE-matched orbits, ephemerides from the
      // stream, Galileo I/NAV only, held code biases (docs/galileo_has.md).
      cmd.input('--ssr-rtcm', opts.hasIdd).flag('--ssr-rtcm-profile', 'has-idd');
    } else if (corrections === 'has-sis') {
      cmd.input('--has-pages', opts.hasPages).flag('--has-pages-format', opts.hasPagesFormat || 'cssrlib');
    } else if (corrections === 'b2b') {
      // The stream of the first BDS GEO in the log; BDS orbits refer to the
      // B-CNAV1 records of the NAV (docs/bds_ppp_b2b.md).
      cmd.input('--b2b', opts.b2bLog);
    } else {
      // A raw L6 log given as --ssr is decoded natively (Compact SSR) for
      // the CLAS OSR PPP-RTK filter.
      cmd.input('--ssr', opts.clasL6).flag('--clas-osr');
    }
    if (opts.antex) cmd.input('--antex', opts.antex);
    cmd.output('--out', out);
    cmd.flag(mode.pppMotion);
    if (elevation !== null) cmd.flag('--elevation-mask', elevation);
    if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
    if (corrections === 'madoca') {
      // L6D ionosphere applies only to the per-frequency (uncombined) filter.
      if (opts.enableAr || l6d.length) cmd.flag('--ar-method', 'per-freq');
      if (opts.enableAr) {
        cmd.flag('--enable-ar', '--ar-ratio-threshold', ratio ?? MADOCA_AR.ratio, ...MADOCA_AR.convergence);
      }
    } else if (corrections === 'products' && opts.enableAr) {
      cmd.flag('--enable-ar');
      if (ratio !== null) cmd.flag('--ar-ratio-threshold', ratio);
    }
    if (opts.kml) cmd.output('--kml', `${stem(out)}.kml`);
    cmd.output('--summary-json', `${stem(out)}_summary.json`);
  }
  cmd.extra(opts.extra, opts.workspaceNames);
  return cmd.result();
}

/**
 * RTCM3 / u-blox UBX -> RINEX (RTKCONV equivalent).
 * opts: { input, format: 'rtcm'|'ubx', gpsWeek, obsOut, navOut, limit, quiet, extra, workspaceNames }
 */
export function buildConvertCommand(opts) {
  if (!opts.input) throw new Error('choose an RTCM3 or UBX input file');
  if (!opts.obsOut && !opts.navOut) throw new Error('enable at least one RINEX output');
  const cmd = new Command('gnss_convert');
  cmd.input('--input', opts.input).flag('--format', opts.format);
  if (opts.format === 'rtcm') {
    const week = finite(opts.gpsWeek);
    if (week !== null) cmd.flag('--gps-week', week);
  }
  if (opts.obsOut) cmd.output('--obs-out', opts.obsOut);
  // UBX: decode RXM-SFRBX into multi-GNSS ephemerides; RTCM: 1019/1020/... messages.
  if (opts.navOut) cmd.output(opts.format === 'ubx' ? '--nav-rinex-out' : '--nav-out', opts.navOut);
  const limit = finite(opts.limit);
  if (limit) cmd.flag('--limit', limit);
  if (opts.quiet) cmd.flag('--quiet');
  cmd.extra(opts.extra, opts.workspaceNames);
  return cmd.result();
}

/** Default RINEX output names for a converter input. */
export function defaultConvertNames(input) {
  const base = stem(input || 'converted');
  return { obsOut: `${base}.obs`, navOut: `${base}.nav` };
}

/**
 * Satellite azimuth / elevation / SNR / LLI per epoch for the observation
 * viewer (RTKPLOT observation / skyplot views).
 * opts: { obs, nav, minElevation, maxEpochs, receiverEcef: [x,y,z] | null, extra, workspaceNames }
 */
export function buildVisibilityCommand(opts) {
  if (!opts.obs) throw new Error('choose a RINEX observation file');
  if (!opts.nav) throw new Error('choose a RINEX navigation file');
  const base = stem(opts.obs);
  const cmd = new Command('gnss_visibility');
  cmd.input('--obs', opts.obs).input('--nav', opts.nav);
  cmd.output('--csv', `${base}_vis.csv`).output('--summary-json', `${base}_vis_summary.json`);
  const minElevation = finite(opts.minElevation);
  if (minElevation !== null) cmd.flag('--min-elevation-deg', minElevation);
  const maxEpochs = finite(opts.maxEpochs);
  if (maxEpochs) cmd.flag('--max-epochs', maxEpochs);
  if (opts.receiverEcef) {
    const xyz = opts.receiverEcef.map(finite);
    if (xyz.length !== 3 || xyz.some((v) => v === null)) throw new Error('receiver ECEF needs X, Y and Z');
    cmd.flag('--receiver-ecef', ...xyz);
  }
  cmd.flag('--quiet');
  cmd.extra(opts.extra, opts.workspaceNames);
  return cmd.result();
}

/** RINEX header / record summary. opts: { files: [names], countRecords } */
export function buildInfoCommand(opts) {
  if (!opts.files || opts.files.length === 0) throw new Error('choose at least one RINEX file');
  const cmd = new Command('gnss_rinex_info');
  if (opts.countRecords) cmd.flag('--count-records');
  for (const name of opts.files) cmd.input(null, name);
  return cmd.result();
}
