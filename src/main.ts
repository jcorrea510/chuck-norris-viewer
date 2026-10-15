import './style.css';
import { registerSW } from 'virtual:pwa-register';
import { Viewer, type Insets, type FrameSpec, type PickHit } from './viewer';
import { UI, type MechView, type ItemView } from './ui';
import { robot, subsystems, subsystemById, MODEL_URL, PARTS_URL } from './config';
import { kindLabel } from './parts';
import { knownFor } from './knownparts';
import { sound } from './sound';
import { ACCENTS, defaults, loadSettings, saveSettings, isLight, type Settings } from './settings';
import type { Simulation } from './sim';

// ------------------------------------------------------------------ iPad Safari guards
for (const t of ['gesturestart', 'gesturechange', 'gestureend']) document.addEventListener(t, (e) => e.preventDefault(), { passive: false });
document.addEventListener('dblclick', (e) => e.preventDefault(), { passive: false });
document.addEventListener('touchmove', (e) => {
  const scroller = (e.target as Element | null)?.closest?.('.panel-body, .chips, .results, .sheet-body, .tray, input[type=range]');
  if (!scroller || e.touches.length > 1) e.preventDefault();
}, { passive: false });
document.addEventListener('contextmenu', (e) => e.preventDefault());

const params = new URLSearchParams(location.search);
const DEBUG = params.get('debug') === '1';
const DEG = Math.PI / 180;
const reducedMotionQuery = matchMedia('(prefers-reduced-motion: reduce)');

// ------------------------------------------------------------------ state
type Selection =
  | { kind: 'sub'; id: string }
  | { kind: 'node'; nodes: number[]; label: string }
  | null;
type Hit = PickHit | null;

const state = {
  sel: null as Selection,
  exploded: false,
  sim: null as Simulation | null,
  lastInteraction: -Infinity,
  interacted: false,
  ready: false,
  autoSpin: true, // tests switch these two off
  idleReset: true,
  doubleTapMs: 260, // the time window for a double tap (tests on slow software rendering raise it)
  hiddenStack: [] as { pids: number[]; label: string }[],
  autoHidden: [] as number[],
};

let settings: Settings = loadSettings(defaults(robot.secondsPerRevolution, robot.idleResetSeconds));

const canvas = document.getElementById('stage') as HTMLCanvasElement;
const viewer = new Viewer(canvas);
viewer.reducedMotion = reducedMotionQuery.matches;
reducedMotionQuery.addEventListener('change', () => { viewer.reducedMotion = reducedMotionQuery.matches; });

const ui = new UI({
  onChip: (id) => { if (!(state.sel?.kind === 'sub' && state.sel.id === id)) selectSubsystem(id); },
  onPin: (id) => selectSubsystem(id),
  onHome: () => resetAll(),
  onMove: () => toggleTray(),
  onExplode: () => setExploded(!state.exploded),
  onSimulate: () => void enterSim(),
  onBack: () => goBack(),
  onPrev: () => stepSubsystem(-1),
  onNext: () => stepSubsystem(1),
  onCrumb: (node) => selectNodes([node]),
  onHide: () => hideSelection(),
  onIsolate: () => isolateSelection(),
  onUp: () => goUp(),
  onShowAll: () => showAll(),
  onUndoHide: () => undoHide(),
  onMechanism: (id, deg, commit) => setMechanism(id, deg, commit),
  onSearchOpen: (open) => { if (open) { touch(); sound.play('tap'); } },
  onSearch: (q) => ui.showResults(viewer.parts?.search(q) ?? [], q),
  onResult: (r) => { ui.openSearch(false); selectNodes(r.nodes, r.count > 1 ? `${r.name} ×${r.count}` : r.name); },
  onFullscreen: () => void toggleFullscreen(),
  onSettings: (patch) => applySettings({ ...settings, ...patch }),
  onSettingsOpen: (open) => { if (open) { touch(); sound.play('select'); } },
  onResetSettings: () => { applySettings(defaults(robot.secondsPerRevolution, robot.idleResetSeconds)); ui.renderSettings(settings); },
  onSimView: () => { state.sim?.setView(state.sim.view === 'follow' ? 'field' : 'follow'); sound.play('tap'); },
  onSimRestart: () => state.sim?.restart(),
  onSimExit: () => exitSim(),
});

// ------------------------------------------------------------------ settings
function applySettings(s: Settings) {
  settings = s;
  saveSettings(s);
  const light = isLight(s);
  document.documentElement.dataset.theme = light ? 'light' : 'dark';
  document.querySelector('meta[name=theme-color]')?.setAttribute('content', light ? '#eef0f4' : '#060708');
  const a = ACCENTS[s.accent];
  document.documentElement.style.setProperty('--accent', a.fill);
  document.documentElement.style.setProperty('--accent-ink', light ? a.fill : a.ink);
  viewer.setTheme(light);
  viewer.setAccent(light ? a.fill : a.ink);
  viewer.setGhostStrength(s.hologram);
  viewer.spinPeriod = s.spinSeconds;
  if (viewer.quality !== s.quality) viewer.setQuality(s.quality);
  sound.enabled = s.sound;
  sound.setVolume(s.volume);
  if (!s.stats && !DEBUG) ui.debug(null);
  viewer.invalidate();
}
matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => { if (settings.appearance === 'system') applySettings(settings); });

// ------------------------------------------------------------------ framing insets
const W = () => ui.app.clientWidth;
const H = () => ui.app.clientHeight;
function insets(): Insets {
  const w = W(), h = H();
  const bottom = h - ui.dockTop() + 16;
  // The inspector (or the mechanisms card) occupies the right side in landscape, the bottom in portrait.
  const side = state.sel ? ui.panelBox() : ui.trayBox();
  if (!side) return { top: ui.titleBottom() + 12, right: 32, bottom: bottom + 34, left: 32 };
  const top = state.sel ? 40 : ui.titleBottom() + 12;
  if (ui.isLandscape) return { top, right: w - side.left + 20, bottom, left: 40 };
  return { top, right: 24, bottom: h - side.top + 20, left: 24 };
}

// ------------------------------------------------------------------ selection
function touch() {
  state.lastInteraction = performance.now();
  viewer.spinning = false;
  sound.unlock();
  void requestWakeLock();
}

function mechViews(ids: string[]): MechView[] {
  const out: MechView[] = [];
  for (const id of ids) { const m = viewer.mechanisms.get(id); if (m) out.push({ mech: m, value: viewer.mechanismAngle(id) }); }
  return out;
}

function restoreAutoHidden() {
  if (!state.autoHidden.length) return;
  const userHidden = new Set(state.hiddenStack.flatMap((e) => e.pids));
  viewer.setHidden(state.autoHidden.filter((p) => !userHidden.has(p)), false);
  state.autoHidden = [];
}

const visible = (pids: number[]) => pids.filter((p) => viewer.state.isVisible(p));

function mechanismsForSubsystem(id: string) {
  const P = viewer.parts;
  if (!P) return [];
  return [...viewer.mechanisms.keys()].filter((m) => P.partsOfMechanism(m).some((p) => P.subsystemOf(P.nodeOfPart(p)) === id));
}

function selectSubsystem(id: string, spec?: FrameSpec) {
  if (!state.ready || state.sim) return;
  touch();
  const s = subsystemById.get(id);
  if (!s) return;
  restoreAutoHidden();
  state.sel = { kind: 'sub', id };
  const pids = viewer.subsystemPids(id);
  if (s.hideWhenFocused?.length && viewer.parts) {
    const inSub = new Set(pids);
    state.autoHidden = viewer.parts.partsMatching(s.hideWhenFocused).filter((p) => inSub.has(p) && viewer.state.isVisible(p));
    viewer.setHidden(state.autoHidden, true);
  }
  viewer.setFocus(pids.length ? pids : null);
  const i = subsystems.findIndex((x) => x.id === id);
  ui.showSubsystem(s, i, subsystems.length, mechViews(mechanismsForSubsystem(id)));
  ui.setSelectedChip(id);
  ui.setFocusedClass(true);
  ui.showHint(false);
  state.interacted = true;
  if (pids.length) viewer.frameParts(visible(pids), insets(), spec ?? viewer.subsystemAngle(id));
  sound.play('focus');
}

function selectNodes(nodes: number[], label?: string) {
  const P = viewer.parts;
  if (!state.ready || !P || state.sim || !nodes.length) return;
  touch();
  restoreAutoHidden();
  const pids = [...new Set(nodes.flatMap((n) => Array.from(P.partsOf(n))))];
  const node = nodes[0];
  const name = label ?? P.name(node);
  state.sel = { kind: 'node', nodes, label: name };
  viewer.setFocus(pids);
  const subId = P.subsystemOf(node);
  const sub = subId ? subsystemById.get(subId) : null;
  const known = knownFor(P.name(node));
  const view: ItemView = {
    name,
    kind: nodes.length > 1 ? `${nodes.length} matches · ${kindLabel(P, node)}` : kindLabel(P, node),
    crumbs: nodes.length === 1 ? P.path(node).map((n) => ({ node: n, name: P.name(n) })) : [],
    subsystemName: sub?.name ?? '',
    summary: known?.summary,
    specs: known?.specs ?? [],
    canUp: nodes.length === 1,
    count: pids.length,
  };
  const mechs = [...new Set(nodes.flatMap((n) => P.mechanismsUnder(n)))];
  ui.showItem(view, mechViews(mechs), backLabel());
  ui.setSelectedChip(subId);
  ui.setFocusedClass(true);
  ui.showHint(false);
  state.interacted = true;
  viewer.frameParts(visible(pids), insets(), { azimuth: 'keep', polar: 'keep' });
  sound.play('select');
}

function backLabel() {
  const P = viewer.parts;
  if (!P || state.sel?.kind !== 'node') return 'Overview';
  const node = state.sel.nodes[0];
  const up = state.sel.nodes.length === 1 ? P.up(node) : -1;
  if (up > 0) return P.name(up);
  return subsystemById.get(P.subsystemOf(node) ?? '')?.short ?? 'Overview';
}

function goUp() {
  const P = viewer.parts;
  if (!P || state.sel?.kind !== 'node') return;
  const node = state.sel.nodes[0];
  const up = state.sel.nodes.length === 1 ? P.up(node) : -1;
  if (up > 0) selectNodes([up]);
  else {
    const s = P.subsystemOf(node);
    if (s) selectSubsystem(s, { azimuth: 'keep', polar: 'keep' }); else overview();
  }
}

function goBack() {
  if (state.sel?.kind === 'node') return goUp();
  overview();
}

function overview(resetAngle = false) {
  touch();
  const had = !!state.sel;
  restoreAutoHidden();
  state.sel = null;
  viewer.setFocus(null);
  ui.hidePanel();
  ui.setSelectedChip(null);
  ui.setFocusedClass(false);
  viewer.frameOverview(insets(), true, resetAngle);
  if (had) sound.play('back');
}

function stepSubsystem(dir: number) {
  const cur = state.sel?.kind === 'sub' ? state.sel.id : state.sel?.kind === 'node' ? viewer.parts?.subsystemOf(state.sel.nodes[0]) : null;
  const i = cur ? subsystems.findIndex((s) => s.id === cur) : -1;
  selectSubsystem(subsystems[(i + dir + subsystems.length) % subsystems.length].id);
}

function selectionPids(): number[] {
  const P = viewer.parts;
  if (!state.sel || !P) return [];
  if (state.sel.kind === 'sub') return viewer.subsystemPids(state.sel.id);
  return [...new Set(state.sel.nodes.flatMap((n) => Array.from(P.partsOf(n))))];
}

// ------------------------------------------------------------------ hide / show
function hidePids(pids: number[], label: string) {
  const vis = visible(pids);
  if (!vis.length) return;
  viewer.setHidden(vis, true);
  state.hiddenStack.push({ pids: vis, label });
  ui.setHidden(viewer.hiddenCount(), label);
  ui.toast(`Hidden: ${label}`, { icon: 'eye-off', action: { label: 'Undo', fn: undoHide } });
  sound.play('hide');
}
function hideSelection() {
  if (!state.sel) return;
  const label = state.sel.kind === 'sub' ? subsystemById.get(state.sel.id)!.name : state.sel.label;
  const pids = selectionPids();
  overview();
  hidePids(pids, label);
}
function isolateSelection() {
  const P = viewer.parts;
  if (!P || !state.sel) return;
  const keep = new Set(selectionPids());
  const others: number[] = [];
  for (let p = 1; p <= P.count; p++) if (!keep.has(p)) others.push(p);
  const label = state.sel.kind === 'node' ? state.sel.label : subsystemById.get(state.sel.id)!.name;
  hidePids(others, `everything except ${label}`);
  viewer.frameParts([...keep], insets(), { azimuth: 'keep', polar: 'keep' });
}
function undoHide() {
  const e = state.hiddenStack.pop();
  if (!e) return;
  viewer.setHidden(e.pids, false);
  ui.setHidden(viewer.hiddenCount(), state.hiddenStack.at(-1)?.label ?? null);
  sound.play('show');
  if (!state.sel) viewer.frameOverview(insets(), true);
}
function showAll() {
  state.hiddenStack = [];
  viewer.showAll();
  state.autoHidden = [];
  if (state.sel?.kind === 'sub') selectSubsystem(state.sel.id);
  ui.setHidden(0, null);
  sound.play('show');
  if (!state.sel) viewer.frameOverview(insets(), true);
}

// ------------------------------------------------------------------ mechanisms / explode
function toggleTray() {
  touch();
  if (ui.trayOpen) { ui.showTray(null); sound.play('toggle'); }
  else { ui.showTray(mechViews([...viewer.mechanisms.keys()])); sound.play('select'); }
  setTimeout(() => { if (!state.sel) viewer.frameOverview(insets(), true); }, 30);
}

let underside = false;
function setMechanism(id: string, deg: number, commit: boolean) {
  touch();
  viewer.setMechanism(id, deg, true);
  const m = viewer.mechanisms.get(id);
  if (commit && m?.liftRobot) {
    const open = Math.abs(deg) > 1;
    sound.play('open');
    // The brain pan opens from below: lift the robot and swing the camera underneath.
    if (open) {
      underside = true;
      setTimeout(() => viewer.frameParts(null, insets(), { azimuth: 'keep', polar: 132 * DEG }), 60);
    } else if (underside) {
      underside = false;
      if (state.sel) setTimeout(() => viewer.frameParts(visible(selectionPids()), insets(), { azimuth: 'keep', polar: 62 * DEG }), 60);
      else viewer.frameOverview(insets(), true, false);
    }
  } else if (commit) sound.play('toggle');
}

function setExploded(on: boolean) {
  if (!state.ready || state.sim) return;
  touch();
  state.exploded = on;
  ui.setExploded(on);
  viewer.setExploded(on);
  sound.play(on ? 'open' : 'toggle');
  if (state.sel) viewer.frameParts(visible(selectionPids()), insets(), { azimuth: 'keep', polar: 'keep' });
  else viewer.frameOverview(insets(), true);
}

function resetAll() {
  if (state.exploded) { state.exploded = false; ui.setExploded(false); viewer.setExploded(false); }
  for (const m of viewer.mechanisms.values()) viewer.setMechanism(m.id, m.def, true);
  underside = false;
  if (ui.trayOpen) ui.showTray(null);
  overview(true);
}

// ------------------------------------------------------------------ simulation
async function enterSim() {
  if (!state.ready || state.sim) return;
  touch();
  if (ui.searchOpen) ui.openSearch(false);
  if (ui.trayOpen) ui.showTray(null);
  if (state.sel) overview();
  if (state.exploded) setExploded(false);
  ui.setSimMode(true);
  sound.play('focus');
  try {
    const { Simulation } = await import('./sim');
    const sim = new Simulation(viewer);
    state.sim = sim;
    sim.onStatus = (s) => ui.simStatus(s, sim.view);
    await sim.start();
  } catch (err) {
    console.error(err);
    ui.toast('The field could not be loaded.', { icon: 'x' });
    exitSim();
  }
}
function exitSim() {
  state.sim?.stop();
  state.sim = null;
  ui.setSimMode(false);
  sound.play('back');
  viewer.frameOverview(insets(), true, true);
}

// ------------------------------------------------------------------ canvas taps vs drags
const TAP_MOVE = 8, TAP_MS = 300;
const pointers = new Map<number, { x: number; y: number }>();
let tapCandidate: { id: number; x: number; y: number; t: number; hit: Hit } | null = null;
let pendingTap: { timer: number; x: number; y: number; hit: Hit } | null = null;

function highlightFor(hit: Hit) {
  const P = viewer.parts;
  if (!hit || !P || !hit.pid) return null;
  return Array.from(P.partsOf(P.meaningful(hit.pid)));
}

canvas.addEventListener('pointerdown', (e) => {
  touch();
  pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
  if (state.sim) return;
  if (pointers.size === 1 && e.button === 0) {
    const hit = state.ready ? viewer.pick(e.clientX, e.clientY) : null;
    tapCandidate = { id: e.pointerId, x: e.clientX, y: e.clientY, t: performance.now(), hit };
    viewer.setHighlight(highlightFor(hit)); // instant touch-down feedback
  } else cancelTap();
});
canvas.addEventListener('pointermove', (e) => {
  if (tapCandidate && e.pointerId === tapCandidate.id && Math.hypot(e.clientX - tapCandidate.x, e.clientY - tapCandidate.y) > TAP_MOVE) {
    cancelTap();
    if (!state.interacted) { state.interacted = true; ui.showHint(false); }
  }
  if (pointers.size) state.lastInteraction = performance.now();
});
const endPointer = (e: PointerEvent) => {
  pointers.delete(e.pointerId);
  if (!tapCandidate || e.pointerId !== tapCandidate.id) return;
  const c = tapCandidate;
  cancelTap();
  if (e.type !== 'pointerup' || performance.now() - c.t > TAP_MS) return;
  if (pendingTap && Math.hypot(c.x - pendingTap.x, c.y - pendingTap.y) < 36) {
    clearTimeout(pendingTap.timer); // double tap: hide what's under the finger
    const hit = pendingTap.hit ?? c.hit;
    pendingTap = null;
    if (hit) doubleTap(hit);
    return;
  }
  const hit = c.hit;
  pendingTap = { x: c.x, y: c.y, hit, timer: window.setTimeout(() => { pendingTap = null; singleTap(hit); }, state.doubleTapMs) };
};
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);
canvas.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') endPointer(e); });
function cancelTap() { tapCandidate = null; viewer.setHighlight(null); }
ui.layer.addEventListener('pointerdown', () => touch());

function singleTap(hit: Hit) {
  const P = viewer.parts;
  if (!hit) { if (state.sel) goBack(); return; }
  if (!P || !hit.pid) { if (hit.sub) selectSubsystem(hit.sub); return; }
  const sel = state.sel;
  // tapping inside the current selection climbs one level (part → assembly → subsystem)
  if (sel?.kind === 'node' && sel.nodes.length === 1 && P.partsOf(sel.nodes[0]).includes(hit.pid)) return goUp();
  selectNodes([P.meaningful(hit.pid)]);
}

function doubleTap(hit: PickHit) {
  const P = viewer.parts;
  if (!P || !hit.pid) return;
  const node = P.meaningful(hit.pid);
  if (state.sel?.kind === 'node' && state.sel.nodes.includes(node)) overview();
  hidePids(Array.from(P.partsOf(node)), P.name(node));
  if (state.sel) viewer.frameParts(visible(selectionPids()), insets(), { azimuth: 'keep', polar: 'keep' });
}

// ------------------------------------------------------------------ keyboard (testing on a Mac)
window.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  if ((e.target as HTMLElement).closest?.('input')) return;
  if (ui.settingsOpen && e.key === 'Escape') return ui.openSettings(false);
  if (state.sim) { if (e.key === 'Escape') exitSim(); return; }
  if (e.key === 'ArrowRight' || e.key === 'ArrowDown') { e.preventDefault(); stepSubsystem(1); }
  else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') { e.preventDefault(); stepSubsystem(-1); }
  else if (e.key === 'Escape') { if (ui.searchOpen) ui.openSearch(false); else goBack(); }
  else if (e.key === 'r' || e.key === 'R') resetAll();
  else if (e.key === '/') { e.preventDefault(); ui.openSearch(true); }
  else if (e.key === ',') ui.openSettings(true, settings);
  else if (e.key === 'h' || e.key === 'H') hideSelection();
});

// ------------------------------------------------------------------ auto-spin + idle reset
setInterval(() => {
  if (!state.ready || viewer.revealing || state.sim) return;
  const idle = (performance.now() - state.lastInteraction) / 1000;
  const anyChange = !!state.sel || state.exploded || viewer.hiddenCount() > 0 || viewer.lifted || ui.trayOpen ||
    [...viewer.mechanisms.values()].some((m) => Math.abs(viewer.mechanismAngle(m.id) - m.def) > 0.5);
  if (state.idleReset && settings.idleReset > 0 && idle > settings.idleReset && anyChange && !ui.settingsOpen) {
    showAll();
    resetAll();
    state.lastInteraction = performance.now() - settings.spinSeconds * 1000;
    state.interacted = false;
    ui.showHint(true);
    return;
  }
  const spin = state.autoSpin && settings.autoSpin && !state.sel && pointers.size === 0 && idle > robot.autoSpinResumeSeconds && !viewer.reducedMotion && !viewer.lifted;
  if (spin !== viewer.spinning) viewer.spinning = spin;
}, 250);

// ------------------------------------------------------------------ resize (rotation, Split View, Stage Manager)
let resizeRaf = 0;
function onResize() {
  cancelAnimationFrame(resizeRaf);
  resizeRaf = requestAnimationFrame(() => {
    viewer.resize(W(), H());
    ui.layout(W(), H());
    if (!state.ready || state.sim) return;
    if (state.sel) viewer.frameParts(visible(selectionPids()), insets(), { azimuth: 'keep', polar: 'keep' });
    else viewer.frameOverview(insets(), true);
  });
}
new ResizeObserver(onResize).observe(ui.app);
viewer.resize(W(), H());
ui.layout(W(), H());

// ------------------------------------------------------------------ per-frame UI
let lastSync = 0;
viewer.onRender = () => {
  ui.updatePins(viewer.pins(), state.ready && !state.sel && !viewer.revealing && settings.labels && !state.sim && !viewer.lifted);
  const label = state.sel ? (state.sel.kind === 'sub' ? subsystemById.get(state.sel.id)!.name : state.sel.label) : null;
  ui.updateBrackets(state.sim ? null : viewer.focusRect(), label);
  const now = performance.now();
  if (now - lastSync > 80) {
    lastSync = now;
    const vals = new Map<string, number>();
    let speed = 0;
    for (const m of viewer.mechanisms.values()) {
      vals.set(m.id, viewer.mechanismValue(m.id));
      speed = Math.max(speed, Math.abs(m.angle.vel) / (180 * DEG));
    }
    if (!state.sim) { ui.syncMechs(vals); sound.servoSpeed(speed); } else sound.servoSpeed(0);
  }
  if (DEBUG || settings.stats) {
    const s = viewer.stats, m = viewer.modelStats();
    ui.debug(`${s.fps.toFixed(0)} fps\n${s.calls} draw calls\n${(s.triangles / 1e6).toFixed(2)}M tris drawn\n${(m.triangles / 1e6).toFixed(2)}M tris model\n${m.parts} parts · ${m.meshes} meshes`);
  }
};
viewer.onContext = (lost) => ui.restoring(lost);

// ------------------------------------------------------------------ full screen + wake lock
type FsEl = HTMLElement & { webkitRequestFullscreen?: () => Promise<void> | void; webkitRequestFullScreen?: () => void };
const docEl = document.documentElement as FsEl;
const doc = document as Document & { webkitFullscreenElement?: Element; webkitExitFullscreen?: () => void; webkitFullscreenEnabled?: boolean };
const standalone = matchMedia('(display-mode: standalone), (display-mode: fullscreen)').matches || (navigator as Navigator & { standalone?: boolean }).standalone === true;
const fsApi = !!(document.fullscreenEnabled || doc.webkitFullscreenEnabled) && !!(docEl.requestFullscreen || docEl.webkitRequestFullscreen || docEl.webkitRequestFullScreen);
const isFs = () => !!(document.fullscreenElement || doc.webkitFullscreenElement);
function syncFs() { ui.setFullscreenState(!standalone, isFs()); }
async function toggleFullscreen() {
  touch();
  sound.play('tap');
  try {
    if (isFs()) {
      if (document.exitFullscreen) await document.exitFullscreen(); else doc.webkitExitFullscreen?.();
      return;
    }
    if (fsApi) {
      if (docEl.requestFullscreen) await docEl.requestFullscreen({ navigationUI: 'hide' });
      else if (docEl.webkitRequestFullscreen) await docEl.webkitRequestFullscreen();
      else docEl.webkitRequestFullScreen?.();
      setTimeout(() => { if (!isFs()) fullscreenHelp(); }, 450);
      return;
    }
  } catch (err) {
    console.info('[fullscreen] not allowed here', err);
  }
  fullscreenHelp();
}
function fullscreenHelp() {
  // Safari on iPad only allows true full screen for pages opened from the Home Screen.
  ui.sheet('Full Screen', `
    <p class="lead">For edge-to-edge full screen on iPad, add this page to your Home Screen. It then opens like an app, with no browser bars, and works offline.</p>
    <ol class="steps"><li>Tap the <b>Share</b> button in Safari’s toolbar.</li><li>Choose <b>Add to Home Screen</b>, then <b>Add</b>.</li><li>Open <b>Chuck Norris</b> from your Home Screen.</li></ol>`);
}
document.addEventListener('fullscreenchange', syncFs);
document.addEventListener('webkitfullscreenchange', syncFs);
syncFs();

let wakeLock: { release(): Promise<void> } | null = null;
async function requestWakeLock() {
  const wl = (navigator as Navigator & { wakeLock?: { request(t: 'screen'): Promise<{ release(): Promise<void>; addEventListener(t: string, f: () => void): void }> } }).wakeLock;
  if (!wl || wakeLock || document.visibilityState !== 'visible') return;
  try {
    const lock = await wl.request('screen');
    wakeLock = lock;
    lock.addEventListener('release', () => { wakeLock = null; });
  } catch { /* needs a user gesture on some versions; retried on the next touch */ }
}
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible') { void requestWakeLock(); viewer.invalidate(); }
});

// ------------------------------------------------------------------ offline (PWA)
registerSW({
  immediate: true,
  onOfflineReady() {
    try {
      if (localStorage.getItem('offline-note')) return;
      localStorage.setItem('offline-note', '1');
    } catch { /* private mode: show it anyway */ }
    ui.toast('Ready offline');
  },
});

// ------------------------------------------------------------------ load + reveal
(async () => {
  void requestWakeLock();
  applySettings(settings);
  await viewer.load(MODEL_URL, PARTS_URL, (f) => ui.progress(f));
  applySettings(settings);
  viewer.resize(W(), H());
  viewer.frameOverview(insets(), false, true);
  ui.loaded();
  state.ready = true;
  viewer.startReveal();
  viewer.buildPickingInBackground();
  ui.showHint(true);
  window.addEventListener('pointerdown', () => sound.play('boot'), { once: true });
  (window as unknown as { __viewer: unknown }).__viewer = {
    viewer, state, ui, stats: () => viewer.modelStats(),
    api: { selectSubsystem, selectNodes, overview, hidePids, showAll, setMechanism, enterSim, exitSim, toggleTray, goUp },
  };
})().catch((err) => {
  console.error(err);
  ui.loadError('Something went wrong loading the robot. Reload the page to try again.');
});
