import './style.css';
import { registerSW } from 'virtual:pwa-register';
import { Viewer, type Insets } from './viewer';
import { UI } from './ui';
import { robot, subsystems, MODEL_URL } from './config';

// ------------------------------------------------------------------ iPad Safari guards
// Safari ignores user-scalable=no, so block pinch/double-tap page zoom here too.
for (const t of ['gesturestart', 'gesturechange', 'gestureend']) {
  document.addEventListener(t, (e) => e.preventDefault(), { passive: false });
}
document.addEventListener('dblclick', (e) => e.preventDefault(), { passive: false });
document.addEventListener('touchmove', (e) => {
  const scroller = (e.target as Element | null)?.closest?.('.panel-body, .chips');
  if (!scroller || e.touches.length > 1) e.preventDefault();
}, { passive: false });
document.addEventListener('contextmenu', (e) => e.preventDefault());

const params = new URLSearchParams(location.search);
const DEBUG = params.get('debug') === '1';
const reducedMotionQuery = matchMedia('(prefers-reduced-motion: reduce)');

// ------------------------------------------------------------------ state
const state = {
  focused: null as string | null,
  tour: false,
  exploded: false,
  lastInteraction: -Infinity,
  interacted: false,
  ready: false,
  autoSpin: true,  // tests switch these two off
  idleReset: true,
};

const canvas = document.getElementById('stage') as HTMLCanvasElement;
const viewer = new Viewer(canvas);
viewer.reducedMotion = reducedMotionQuery.matches;
reducedMotionQuery.addEventListener('change', () => { viewer.reducedMotion = reducedMotionQuery.matches; });

const ui = new UI({
  onChip: (id) => focus(id),
  onPin: (id) => focus(id),
  onHome: () => resetAll(),
  onTour: () => (state.tour ? endTour() : startTour()),
  onExplode: () => setExploded(!state.exploded),
  onBack: () => (state.tour ? endTour() : overview()),
  onPrev: () => step(-1),
  onNext: () => step(1),
  onFullscreen: () => toggleFullscreen(),
});

// ------------------------------------------------------------------ framing insets
const W = () => ui.app.clientWidth;
const H = () => ui.app.clientHeight;
function insets(): Insets {
  const w = W(), h = H();
  const bottom = h - ui.dockTop() + 16;
  if (!state.focused) return { top: ui.titleBottom() + 12, right: 32, bottom: bottom + 36, left: 32 };
  const p = ui.panelBox();
  if (ui.isLandscape) return { top: 32, right: w - p.left + 16, bottom, left: 32 };
  return { top: Math.max(24, ui.titleBottom() - 40), right: 20, bottom: h - p.top + 16, left: 20 };
}

// ------------------------------------------------------------------ actions
function touch() {
  state.lastInteraction = performance.now();
  viewer.spinning = false;
  requestWakeLock();
}

function focus(id: string) {
  if (!state.ready) return;
  touch();
  state.focused = id;
  const i = subsystems.findIndex((s) => s.id === id);
  ui.showPanel(subsystems[i], i, subsystems.length, state.tour);
  ui.setFocused(id);
  ui.showHint(false);
  state.interacted = true;
  viewer.focus(id, insets());
}

function overview(resetAngle = false) {
  touch();
  state.focused = null;
  ui.hidePanel();
  ui.setFocused(null);
  viewer.frameOverview(insets(), true, resetAngle);
}

function step(dir: number) {
  const i = state.focused ? subsystems.findIndex((s) => s.id === state.focused) : -1;
  const n = i + dir;
  if (state.tour) {
    if (n >= subsystems.length) return endTour();
    if (n < 0) return;
    return focus(subsystems[n].id);
  }
  focus(subsystems[(n + subsystems.length) % subsystems.length].id);
}

function startTour() {
  state.tour = true;
  ui.setTour(true);
  focus(subsystems[0].id);
}

function endTour() {
  state.tour = false;
  ui.setTour(false);
  overview();
}

function setExploded(on: boolean) {
  if (!state.ready) return;
  touch();
  state.exploded = on;
  ui.setExploded(on);
  viewer.setExploded(on, insets());
}

function resetAll() {
  state.tour = false;
  ui.setTour(false);
  if (state.exploded) { state.exploded = false; ui.setExploded(false); viewer.setExploded(false, insets()); }
  overview(true);
}

// ------------------------------------------------------------------ canvas taps vs drags
const TAP_MOVE = 8, TAP_MS = 300;
const pointers = new Map<number, { x: number; y: number; t: number }>();
let tapCandidate: { id: number; x: number; y: number; t: number; hit: string | null } | null = null;

canvas.addEventListener('pointerdown', (e) => {
  touch();
  pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, t: performance.now() });
  if (pointers.size === 1 && e.button === 0) {
    const hit = state.ready ? viewer.pick(e.clientX, e.clientY) : null;
    tapCandidate = { id: e.pointerId, x: e.clientX, y: e.clientY, t: performance.now(), hit };
    if (hit) viewer.setHighlight(hit); // instant touch-down feedback
  } else {
    cancelTap(); // a second finger means pinch/pan, never a tap
  }
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
  if (c.hit) focus(c.hit);
  else if (state.focused) state.tour ? endTour() : overview();
};
canvas.addEventListener('pointerup', endPointer);
canvas.addEventListener('pointercancel', endPointer);
canvas.addEventListener('pointerleave', (e) => { if (e.pointerType === 'mouse') endPointer(e); });
function cancelTap() {
  tapCandidate = null;
  viewer.setHighlight(null);
}
ui.layer.addEventListener('pointerdown', () => touch());

// ------------------------------------------------------------------ keyboard (testing on a Mac)
window.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  if (e.key === 'ArrowRight' || e.key === 'ArrowDown') { e.preventDefault(); step(1); }
  else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') { e.preventDefault(); step(-1); }
  else if (e.key === 'Escape') { state.tour ? endTour() : overview(); }
  else if (e.key === 'r' || e.key === 'R') resetAll();
});

// ------------------------------------------------------------------ auto-spin + idle reset
setInterval(() => {
  if (!state.ready || viewer.revealing) return;
  const idle = (performance.now() - state.lastInteraction) / 1000;
  if (state.idleReset && idle > robot.idleResetSeconds && (state.focused || state.exploded || state.tour)) {
    resetAll();
    state.lastInteraction = performance.now() - robot.autoSpinResumeSeconds * 1000; // spin right away
    state.interacted = false;
    ui.showHint(true);
    return;
  }
  const spin = state.autoSpin && !state.focused && pointers.size === 0 && idle > robot.autoSpinResumeSeconds && !viewer.reducedMotion;
  if (spin !== viewer.spinning) viewer.spinning = spin;
}, 250);

// ------------------------------------------------------------------ resize (rotation, Split View, Stage Manager)
let resizeRaf = 0;
function onResize() {
  cancelAnimationFrame(resizeRaf);
  resizeRaf = requestAnimationFrame(() => {
    viewer.resize(W(), H());
    ui.layout(W(), H());
    if (state.ready) viewer.reframe(insets());
  });
}
new ResizeObserver(onResize).observe(ui.app);
viewer.resize(W(), H());
ui.layout(W(), H());

// ------------------------------------------------------------------ pins follow the robot
viewer.onRender = () => {
  ui.updatePins(viewer.pins(), state.ready && !state.focused && !viewer.revealing);
  if (DEBUG) {
    const s = viewer.stats, m = viewer.modelStats();
    ui.debug(`${s.fps.toFixed(0)} fps\n${s.calls} draw calls\n${(s.triangles / 1e6).toFixed(2)}M tris drawn\n${(m.triangles / 1e6).toFixed(2)}M tris model\n${m.meshes} meshes · ${m.source}`);
  }
};
viewer.onContext = (lost) => ui.restoring(lost);

// ------------------------------------------------------------------ fullscreen + wake lock
const docEl = document.documentElement as HTMLElement & { webkitRequestFullscreen?: () => void };
const doc = document as Document & { webkitFullscreenElement?: Element; webkitExitFullscreen?: () => void };
const standalone = matchMedia('(display-mode: standalone), (display-mode: fullscreen)').matches || (navigator as Navigator & { standalone?: boolean }).standalone === true;
const fsSupported = !standalone && !!(docEl.requestFullscreen || docEl.webkitRequestFullscreen);
const isFs = () => !!(document.fullscreenElement || doc.webkitFullscreenElement);
function syncFs() { ui.setFullscreenAvailable(fsSupported, isFs()); }
function toggleFullscreen() {
  if (isFs()) (document.exitFullscreen?.bind(document) ?? doc.webkitExitFullscreen?.bind(doc))?.();
  else (docEl.requestFullscreen?.bind(docEl) ?? docEl.webkitRequestFullscreen?.bind(docEl))?.();
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
  if (document.visibilityState === 'visible') { requestWakeLock(); viewer.invalidate(); }
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
  requestWakeLock();
  await viewer.load(MODEL_URL, (f) => ui.progress(f));
  viewer.resize(W(), H());
  viewer.frameOverview(insets(), false, true);
  ui.loaded();
  state.ready = true;
  viewer.startReveal();
  viewer.buildPickingInBackground();
  ui.showHint(true);
  (window as unknown as { __viewer: unknown }).__viewer = { viewer, state, stats: () => viewer.modelStats() };
})().catch((err) => {
  console.error(err);
  ui.loadError('Something went wrong loading the robot. Reload the page to try again.');
});
