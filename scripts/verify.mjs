#!/usr/bin/env node
// End-to-end check (run `npm run build` first). Serves dist/, drives it with a headless browser and
// touch emulation at three iPad sizes plus a laptop, saves screenshots to shots/<browser>/, and fails
// on console errors or broken interactions.
//   node scripts/verify.mjs [--only=portrait] [--browser=chromium|firefox|webkit] [--sim-only]
import { preview } from 'vite';
import { chromium, firefox, webkit } from 'playwright';
import fs from 'node:fs';

const SIZES = [
  { name: 'ipad-air-landscape', width: 1180, height: 820 },
  { name: 'ipad-air-portrait', width: 820, height: 1180 },
  { name: 'ipad-pro-landscape', width: 1366, height: 1024 },
  { name: 'laptop', width: 1440, height: 900, mouse: true },
];
const only = process.argv.find((a) => a.startsWith('--only='))?.slice(7);
const BROWSER = process.argv.find((a) => a.startsWith('--browser='))?.slice(10) ?? 'chromium';
const SIM_ONLY = process.argv.includes('--sim-only');
const OUT = `shots/${BROWSER}`;
fs.mkdirSync(OUT, { recursive: true });

const PORT = 4173 + ['chromium', 'firefox', 'webkit'].indexOf(BROWSER);
const server = await preview({ preview: { port: PORT, strictPort: true, host: '127.0.0.1' }, logLevel: 'warn' });
const URL = `http://127.0.0.1:${PORT}/`;
const browser = BROWSER === 'firefox' ? await firefox.launch() : BROWSER === 'webkit' ? await webkit.launch()
  : await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
console.log(`Browser: ${BROWSER} ${browser.version()}`);

const failures = [];
const check = (ok, msg) => { if (!ok) { failures.push(msg); console.log('  ✗', msg); } else console.log('  ✓', msg); };

async function open(size, opts = {}) {
  const ctx = await browser.newContext({
    viewport: { width: size.width, height: size.height }, deviceScaleFactor: 1, hasTouch: !size.mouse, isMobile: false,
    reducedMotion: opts.reducedMotion ? 'reduce' : 'no-preference', serviceWorkers: 'block',
  });
  const page = await ctx.newPage();
  const errors = [];
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  if (opts.light) await page.addInitScript(() => localStorage.setItem('chuck-norris-settings-v2', JSON.stringify({ appearance: 'light' })));
  if (opts.local) await page.addInitScript(() => { window.__simLocal = true; });
  await page.goto(URL + (opts.query ?? ''));
  await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
  await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; window.__viewer.viewer.spinning = false; });
  await settle(page);
  return { ctx, page, errors };
}
async function settle(page, ms = 450) {
  await page.evaluate(() => window.__viewer.viewer.settle());
  await page.waitForTimeout(ms);
  await page.evaluate(() => window.__viewer.viewer.settle());
}
const shot = (page, size, name) => page.screenshot({ path: `${OUT}/${size.name}--${name}.png`, timeout: 240000 });
const tapEl = async (page, selector) => {
  const box = await page.locator(selector).first().boundingBox();
  await page.touchscreen.tap(box.x + box.width / 2, box.y + box.height / 2);
};
const tool = (label) => `.dock .tool:has(.label:text-is("${label}"))`;
async function touchDrag(page, x, y, dx, dy, ms = 400) {
  if (BROWSER !== 'chromium') {
    // synthesized pointer events (CDP touch is Chromium-only)
    await page.evaluate(async ([x, y, dx, dy, ms]) => {
      const el = document.elementFromPoint(x, y);
      const ev = (type, px, py) => el.dispatchEvent(new PointerEvent(type, { pointerId: 7, pointerType: 'touch', isPrimary: true, clientX: px, clientY: py, bubbles: true, cancelable: true, buttons: type === 'pointerup' ? 0 : 1 }));
      ev('pointerdown', x, y);
      for (let i = 1; i <= 12; i++) { await new Promise((r) => setTimeout(r, ms / 12)); ev('pointermove', x + (dx * i) / 12, y + (dy * i) / 12); }
      ev('pointerup', x + dx, y + dy);
    }, [x, y, dx, dy, ms]);
    return;
  }
  const cdp = await page.context().newCDPSession(page);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y }] });
  for (let i = 1; i <= 12; i++) {
    await page.waitForTimeout(ms / 12);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x + (dx * i) / 12, y: y + (dy * i) / 12 }] });
  }
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
}
async function touchHold(page, x, y, ms) {
  if (BROWSER === 'chromium') {
    const cdp = await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y }] });
    await page.waitForTimeout(ms);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    return;
  }
  await page.evaluate(async ([x, y, ms]) => {
    const el = document.elementFromPoint(x, y);
    const ev = (type) => el.dispatchEvent(new PointerEvent(type, { pointerId: 9, pointerType: 'touch', isPrimary: true, clientX: x, clientY: y, bubbles: true, cancelable: true, buttons: type === 'pointerup' ? 0 : 1 }));
    ev('pointerdown'); await new Promise((r) => setTimeout(r, ms)); ev('pointerup');
  }, [x, y, ms]);
}
const findHit = (page) => page.evaluate(() => {
  const v = window.__viewer.viewer, c = document.getElementById('stage');
  for (let y = 0.35; y < 0.75; y += 0.03) for (let x = 0.3; x < 0.7; x += 0.03) {
    const px = x * innerWidth, py = y * innerHeight;
    if (document.elementFromPoint(px, py) !== c) continue;
    const h = v.pick(px, py);
    if (h && h.pid) return { x: px, y: py, pid: h.pid };
  }
  return null;
});

let report = null;
for (const size of SIZES) {
  if (only && !size.name.includes(only)) continue;
  if (size.mouse || SIM_ONLY) continue; // the CAD regression runs at the iPad sizes
  console.log(`\n=== ${size.name} (${size.width}×${size.height}) ===`);
  const { ctx, page, errors } = await open(size);
  await shot(page, size, '01-overview');

  const ids = await page.evaluate(() => window.__viewer.viewer.ids);
  for (const [i, id] of ids.entries()) {
    await tapEl(page, `.chip:nth-of-type(${i + 1})`);
    await settle(page);
    check(await page.evaluate((id) => window.__viewer.state.sel?.id === id, id), `chip focuses ${id}`);
    await shot(page, size, `02-focus-${String(i + 1).padStart(2, '0')}-${id}`);
  }
  if (await page.evaluate(() => window.__viewer.state.sel?.id === 'frame')) {
    const turretsHidden = await page.evaluate(() => { const s = window.__viewer.state; window.__viewer.api.selectSubsystem('turrets'); return s.autoHidden.length; });
    check(turretsHidden > 0, 'turret side panels hide while turrets are focused');
  }
  await tapEl(page, '.panel .back');
  await settle(page);
  check(await page.evaluate(() => window.__viewer.state.sel === null), 'Back returns to overview');

  // Geometry: tap selects a part, tap again climbs, drag never selects, double-tap hides.
  // (Subsystem labels drift over the robot as it turns; tapping one correctly selects that
  // subsystem, so they're hidden for this part-tapping check.)
  await page.evaluate(() => { document.getElementById('pins').style.display = 'none'; });
  await page.waitForFunction(() => window.__viewer.viewer.pickingReady, null, { timeout: 240000 });
  let hit = await findHit(page);
  check(!!hit, 'found a point on the robot');
  if (hit) {
    await touchDrag(page, hit.x, hit.y, 140, 10);
    await settle(page);
    check(await page.evaluate(() => window.__viewer.state.sel === null), 'drag starting on the robot does not select');
    await page.waitForTimeout(800); // subsystem labels fade back in; don't pick a spot under one
    await settle(page);
    hit = await findHit(page);
    await page.touchscreen.tap(hit.x, hit.y);
    await page.waitForFunction(() => window.__viewer.state.sel !== null, null, { timeout: 240000 }).catch(() => {});
    await settle(page);
    const s1 = await page.evaluate(() => window.__viewer.state.sel);
    check(s1?.kind === 'node', `tap on geometry selects a part (${s1?.label})`);
    await shot(page, size, '03-part');
    await page.touchscreen.tap(hit.x, hit.y);
    await page.waitForFunction((a) => JSON.stringify(window.__viewer.state.sel) !== a, JSON.stringify(s1), { timeout: 240000 }).catch(() => {});
    await settle(page);
    const s2 = await page.evaluate(() => window.__viewer.state.sel);
    check(s2 && JSON.stringify(s2) !== JSON.stringify(s1), `tapping again selects the parent (${s2?.label ?? s2?.id})`);
    await page.waitForTimeout(600);
    await page.evaluate(() => window.__viewer.api.overview());
    await settle(page, 800);
    await page.waitForTimeout(800);
    await settle(page);
    hit = await findHit(page);
    const before = await page.evaluate(() => { window.__viewer.state.doubleTapMs = 30000; return window.__viewer.viewer.hiddenCount(); });
    await page.touchscreen.tap(hit.x, hit.y);
    await page.waitForTimeout(90);
    await page.touchscreen.tap(hit.x, hit.y);
    await page.waitForFunction((b) => window.__viewer.viewer.hiddenCount() > b, before, { timeout: 240000 }).catch(() => {});
    await settle(page);
    const after = await page.evaluate(() => window.__viewer.viewer.hiddenCount());
    await page.evaluate(() => { window.__viewer.state.doubleTapMs = 260; });
    check(after > before, `double-tap hides a part (${after - before} part(s))`);
    await shot(page, size, '04-hidden');
    await tapEl(page, '.pill [data-act=all]');
    await settle(page);
    check(await page.evaluate(() => window.__viewer.viewer.hiddenCount() === 0), 'Show All restores hidden parts');
  }

  await page.evaluate(() => { document.getElementById('pins').style.display = ''; });
  // Explode
  await tapEl(page, tool('Explode'));
  await settle(page);
  await shot(page, size, '05-explode');
  await tapEl(page, tool('Collapse'));
  await settle(page);

  // Mechanisms
  await tapEl(page, tool('Move'));
  await page.evaluate(() => { const a = window.__viewer.api; a.setMechanism('turret-left', 70, true); a.setMechanism('turret-right', -70, true); a.setMechanism('intake', -110, true); });
  await settle(page);
  check(await page.evaluate(() => Math.abs(window.__viewer.viewer.mechanismValue('turret-left') - 70) < 1), 'left turret rotates');
  await shot(page, size, '06-mechanisms');
  await page.evaluate(() => { const a = window.__viewer.api; a.setMechanism('turret-left', 0, true); a.setMechanism('turret-right', 0, true); a.setMechanism('intake', 0, true); a.setMechanism('brainpan', -95, true); });
  await settle(page, 600);
  check(await page.evaluate(() => window.__viewer.viewer.lifted), 'opening the brain pan lifts the robot');
  await shot(page, size, '07-brain-pan-underside');
  await page.evaluate(() => window.__viewer.api.setMechanism('brainpan', 0, true));
  await tapEl(page, '.tray .tray-head button');
  await settle(page);

  // Search
  await tapEl(page, '.corner .icon-btn:first-child');
  await page.locator('.search input').fill('kraken');
  await page.waitForTimeout(200);
  check(await page.locator('.result').count() > 0, 'search finds Kraken motors');
  await shot(page, size, '08-search');
  await tapEl(page, '.result');
  await settle(page);
  check(await page.evaluate(() => window.__viewer.state.sel?.kind === 'node'), 'search result selects the part');
  await shot(page, size, '09-search-result');
  await page.evaluate(() => window.__viewer.api.overview());
  await settle(page);

  // Settings (hidden: press and hold the title)
  const t = await page.locator('.title').boundingBox();
  await touchHold(page, t.x + 40, t.y + 20, 900);
  await page.waitForTimeout(400);
  check(await page.evaluate(() => window.__viewer.ui.settingsOpen), 'press-and-hold the title opens settings');
  await shot(page, size, '10-settings');
  await tapEl(page, '.sheet [data-act=close]');

  if (size.name === SIZES[0].name) {
    const d = await open(size, { query: '?debug=1' });
    report = await d.page.evaluate(() => ({ ...window.__viewer.viewer.stats, model: window.__viewer.viewer.modelStats ? window.__viewer.stats() : window.__viewer.stats() }));
    errors.push(...d.errors);
    await d.ctx.close();
  }

  // Light mode + reduced motion
  const lm = await open(size, { light: true, reducedMotion: true });
  await shot(lm.page, size, '13-light-reduced-motion');
  check(await lm.page.evaluate(() => !window.__viewer.viewer.spinning), 'reduced motion: no auto-spin');
  await tapEl(lm.page, '.chip:nth-of-type(4)');
  await lm.page.waitForTimeout(400);
  await settle(lm.page);
  await shot(lm.page, size, '14-light-focus');
  errors.push(...lm.errors);
  await lm.ctx.close();

  check(errors.length === 0, `no console errors${errors.length ? ': ' + errors.slice(0, 3).join(' | ') : ''}`);
  await ctx.close();
}

await simTests();

await browser.close();
await new Promise((r) => server.httpServer.close(r));

async function simTests() {
  // ---------------------------------------------------------------- the simulation
  const size = SIZES[0];
  console.log(`\n=== simulation (${size.width}×${size.height}) ===`);
  const { ctx, page, errors } = await open(size, { local: true });
  await tapEl(page, tool('Simulate'));
  await page.waitForFunction(() => window.__viewer.state.sim?.active && window.__viewer.state.sim.backend?.frame, null, { timeout: 180000 });
  const step = (s) => page.evaluate((s) => { const sim = window.__viewer.state.sim; for (let i = 0; i < Math.round(s * 60); i++) sim.frame(1 / 60); }, s);
  await page.evaluate(() => window.__viewer.state.sim.setTouchControls(true));
  await step(4);
  check(await page.evaluate(() => document.querySelector('.sim-score')?.offsetParent !== null), 'HUD: score pill is visible');
  check(await page.evaluate(() => !document.querySelector('.sim-status') && !document.querySelector('.sim-controls')), 'HUD: the old crowded bar is gone');
  check(await page.evaluate(() => [...document.querySelectorAll('.sim-hud button')].filter((b) => b.offsetParent).length <= 4), 'HUD: at most four small buttons');
  // the AI drives it until we touch something
  check(await page.evaluate(() => window.__viewer.state.sim.status().control === 'ai'), 'free drive: the robot drives itself at first');

  // touch: the left floating stick drives (up = toward the opponent = +x internally)
  await page.evaluate(() => { const sim = window.__viewer.state.sim; sim.world.takeOver(); sim.world.robot.place(-6.4, -0.4, Math.PI / 2); sim.world.robot.setIntakeNow(true); });
  await step(0.5);
  const x0 = await page.evaluate(() => window.__viewer.state.sim.robot.x);
  const zone = await page.locator('.dp-move').boundingBox();
  const sx = zone.x + zone.width / 2, sy = zone.y + zone.height * 0.6;
  // hold the stick up while stepping the simulation
  await page.evaluate(([sx, sy]) => {
    const el = document.elementFromPoint(sx, sy);
    const ev = (type, x, y) => el.dispatchEvent(new PointerEvent(type, { pointerId: 21, pointerType: 'touch', isPrimary: true, clientX: x, clientY: y, bubbles: true, cancelable: true, buttons: 1 }));
    ev('pointerdown', sx, sy); ev('pointermove', sx, sy - 70);
    window.__stickUp = () => { ev('pointerup', sx, sy - 70); };
  }, [sx, sy]);
  await step(1.2);
  const after = await page.evaluate(() => { const s = window.__viewer.state.sim; return { x: s.robot.x, z: s.robot.z, control: s.status().control }; });
  check(after.x - x0 > 1.0 && Math.abs(after.z + 0.4) < 0.4, `touch stick up drives toward the opponent's wall (Δx ${(after.x - x0).toFixed(2)} m, Δz ${(after.z + 0.4).toFixed(2)} m)`);
  check(after.control === 'driver', 'touching the stick takes control');
  // multitouch: shoot while driving
  const shootBox = await page.locator('.dp-shoot').boundingBox();
  await page.evaluate(([x, y]) => {
    const el = document.elementFromPoint(x, y);
    el.dispatchEvent(new PointerEvent('pointerdown', { pointerId: 22, pointerType: 'touch', clientX: x, clientY: y, bubbles: true, cancelable: true, buttons: 1 }));
    window.__shootUp = () => el.dispatchEvent(new PointerEvent('pointerup', { pointerId: 22, pointerType: 'touch', clientX: x, clientY: y, bubbles: true }));
  }, [shootBox.x + shootBox.width / 2, shootBox.y + shootBox.height / 2]);
  const shots0 = await page.evaluate(() => window.__viewer.state.sim.world.robot.shotsFired);
  await step(0.3);
  await page.evaluate(() => window.__stickUp());
  await page.evaluate(() => { const r = window.__viewer.state.sim.world.robot; r.place(-6.1, 0.3, Math.PI / 2); r.preload(window.__viewer.state.sim.world.preloadIdx); });
  await step(2.5);
  const shots1 = await page.evaluate(() => window.__viewer.state.sim.world.robot.shotsFired);
  await page.evaluate(() => window.__shootUp());
  check(shots1 - shots0 >= 3, `holding Shoot fires (${shots1 - shots0} shots), even with a second finger on the stick`);
  await step(1);
  const shots2 = await page.evaluate(() => window.__viewer.state.sim.world.robot.shotsFired);
  await step(1);
  check(await page.evaluate((s) => window.__viewer.state.sim.world.robot.shotsFired === s, shots2), 'releasing Shoot stops the indexer');
  // intake toggle
  const intakeBox = await page.locator('.dp-intake').boundingBox();
  await page.touchscreen.tap(intakeBox.x + intakeBox.width / 2, intakeBox.y + intakeBox.height / 2);
  await step(0.3);
  check(await page.evaluate(() => window.__viewer.state.sim.robot.rollers === 'in' && document.querySelector('.dp-intake').getAttribute('aria-pressed') === 'true'), 'Intake button turns the rollers on (and shows it)');
  await page.touchscreen.tap(intakeBox.x + intakeBox.width / 2, intakeBox.y + intakeBox.height / 2);
  await step(0.6);
  check(await page.evaluate(() => window.__viewer.state.sim.robot.rollers === 'off' && window.__viewer.state.sim.robot.intakeAngle < -60), 'tapping Intake again stops and stows it');
  // ferry from the neutral zone, with the subtle indicator
  await page.evaluate(() => { const w = window.__viewer.state.sim.world; w.robot.place(0.5, -1.2, Math.PI / 2); w.robot.preload(w.preloadIdx); w.robot.shoot = true; });
  await page.evaluate(() => { window.__viewer.state.sim.controls.touch.shoot = true; });
  await step(1.5);
  check(await page.evaluate(() => window.__viewer.state.sim.status().ferry && !document.querySelector('.sim-ferry').hidden), 'shooting from the neutral zone ferries (and says "Ferry")');
  await shot(page, size, 'sim-ferry');
  await page.evaluate(() => { window.__viewer.state.sim.controls.touch.shoot = false; });
  await step(0.5);

  // cameras: exactly three, cycling smoothly (critically damped: no jumps, and it gets there)
  const views = [];
  for (let k = 0; k < 4; k++) {
    views.push(await page.evaluate(() => window.__viewer.state.sim.view));
    await tapEl(page, '.sim-tr [data-act=camera]');
    const path = await page.evaluate(() => {
      const sim = window.__viewer.state.sim, out = [];
      for (let i = 0; i < 90; i++) { sim.frame(1 / 60); out.push(sim.cameraRig().toArray()); }
      return out;
    });
    let maxStep = 0;
    for (let i = 1; i < path.length; i++) maxStep = Math.max(maxStep, Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1], path[i][2] - path[i - 1][2]));
    const settle = Math.hypot(...path[89].map((v, j) => v - path[88][j]));
    let total = 0;
    for (let i = 1; i < path.length; i++) total += Math.hypot(...path[i].map((v, j) => v - path[i - 1][j]));
    // no jump: no single frame covers more than 6 % of the move, and it comes to rest
    check(maxStep <= Math.max(0.05, total * 0.06) && settle < 0.02, `camera switch ${k + 1}: smooth blend (${total.toFixed(1)} m in ${(path.length / 60).toFixed(1)} s, largest frame ${(maxStep / Math.max(total, 1e-6) * 100).toFixed(1)} %, settled)`);
  }
  check(JSON.stringify(views.slice(0, 3).sort()) === JSON.stringify(['driver', 'follow', 'overhead']) && views[3] === views[0], `camera button cycles exactly Driver → Follow → Overhead (${views.join(' → ')})`);
  for (const v of ['driver', 'follow', 'overhead']) {
    await page.evaluate((v) => window.__viewer.state.sim.setView(v), v);
    await step(1.5);
    await shot(page, size, `sim-${v}`);
  }

  // settings sheet: exactly the six things
  await tapEl(page, '.sim-tr [data-act=settings]');
  await page.waitForTimeout(300);
  const labels = await page.evaluate(() => [...document.querySelectorAll('.sheet .row-label, .sheet .row > span, .sheet .sheet-actions .act span:not(.icon)')].map((e) => e.textContent.trim()).filter(Boolean));
  check(JSON.stringify(labels) === JSON.stringify(['Alliance', 'Bumper Color in CAD View', 'Camera', 'Robot Sound', 'Reset Field', 'Controls Help']), `sim settings: ${labels.join(', ')}`);
  await shot(page, size, 'sim-settings');
  await tapEl(page, '.sheet [data-key=alliance] [data-v=blue]');
  await tapEl(page, '.sheet [data-key=bumper] [data-v=blue]');
  await page.waitForTimeout(200);
  await step(1);
  check(await page.evaluate(() => window.__viewer.state.sim.alliance === 'blue'), 'alliance: blue');
  await shot(page, size, 'sim-blue');
  await tapEl(page, '.sheet [data-act=close]');

  // High Score: 3-2-1, 60 s, disabled at 0:00, 3 s more for fuel in the air, results
  await tapEl(page, '.sim-tr [data-act=highscore]');
  await step(0.5);
  check(await page.evaluate(() => { const s = window.__viewer.state.sim.status(); return s.hs?.phase === 'countdown' && !document.querySelector('.sim-count').hidden; }), 'High Score starts with a 3-2-1 countdown');
  await shot(page, size, 'hs-countdown');
  await step(3);
  check(await page.evaluate(() => window.__viewer.state.sim.status().hs?.phase === 'play'), 'then the 60 s round');
  // drive and shoot a bit
  await page.evaluate(() => { const c = window.__viewer.state.sim.controls.touch; c.shoot = true; });
  await step(8);
  await page.evaluate(() => { const c = window.__viewer.state.sim.controls.touch; c.shoot = false; c.mx = 0.6; c.my = 0.8; });
  await step(52.5);
  check(await page.evaluate(() => window.__viewer.state.sim.status().hs?.phase === 'post' && window.__viewer.state.sim.world.robot.disabled), 'at 0:00 the robot is disabled');
  await step(3.5);
  const res = await page.evaluate(() => window.__viewer.state.sim.status().hs?.result);
  check(!!res && await page.evaluate(() => !document.querySelector('.sim-result').hidden), `results: ${res?.score} FUEL, ${res?.shots} shots, best ${res?.bestRate}/s, personal best ${res?.best}`);
  await shot(page, size, 'hs-results');
  const stored = await page.evaluate(() => localStorage.getItem('chuck-norris-highscore'));
  check(stored !== null && Number(stored) === res.best, `personal best saved (${stored})`);
  await page.evaluate(() => { const c = window.__viewer.state.sim.controls.touch; c.mx = 0; c.my = 0; });
  await tapEl(page, '.sim-result [data-act=hs-exit]');
  await step(0.3);
  check(await page.evaluate(() => window.__viewer.state.sim.status().mode === 'free'), 'Exit returns to Free Drive');

  // flicker: a still scene must render identically frame after frame, from every camera
  await page.waitForFunction(() => !window.__viewer.viewer.revealing, null, { timeout: 60000 });
  await page.evaluate(() => { window.__viewer.state.sim.paused = true; });
  for (const v of ['driver', 'follow', 'overhead']) {
    await page.evaluate((v) => { const sim = window.__viewer.state.sim; sim.paused = false; sim.setView(v); for (let i = 0; i < 120; i++) sim.frame(1 / 60); sim.paused = true; }, v);
    const a = await page.screenshot({ timeout: 240000 });
    await page.waitForTimeout(500);
    const b = await page.screenshot({ timeout: 240000 });
    await page.waitForTimeout(500);
    const c = await page.screenshot({ timeout: 240000 });
    check(a.equals(b) && b.equals(c), `no flicker: ${v} view renders identically over three frames`);
  }
  await page.evaluate(() => { window.__viewer.state.sim.paused = false; });

  // input fuzzing: random touches, keys and stick motion for a while; nothing may break
  await page.evaluate(() => {
    const sim = window.__viewer.state.sim;
    let seed = 7; const rnd = () => (seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296;
    const keys = ['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'Space', 'KeyF', 'KeyR', 'KeyV', 'KeyX', 'KeyC', 'ShiftLeft', 'KeyH'];
    for (let k = 0; k < 60 * 40; k++) {
      if (rnd() < 0.05) { const code = keys[Math.floor(rnd() * keys.length)]; window.dispatchEvent(new KeyboardEvent(rnd() < 0.5 ? 'keydown' : 'keyup', { code, bubbles: true })); }
      if (rnd() < 0.03) Object.assign(sim.controls.touch, { mx: rnd() * 2 - 1, my: rnd() * 2 - 1, turn: rnd() * 2 - 1, shoot: rnd() < 0.4, intakeTap: rnd() < 0.2 });
      sim.frame(1 / 60 * (0.5 + rnd()));
    }
    for (const code of keys) window.dispatchEvent(new KeyboardEvent('keyup', { code }));
    Object.assign(sim.controls.touch, { mx: 0, my: 0, turn: 0, shoot: false });
  });
  const fz = await page.evaluate(() => { const r = window.__viewer.state.sim.robot; return [r.x, r.z, r.th].every(Number.isFinite) && Math.abs(r.x) < 8.3 && Math.abs(r.z) < 4.05; });
  check(fz, 'input fuzzing (40 s of random keys and touches): robot sane');

  await tapEl(page, '.sim-hud [data-act=exit]');
  await page.waitForTimeout(300);
  check(await page.evaluate(() => !window.__viewer.state.sim && !window.__viewer.viewer.simActive), 'exit returns to the robot');
  check(errors.length === 0, `sim: no console errors${errors.length ? ': ' + errors.slice(0, 3).join(' | ') : ''}`);

  // settings persist across reloads
  await page.reload();
  await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 180000 });
  check(await page.evaluate(() => { const s = JSON.parse(localStorage.getItem('chuck-norris-settings-v2') || '{}'); return s.alliance === 'blue' && s.bumper === 'blue'; }), 'alliance and bumper color are remembered');
  check(await page.evaluate(() => window.__viewer.viewer.bumperColor === 'blue'), 'CAD view shows blue bumpers after reload');
  await settle(page);
  await shot(page, size, 'cad-blue-bumpers');
  await ctx.close();

  // the real thing: physics in its worker, in real time, at every size
  for (const sz of SIZES) {
    const q = await open(sz);
    await q.page.evaluate(() => { localStorage.clear(); });
    await q.page.evaluate(() => window.__viewer.api.enterSim('free'));
    await q.page.waitForFunction(() => window.__viewer.state.sim?.active && window.__viewer.state.sim.backend?.frame, null, { timeout: 180000 });
    await q.page.waitForTimeout(4000);
    const st = await q.page.evaluate(() => ({ clock: window.__viewer.state.sim.f.clock, worker: !window.__viewer.state.sim.world }));
    check(st.worker && st.clock > 0.1, `${sz.name}: physics runs in its worker (${st.clock.toFixed(1)} s simulated)`);
    // nothing overlaps: HUD elements keep clear of each other
    const overlap = await q.page.evaluate(() => {
      const els = ['.sim-exit', '.sim-score', '.sim-tr', '.dp-move', '.dp-turn', '.dp-buttons'].map((s) => document.querySelector(s)).filter((e) => e && e.offsetParent !== null);
      const r = els.map((e) => ({ s: e.className, b: e.getBoundingClientRect() }));
      const hits = [];
      const tops = r.filter((x) => /sim-(exit|score|tr)/.test(x.s));
      for (let i = 0; i < tops.length; i++) for (let j = i + 1; j < tops.length; j++) { const a = tops[i].b, b = tops[j].b; if (a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom) hits.push(`${tops[i].s} × ${tops[j].s}`); }
      const btn = document.querySelector('.dp-buttons')?.getBoundingClientRect();
      const turn = document.querySelector('.dp-turn')?.getBoundingClientRect();
      if (btn && turn && document.querySelector('.drivepad.on') && btn.left < turn.right && turn.left < btn.right && btn.top < turn.bottom && turn.top < btn.bottom) hits.push('buttons × turn area');
      return hits;
    });
    check(overlap.length === 0, `${sz.name}: HUD and controls don't overlap${overlap.length ? ': ' + overlap.join(', ') : ''}`);
    await shot(q.page, sz, 'sim-live');
    check(q.errors.length === 0, `${sz.name}: no console errors${q.errors.length ? ': ' + q.errors.slice(0, 3).join(' | ') : ''}`);
    await q.ctx.close();
  }
}

const glb = fs.statSync('public/models/chuck-norris.glb').size / 1e6;
const field = fs.statSync('public/field/field-2026.glb').size / 1e6;
console.log('\n=== Report ===');
console.log(`Robot model: ${glb.toFixed(2)} MB · Field: ${field.toFixed(2)} MB (loaded only for simulation)`);
if (report) console.log(`Draw calls (overview): ${report.calls}, triangles drawn: ${report.triangles.toLocaleString()}, model: ${report.model.triangles.toLocaleString()} tris, ${report.model.parts} parts in ${report.model.meshes} meshes`);
console.log(failures.length ? `\n${failures.length} check(s) FAILED` : '\nAll checks passed');
process.exit(failures.length ? 1 : 0);
