#!/usr/bin/env node
// End-to-end check (run `npm run build` first). Serves dist/, drives it with headless Chromium and
// touch emulation at three iPad sizes, saves screenshots to shots/, and fails on console errors
// or broken interactions.   node scripts/verify.mjs [--only=portrait]
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const SIZES = [
  { name: 'ipad-air-landscape', width: 1180, height: 820 },
  { name: 'ipad-air-portrait', width: 820, height: 1180 },
  { name: 'ipad-pro-landscape', width: 1366, height: 1024 },
];
const only = process.argv.find((a) => a.startsWith('--only='))?.slice(7);
const OUT = 'shots';
fs.mkdirSync(OUT, { recursive: true });

const server = await preview({ preview: { port: 4173, strictPort: true }, logLevel: 'warn' });
const URL = 'http://localhost:4173/';
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });

const failures = [];
const check = (ok, msg) => { if (!ok) { failures.push(msg); console.log('  ✗', msg); } else console.log('  ✓', msg); };

async function open(size, opts = {}) {
  const ctx = await browser.newContext({
    viewport: { width: size.width, height: size.height }, deviceScaleFactor: 1, hasTouch: true,
    reducedMotion: opts.reducedMotion ? 'reduce' : 'no-preference', serviceWorkers: 'block',
  });
  const page = await ctx.newPage();
  const errors = [];
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  if (opts.light) await page.addInitScript(() => localStorage.setItem('chuck-norris-settings-v2', JSON.stringify({ appearance: 'light' })));
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
  const cdp = await page.context().newCDPSession(page);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y }] });
  for (let i = 1; i <= 12; i++) {
    await page.waitForTimeout(ms / 12);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x + (dx * i) / 12, y: y + (dy * i) / 12 }] });
  }
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
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
  const cdp = await page.context().newCDPSession(page);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: t.x + 40, y: t.y + 20 }] });
  await page.waitForTimeout(900);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  await page.waitForTimeout(400);
  check(await page.evaluate(() => window.__viewer.ui.settingsOpen), 'press-and-hold the title opens settings');
  await shot(page, size, '10-settings');
  await tapEl(page, '.sheet [data-act=close]');

  // Simulation (landscape iPad Air only; it loads the whole field)
  if (size.name === SIZES[0].name) {
    await tapEl(page, tool('Simulate'));
    await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
    const bumpMax = await page.evaluate(() => { const sim = window.__viewer.state.sim; let m = 0; for (let i = 0; i < 60 * 12; i++) { sim.frame(1 / 60); m = Math.max(m, sim.grid.floor(sim.robot.x, sim.robot.z)); } return m; });
    await settle(page);
    await shot(page, size, '11-simulation');
    check(bumpMax < 0.02, `auto never drives over a bump (max floor ${bumpMax.toFixed(3)} m)`);
    check(await page.evaluate(() => window.__viewer.state.sim.balls.scored > 0), 'auto scores fuel in the HUB');
    for (const view of ['follow', 'field', 'driver', 'tv']) {
      await tapEl(page, `.sim-view button[data-view=${view}]`);
      check(await page.evaluate((v) => window.__viewer.state.sim.view === v && document.querySelector(`.sim-view button[data-view=${v}]`).getAttribute('aria-pressed') === 'true', view), `camera view ${view}`);
    }
    await tapEl(page, '.sim-mode button[data-mode=endless]');
    await page.evaluate(() => { const sim = window.__viewer.state.sim; for (let i = 0; i < 60 * 25; i++) sim.frame(1 / 60); });
    await settle(page);
    await shot(page, size, '11b-simulation-endless');
    check(await page.evaluate(() => window.__viewer.state.sim.mode === 'endless' && window.__viewer.state.sim.balls.scored > 0), 'endless mode scores fuel');
    await tapEl(page, '.sim-controls [data-act=exit]');
    await settle(page);
    check(await page.evaluate(() => !window.__viewer.state.sim && !window.__viewer.viewer.simActive), 'exit simulation returns to the robot');

    const d = await open(size, { query: '?debug=1' });
    report = await d.page.evaluate(() => ({ ...window.__viewer.viewer.stats, model: window.__viewer.stats() }));
    await shot(d.page, size, '12-debug');
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

await browser.close();
await new Promise((r) => server.httpServer.close(r));

const glb = fs.statSync('public/models/chuck-norris.glb').size / 1e6;
const field = fs.statSync('public/field/field-2026.glb').size / 1e6;
console.log('\n=== Report ===');
console.log(`Robot model: ${glb.toFixed(2)} MB · Field: ${field.toFixed(2)} MB (loaded only for simulation)`);
if (report) console.log(`Draw calls (overview): ${report.calls}, triangles drawn: ${report.triangles.toLocaleString()}, model: ${report.model.triangles.toLocaleString()} tris, ${report.model.parts} parts in ${report.model.meshes} meshes`);
console.log(failures.length ? `\n${failures.length} check(s) FAILED` : '\nAll checks passed');
process.exit(failures.length ? 1 : 0);
