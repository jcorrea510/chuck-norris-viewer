#!/usr/bin/env node
// End-to-end check: builds must already exist in dist/ (npm run build).
// Serves dist/, drives it with headless Chromium + touch emulation at three iPad sizes,
// saves screenshots to shots/, and fails on console errors or broken interactions.
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
    viewport: { width: size.width, height: size.height },
    deviceScaleFactor: 1,
    hasTouch: true,
    reducedMotion: opts.reducedMotion ? 'reduce' : 'no-preference',
    serviceWorkers: 'block',
  });
  const page = await ctx.newPage();
  const errors = [];
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(URL + (opts.query ?? ''));
  await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
  await page.evaluate(() => { window.__viewer.state.autoSpin = false; window.__viewer.state.idleReset = false; window.__viewer.viewer.spinning = false; });
  await settle(page);
  return { ctx, page, errors };
}

async function settle(page) {
  await page.evaluate(() => window.__viewer.viewer.settle());
  await page.waitForTimeout(450); // CSS transitions (panel slide, fades)
  await page.evaluate(() => window.__viewer.viewer.settle());
}
const shot = (page, size, name) => page.screenshot({ path: `${OUT}/${size.name}--${name}.png`, timeout: 180000 });
const tapEl = async (page, selector) => {
  const box = await page.locator(selector).first().boundingBox();
  await page.touchscreen.tap(box.x + box.width / 2, box.y + box.height / 2);
};

async function touchDrag(page, x, y, dx, dy, ms = 400) {
  const cdp = await page.context().newCDPSession(page);
  const steps = 12;
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x, y }] });
  for (let i = 1; i <= steps; i++) {
    await page.waitForTimeout(ms / steps);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x + (dx * i) / steps, y: y + (dy * i) / steps }] });
  }
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
}

let report = null;
for (const size of SIZES) {
  if (only && !size.name.includes(only)) continue;
  console.log(`\n=== ${size.name} (${size.width}×${size.height}) ===`);
  const { ctx, page, errors } = await open(size);
  await shot(page, size, '01-overview');

  const ids = await page.evaluate(() => window.__viewer.viewer.ids);
  // Every subsystem chip
  for (const [i, id] of ids.entries()) {
    await tapEl(page, `.chip:nth-child(${i + 1})`);
    await settle(page);
    const f = await page.evaluate(() => window.__viewer.state.focused);
    check(f === id, `chip tap focuses ${id}`);
    await shot(page, size, `02-focus-${String(i + 1).padStart(2, '0')}-${id}`);
  }
  // Back
  await tapEl(page, '.panel .back');
  await settle(page);
  check(await page.evaluate(() => window.__viewer.state.focused === null), 'Back returns to overview');

  // Tap on robot geometry, and a drag that must not focus
  await page.waitForFunction(() => window.__viewer.viewer.pickingReady, null, { timeout: 180000 });
  const hit = await page.evaluate(() => {
    const v = window.__viewer.viewer;
    const c = document.getElementById('stage');
    for (let y = 0.3; y < 0.75; y += 0.03) for (let x = 0.3; x < 0.7; x += 0.03) {
      const px = x * innerWidth, py = y * innerHeight;
      if (document.elementFromPoint(px, py) !== c) continue;
      const id = v.pick(px, py);
      if (id) return { x: px, y: py, id };
    }
    return null;
  });
  check(!!hit, 'found a point on the robot geometry');
  if (hit) {
    const az0 = await page.evaluate(() => window.__viewer.viewer.controls.azimuthAngle);
    await touchDrag(page, hit.x, hit.y, 140, 10);
    await settle(page);
    const after = await page.evaluate(() => ({ f: window.__viewer.state.focused, az: window.__viewer.viewer.controls.azimuthAngle }));
    check(after.f === null, 'drag starting on the robot does not focus');
    check(Math.abs(after.az - az0) > 0.05, 'drag orbits the camera');
    // re-find a hit point after the orbit
    const hit2 = await page.evaluate(() => {
      const v = window.__viewer.viewer, c = document.getElementById('stage');
      for (let y = 0.3; y < 0.75; y += 0.03) for (let x = 0.3; x < 0.7; x += 0.03) {
        const px = x * innerWidth, py = y * innerHeight;
        if (document.elementFromPoint(px, py) === c && v.pick(px, py)) return { x: px, y: py, id: v.pick(px, py) };
      }
      return null;
    });
    await page.touchscreen.tap(hit2.x, hit2.y);
    await settle(page);
    check(await page.evaluate(() => window.__viewer.state.focused) === hit2.id, `tap on geometry focuses ${hit2.id}`);
    await shot(page, size, '03-tap-geometry');
    // tap empty space returns to overview
    await page.touchscreen.tap(size.width * 0.04, size.height * 0.5);
    await settle(page);
    check(await page.evaluate(() => window.__viewer.state.focused === null), 'tap on empty space returns to overview');
  }

  // Explode
  await tapEl(page, '.dock .tool:has-text("Explode")');
  await settle(page);
  await shot(page, size, '04-explode');
  await tapEl(page, '.chip:nth-child(4)');
  await settle(page);
  await shot(page, size, '05-explode-focus');
  await tapEl(page, '.dock .tool:has-text("Collapse")');
  await tapEl(page, '.dock .tool:first-child');
  await settle(page);

  // Tour
  await tapEl(page, '.dock .tool:has-text("Tour")');
  await settle(page);
  await shot(page, size, '06-tour-1');
  await tapEl(page, '.panel [data-act=next]');
  await settle(page);
  await shot(page, size, '07-tour-2');
  check(await page.evaluate(() => window.__viewer.state.tour && window.__viewer.state.focused === window.__viewer.viewer.ids[1]), 'tour Next advances');
  await tapEl(page, '.panel .back');
  await settle(page);

  // Debug view
  if (size.name === SIZES[0].name) {
    const d = await open(size, { query: '?debug=1' });
    await d.page.evaluate(() => window.__viewer.viewer.settle());
    await shot(d.page, size, '08-debug');
    report = await d.page.evaluate(() => ({ ...window.__viewer.viewer.stats, model: window.__viewer.stats() }));
    errors.push(...d.errors);
    await d.ctx.close();
  }
  // Reduced motion
  const rm = await open(size, { reducedMotion: true });
  await shot(rm.page, size, '09-reduced-motion-overview');
  check(await rm.page.evaluate(() => !window.__viewer.viewer.spinning), 'reduced motion: no auto-spin');
  await tapEl(rm.page, '.chip:nth-child(2)');
  await rm.page.waitForTimeout(400);
  await settle(rm.page);
  await shot(rm.page, size, '10-reduced-motion-focus');
  errors.push(...rm.errors);
  await rm.ctx.close();

  check(errors.length === 0, `no console errors${errors.length ? ': ' + errors.slice(0, 3).join(' | ') : ''}`);
  await ctx.close();
}

await browser.close();
await new Promise((r) => server.httpServer.close(r));

const glb = fs.statSync('public/models/chuck-norris.glb').size / 1e6;
console.log('\n=== Report ===');
console.log(`Model: ${glb.toFixed(2)} MB`);
if (report) console.log(`Draw calls (overview): ${report.calls}, triangles drawn: ${report.triangles.toLocaleString()}, model: ${report.model.triangles.toLocaleString()} tris in ${report.model.meshes} meshes (${report.model.source})`);
console.log(failures.length ? `\n${failures.length} check(s) FAILED` : '\nAll checks passed');
process.exit(failures.length ? 1 : 0);
