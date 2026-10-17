#!/usr/bin/env node
// Screenshots of the simulation from custom camera poses (robot-relative), for visual checks:
//   node scripts/simshot.mjs <port> <outdir> <shot>[,<shot>…]
// Shots: left, right, front, back, top, turret-close, hood-lo, hood-hi, <view id> (a sim camera).
// Serves the source with Vite's dev server (no build step); the sim is paused for the pose shots.
import { createServer } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';
const port = Number(process.argv[2]), out = process.argv[3], shots = process.argv[4].split(',');
fs.mkdirSync(out, { recursive: true });
const server = await createServer({ server: { port, strictPort: true, host: '127.0.0.1' }, logLevel: 'warn' });
await server.listen();
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1100, height: 760 } });
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
await page.addInitScript(() => { const s = document.createElement('style'); s.textContent = '#ui{display:none!important}'; document.addEventListener('DOMContentLoaded', () => document.head.append(s)); });
await page.goto(`http://127.0.0.1:${port}/?physics=main`);
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 180000 });
const cad = process.argv.includes('--cad');
if (!cad) {
  await page.evaluate(() => window.__viewer.api.enterSim('free'));
  await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 180000 });
  await page.evaluate(() => { const sim = window.__viewer.state.sim; for (let i = 0; i < 120; i++) sim.frame(1 / 60); });
}
// robot-relative poses: [camera offset, target offset] in the robot frame (+z front, +x left, +y up)
const POSES = {
  left: [[1.4, 0.8, 0.5], [0, 0.35, 0]],
  right: [[-1.4, 0.8, 0.5], [0, 0.35, 0]],
  front: [[0, 0.5, 1.7], [0, 0.35, 0]],
  back: [[0, 0.5, -1.7], [0, 0.35, 0]],
  'left-front': [[1.1, 0.55, 1.0], [0.25, 0.2, 0.45]],
  'left-mid': [[1.2, 0.5, 0.0], [0.25, 0.25, 0.0]],
  top: [[0, 1.8, 0.01], [0, 0.3, 0]],
  'turret-close': [[0.95, 0.75, 0.55], [0.2, 0.45, -0.3]],
  'hood-lo': [[0.9, 0.8, 0.6], [0.2, 0.45, 0.0]],
  'hood-hi': [[0.9, 0.8, 0.6], [0.2, 0.45, 0.0]],
};
const hideArg = process.argv.find((a) => a.startsWith('--hide='));
if (hideArg) await page.evaluate((ids) => { const v = window.__viewer.viewer; for (const id of ids) v.setHidden(v.subsystemPids(id), true); }, hideArg.slice(7).split(','));
const hidePid = process.argv.find((a) => a.startsWith('--hidepid='));
if (hidePid) await page.evaluate((ids) => window.__viewer.viewer.setHidden(ids, true), hidePid.slice(10).split(',').map(Number));
const bboxPid = process.argv.find((a) => a.startsWith('--bbox='));
if (bboxPid) console.log('bbox', JSON.stringify(await page.evaluate((pid) => {
  const v = window.__viewer.viewer, out = [];
  v.stage.updateMatrixWorld(true);
  const inv = v.stage.matrixWorld.clone().invert();
  for (const m of v.meshes) {
    const a = m.geometry.getAttribute('_partid'), pos = m.geometry.getAttribute('position');
    if (!a) continue;
    const mw = inv.clone().multiply(m.matrixWorld), p = v.camera.position.clone();
    const lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9]; let n = 0;
    for (let i = 0; i < a.count; i++) if (a.getX(i) === pid) { p.fromBufferAttribute(pos, i).applyMatrix4(mw); n++; for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], [p.x, p.y, p.z][k]); hi[k] = Math.max(hi[k], [p.x, p.y, p.z][k]); } }
    if (n) out.push({ mesh: m.name, parent: m.parent?.name, n, lo, hi });
  }
  return out;
}, Number(bboxPid.slice(7)))));
for (const s of shots) {
  if (POSES[s]) {
    await page.evaluate(([cad, [c, t]]) => {
      const v = window.__viewer.viewer, sim = window.__viewer.state.sim;
      if (sim) sim.paused = true;
      const base = cad ? v.boxOfParts(v.subsystemPids('swerve'), false).getCenter(v.camera.position.clone()) : null;
      const toWorld = (o) => cad ? base.clone().add(v.camera.position.clone().set(o[0], o[1], o[2])) : v.camera.position.clone().set(o[0], o[1], o[2]).applyMatrix4(v.stage.matrixWorld);
      const p = toWorld(c), q = toWorld(t);
      v.setFov(40);
      v.controls.setLookAt(p.x, p.y, p.z, q.x, q.y, q.z, false);
      v.invalidate(true);
    }, [cad, POSES[s]]);
    if (s.startsWith('hood-')) await page.evaluate((deg) => { const v = window.__viewer.viewer; for (const side of ['left', 'right']) { v.setMechanismNow(`hood-${side}`, deg - 17.2); v.setMechanismNow(`hoodgear-${side}`, -(deg - 17.2) * 16); } v.syncPose(); v.invalidate(true); }, s === 'hood-lo' ? 13 : 29.85);
  } else {
    await page.evaluate((id) => { const sim = window.__viewer.state.sim; sim.paused = false; sim.setView(id); for (let i = 0; i < 120; i++) sim.frame(1 / 60); sim.paused = true; window.__viewer.viewer.invalidate(true); }, s);
  }
  await page.waitForTimeout(1500);
  await page.screenshot({ path: `${out}/${s}.png`, timeout: 240000 });
  console.log('shot', s);
  const pickArg = process.argv.find((a) => a.startsWith('--pick='));
  if (pickArg) for (const xy of pickArg.slice(7).split(';')) {
    const [x, y] = xy.split(',').map(Number);
    console.log('pick', xy, JSON.stringify(await page.evaluate(([x, y]) => {
      const v = window.__viewer.viewer, r = v.canvas.getBoundingClientRect();
      const ndc = v.camera.position.clone().set(((x - r.left) / r.width) * 2 - 1, -((y - r.top) / r.height) * 2 + 1, 0.5);
      v.raycaster.setFromCamera(ndc, v.camera);
      const hits = v.raycaster.intersectObjects(v.meshes.filter((m) => m.visible), false);
      return hits.slice(0, 3).map((h) => { const a = h.object.geometry.getAttribute('_partid'); return { pid: a && h.face ? a.getX(h.face.a) : -1, d: +h.distance.toFixed(3) }; });
    }, [x, y])));
  }
}
console.log('errors', errors.slice(0, 5));
await browser.close();
await server.close();
