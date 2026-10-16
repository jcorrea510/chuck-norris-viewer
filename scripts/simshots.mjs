#!/usr/bin/env node
// Close-ups of the robot handling fuel in Endless: node scripts/simshots.mjs → shots/sim/close-*.png
// Waits for moments (intaking, a full hopper, shooting) and frames the robot from fixed robot-relative angles.
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';
fs.mkdirSync('shots/sim', { recursive: true });
const server = await preview({ preview: { port: 4172 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1000, height: 700 } });
const errors = [];
page.on('pageerror', (e) => errors.push('pageerror: ' + String(e).slice(0, 600)));
await page.goto('http://localhost:4172/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; document.getElementById('ui').style.opacity = '0'; });
await page.evaluate(() => window.__viewer.api.enterSim('endless'));
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
// run until a condition holds (max seconds of sim time)
const until = (cond, max) => page.evaluate(({ cond, max }) => {
  const s = window.__viewer.state.sim; const f = new Function('s', 'r', `return (${cond})`);
  for (let i = 0; i < max * 60; i++) { s.frame(1 / 60); if (f(s, s.robot)) return true; }
  return false;
}, { cond, max });
/** camera in the robot's frame: (right, up, forward) offset, looking at a robot-frame point */
const frame = (name, off, look) => page.evaluate(({ off, look }) => {
  const s = window.__viewer.state.sim, r = s.robot, v = window.__viewer.viewer;
  s.paused = true;
  const c = Math.cos(r.th), sn = Math.sin(r.th);
  const w = (x, y, z) => [r.x + x * c + z * sn, r.py + y, r.z - x * sn + z * c];
  const p = w(...off), t = w(...look);
  v.controls.setLookAt(p[0], p[1], p[2], t[0], t[1], t[2], false);
  for (let i = 0; i < 5; i++) v.controls.update(0.1);
}, { off, look }).then(async () => { await page.waitForTimeout(900); await page.screenshot({ path: `shots/sim/close-${name}.png`, timeout: 240000 }); await page.evaluate(() => { window.__viewer.state.sim.paused = false; }); console.log('shot', name); });

console.log('intaking:', await until('r.intaking.length >= 2', 40));
await frame('intake-side', [1.5, 0.45, 0.9], [0, 0.2, 0.55]);
console.log('loaded:', await until('r.load >= 20', 60));
await frame('hopper-top', [0.9, 1.6, -0.6], [0, 0.25, 0.2]);
await frame('hopper-side', [1.3, 0.7, 0.2], [0, 0.3, 0.15]);
console.log('shooting:', await until('r.turrets.some((t) => t.lane[0]?.firing)', 60));
await frame('turret', [0.9, 1.1, -1.3], [0, 0.45, -0.15]);
await until('false', 0.12);
await frame('shot-arc', [2.5, 1.5, -2.2], [0, 0.6, 0]);
console.log(errors.length ? errors.join('\n') : 'no page errors');
await browser.close(); server.httpServer.close();
