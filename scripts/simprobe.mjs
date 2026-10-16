#!/usr/bin/env node
// Simulation screenshots and a keyboard-driving check: node scripts/simprobe.mjs [width] [height]
// Steps the simulation by hand (fast even with software rendering), shoots shots/sim/*.png.
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';
const [w = 1180, h = 820] = process.argv.slice(2, 4).map(Number);
const touch = process.argv.includes('--touch');
fs.mkdirSync('shots/sim', { recursive: true });
const server = await preview({ preview: { port: 4174 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const ctx = await browser.newContext({ viewport: { width: w, height: h }, hasTouch: touch, isMobile: touch, serviceWorkers: 'block', colorScheme: 'dark' });
const page = await ctx.newPage();
const errors = [];
page.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') errors.push(m.type() + ': ' + m.text().slice(0, 300)); });
page.on('pageerror', (e) => errors.push('pageerror: ' + String(e).slice(0, 600)));
await page.goto('http://localhost:4174/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; });
await page.evaluate(() => window.__viewer.api.enterSim('endless'));
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
const step = (secs) => page.evaluate((n) => { const sim = window.__viewer.state.sim; for (let i = 0; i < n; i++) sim.frame(1 / 60); }, Math.round(secs * 60));
const shot = async (name) => { await page.evaluate(() => { const v = window.__viewer.viewer; for (let i = 0; i < 20; i++) v.controls.update(0.2); v.invalidate?.(); }); await page.waitForTimeout(600); await page.screenshot({ path: `shots/sim/${name}.png`, timeout: 240000 }); console.log('shot', name); };
const info = () => page.evaluate(() => { const s = window.__viewer.state.sim, r = s.robot; return `t=${s.world.clock.toFixed(1)} ctl=${s.world.control} pos=(${r.x.toFixed(2)},${r.z.toFixed(2)}) th=${(r.th * 57.3).toFixed(0)} v=${r.speed.toFixed(2)} load=${r.load} scored=${s.balls.scored} intake=${r.intakeAngle.toFixed(0)} phase="${s.status().phase}" chip="${document.querySelector('.sim-driver')?.hidden ? '' : document.querySelector('.sim-driver-text')?.textContent}"`; });

await step(8);
console.log(await info());
await shot('endless-tv');
await page.evaluate(() => window.__viewer.state.sim.setView('follow', false));
await step(3);
await shot('endless-chase');
// close look at the robot holding fuel
await page.evaluate(() => { const s = window.__viewer.state.sim; s.setView('tv', false); s.paused = true; const r = s.robot; window.__viewer.viewer.controls.setLookAt(r.x + 1.6, 1.5, r.z + 1.4, r.x, 0.3, r.z, false); s.paused = false; });
await step(1 / 60);
await page.evaluate(() => { const s = window.__viewer.state.sim; const r = s.robot; window.__viewer.viewer.controls.setLookAt(r.x + 1.6, 1.5, r.z + 1.4, r.x, 0.3, r.z, false); s.paused = true; });
await shot('robot-close');
await page.evaluate(() => { window.__viewer.state.sim.paused = false; window.__viewer.state.sim.setView('tv', false); });

// drive with the keyboard: W (forward relative to the camera) + E (turn) for 2 s
if (!touch) {
  await page.keyboard.down('KeyW');
  for (let k = 0; k < 4; k++) { await step(0.5); }
  console.log('W held 2s:', await info());
  await page.keyboard.down('KeyF');
  await step(1);
  await page.keyboard.up('KeyW');
  await page.keyboard.up('KeyF');
  await page.keyboard.down('Space');
  await step(1);
  console.log('shooting:', await info());
  await shot('driving');
  await page.keyboard.up('Space');
  await step(10);
  console.log('10 s after letting go:', await info());
  await step(6);
  console.log('16 s after letting go:', await info());
}
await page.click('.sim-controls [data-act=controls]').catch((e) => console.log('controls click failed', String(e).slice(0, 200)));
await page.waitForTimeout(3000);
console.log('sheet open:', await page.evaluate(() => !document.querySelector('.sheet-backdrop').hidden && document.querySelector('.sheet h2')?.textContent));
await shot('controls-page');
console.log(errors.length ? errors.join('\n') : 'no console errors');
await browser.close(); server.httpServer.close();
