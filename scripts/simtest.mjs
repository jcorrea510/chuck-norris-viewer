#!/usr/bin/env node
// Headless simulation soak test: runs the real physics (no rendering) for a long time.
//   node scripts/simtest.mjs [endless|auto] [simulatedSeconds]
import { preview } from 'vite';
import { chromium } from 'playwright';
const mode = process.argv[2] ?? 'endless';
const seconds = Number(process.argv[3] ?? 600);
const server = await preview({ preview: { port: 4178 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 640, height: 480 } });
const errors = [];
page.on('pageerror', (e) => errors.push(String(e)));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
await page.goto('http://localhost:4178/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; });
await page.evaluate((m) => window.__viewer.api.enterSim(m), mode);
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
// stop the render loop from doing sim work; we drive it by hand
const t0 = Date.now();
const report = await page.evaluate(async ({ seconds }) => {
  const sim = window.__viewer.state.sim;
  const v = window.__viewer.viewer;
  v.hooks.clear();
  const out = [];
  const dt = 1 / 60;
  let lastScored = 0, maxGap = 0, gapStart = 0, nan = 0, escaped = 0, clip = 0;
  const g = sim.grid;
  for (let f = 0; f < seconds * 60; f++) {
    sim.frame(dt);
    const t = (f + 1) * dt;
    const r = sim.robot;
    if (g.distAt(r.x, r.z) < 0.34) clip++;
    if (sim.balls.scored !== lastScored) { maxGap = Math.max(maxGap, t - gapStart); gapStart = t; lastScored = sim.balls.scored; }
    if (f % 600 === 599) {
      const st = sim.balls.st, pos = sim.balls.pos;
      const counts = [0, 0, 0, 0, 0, 0, 0];
      for (let i = 0; i < sim.balls.n; i++) {
        counts[st[i]]++;
        if (!Number.isFinite(pos[i * 3]) || !Number.isFinite(pos[i * 3 + 1])) nan++;
        if (st[i] !== 6 && (Math.abs(pos[i * 3]) > 8.4 || Math.abs(pos[i * 3 + 2]) > 4.2 || pos[i * 3 + 1] < 0)) escaped++;
      }
      out.push(`${t.toFixed(0).padStart(5)}s scored=${String(sim.balls.scored).padStart(4)} robot=(${r.x.toFixed(2)},${r.z.toFixed(2)}) load=${String(r.load).padStart(2)} phase="${sim.brain.phase}" rest=${counts[0]} moving=${counts[1]} stored=${counts[2]} flying=${counts[3]} hub=${counts[4]}`);
      await new Promise((res) => setTimeout(res, 0));
    }
  }
  maxGap = Math.max(maxGap, seconds - gapStart);
  return { out, maxGap, nan, escaped, clip, shots: sim.robot.shotsFired, scored: sim.balls.scored };
}, { seconds });
console.log(report.out.join('\n'));
console.log(`\nshots=${report.shots} scored=${report.scored} accuracy=${(report.scored / Math.max(1, report.shots) * 100).toFixed(1)}%  longest scoring gap=${report.maxGap.toFixed(1)}s  NaN=${report.nan} escaped=${report.escaped} wallClipFrames=${report.clip}`);
console.log(`real time ${((Date.now() - t0) / 1000).toFixed(1)}s`, errors.length ? '\nERRORS:\n' + errors.join('\n') : '');
await browser.close(); server.httpServer.close();
