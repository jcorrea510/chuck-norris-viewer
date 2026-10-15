#!/usr/bin/env node
// Prints the robot's state every half second of simulated time: node scripts/simtrace.mjs [auto|endless] [seconds]
import { preview } from 'vite';
import { chromium } from 'playwright';
const mode = process.argv[2] ?? 'auto';
const seconds = Number(process.argv[3] ?? 20);
const server = await preview({ preview: { port: 4179 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 640, height: 480 } });
page.on('pageerror', (e) => console.log('pageerror', String(e)));
await page.goto('http://localhost:4179/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate((m) => window.__viewer.api.enterSim(m), mode);
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
const out = await page.evaluate(({ seconds }) => {
  const sim = window.__viewer.state.sim;
  window.__viewer.viewer.hooks.clear();
  const lines = [];
  for (let f = 0; f < seconds * 60; f++) {
    sim.frame(1 / 60);
    if (f % 30 === 29) {
      const r = sim.robot;
      lines.push(`${((f + 1) / 60).toFixed(1).padStart(5)}s (${r.x.toFixed(2)},${r.z.toFixed(2)}) th=${(r.th * 57.3).toFixed(0)} v=${r.speed.toFixed(2)} load=${r.load} intake=${r.intakeAngle.toFixed(0)} scored=${sim.balls.scored} "${sim.brain.phase}"`);
    }
  }
  return lines;
}, { seconds });
console.log(out.join('\n'));
await browser.close(); server.httpServer.close();
