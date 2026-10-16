#!/usr/bin/env node
// Opens the simulation in headless Chromium and saves screenshots of each camera (and any extra
// poses) for a visual check:  node scripts/simlook.mjs [outdir] [--debug] [--blue]
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';
const out = process.argv[2] && !process.argv[2].startsWith('--') ? process.argv[2] : 'shots/sim-look';
const debug = process.argv.includes('--debug');
const blue = process.argv.includes('--blue');
fs.mkdirSync(out, { recursive: true });
const server = await preview({ preview: { port: 4181, strictPort: true }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1180, height: 820 }, hasTouch: true });
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
if (blue) await page.addInitScript(() => localStorage.setItem('chuck-norris-settings-v2', JSON.stringify({ alliance: 'blue' })));
await page.goto(`http://localhost:4181/?physics=main${debug ? '&debug=physics' : ''}`);
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => window.__viewer.api.enterSim('free'));
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
const run = (s) => page.evaluate((s) => { const sim = window.__viewer.state.sim; for (let i = 0; i < s * 60; i++) sim.frame(1 / 60); }, s);
await run(6);
for (const v of ['driver', 'follow', 'overhead']) {
  await page.evaluate((v) => { const sim = window.__viewer.state.sim; sim.setView(v); for (let i = 0; i < 90; i++) sim.frame(1 / 60); }, v);
  await page.screenshot({ path: `${out}/${v}.png`, timeout: 240000 });
}
console.log(JSON.stringify(await page.evaluate(() => { const s = window.__viewer.state.sim; return { status: s.status(), robot: [s.robot.x, s.robot.z, s.robot.th] }; })));
console.log('errors', errors.slice(0, 5));
await browser.close(); server.httpServer.close();
