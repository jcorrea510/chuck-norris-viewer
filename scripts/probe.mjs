// Quick visual probe: node scripts/probe.mjs [width] [height] [steps...]
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';
const [w = 1180, h = 820] = process.argv.slice(2, 4).map(Number);
const steps = process.argv.slice(4);
fs.mkdirSync('shots/probe', { recursive: true });
const server = await preview({ preview: { port: 4175 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const ctx = await browser.newContext({ viewport: { width: w, height: h }, hasTouch: true, serviceWorkers: 'block', colorScheme: 'dark' });
const page = await ctx.newPage();
const errors = [];
page.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') errors.push(m.type() + ': ' + m.text().slice(0, 400)); });
page.on('pageerror', (e) => errors.push('pageerror: ' + String(e).slice(0, 600)));
if (process.env.LIGHT) await page.addInitScript(() => localStorage.setItem('chuck-norris-settings-v1', JSON.stringify({ appearance: 'light' })));
await page.goto('http://localhost:4175/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; window.__viewer.viewer.spinning = false; });
const settle = async () => { await page.evaluate(() => window.__viewer.viewer.settle()); await page.waitForTimeout(500); await page.evaluate(() => window.__viewer.viewer.settle()); };
await settle();
for (const step of steps) {
  const i = step.indexOf('='); const name = i < 0 ? step : step.slice(0, i); const code = i < 0 ? '' : step.slice(i + 1);
  if (code) await page.evaluate(code);
  await settle();
  await page.screenshot({ path: `shots/probe/${name}.png`, timeout: 240000 });
  console.log('shot', name);
}
console.log(errors.length ? errors.join('\n') : 'no console errors');
await browser.close(); server.httpServer.close();
