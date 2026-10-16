import { preview } from 'vite';
import { chromium } from 'playwright';
import { PNG } from 'pngjs';
const server = await preview({ preview: { port: 4185, strictPort: true }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1180, height: 820 } });
await page.addInitScript(() => { window.__simLocal = true; });
await page.goto('http://localhost:4185/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 180000 });
await page.evaluate(() => window.__viewer.api.enterSim('free'));
await page.waitForFunction(() => window.__viewer.state.sim?.active && window.__viewer.state.sim.backend?.frame, null, { timeout: 180000 });
await page.evaluate(() => { const sim = window.__viewer.state.sim; sim.setView('driver'); for (let i = 0; i < 240; i++) sim.frame(1 / 60); sim.paused = true; document.getElementById('ui').style.visibility = 'hidden'; });
await page.waitForTimeout(1500);
const shots = [];
for (let k = 0; k < 3; k++) { shots.push(await page.screenshot({ timeout: 240000 })); await page.waitForTimeout(600); }
const imgs = shots.map((b) => PNG.sync.read(b));
for (let k = 1; k < 3; k++) {
  const a = imgs[k - 1].data, b = imgs[k].data; let n = 0, minx = 1e9, maxx = -1, miny = 1e9, maxy = -1;
  for (let i = 0; i < a.length; i += 4) if (Math.abs(a[i] - b[i]) + Math.abs(a[i + 1] - b[i + 1]) + Math.abs(a[i + 2] - b[i + 2]) > 6) { n++; const p = i / 4, x = p % 1180, y = Math.floor(p / 1180); minx = Math.min(minx, x); maxx = Math.max(maxx, x); miny = Math.min(miny, y); maxy = Math.max(maxy, y); }
  console.log(`frame ${k - 1}→${k}: ${n} pixels differ, box x ${minx}-${maxx} y ${miny}-${maxy}`);
}
import fs from 'node:fs';
fs.writeFileSync('/tmp/claude-1000/-home-jackcorrea-chuck-norris-viewer/0bd85122-6393-47cd-9f6d-65a6542f0d10/scratchpad/flick0.png', shots[0]);
fs.writeFileSync('/tmp/claude-1000/-home-jackcorrea-chuck-norris-viewer/0bd85122-6393-47cd-9f6d-65a6542f0d10/scratchpad/flick1.png', shots[1]);
await browser.close(); server.httpServer.close();
