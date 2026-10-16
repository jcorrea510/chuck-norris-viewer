// Prints the kicker ramp's solved swivel angle across the intake's travel: node scripts/swivel-trace.mjs
import { preview } from 'vite';
import { chromium } from 'playwright';
const server = await preview({ preview: { port: 4176 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 400, height: 300 } });
await page.goto('http://localhost:4176/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
const out = await page.evaluate(() => {
  const v = window.__viewer.viewer;
  const m = v.mechanisms.get('intake');
  const sw = m.nodes.filter((n) => n.role === 'swivel');
  const rows = [];
  for (let d = 0; d >= -73; d -= 1) {
    v.setMechanismNow('intake', d); v.syncPose();
    rows.push(`${d}:${sw.map((n) => ((n.swivelAngle ?? 0) * 57.2958).toFixed(0)).join('/')}`);
  }
  return `${sw.length} swivel nodes, bumperBox ${JSON.stringify(v.bumperBox)}\n` + rows.join(' ');
});
console.log(out);
await browser.close(); server.httpServer.close();
