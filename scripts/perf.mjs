#!/usr/bin/env node
// Simulation performance in headless Chromium (software GL, so absolute frame times are pessimistic;
// compare before/after): 20 s of AI play, real rAF, the physics in its worker.
//   node scripts/perf.mjs <port> [seconds]
// Prints, per viewport (1280x800 @1x, 1024x768 @2x): fps, renderer frame ms (the sim's own frame()
// cost), worker physMs, draw calls, triangles, JS heap churn.
import { createServer } from 'vite';
import { chromium } from 'playwright';
const port = Number(process.argv[2]), secs = Number(process.argv[3] ?? 20);
const server = await createServer({ server: { port, strictPort: true, host: '127.0.0.1' }, logLevel: 'warn' });
await server.listen();
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
for (const [w, h, dpr] of [[1280, 800, 1], [1024, 768, 2]]) {
  const ctx = await browser.newContext({ viewport: { width: w, height: h }, deviceScaleFactor: dpr, hasTouch: false });
  const page = await ctx.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(`http://127.0.0.1:${port}/`);
  await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 180000 });
  await page.evaluate(() => window.__viewer.api.enterSim('free'));
  await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 180000 });
  await page.evaluate(() => window.__viewer.viewer.scene.traverse((o) => { if (o.userData.ghost) o.visible = false; }));
  await page.waitForTimeout(3000);
  const r = await page.evaluate(async (secs) => {
    const sim = window.__viewer.state.sim, v = window.__viewer.viewer, info = v.renderer.info;
    const s = { frames: 0, frameMs: [], phys: [], calls: [], tris: [], t0: performance.now() };
    // time each sim.frame() call
    const orig = sim.frame.bind(sim);
    sim.frame = (dt) => { const t = performance.now(); const r = orig(dt); s.frameMs.push(performance.now() - t); return r; };
    const heap0 = performance.memory?.usedJSHeapSize ?? 0;
    await new Promise((res) => {
      const tick = () => {
        s.frames++;
        s.phys.push(sim.f.physMs); s.calls.push(info.render.calls); s.tris.push(info.render.triangles);
        if (performance.now() - s.t0 < secs * 1000) requestAnimationFrame(tick); else res();
      };
      requestAnimationFrame(tick);
    });
    sim.frame = orig;
    const q = (a, p) => { const b = [...a].sort((x, y) => x - y); return +b[Math.min(b.length - 1, Math.floor(b.length * p))].toFixed(2); };
    const avg = (a) => +(a.reduce((x, y) => x + y, 0) / a.length).toFixed(2);
    return {
      fps: +(s.frames / ((performance.now() - s.t0) / 1000)).toFixed(1),
      simFrameMs: { avg: avg(s.frameMs), p95: q(s.frameMs, 0.95) }, physMs: { avg: avg(s.phys), p95: q(s.phys, 0.95) },
      drawCalls: q(s.calls, 0.5), triangles: Math.round(q(s.tris, 0.5)), heapGrowthMB: +(((performance.memory?.usedJSHeapSize ?? 0) - heap0) / 1e6).toFixed(1),
    };
  }, secs);
  console.log(`${w}x${h}@${dpr}`, JSON.stringify(r), errors.length ? errors.slice(0, 3) : '');
  await ctx.close();
}
await browser.close();
await server.close();
