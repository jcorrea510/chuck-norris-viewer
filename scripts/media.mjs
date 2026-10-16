#!/usr/bin/env node
// Frame sequences of the physics for the morning report: node scripts/media.mjs [outdir]
// Each scenario is set up through the world API (physics on this thread, ?physics=main), stepped
// in exact time, and photographed from a fixed camera every few frames.
import { preview } from 'vite';
import { chromium } from 'playwright';
import fs from 'node:fs';

const out = process.argv[2] ?? 'overnight/media/frames';
fs.mkdirSync(out, { recursive: true });
const server = await preview({ preview: { port: 4190, strictPort: true }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 960, height: 600 } });
page.on('pageerror', (e) => console.log('pageerror', String(e)));
await page.addInitScript(() => { window.__simLocal = true; localStorage.setItem('chuck-norris-settings-v2', JSON.stringify({ alliance: 'red', pitMode: 0 })); });
await page.goto('http://localhost:4190/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 180000 });
await page.evaluate(() => window.__viewer.api.enterSim('free'));
await page.waitForFunction(() => window.__viewer.state.sim?.active && window.__viewer.state.sim.backend?.frame, null, { timeout: 180000 });
await page.waitForFunction(() => !window.__viewer.viewer.revealing, null, { timeout: 400000 });
await page.evaluate(() => { document.getElementById('ui').style.visibility = 'hidden'; window.__viewer.state.idleReset = false; });

/** Runs a scenario: setup(sim) in the page, then `frames` shots `every` sim-frames apart from a fixed camera. */
async function scenario(name, setup, cam, frames, every, input = {}) {
  await page.evaluate(([setup, input]) => {
    const sim = window.__viewer.state.sim, w = sim.world;
    w.setMode('free');
    w.takeOver();
    sim.setTouchControls(true);   // the on-screen controls carry our input (hidden pads zero it)
    new Function('sim', 'w', setup)(sim, w);
    sim.paused = false;
    Object.assign(sim.controls.touch, { mx: 0, my: 0, turn: 0, shoot: false, ...input });
    sim.frame(1 / 60);
  }, [setup, input]);
  for (let k = 0; k < frames; k++) {
    await page.evaluate(([cam, every, input]) => {
      const sim = window.__viewer.state.sim;
      sim.paused = false;
      for (let i = 0; i < every; i++) { Object.assign(sim.controls.touch, input); sim.frame(1 / 60); }
      // hold the simulation (and its camera rig) while this frame is photographed from our camera
      sim.paused = true;
      window.__viewer.viewer.controls.setLookAt(...cam, false);
      window.__viewer.viewer.controls.update(0);
      window.__viewer.viewer.invalidate(true);
    }, [cam, every, input]);
    await page.waitForTimeout(250);
    await page.screenshot({ path: `${out}/${name}-${String(k).padStart(2, '0')}.png`, timeout: 240000 });
  }
  console.log('done', name);
}

// a ferry shot from the neutral zone, seen from the side: drag bends the arc steeper on the way down
await scenario('ferry', `w.robot.place(0.4, -1.5, Math.PI / 2); w.robot.preload(w.preloadIdx);`, [-2.5, 2.2, -9.5, -2.5, 1.5, -1.0], 8, 12, { shoot: true });
// shots from our zone into the HUB, some over it into the net
await scenario('net', `w.robot.place(-7.2, 0.15, Math.PI / 2); w.robot.preload(w.preloadIdx);`, [-2.0, 2.6, -3.2, -3.0, 2.0, 0.0], 8, 9, { shoot: true });
// stowed intake into a TRENCH arm: stopped (a short wheelie)
await scenario('trench-blocked', `w.robot.place(-6.0, -3.38, Math.PI / 2); w.robot.setIntakeNow(false); for (let i = 0; i < w.fuel.n; i++) { const x = w.fuel.pos[i*3], z = w.fuel.pos[i*3+2]; if (z < -2.5 && x < -2 && x > -7) w.fuel.park(i, [20, 0, 0]); }`, [-5.1, 1.15, -1.9, -3.9, 0.45, -3.38], 8, 6, { my: 1 });
// intake down: under the TRENCH
await scenario('trench-under', `w.robot.place(-6.0, -3.38, Math.PI / 2); w.robot.setIntakeNow(true); for (let i = 0; i < w.fuel.n; i++) { const x = w.fuel.pos[i*3], z = w.fuel.pos[i*3+2]; if (z < -2.5 && x < -2 && x > -7) w.fuel.park(i, [20, 0, 0]); }`, [-5.1, 1.15, -1.9, -3.4, 0.45, -3.38], 8, 8, { my: 1 });
// over a BUMP at speed, side view
await scenario('bump', `w.robot.place(-6.0, 1.5, Math.PI / 2);`, [-5.4, 0.9, 3.3, -3.6, 0.2, 1.5], 10, 5, { my: 1 });
// the hopper full of FUEL while the robot spins in place: it sloshes outward
await scenario('hopper', `w.robot.place(-6.5, -1.6, Math.PI / 2); const idx = []; for (let i = 0; i < w.fuel.n && idx.length < 30; i++) { const x = w.fuel.pos[i*3]; if (w.fuel.st[i] === 0 && x > -1 && x < 1) idx.push(i); } w.robot.preload(idx.slice(0, 24));`, [-6.5, 2.1, -0.3, -6.5, 0.3, -1.6], 10, 6, { turn: 1 });

await browser.close();
server.httpServer.close();
