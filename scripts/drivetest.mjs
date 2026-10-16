#!/usr/bin/env node
// Driving checks in the real page: a (simulated) game controller, the on-screen touch sticks and
// buttons, the hand-back to the robot, and the sound engine running with all of it.
//   node scripts/drivetest.mjs
import { preview } from 'vite';
import { chromium } from 'playwright';
const server = await preview({ preview: { port: 4171 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', '--autoplay-policy=no-user-gesture-required'] });
const ctx = await browser.newContext({ viewport: { width: 1180, height: 820 }, hasTouch: true, serviceWorkers: 'block' });
const page = await ctx.newPage();
const errors = [];
page.on('pageerror', (e) => errors.push('pageerror: ' + String(e).slice(0, 600)));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text().slice(0, 300)); });
// a fake standard-mapping controller the test can move
await page.addInitScript(() => {
  const pad = { id: 'Test Controller (STANDARD GAMEPAD)', index: 0, connected: true, mapping: 'standard', timestamp: 0,
    axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => ({ pressed: false, touched: false, value: 0 })) };
  window.__pad = pad;
  navigator.getGamepads = () => [pad, null, null, null];
});
await page.goto('http://localhost:4171/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => { const s = window.__viewer.state; s.autoSpin = false; s.idleReset = false; });
await page.mouse.click(600, 300); // a gesture unlocks audio
await page.evaluate(() => window.__viewer.api.enterSim('endless'));
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
const step = (secs) => page.evaluate((n) => { const s = window.__viewer.state.sim; for (let i = 0; i < n; i++) s.frame(1 / 60); }, Math.round(secs * 60));
const st = () => page.evaluate(() => { const s = window.__viewer.state.sim, r = s.robot; return { ctl: s.world.control, x: r.x, z: r.z, v: r.speed, shots: r.shotsFired, down: r.intakeDown, load: r.load, mode: s.mode, phase: s.status().phase }; });
let fails = 0;
const check = (ok, msg) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${msg}`); if (!ok) fails++; };

await step(4);
// ---- controller
let a = await st();
await page.evaluate(() => { window.__pad.axes[1] = -1; });
await step(1.2);
let b = await st();
check(b.ctl === 'driver', `controller stick takes over (${b.ctl})`);
check(Math.hypot(b.x - a.x, b.z - a.z) > 1.0 || b.v > 2, `controller drives the robot (${Math.hypot(b.x - a.x, b.z - a.z).toFixed(2)} m, ${b.v.toFixed(2)} m/s)`);
await page.evaluate(() => { window.__pad.axes[1] = 0; window.__pad.buttons[0].pressed = true; window.__pad.buttons[0].value = 1; });
await step(0.1);
await page.evaluate(() => { window.__pad.buttons[0].pressed = false; window.__pad.buttons[0].value = 0; });
await step(0.7);
const c = await st();
check(c.down !== b.down, `A toggles the intake (down ${b.down} → ${c.down})`);
// drive to the alliance zone with fuel and shoot with RT
await page.evaluate(() => { const s = window.__viewer.state.sim, r = s.robot; s.world.respawn(); r.drive.place(-6.0, 1.0, Math.PI / 2); r.setIntakeNow(true); const pre = []; for (let i = 0; i < s.balls.n && pre.length < 10; i++) if (s.balls.st[i] === 0) pre.push(i); r.preload(pre); });
await page.evaluate(() => { window.__pad.buttons[7].pressed = true; window.__pad.buttons[7].value = 1; });
await step(2.5);
const d = await st();
check(d.shots >= 4, `RT shoots (${d.shots} shots, phase "${d.phase}")`);
await page.evaluate(() => { window.__pad.buttons[7].pressed = false; window.__pad.buttons[7].value = 0; });
// ---- hand back with Menu
await page.evaluate(() => { window.__pad.buttons[9].pressed = true; });
await step(0.1);
await page.evaluate(() => { window.__pad.buttons[9].pressed = false; });
await step(0.5);
check((await st()).ctl === 'ai', 'Menu gives control back to the robot');

// ---- touch: drag the left stick up, hold Shoot
await page.evaluate(() => window.__viewer.state.sim.setTouchControls(true));
await step(0.1);
const zone = await page.locator('.dp-move').boundingBox();
const before = await st();
const cx = zone.x + zone.width / 2, cy = zone.y + zone.height * 0.62;
const cdp = await ctx.newCDPSession(page);
await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: cx, y: cy, id: 1 }] });
await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: cx, y: cy - 60, id: 1 }] });
await step(1.2);
const after = await st();
check(after.ctl === 'driver', `touch stick takes over (${after.ctl})`);
check(Math.hypot(after.x - before.x, after.z - before.z) > 0.8, `touch stick drives (${Math.hypot(after.x - before.x, after.z - before.z).toFixed(2)} m)`);
await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
await step(0.2);
const tstate = await page.evaluate(() => window.__viewer.state.sim.controls.touch);
check(tstate.mx === 0 && tstate.my === 0, 'stick re-centers on release');
// arm button
const arm = await page.locator('.dp-arm').boundingBox();
const downBefore = (await st()).down;
await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: arm.x + arm.width / 2, y: arm.y + arm.height / 2, id: 2 }] });
await step(0.1);
await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
await step(0.6);
check((await st()).down !== downBefore, 'Raise/Lower button moves the intake');
// ---- after 15 s untouched the robot drives itself
await step(15.5);
check((await st()).ctl === 'ai', 'robot takes back over 15 s after the last input');
// ---- auto: touching a control ends Auto and hands over the robot
await page.evaluate(() => window.__viewer.state.sim.setMode('auto'));
await step(2);
await page.keyboard.down('KeyD');
await step(0.5);
await page.keyboard.up('KeyD');
const e = await st();
check(e.ctl === 'driver' && e.mode === 'endless', `driving during Auto hands the robot to you (${e.ctl}, ${e.mode})`);
check(await page.evaluate(() => !!document.querySelector('.sim-driver') && !document.querySelector('.sim-driver').hidden), 'HUD shows the driver chip');
console.log(errors.length ? 'ERRORS:\n' + errors.join('\n') : 'no page errors');
console.log(fails ? `${fails} FAILED` : 'all passed');
await browser.close(); server.httpServer.close();
process.exit(fails || errors.length ? 1 : 0);
