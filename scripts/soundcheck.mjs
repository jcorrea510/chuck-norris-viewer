#!/usr/bin/env node
// Renders every sound offline through the real engine and prints its peak and loudness, so none
// clip, none vanish, and the mix stays balanced: node scripts/soundcheck.mjs
import { preview } from 'vite';
import { chromium } from 'playwright';
const server = await preview({ preview: { port: 4170 }, build: process.env.OUTDIR ? { outDir: process.env.OUTDIR } : undefined, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: 400, height: 300 } });
const errors = [];
page.on('pageerror', (e) => errors.push(String(e)));
await page.goto('http://localhost:4170/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
const out = await page.evaluate(async () => {
  const snd = window.__viewer.sound;
  const kinds = ['tap', 'select', 'focus', 'back', 'hide', 'show', 'toggle', 'open', 'boot', 'error', 'shot', 'score', 'intake', 'bounce', 'impact', 'takeover', 'handback', 'respawn', 'connect'];
  const rows = [];
  const measure = async (fn, secs) => {
    const Off = class extends OfflineAudioContext { constructor() { super(2, Math.round(44100 * secs), 44100); } get state() { return 'running'; } resume() { return Promise.resolve(); } };
    const saved = window.AudioContext;
    window.AudioContext = Off;
    snd.ctx = null; snd.sim = null; snd.last = new Map();
    snd.enabled = true; snd.volume = 0.6;
    snd.unlock();
    const ctx = snd.ctx;
    fn(snd);
    window.AudioContext = saved;
    const buf = await ctx.startRendering();
    let peak = 0, sum = 0;
    for (let c = 0; c < 2; c++) { const d = buf.getChannelData(c); for (let i = 0; i < d.length; i++) { const v = Math.abs(d[i]); if (v > peak) peak = v; sum += d[i] * d[i]; } }
    const rms = Math.sqrt(sum / (buf.length * 2));
    return { peak: 20 * Math.log10(peak || 1e-9), rms: 20 * Math.log10(rms || 1e-9) };
  };
  for (const k of kinds) { const m = await measure((s) => s.play(k), 2); rows.push(`${k.padEnd(9)} peak ${m.peak.toFixed(1).padStart(6)} dBFS  rms ${m.rms.toFixed(1).padStart(6)} dB`); }
  // rapid fire: ten shots in a second
  const rf = await measure((s) => { const c = s.ctx; for (let i = 0; i < 10; i++) setTimeout(() => {}, 0); for (let i = 0; i < 10; i++) { s.last = new Map(); const t0 = c.currentTime; s.play('shot'); } }, 1);
  rows.push(`10 shots stacked: peak ${rf.peak.toFixed(1)} dBFS`);
  // the robot's machinery at full tilt
  const m = await measure((s) => { s.simStart(); s.simUpdate({ speed: 4.6, turn: 3, effort: 1, slipping: true, flywheels: [420, 425], rollers: 1, arm: 1, feeding: 1, near: 1, pan: 0 }); }, 2);
  rows.push(`machinery full: peak ${m.peak.toFixed(1)} dBFS  rms ${m.rms.toFixed(1)} dB`);
  const idle = await measure((s) => { s.simStart(); s.simUpdate({ speed: 0, turn: 0, effort: 0, slipping: false, flywheels: [230, 230], rollers: 1, arm: 0, feeding: 0, near: 0.5, pan: 0 }); }, 2);
  rows.push(`machinery idle: peak ${idle.peak.toFixed(1)} dBFS  rms ${idle.rms.toFixed(1)} dB`);
  return rows;
});
console.log(out.join('\n'));
console.log(errors.length ? 'ERRORS:\n' + errors.join('\n') : 'no page errors');
await browser.close(); server.httpServer.close();
