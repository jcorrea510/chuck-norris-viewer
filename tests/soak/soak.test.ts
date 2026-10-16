// Soak: 10 simulated minutes (SOAK_SECONDS) of randomized driving and shooting, with the robot
// handed back to its own brain now and then. Invariants checked every step, memory sampled.
//   npm run soak            (or SOAK_SECONDS=3600 npm run soak)
import { test, expect } from 'vitest';
import { makeWorld } from '../phys/helpers';
import { NO_INPUT, type DriverInput } from '../../src/sim/world';
import { FUEL, STEP_HZ } from '../../src/sim/phys/config';
import { Rng } from '../../src/sim/phys/rng';

const SECONDS = Number(process.env.SOAK_SECONDS ?? 600);

test(`soak: ${SECONDS} s of random driving`, async () => {
  const w = await makeWorld(11);
  const rng = new Rng(99);
  const f = w.fuel, r = w.robot;
  let inp: DriverInput = NO_INPUT, until = 0, lastScore = 0, problems: string[] = [];
  const mem: number[] = [];
  const wasm = () => (w.engine.R as unknown as { __wasm?: { memory: { buffer: ArrayBuffer } } }).__wasm?.memory?.buffer?.byteLength ?? 0;
  const steps = SECONDS * STEP_HZ;
  const t0 = performance.now();
  let worstStep = 0;
  for (let k = 0; k < steps; k++) {
    const t = k / STEP_HZ;
    if (t >= until) {
      // a new burst: drive somewhere, maybe shoot, maybe toggle the intake, or let go (the AI takes over after 15 s)
      const roll = rng.next();
      until = t + rng.range(0.3, 4);
      if (roll < 0.12) inp = NO_INPUT;
      else {
        const a = rng.range(0, Math.PI * 2), m = rng.range(0.2, 1);
        inp = { ...NO_INPUT, mx: Math.cos(a) * m, mz: Math.sin(a) * m, turn: rng.next() < 0.4 ? rng.range(-1, 1) : 0, shoot: rng.next() < 0.35, toggleIntake: rng.next() < 0.15, toggleArm: rng.next() < 0.05, lock: rng.next() < 0.05 };
      }
      if (roll > 0.97) { inp = NO_INPUT; until = t + 18; }  // long enough for the robot to drive itself
    }
    const s0 = performance.now();
    w.step(k === 0 || t < until - 1 / STEP_HZ ? { ...inp, toggleIntake: false, toggleArm: false } : inp);
    worstStep = Math.max(worstStep, performance.now() - s0);
    // invariants, every step
    if (!Number.isFinite(r.x + r.y + r.z + r.vx + r.vz + r.th)) problems.push(`robot NaN at ${t.toFixed(2)}`);
    if (Math.abs(r.x) > 8.3 || Math.abs(r.z) > 4.05) problems.push(`robot outside the field at ${t.toFixed(2)}: (${r.x.toFixed(2)}, ${r.z.toFixed(2)})`);
    if (w.hubs.scoredBy[0] < lastScore) problems.push(`score went down at ${t.toFixed(2)}`);
    lastScore = w.hubs.scoredBy[0];
    // balls, every 0.1 s
    if (k % (STEP_HZ / 10) === 0) {
      for (let i = 0; i < f.n; i++) {
        if (!f.isFree(i)) continue;
        const x = f.pos[i * 3], y = f.pos[i * 3 + 1], z = f.pos[i * 3 + 2];
        if (!Number.isFinite(x + y + z)) problems.push(`ball ${i} NaN at ${t.toFixed(2)}`);
        else if (Math.abs(x) > 8.35 || Math.abs(z) > 4.1 || y < FUEL.radius - 0.03 || y > 12) problems.push(`ball ${i} out at (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) t=${t.toFixed(2)}`);
        else if (Math.abs(x - -3.645) < 0.55 && Math.abs(z) < 0.55 && y < 1.3) problems.push(`ball ${i} inside the HUB body at t=${t.toFixed(2)}`);
      }
    }
    if (k % (STEP_HZ * 60) === 0) { (globalThis as { gc?: () => void }).gc?.(); mem.push(process.memoryUsage().heapUsed / 1e6 + wasm() / 1e6); }
    if (problems.length > 20) break;
  }
  const secs = (performance.now() - t0) / 1000;
  console.log(`soak ${SECONDS} s simulated in ${secs.toFixed(0)} s: scored ${w.hubs.scoredBy[0]}, shots ${r.shotsFired}, respawns? robot at (${r.x.toFixed(2)}, ${r.z.toFixed(2)}); worst step ${worstStep.toFixed(1)} ms; memory (MB) ${mem.map((m) => m.toFixed(0)).join(' ')}`);
  if (problems.length) console.log(problems.slice(0, 20).join('\n'));
  expect(problems).toEqual([]);
  if (mem.length >= 4) expect(mem[mem.length - 1] - mem[2]).toBeLessThan(40);
}, 3_600_000);
