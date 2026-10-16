import { test, expect } from 'vitest';
import { makeEngine } from './helpers';
import { Fuel, BS } from '../../src/sim/phys/fuel';
import { Rng } from '../../src/sim/phys/rng';
import { STEP } from '../../src/sim/phys/config';

async function one(x = -6, y = 1.075, z = -2) {
  const eng = await makeEngine(false);
  const f = new Fuel(eng, [[x, y, z]], new Rng(1), () => 0);
  const step = () => { f.preStep(); eng.step(); f.postStep(() => {}); };
  return { eng, f, step };
}

test('drop from 1 m onto carpet: first rebound', async () => {
  const { f, step } = await one();
  let max = 0, up = false;
  for (let k = 0; k < 2 * 240; k++) {
    step();
    const vy = f.bodies[0].linvel().y, y = f.pos[1] - 0.075;
    if (vy > 0.05) up = true;
    if (up && vy < 0 && max === 0) max = y;
  }
  console.log(`drop 1 m: rebound ${(max * 100).toFixed(1)} cm`);
  expect(max).toBeGreaterThan(0.18);
  expect(max).toBeLessThan(0.32);
});

test('roll at 2 m/s on carpet: roll-out distance', async () => {
  const { f, step } = await one(-6, 0.075, -2);
  f.bodies[0].setLinvel({ x: 2, y: 0, z: 0 }, true);
  f.bodies[0].setAngvel({ x: 0, y: 0, z: -2 / 0.075 }, true);
  let t = 0;
  while (t < 10) { step(); t += STEP; if (f.bodies[0].linvel().x < 0.01) break; }
  const d = f.pos[0] + 6;
  console.log(`roll 2 m/s: ${d.toFixed(2)} m in ${t.toFixed(2)} s`);
  expect(d).toBeGreaterThan(1.6);
  expect(d).toBeLessThan(2.4);
});
