import fs from 'node:fs';
import { Engine, initRapier, type CollisionIndex } from '../../src/sim/phys/engine';
import { SimWorld, NO_INPUT, type DriverInput } from '../../src/sim/world';
import { STEP_HZ } from '../../src/sim/phys/config';

export async function makeEngine(withField = true) {
  await initRapier();
  const eng = new Engine();
  if (withField) {
    const { bin, index } = collision();
    eng.addField(bin, index);
  }
  return eng;
}
export const fieldData = () => JSON.parse(fs.readFileSync('public/field/field-2026.json', 'utf8'));
export function collision() {
  const buf = fs.readFileSync('public/field/field-collision.bin');
  const bin = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength) as ArrayBuffer;
  const index = JSON.parse(fs.readFileSync('public/field/field-collision.json', 'utf8')) as CollisionIndex;
  return { bin, index };
}

export async function makeWorld(seed = 1) {
  await initRapier();
  return new SimWorld(fieldData(), collision(), seed);
}

/** A world where we drive (the AI is off), robot placed, field cleared of fuel near the robot if asked. */
export async function driverWorld(x: number, z: number, th: number, opts: { clearFuel?: boolean; intakeDown?: boolean } = {}) {
  const w = await makeWorld(1);
  w.takeOver();
  if (opts.clearFuel) for (let i = 0; i < w.fuel.n; i++) w.fuel.park(i, [20, 0, i * 0.01]);
  w.robot.place(x, z, th);
  w.robot.setIntakeNow(opts.intakeDown ?? true);
  // the preloads ride along
  if (!opts.clearFuel) w.robot.preload(w.preloadIdx);
  for (let k = 0; k < STEP_HZ / 2; k++) w.step(NO_INPUT);
  return w;
}

export const input = (p: Partial<DriverInput>): DriverInput => ({ ...NO_INPUT, ...p });
export function run(w: SimWorld, seconds: number, inp: DriverInput = NO_INPUT, each?: (t: number) => void) {
  const n = Math.round(seconds * STEP_HZ);
  for (let k = 0; k < n; k++) { w.step(inp); each?.(k / STEP_HZ); }
}

/** Deepest overlap (m, positive) between any robot collider and the field right now. */
export function robotPenetration(w: SimWorld) {
  const W = w.engine.world, b = w.robot.body;
  let worst = 0;
  for (let k = 0; k < b.numColliders(); k++) {
    const c = b.collider(k);
    W.contactPairsWith(c, (o) => {
      if (o.parent()?.handle === b.handle || !w.engine.surfaceOf.has(o.handle) || ['fuel'].includes(w.engine.surfaceOf.get(o.handle)!)) return;
      W.contactPair(c, o, (m) => { for (let i = 0; i < m.numContacts(); i++) worst = Math.max(worst, -m.contactDist(i)); });
    });
  }
  return worst;
}
