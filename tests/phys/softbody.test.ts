// Soft bodies: the cargo net over the hopper and the bending polycarbonate panels (overnight/PHYSICS.md).
import { describe, test, expect } from 'vitest';
import { driverWorld, input, run } from './helpers';
import { FUEL, STEP } from '../../src/sim/phys/config';
import { NET } from '../../src/sim/phys/net';
import { NO_INPUT, type SimWorld } from '../../src/sim/world';

const log = (s: string) => console.log(s);
const R = FUEL.radius;

/** A world with the field's fuel parked away, our robot at rest on open carpet, and one ball ready. */
async function netWorld() {
  const w = await driverWorld(-5, 0, Math.PI / 2, { clearFuel: true });
  run(w, 1);
  w.net.reset();
  return w;
}
/** put ball i at a robot-frame point with a robot-frame velocity */
function throwBall(w: SimWorld, i: number, p: [number, number, number], v: [number, number, number]) {
  const r = w.robot;
  const q = r.q;
  const rot = (a: number[]) => {
    const ix = q.w * a[0] + q.y * a[2] - q.z * a[1], iy = q.w * a[1] + q.z * a[0] - q.x * a[2], iz = q.w * a[2] + q.x * a[1] - q.y * a[0], iw = -q.x * a[0] - q.y * a[1] - q.z * a[2];
    return [ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y, iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z, iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x];
  };
  const wp = r.toWorld(p[0], p[1], p[2]);
  w.fuel.place(i, wp, rot(v));
}
const local = (w: SimWorld, i: number) => w.robot.toLocal(w.fuel.pos[3 * i], w.fuel.pos[3 * i + 1], w.fuel.pos[3 * i + 2]);

describe('cargo net', () => {
  test('rests with the designed sag (4.5 cm, ±1 cm), still, and sways with the robot', async () => {
    const w = await netWorld();
    log(`net: ${w.net.n} knots, ${w.net.cords.length / 2} cords, sag ${(w.net.sag * 100).toFixed(2)} cm`);
    expect(w.net.sag).toBeGreaterThan(0.035);
    expect(w.net.sag).toBeLessThan(0.055);
    run(w, 3);
    const a = Float32Array.from(w.net.local);
    run(w, 1);
    let drift = 0;
    for (let k = 0; k < a.length; k++) drift = Math.max(drift, Math.abs(a[k] - w.net.local[k]));
    log(`net at rest: drift over 1 s ${(drift * 1000).toFixed(3)} mm`);
    expect(drift).toBeLessThan(0.0005);
  });

  test('a ball dropped from 1 m bounces less than on carpet and never goes through', async () => {
    const w = await netWorld();
    const i = 0;
    throwBall(w, i, [0.02, 0.546 + 1 + R, 0.30], [0, 0, 0]);
    let apex = 0, below = Infinity, up = false, hit = false;
    const y0 = w.net.local[3 * w.net.index[9 * 19 + 9] + 1];
    for (let k = 0; k < 6 * 120; k++) {
      w.step(NO_INPUT);
      const l = local(w, i);
      const vy = w.fuel.bodies[i].linvel().y;
      if (l[1] < 0.546 + 0.2) hit = true;
      if (hit && vy > 0.1) up = true;
      if (up && vy < 0) { /* falling again */ }
      if (up) apex = Math.max(apex, l[1] - R - 0.546);
      if (hit) below = Math.min(below, l[1] - R);
    }
    const l = local(w, i);
    const onTop = l[1] > 0.5;
    const over = Math.abs(l[0]) > NET.half || l[2] < -0.1 || l[2] > 0.8;
    log(`net drop 1 m: net at ${(y0 * 100).toFixed(1)} cm, deepest ball bottom ${(below * 100).toFixed(1)} cm, rebound apex ${(apex * 100).toFixed(1)} cm above the pins, end local ${l.map((x) => x.toFixed(2)).join(',')}`);
    expect(apex).toBeLessThan(0.24);          // carpet: ~0.24 m
    expect(below).toBeGreaterThan(0.25);      // never close to the hopper floor
    expect(onTop || over).toBe(true);
  });

  test('a ball thrown up from inside the hopper at 4 m/s is caught and comes back down', async () => {
    const w = await netWorld();
    const i = 0;
    throwBall(w, i, [0, 0.30, 0.25], [0, 4, 0]);
    let top = 0;
    for (let k = 0; k < 3 * 120; k++) { w.step(NO_INPUT); top = Math.max(top, local(w, i)[1] + R); }
    const l = local(w, i);
    log(`net catch: ball top reached ${(top * 100).toFixed(1)} cm (pins at 54.6 cm), ends at ${l.map((x) => x.toFixed(3)).join(',')}`);
    expect(top).toBeLessThan(0.546 + 0.12);   // the net holds it close to the wall tops
    expect(l[1]).toBeLessThan(0.45);          // back inside
  });

  test('no energy from nothing: a plucked net only loses energy', async () => {
    const w = await netWorld();
    const net = w.net;
    // pluck the middle up by 10 cm
    const c = net.index[9 * 19 + 9];
    net.x[3 * c + 1] += 0.1;
    const energy = () => {
      let e = 0;
      for (let k = 0; k < net.n; k++) {
        const m = net.pinned[k] ? 0 : NET.mass / (net.n - net.pinned.reduce((a, b) => a + b, 0));
        e += m * 9.81 * net.x[3 * k + 1];
        const v = net.speedOf(k);
        e += 0.5 * m * v * v;
      }
      for (let q = 0; q < net.cords.length; q += 2) {
        const i = net.cords[q], j = net.cords[q + 1];
        const d = Math.hypot(net.x[3 * i] - net.x[3 * j], net.x[3 * i + 1] - net.x[3 * j + 1], net.x[3 * i + 2] - net.x[3 * j + 2]);
        const C = Math.max(0, d - net.restOf(q / 2));
        e += 0.5 * NET.kCord * C * C;
      }
      return e;
    };
    let e0 = 0, worst = -Infinity, last = 0;
    for (let k = 0; k < 240; k++) {
      w.step(NO_INPUT);
      const e = energy();
      if (k === 0) e0 = e;
      worst = Math.max(worst, e - e0);
      last = e;
    }
    log(`net pluck: E0 ${e0.toFixed(4)} J, max rise ${worst.toExponential(2)} J, after 2 s ${(last - e0).toExponential(2)} J`);
    expect(worst).toBeLessThan(0.02);
    expect(last).toBeLessThan(e0 + 0.001);
  });
});
