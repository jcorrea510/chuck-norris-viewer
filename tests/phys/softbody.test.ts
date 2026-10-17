// Soft bodies: the cargo net over the hopper and the bending polycarbonate panels (overnight/PHYSICS.md).
import { describe, test, expect } from 'vitest';
import { driverWorld, input, run } from './helpers';
import { FUEL, STEP } from '../../src/sim/phys/config';
import { NET } from '../../src/sim/phys/net';
import { PANELS, PLATE } from '../../src/sim/phys/panels';
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

  test('60 s of hard driving, shooting and intake toggles: bounded, finite, cheap', async () => {
    const w = await driverWorld(-5.2, 0, Math.PI / 2, { intakeDown: true });
    const net = w.net;
    const pre = net.preStep.bind(net);
    let ms = 0, steps = 0, worstMs = 0;
    net.preStep = (f) => { const t = performance.now(); pre(f); const d = performance.now() - t; ms += d; steps++; worstMs = Math.max(worstMs, d); };
    let vmax = 0, box = 0, contacts = 0, finite = true;
    for (let k = 0; k < 60 * 120; k++) {
      // yield every 10 simulated seconds: a synchronous block over a minute trips vitest's worker RPC timeout
      if (k % 1200 === 0) await new Promise((r) => setTimeout(r, 0));
      const t = k / 120;
      const inp = input({ mx: Math.sin(t * 0.9), mz: Math.cos(t * 1.3), turn: Math.sin(t * 0.7) > 0 ? 1 : -1, shoot: t % 12 > 4, intake: t % 9 < 6, retract: k % 600 === 0 });
      w.step(inp);
      contacts += net.contacts;
      for (let n = 0; n < net.n; n++) {
        vmax = Math.max(vmax, net.speedOf(n));
        for (let c = 0; c < 3; c++) { const x = net.local[3 * n + c]; if (!Number.isFinite(x)) finite = false; }
        box = Math.max(box, Math.abs(net.local[3 * n]), Math.abs(net.local[3 * n + 2]) - 0.3, net.local[3 * n + 1] - 0.9);
      }
    }
    log(`net 60 s hard drive: max knot speed ${vmax.toFixed(1)} m/s, max excursion ${box.toFixed(2)} m, ${(ms / steps * 1000).toFixed(0)} µs/step (worst ${(worstMs * 1000).toFixed(0)}), ${contacts} contact-steps, scored ${w.scored}`);
    expect(finite).toBe(true);
    expect(vmax).toBeLessThan(25);
    expect(box).toBeLessThan(0.5);
  });
});

describe('polycarbonate panels', () => {
  /** shoot ball 0 at v (robot frame) from p; returns the strike's deflection under the ball (m) and the ball's velocity right after it */
  async function hit(k: number, p: [number, number, number], v: [number, number, number]) {
    const w = await netWorld();
    w.panels.clear();
    throwBall(w, 0, p, v);
    let strikes = 0, after = [0, 0, 0], ring = 0;
    for (let s = 0; s < 20; s++) {
      w.step(NO_INPUT);
      if (w.panels.strikes && !strikes) { const l = w.fuel.bodies[0].linvel(); after = [Math.hypot(l.x, l.y, l.z), 0, 0]; }
      strikes += w.panels.strikes;
      ring = Math.max(ring, w.panels.amplitude(k));
    }
    return { w, peak: w.panels.lastPeak, strikes, after, ring };
  }

  test('modes: 170 Hz first mode from plate theory, ζ = 0.05', async () => {
    const w = await netWorld();
    const m = w.panels.modes[0];
    log(`panel modes: f1 ${(m.omega[0] / 2 / Math.PI).toFixed(0)} Hz, f2 ${(m.omega[1] / 2 / Math.PI).toFixed(0)} Hz, M1 ${(m.mass[0] * 1000).toFixed(0)} g, Γ1 ${m.gamma[0].toFixed(2)}, k1 ${(m.mass[0] * m.omega[0] ** 2 / 1000).toFixed(0)} kN/m`);
    expect(m.omega[0] / 2 / Math.PI).toBeGreaterThan(120);
    expect(m.omega[0] / 2 / Math.PI).toBeLessThan(250);
    expect(PLATE.zeta).toBeGreaterThanOrEqual(0.03);
    expect(PLATE.zeta).toBeLessThanOrEqual(0.08);
  });

  test('a ball at 10 m/s into a hopper wall flexes it 1–6 mm', async () => {
    const { peak, strikes, after } = await hit(0, [0, 0.36, 0.2], [10, 0, 0]);
    log(`10 m/s ball into the middle of the left hopper wall: ${strikes} strike, ${(peak * 1000).toFixed(2)} mm under the ball, rebound ${after[0].toFixed(2)} m/s`);
    expect(strikes).toBe(1);
    expect(peak).toBeGreaterThan(0.001);
    expect(peak).toBeLessThan(0.006);
  });

  test('ringing decays at the log decrement of ζ and is gone in a fraction of a second', async () => {
    const w = await netWorld();
    const P = w.panels, w1 = P.modes[0].omega[0];
    P.state[1] = 0.5;                                // kick the first mode: 0.5 m/s
    const a = [P.amplitude(0)];
    for (let s = 0; s < 12; s++) { P.postStep(w.fuel); a.push(P.amplitude(0)); }
    const per = Math.log(a[1] / a[2]);
    const delta = per * (2 * Math.PI / (w1 * Math.sqrt(1 - PLATE.zeta ** 2))) * 120;
    log(`log decrement from the ringing: ${delta.toFixed(3)} (theory ${(2 * Math.PI * PLATE.zeta / Math.sqrt(1 - PLATE.zeta ** 2)).toFixed(3)}); after 0.1 s ${(a[12] / a[0] * 100).toFixed(2)} % of the first`);
    expect(delta).toBeGreaterThan(0.28);
    expect(delta).toBeLessThan(0.35);
    expect(a[12] / a[0]).toBeLessThan(0.02);
  });

  test('the wall is compliant: the ball leaves slower than off a rigid wall, never faster', async () => {
    const mid = await hit(0, [0, 0.36, 0.2], [10, 0, 0]);
    // the same shot into the panel's edge, where it hardly moves: the rigid-wall bounce
    const edge = await hit(0, [0, 0.36, 0.425], [10, 0, 0]);
    log(`rebound from the middle of a panel ${mid.after[0].toFixed(2)} m/s (${(mid.peak * 1000).toFixed(2)} mm), from its edge ${edge.after[0].toFixed(2)} m/s (${(edge.peak * 1000).toFixed(2)} mm)`);
    expect(mid.after[0]).toBeLessThanOrEqual(edge.after[0] + 0.01);
    expect(mid.after[0]).toBeGreaterThan(1);
  });

  test('a robot crash into a wall rings the front panel more than a 10 m/s ball does', async () => {
    const ball = await hit(3, [0, 0.396, 0.35], [0, 0, 10]);
    // face the wall at x = −8.27 and drive into it from 2.4 m
    const w = await driverWorld(-5.4, 0, -Math.PI / 2, { clearFuel: true, intakeDown: true });
    w.panels.clear();
    let peak = 0, vmax = 0;
    for (let k = 0; k < 4 * 120; k++) { w.step(input({ mx: -1 })); peak = Math.max(peak, w.panels.amplitude(3)); vmax = Math.max(vmax, w.robot.speed); }
    log(`front panel: robot crash at ${vmax.toFixed(1)} m/s rings ${(peak * 1000).toFixed(2)} mm; 10 m/s ball deflects it ${(ball.peak * 1000).toFixed(2)} mm`);
    expect(peak).toBeGreaterThan(ball.peak);
    expect(peak).toBeLessThan(0.02);
  });

  test('driving normally does not drift the panels', async () => {
    const w = await driverWorld(-5, 0, Math.PI / 2, { clearFuel: true });
    w.panels.clear();
    let worst = 0;
    for (let k = 0; k < 10 * 120; k++) {
      w.step(input({ mx: Math.sin(k / 90) * 0.6, mz: Math.cos(k / 70) * 0.6, turn: 0.4 }));
      for (let p = 0; p < PANELS.length; p++) worst = Math.max(worst, w.panels.amplitude(p));
    }
    for (let k = 0; k < 120; k++) w.step(NO_INPUT);
    log(`driving: largest panel ringing ${(worst * 1000).toFixed(3)} mm, at rest after: ${(Math.max(...PANELS.map((_, p) => w.panels.amplitude(p))) * 1000).toFixed(4)} mm`);
    expect(worst).toBeLessThan(0.002);
    expect(Math.max(...PANELS.map((_, p) => w.panels.amplitude(p)))).toBeLessThan(1e-5);
  });
});
