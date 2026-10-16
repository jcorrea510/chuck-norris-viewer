// The realism checks of OVERNIGHT.md §3.7 and the collision/scoring rules of §2, with tolerances.
// Every number here is also in overnight/PHYSICS.md with its source.
import { describe, test, expect } from 'vitest';
import { makeEngine, makeWorld, driverWorld, input, run, robotPenetration } from './helpers';
import { Fuel, BS } from '../../src/sim/phys/fuel';
import { Rng } from '../../src/sim/phys/rng';
import { STEP, FUEL, SHOOTER } from '../../src/sim/phys/config';
import { AimTable, exitFor, fly } from '../../src/sim/phys/aim';
import { HUB } from '../../src/sim/phys/hub';
import { NO_INPUT, ALLIANCE_LINE } from '../../src/sim/world';

async function oneBall(p: number[], withField = false) {
  const eng = await makeEngine(withField);
  const f = new Fuel(eng, [p], new Rng(1), () => 0);
  eng.step();
  const step = () => { f.preStep(); eng.step(); f.postStep(() => {}); };
  return { eng, f, step };
}
const log = (s: string) => console.log(s);

describe('FUEL', () => {
  test('1 m drop onto carpet rebounds well under half (18–32 cm)', async () => {
    const { f, step } = await oneBall([-6, 1 + FUEL.radius, -2]);
    let max = 0, up = false;
    for (let k = 0; k < 480; k++) {
      step();
      const vy = f.bodies[0].linvel().y;
      if (vy > 0.05) up = true;
      if (up && vy < 0 && max === 0) max = f.pos[1] - FUEL.radius;
    }
    log(`drop 1 m: rebound ${(max * 100).toFixed(1)} cm`);
    expect(max).toBeGreaterThan(0.18);
    expect(max).toBeLessThan(0.32);
  });

  test('a passive bounce never gains energy (any drop height)', async () => {
    for (const h of [0.3, 1, 2, 4]) {
      const { f, step } = await oneBall([-6, h + FUEL.radius, -2]);
      let max = 0, up = false;
      for (let k = 0; k < 1200; k++) {
        step();
        const vy = f.bodies[0].linvel().y;
        if (vy > 0.02) up = true;
        if (up) max = Math.max(max, f.pos[1] - FUEL.radius);
        if (up && vy < 0) break;
      }
      log(`drop ${h} m → ${(max * 100).toFixed(1)} cm (${(max / h * 100).toFixed(0)}%)`);
      expect(max).toBeLessThan(h * 0.5);
    }
  });

  test('harder hits lose more energy (restitution falls with impact speed)', async () => {
    const ratio = async (h: number) => {
      const { f, step } = await oneBall([-6, h + FUEL.radius, -2]);
      let max = 0, up = false;
      for (let k = 0; k < 1200; k++) { step(); const vy = f.bodies[0].linvel().y; if (vy > 0.02) up = true; if (up) max = Math.max(max, f.pos[1] - FUEL.radius); if (up && vy < 0) break; }
      return max / h;
    };
    const low = await ratio(0.3), high = await ratio(3);
    log(`rebound ratio: 0.3 m drop ${low.toFixed(3)}, 3 m drop ${high.toFixed(3)}`);
    expect(high).toBeLessThan(low);
  });

  test('a 2 m/s roll on carpet stops within 1.6–2.4 m', async () => {
    const { f, step } = await oneBall([-6, FUEL.radius, -2]);
    f.bodies[0].setLinvel({ x: 2, y: 0, z: 0 }, true);
    f.bodies[0].setAngvel({ x: 0, y: 0, z: -2 / FUEL.radius }, true);
    let t = 0;
    while (t < 10) { step(); t += STEP; if (f.bodies[0].linvel().x < 0.01) break; }
    const d = f.pos[0] + 6;
    log(`roll 2 m/s: ${d.toFixed(2)} m in ${t.toFixed(2)} s`);
    expect(d).toBeGreaterThan(1.6);
    expect(d).toBeLessThan(2.4);
  });

  test('drag: terminal velocity ~20 m/s, shots lose speed and come down steeper than they went up', () => {
    const vt = Math.sqrt(9.81 / (0.5 * 1.2 * FUEL.cd * Math.PI * FUEL.radius ** 2 / FUEL.mass));
    log(`terminal velocity ${vt.toFixed(1)} m/s`);
    expect(vt).toBeGreaterThan(17); expect(vt).toBeLessThan(23);
    const r = fly(12, 50 * Math.PI / 180, 0, 0.57, 0)!;
    const vac = (12 * Math.cos(50 * Math.PI / 180)) * ((12 * Math.sin(50 * Math.PI / 180)) + Math.sqrt((12 * Math.sin(50 * Math.PI / 180)) ** 2 + 2 * 9.81 * 0.57)) / 9.81;
    log(`12 m/s at 50°: range with drag ${r.d.toFixed(2)} m vs vacuum ${vac.toFixed(2)} m; apex ${r.apex.toFixed(2)} m`);
    expect(r.d).toBeLessThan(vac * 0.85);
    // descent steeper than ascent
    const p = r.path, n = p.length;
    const up = Math.atan2(p[1][1] - p[0][1], p[1][0] - p[0][0]), down = Math.atan2(p[n - 2][1] - p[n - 1][1], p[n - 1][0] - p[n - 2][0]);
    log(`launch ${(up * 57.3).toFixed(0)}°, landing ${(down * 57.3).toFixed(0)}°`);
    expect(down).toBeGreaterThan(up);
  });

  test('Magnus: shooter backspin floats a shot (higher apex; longer carry on a flat shot)', () => {
    const e = exitFor(450);
    const steep = fly(e.speed, 55 * Math.PI / 180, e.spin, 0.57, 0)!, steep0 = fly(e.speed, 55 * Math.PI / 180, 0, 0.57, 0)!;
    const flat = fly(e.speed, 30 * Math.PI / 180, e.spin, 0.57, 0)!, flat0 = fly(e.speed, 30 * Math.PI / 180, 0, 0.57, 0)!;
    log(`exit ${e.speed.toFixed(1)} m/s, backspin ${(e.spin / (2 * Math.PI)).toFixed(0)} rev/s: 55° apex ${steep.apex.toFixed(2)} vs ${steep0.apex.toFixed(2)} m; 30° range ${flat.d.toFixed(2)} vs ${flat0.d.toFixed(2)} m`);
    expect(steep.apex).toBeGreaterThan(steep0.apex + 0.08);
    expect(flat.d).toBeGreaterThan(flat0.d * 1.05);
  });
});

describe('net and thin panels', () => {
  test('a 20 m/s ball into the HUB net is caught (no tunneling) and drops with almost no rebound', async () => {
    const w = await makeWorld(1);
    const f = w.fuel, i = 0;
    const netX = -HUB.cx + HUB.net.off;
    // from the neutral zone, straight at the net
    f.launch(i, [netX + 1.5, 2.4, 0.1], [-20, 0.5, 0], [0, 0, 0]);
    let minX = Infinity, vAfter = Infinity, caught = false;
    for (let k = 0; k < 240; k++) {
      w.step(NO_INPUT);
      const x = f.pos[0];
      minX = Math.min(minX, x);
      if (!caught && f.vel[0] > -0.2 && x < netX + 0.3) { caught = true; }
      if (caught && k % 1 === 0 && f.pos[1] > 1.5) vAfter = Math.min(vAfter, Math.abs(f.vel[0]));
    }
    const rebound = Math.max(0, f.vel[0]);
    log(`net: deepest x ${minX.toFixed(3)} (net at ${netX.toFixed(3)}), x-speed after catch ${vAfter.toFixed(2)} m/s, now at y=${f.pos[1].toFixed(2)}`);
    expect(minX).toBeGreaterThan(netX - 0.35);
    expect(f.st[0] === BS.HUB).toBe(false);
    expect(rebound).toBeLessThan(1.5);
  });

  test('a 25 m/s ball never tunnels through a wall panel', async () => {
    const w = await makeWorld(1);
    const f = w.fuel;
    f.launch(0, [-6, 0.5, 3.2], [0, 0.5, 25], [0, 0, 0]);
    let maxZ = -Infinity;
    for (let k = 0; k < 240; k++) { w.step(NO_INPUT); maxZ = Math.max(maxZ, f.pos[2]); }
    log(`side wall: max z ${maxZ.toFixed(3)}`);
    expect(maxZ).toBeLessThan(4.06);
  });
});

describe('robot', () => {
  test('0 → top speed, stop, 180° reversal', async () => {
    // the open carpet of our ALLIANCE ZONE, driving across the field (sideways: swerve doesn't care)
    const w = await driverWorld(-6.0, -3.45, Math.PI / 2, { clearFuel: true });
    const r = w.robot;
    let t90 = -1;
    const x0 = r.z;
    run(w, 1.3, input({ mz: 1 }), (t) => { if (t90 < 0 && r.speed > 0.9 * 4.7) t90 = t; });
    const top = r.speed, dist = r.z - x0;
    log(`0→90% of 4.7 m/s: ${t90.toFixed(2)} s; ${top.toFixed(2)} m/s after 2 s (${dist.toFixed(2)} m)`);
    expect(t90).toBeGreaterThan(0.45); expect(t90).toBeLessThan(1.2);
    const xs = r.z;
    let ts = -1;
    run(w, 2, NO_INPUT, (t) => { if (ts < 0 && r.speed < 0.05) ts = t; });
    log(`stop from ${top.toFixed(2)} m/s: ${(r.z - xs).toFixed(2)} m in ${ts.toFixed(2)} s`);
    expect(r.z - xs).toBeGreaterThan(0.6); expect(r.z - xs).toBeLessThan(2.4);
    // full reversal
    const w2 = await driverWorld(-6.0, -3.45, Math.PI / 2, { clearFuel: true });
    run(w2, 1.3, input({ mz: 1 }));
    let tr = -1;
    run(w2, 2, input({ mz: -1 }), (t) => { if (tr < 0 && w2.robot.vz < -2) tr = t; });
    log(`180° reversal from ${4.7} m/s to −2 m/s: ${tr.toFixed(2)} s`);
    expect(tr).toBeGreaterThan(0.5); expect(tr).toBeLessThan(2);
  });

  test('driving over a BUMP at speed pitches the chassis like the 15° ramps', async () => {
    const w = await driverWorld(-6, 1.5, Math.PI / 2, { clearFuel: true });
    let maxP = 0, maxR = 0;
    run(w, 3, input({ mx: 1 }), () => { maxP = Math.max(maxP, Math.abs(w.robot.pitch)); maxR = Math.max(maxR, Math.abs(w.robot.roll)); });
    log(`bump: max pitch ${(maxP * 57.3).toFixed(1)}°, max roll ${(maxR * 57.3).toFixed(1)}°, ended at x=${w.robot.x.toFixed(2)}`);
    expect(maxP * 57.3).toBeGreaterThan(8);
    expect(maxP * 57.3).toBeLessThan(35);
    expect(w.robot.x).toBeGreaterThan(-2);
  });

  test('TRENCH: intake down (0.549 m) fits under; intake up (0.728 m) is blocked', async () => {
    for (const down of [true, false]) {
      const w = await driverWorld(-6.0, -3.38, Math.PI / 2, { clearFuel: true, intakeDown: down });
      let worst = 0;
      run(w, 3, input({ mx: 1 }), () => { worst = Math.max(worst, robotPenetration(w)); });
      log(`trench, intake ${down ? 'down' : 'up'}: x=${w.robot.x.toFixed(2)}, worst overlap ${(worst * 1000).toFixed(1)} mm`);
      if (down) expect(w.robot.x).toBeGreaterThan(-2.5);
      else expect(w.robot.x).toBeLessThan(-3.8);
      expect(worst).toBeLessThan(0.012);
    }
  });

  test('full speed into every field element: no tunneling, no overlap', async () => {
    const cases: [string, number, number, number, number, (x: number, z: number, minX: number) => boolean][] = [
      ['HUB', -6.5, 0, 1, 0, (x) => x < -4.24 - 0.42],
      // backing into our TOWER: the bumpers stop at the uprights (x = −7.16), centered or off-center
      // (pushed on, the robot can pivot on a post, so allow for its half-diagonal)
      ['TOWER (centered)', -5.6, 0.295, -1, 0, (x, z) => !(x < -7.16 && Math.abs(z - 0.295) < 0.45)],
      ['TOWER (one upright)', -5.6, -0.4, -1, 0, (x, z) => !(x < -7.16 && Math.abs(z - 0.295) < 0.45)],
      ['alliance wall', -6.5, 2.5, -1, 0, (x) => x > -8.27 + 0.42],
      ['side guardrail', -6, 1, 0, 1, (_x, z) => z < 4.03 - 0.42],
      ['side guardrail (other)', -6, -1, 0, -1, (_x, z) => z > -4.03 + 0.42],
      ['TRENCH support', -6, 2.7, 1, 0, (x) => x < -3.0],
      ['DEPOT', -6.5, -2.5, -1, 0, (x) => x > -8.27 + 0.42],
    ];
    for (const [name, x, z, mx, mz, ok] of cases) {
      const w = await driverWorld(x, z, Math.PI / 2, { clearFuel: true });
      let worst = 0, bad = false;
      run(w, 2.5, input({ mx, mz }), () => {
        worst = Math.max(worst, robotPenetration(w));
        if (!ok(w.robot.x, w.robot.z, 0)) bad = true;
      });
      log(`${name}: end (${w.robot.x.toFixed(2)}, ${w.robot.z.toFixed(2)}), worst overlap ${(worst * 1000).toFixed(1)} mm`);
      expect(bad).toBe(false);
      // a full-speed hit on a 4 cm post concentrates the impact: allow a one-step transient there
      expect(worst).toBeLessThan(name.startsWith('TOWER') ? 0.025 : 0.012);
    }
  });
});

describe('shooter, indexer, scoring', () => {
  test('flywheel spin-up from rest, and the RPM dip and recovery of each shot', async () => {
    const w = await driverWorld(-6.1, 0, Math.PI / 2, { clearFuel: false });
    const r = w.robot, t = r.turrets[0];
    for (const tu of r.turrets) tu.fly = 0;
    let tReady = -1, last = 0, recover = -1, shotAt = -1;
    const dips: number[] = [];
    r.onShot = (_i, side) => { if (side === t.side) { dips.push(1 - t.fly / Math.max(1, last)); shotAt = w.clock; } };
    run(w, 3, input({ shoot: true }), (tt) => {
      if (tReady < 0 && t.ready) tReady = tt;
      if (shotAt > 0 && recover < 0 && dips.length === 1 && t.fly > t.flyGoal * 0.995) recover = w.clock - shotAt;
      last = t.fly;
    });
    log(`spin-up from rest to ready: ${tReady.toFixed(2)} s (goal ${(t.flyGoal * 9.549).toFixed(0)} rpm); dip per shot ${dips.map((d) => (d * 100).toFixed(1) + '%').join(' ')}; first recovery ${(recover * 1000).toFixed(0)} ms`);
    expect(tReady).toBeGreaterThan(0.25); expect(tReady).toBeLessThan(1.6);
    expect(dips.length).toBeGreaterThan(2);
    for (const d of dips) { expect(d).toBeGreaterThan(0.01); expect(d).toBeLessThan(0.25); }
    expect(recover).toBeGreaterThan(0.01); expect(recover).toBeLessThan(0.6);
  });

  test('the indexer only feeds while shoot is held', async () => {
    const w = await driverWorld(-6.1, 0, Math.PI / 2);
    const r = w.robot;
    run(w, 3, NO_INPUT);
    expect(r.shotsFired).toBe(0);
    expect(r.turrets[0].lane.length + r.turrets[1].lane.length).toBe(0);
    run(w, 2.5, input({ shoot: true }));
    const fired = r.shotsFired;
    log(`held 2.5 s: ${fired} shots`);
    expect(fired).toBeGreaterThan(4);
    run(w, 0.3, NO_INPUT);
    const after = r.shotsFired;
    run(w, 2, NO_INPUT);
    expect(r.shotsFired).toBe(after);
  });

  test('every scored ball counts exactly once, then leaves through the exits', async () => {
    const w = await makeWorld(1);
    w.takeOver();
    const f = w.fuel;
    const ids = [10, 11, 12, 13, 14, 15];
    const before = w.hubs.scoredBy[0];
    ids.forEach((i, k) => f.launch(i, [-HUB.cx + (k % 3 - 1) * 0.15, 2.4 + k * 0.2, (k < 3 ? -0.1 : 0.1)], [0, -1, 0], [0, 0, 0]));
    run(w, 4, NO_INPUT);
    log(`dropped ${ids.length} into the HUB: scored ${w.hubs.scoredBy[0] - before}; now ${ids.map((i) => f.st[i] === BS.FREE ? `free@x=${f.pos[i * 3].toFixed(2)}` : `st${f.st[i]}`).join(' ')}`);
    expect(w.hubs.scoredBy[0] - before).toBe(ids.length);
    for (const i of ids) { expect(f.st[i]).toBe(BS.FREE); expect(f.pos[i * 3]).toBeGreaterThan(-3.0); }
    run(w, 3, NO_INPUT);
    expect(w.hubs.scoredBy[0] - before).toBe(ids.length);
  });

  test('a ball only scores by flying into the opening: shots from the neutral zone never score', async () => {
    const w = await driverWorld(0, 1.2, Math.PI / 2);
    const r = w.robot;
    run(w, 0.1, NO_INPUT);
    const before = w.hubs.scoredBy[0];
    run(w, 5, input({ shoot: true }));
    run(w, 3, NO_INPUT);
    log(`ferry from x=0: ${r.shotsFired} shots, HUB scored ${w.hubs.scoredBy[0] - before}`);
    expect(r.shotsFired).toBeGreaterThan(3);
    expect(w.hubs.scoredBy[0] - before).toBe(0);
  });

  test('ferry vs HUB by zone, and ferried FUEL lands in our ALLIANCE ZONE', async () => {
    const a = await driverWorld(-6, 0, Math.PI / 2);
    run(a, 0.2, input({ shoot: true }));
    expect(a.robot.ferry).toBe(false);
    const b = await driverWorld(1.0, -1.2, Math.PI / 2);
    const landed: number[] = [];
    const flying = new Set<number>();
    b.robot.onShot = (i) => flying.add(i);
    run(b, 6, input({ shoot: true }), () => {
      for (const i of [...flying]) if (b.fuel.st[i] !== BS.FLYING) { landed.push(b.fuel.pos[i * 3]); flying.delete(i); }
    });
    expect(b.robot.ferry).toBe(true);
    const home = landed.filter((x) => x < ALLIANCE_LINE).length;
    log(`ferry from x=1.0: ${landed.length} landed, ${home} in our zone; first-contact x ${landed.slice(0, 8).map((x) => x.toFixed(1)).join(' ')}`);
    expect(landed.length).toBeGreaterThan(3);
    expect(home / landed.length).toBeGreaterThan(0.6);
  });

  test('from our zone most shots score; 30 shots from one spot land with a realistic spread', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2);
    const f = w.fuel, r = w.robot;
    const cross: [number, number][] = [];
    const watch = new Map<number, number>();
    r.onShot = (i) => watch.set(i, 0);
    let shots = 0;
    const before = w.hubs.scoredBy[0];
    // refill the hopper from staged fuel as it empties, so the robot keeps shooting from the same spot
    const spare = Array.from({ length: f.n }, (_, i) => i).filter((i) => f.st[i] === BS.FREE && f.pos[i * 3] > -1 && f.pos[i * 3] < 1).slice(0, 40);
    while (shots < 30) {
      if (r.load < 4 && spare.length) r.preload(spare.splice(0, 8));
      run(w, 0.5, input({ shoot: true }), () => {
        for (const [i, apex] of watch) {
          const y = f.pos[i * 3 + 1];
          if (y > apex) watch.set(i, y);
          else if (apex > HUB.rim && y < HUB.rim) { cross.push([f.pos[i * 3], f.pos[i * 3 + 2]]); watch.delete(i); }
          if (f.st[i] === BS.HUB) { watch.delete(i); }
        }
      });
      shots = r.shotsFired;
      if (w.clock > 60) break;
    }
    run(w, 2, NO_INPUT);
    const scored = w.hubs.scoredBy[0] - before;
    const mx = cross.reduce((a, c) => a + c[0], 0) / cross.length, mz = cross.reduce((a, c) => a + c[1], 0) / cross.length;
    const sd = Math.sqrt(cross.reduce((a, c) => a + (c[0] - mx) ** 2 + (c[1] - mz) ** 2, 0) / cross.length);
    log(`${r.shotsFired} shots from (-6.1, 0.4): ${scored} scored (${(scored / r.shotsFired * 100).toFixed(0)}%); ${cross.length} crossed the rim plane, spread (1σ radial) ${(sd * 100).toFixed(1)} cm around (${mx.toFixed(2)}, ${mz.toFixed(2)})`);
    expect(cross.length).toBeGreaterThan(15);
    expect(sd).toBeGreaterThan(0.03);
    expect(sd).toBeLessThan(0.4);
    expect(scored / r.shotsFired).toBeGreaterThan(0.7);
    expect(scored / r.shotsFired).toBeLessThan(1.0001);
  });
});

describe('shooting on the move', () => {
  test('strafing across our zone at speed, the lead keeps most shots in', async () => {
    for (const v of [0, 0.6]) {
      const w = await driverWorld(-6.3, -2.4, Math.PI / 2);
      const r = w.robot;
      const spare = Array.from({ length: w.fuel.n }, (_, i) => i).filter((i) => w.fuel.st[i] === BS.FREE && w.fuel.pos[i * 3] > -1 && w.fuel.pos[i * 3] < 1).slice(0, 16);
      r.preload(spare);
      run(w, 0.8, input({ shoot: true }));   // spin up, settle
      const before = w.hubs.scoredBy[0], shots0 = r.shotsFired;
      // drive across the zone (z from −2.4 toward +2.4) at v × 4.7 m/s while holding Shoot
      run(w, 2.0, input({ shoot: true, mz: v }));
      run(w, 0.4, input({ shoot: true }));
      run(w, 2.5, NO_INPUT);
      const shots = r.shotsFired - shots0, scored = w.hubs.scoredBy[0] - before;
      log(`shooting while strafing at ${(v * 4.7).toFixed(1)} m/s: ${scored}/${shots} scored`);
      expect(shots).toBeGreaterThan(8);
      expect(scored / shots).toBeGreaterThan(v ? 0.5 : 0.8);
    }
  });
});

describe('determinism and invariants', () => {
  test('same seed, same match', async () => {
    const sig = async () => {
      const w = await makeWorld(5);
      run(w, 12, NO_INPUT);
      let h = 0;
      for (let i = 0; i < w.fuel.n; i++) h = (h * 31 + Math.round(w.fuel.pos[i * 3] * 1000) + Math.round(w.fuel.pos[i * 3 + 2] * 1000)) | 0;
      return `${w.robot.x.toFixed(6)},${w.robot.z.toFixed(6)},${w.hubs.scoredBy[0]},${w.robot.shotsFired},${h}`;
    };
    const a = await sig(), b = await sig();
    log(`signature ${a}`);
    expect(b).toBe(a);
  });

  test('60 s of the robot playing itself: no NaN, no FUEL out of the field, score only goes up', async () => {
    const w = await makeWorld(3);
    let last = 0, bad = '';
    run(w, 60, NO_INPUT, (t) => {
      if (w.hubs.scoredBy[0] < last) bad ||= `score dropped at ${t}`;
      last = w.hubs.scoredBy[0];
      const r = w.robot;
      if (!Number.isFinite(r.x + r.z + r.th)) bad ||= `robot NaN at ${t}`;
      if (Math.round(t * 120) % 12 === 0) for (let i = 0; i < w.fuel.n; i++) {
        if (!w.fuel.isFree(i)) continue;
        const x = w.fuel.pos[i * 3], y = w.fuel.pos[i * 3 + 1], z = w.fuel.pos[i * 3 + 2];
        if (!Number.isFinite(x + y + z)) bad ||= `ball ${i} NaN`;
        if (Math.abs(x) > 8.35 || Math.abs(z) > 4.1 || y < FUEL.radius - 0.03) bad ||= `ball ${i} out at (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) t=${t.toFixed(2)}`;
      }
    });
    log(`60 s: scored ${w.hubs.scoredBy[0]}, shots ${w.robot.shotsFired}${bad ? ' — ' + bad : ''}`);
    expect(bad).toBe('');
    expect(w.hubs.scoredBy[0]).toBeGreaterThan(5);
  });
});

void SHOOTER; void AimTable;
