// The team's robot code (Rebuilt26 Java) as ported to src/sim/phys/teamcode.ts, and the robot it
// drives: shot table, turret wrapping, buttons, ferry, intake, feeding.
import { describe, test, expect } from 'vitest';
import { driverWorld, input, makeWorld, run } from './helpers';
import { FIELD_X0, FIELD_Y0, HUB_WPI, INTAKE, SHOOTER, SHOT_TABLE, TURRET } from '../../src/sim/phys/config';
import { exitFor, fly } from '../../src/sim/phys/aim';
import { Hood } from '../../src/sim/phys/mechanisms';
import { trapezoid } from '../../src/sim/phys/motors';
import {
  AngleController, Kinematics, NO_CODE_INPUT, RobotCode, discretize, getParameters, getShotForDistance, optimizeAndScale,
  selectSafeSetpointDegrees, stickToLinear, stickToOmega, wrapPi, type CodeInput, type Sensors,
} from '../../src/sim/phys/teamcode';
import { BS } from '../../src/sim/phys/fuel';
import { NO_INPUT } from '../../src/sim/world';

const log = (s: string) => console.log(s);
const DEG = Math.PI / 180;

describe('shot table (GetAdjustedShot.getShotForDistance)', () => {
  const close = (d: number, hood: number, rps: number, tof: number) => {
    const s = getShotForDistance(d);
    expect(s.hoodDeg).toBeCloseTo(hood, 9); expect(s.rps).toBeCloseTo(rps, 9); expect(s.tof).toBeCloseTo(tof, 9);
  };
  test('below the table: linear extrapolation from the first two rows', () => {
    // rows 1.3 m (13°, 40.5 rps, 0.81 s) and 2.0 m (13°, 44 rps, 1.01 s); t = (1.0 − 1.3) / 0.7
    close(1.0, 13, 40.5 + 3.5 * (-0.3 / 0.7), 0.81 + 0.2 * (-0.3 / 0.7));
    close(1.0, 13, 39, 0.7242857142857143);
  });
  test('on a row: that row', () => {
    close(1.3, 13, 40.5, 0.81);
    close(6.0, 25.5, 64.75, 1.415);
  });
  test('between rows: linear', () => {
    // 2.0 → 2.25 m, t = 0.4
    close(2.1, 13, 45, 1.032);
    // 5.5 (24.5°, 62 rps, 1.315 s) → 6.0 m (25.5°, 64.75, 1.415), t = 0.8
    close(5.9, 25.3, 64.2, 1.395);
  });
  test('above the table: linear extrapolation from the last two rows', () => {
    // t = (7.5 − 5.5) / 0.5 = 4
    close(7.5, 28.5, 73, 1.715);
  });
  test('the table is the deployed shot_table.json', () => {
    expect(SHOT_TABLE).toHaveLength(17);
    expect(SHOT_TABLE[0]).toEqual({ d: 1.3, hoodDeg: 13, rps: 40.5, tof: 0.81 });
    expect(SHOT_TABLE[16]).toEqual({ d: 6, hoodDeg: 25.5, rps: 64.75, tof: 1.415 });
  });
  test('the fit: every row, flown through the sim from the CAD exit height at 82° − hood, lands within a few cm of its distance', () => {
    let sq = 0, worst = 0, tofSq = 0;
    for (const row of SHOT_TABLE) {
      const e = exitFor(row.rps);
      const r = fly(e.speed, (SHOOTER.elevation - row.hoodDeg) * DEG, e.spin, 0.566, 1.83)!;
      const err = r.d - row.d;
      sq += err * err; tofSq += (r.t - row.tof) ** 2; worst = Math.max(worst, Math.abs(err));
    }
    const rms = Math.sqrt(sq / SHOT_TABLE.length), tofRms = Math.sqrt(tofSq / SHOT_TABLE.length);
    log(`table fit through the sim: distance rms ${rms.toFixed(3)} m (worst ${worst.toFixed(2)} m), time-of-flight rms ${tofRms.toFixed(3)} s`);
    expect(rms).toBeLessThan(0.2);
    expect(worst).toBeLessThan(0.45);
    expect(tofRms).toBeLessThan(0.08);
  });
});

describe('turret wrapping (Turret.selectSafeSetpointDegrees)', () => {
  test('ported cases of TurretWrapSelectionTest.java', () => {
    expect(selectSafeSetpointDegrees(0, -355, -380, 1)).toBe(-360);
    expect(selectSafeSetpointDegrees(0, -355, -365, 10)).toBe(-360);
    expect(selectSafeSetpointDegrees(0, -5, -380, 1)).toBe(0);
    expect(selectSafeSetpointDegrees(0, -5, -365, 10)).toBe(0);
    const first = selectSafeSetpointDegrees(0, -355, -380, 1);
    expect(selectSafeSetpointDegrees(1, first, -380, 1)).toBe(-359);
    expect(selectSafeSetpointDegrees(-1, first, -380, 1)).toBe(-361);
    expect(selectSafeSetpointDegrees(10, 0, -380, 1)).toBe(-350);
    expect(selectSafeSetpointDegrees(-10, -360, -365, 10)).toBe(-10);
  });
  test('no candidate in range: the target is clamped', () => {
    expect(selectSafeSetpointDegrees(500, 0, -100, 100)).toBe(100);
    expect(selectSafeSetpointDegrees(-500, 0, -100, 100)).toBe(-100);
  });
  test('the turret lock is −135 radians, which wraps to −174.93°', () => {
    const deg = TURRET.lockedAngleRad / DEG;
    expect(selectSafeSetpointDegrees(deg, -180, TURRET.min[0], TURRET.max[0])).toBeCloseTo(-174.93, 1);
    expect(selectSafeSetpointDegrees(deg, -180, TURRET.min[1], TURRET.max[1])).toBeCloseTo(-174.93, 1);
  });
});

describe('the drive command (DriveCommands)', () => {
  test('joystick shaping: deadband on the magnitude (rescaled), then squared; omega the same with its sign', () => {
    expect(stickToLinear(0.05, 0.05)).toEqual({ x: 0, y: 0 });
    const s = stickToLinear(0.55, 0);
    expect(s.x).toBeCloseTo(0.25, 12); expect(s.y).toBeCloseTo(0, 12);
    const d = stickToLinear(0.55 * Math.SQRT1_2, -0.55 * Math.SQRT1_2);
    expect(d.x).toBeCloseTo(0.25 * Math.SQRT1_2, 12); expect(d.y).toBeCloseTo(-0.25 * Math.SQRT1_2, 12);
    expect(stickToOmega(-1)).toBeCloseTo(-1, 12);
    expect(stickToOmega(0.55)).toBeCloseTo(0.25, 12);
    expect(stickToOmega(-0.08)).toBe(0);
  });
  test('ChassisSpeeds.discretize: translation only is unchanged, a turning chassis gets the arc correction', () => {
    const t = discretize(2, -1, 0, 0.02);
    expect(t.vx).toBeCloseTo(2, 12); expect(t.vy).toBeCloseTo(-1, 12);
    // Pose2d.exp of (vx dt, vy dt, ω dt): the same twist as a robot driving an arc
    const a = discretize(2, 0, 3, 0.02);
    expect(a.omega).toBeCloseTo(3, 12);
    expect(a.vy).toBeLessThan(0);
    expect(Math.hypot(a.vx, a.vy)).toBeCloseTo(2, 2);
  });
  test('kinematics: a zero request keeps the last module headings; desaturation scales every module; optimize flips a >90° turn and cosineScale shrinks the speed by the remaining error', () => {
    const k = new Kinematics();
    const moving = k.toStates(0, 1, 0);
    expect(moving.every((s) => Math.abs(s.angle - Math.PI / 2) < 1e-12 && Math.abs(s.speed - 1) < 1e-12)).toBe(true);
    const stopped = k.toStates(0, 0, 0);
    expect(stopped.every((s) => s.speed === 0 && Math.abs(s.angle - Math.PI / 2) < 1e-12)).toBe(true);
    const spin = k.toStates(0, 0, 20);
    const states = spin.map((s) => ({ ...s }));
    Kinematics.desaturate(states, 4.58);
    expect(Math.max(...states.map((s) => s.speed))).toBeCloseTo(4.58, 12);
    expect(states[0].speed / states[1].speed).toBeCloseTo(spin[0].speed / spin[1].speed, 12);
    const flipped = optimizeAndScale({ speed: 2, angle: Math.PI - 0.1 }, 0);
    expect(flipped.speed).toBeCloseTo(-2 * Math.cos(0.1), 12);
    expect(flipped.angle).toBeCloseTo(-0.1, 12);
    expect(optimizeAndScale({ speed: 2, angle: 0.5 }, 0).speed).toBeCloseTo(2 * Math.cos(0.5), 12);
    // module speeds → chassis speeds round trip
    const c = Kinematics.toChassis(k.toStates(1.2, -0.7, 2.1));
    expect(c.vx).toBeCloseTo(1.2, 9); expect(c.vy).toBeCloseTo(-0.7, 9); expect(c.omega).toBeCloseTo(2.1, 9);
  });
  test('the angle controller (ProfiledPID 5, 0.4, 8 rad/s, 20 rad/s²) turns 2 rad in about a second and stays', () => {
    const c = new AngleController();
    c.reset(0);
    let th = 0, w = 0, tReach = -1, maxW = 0, over = 0;
    for (let k = 0; k < 150; k++) {
      const u = c.calculate(th, 2);
      w = Math.max(-8, Math.min(8, u)); th += w * 0.02; maxW = Math.max(maxW, Math.abs(w)); over = Math.max(over, th - 2);
      if (tReach < 0 && Math.abs(th - 2) < 0.05) tReach = (k + 1) * 0.02;
    }
    log(`angle controller: 2 rad in ${tReach.toFixed(2)} s, peak ${maxW.toFixed(1)} rad/s, overshoot ${over.toFixed(3)} rad`);
    expect(tReach).toBeGreaterThan(0.4); expect(tReach).toBeLessThan(1.5);
    expect(maxW).toBeLessThanOrEqual(8 + 1e-9);
    expect(Math.abs(th - 2)).toBeLessThan(0.02);
  });
  test('the trapezoid profile is WPILib\'s: it reaches the goal at rest, never over the velocity limit', () => {
    let s = { pos: 0, vel: 0 };
    let vmax = 0;
    for (let k = 0; k < 400; k++) { s = trapezoid(0.01, s, 10, 3, 6); vmax = Math.max(vmax, s.vel); }
    expect(vmax).toBeLessThanOrEqual(3 + 1e-9);
    expect(s.pos).toBeCloseTo(10, 6); expect(s.vel).toBeCloseTo(0, 6);
  });
});

describe('targeting (GetAdjustedShot.getParameters)', () => {
  const still = { vx: 0, vy: 0, omega: 0 };
  test('a stationary robot aims at the HUB: bearing and the table row at the distance', () => {
    const p = getParameters({ x: 2, y: 4.0346376, theta: 0 }, still, HUB_WPI, TURRET.mountsWpi[0]);
    const m = TURRET.mountsWpi[0];
    const d = Math.hypot(HUB_WPI[0] - (2 + m[0]), HUB_WPI[1] - (4.0346376 + m[1]));
    expect(p.rps).toBeCloseTo(getShotForDistance(d).rps, 6);
    // the HUB is straight ahead (+x) a little to the right of the left turret
    expect(p.turretAngle).toBeLessThan(0); expect(p.turretAngle).toBeGreaterThan(-0.2);
  });
  test('driving sideways, the lookahead leads the target: the turret points ahead of the HUB and the distance changes', () => {
    const base = getParameters({ x: 2, y: 3, theta: 0 }, still, HUB_WPI, TURRET.mountsWpi[0]);
    const moving = getParameters({ x: 2, y: 3, theta: 0 }, { vx: 0, vy: 3, omega: 0 }, HUB_WPI, TURRET.mountsWpi[0]);
    expect(Math.abs(moving.turretAngle - base.turretAngle)).toBeGreaterThan(0.2);
    expect(moving.rps).not.toBeCloseTo(base.rps, 1);
  });
  test('the robot-relative angle subtracts the heading', () => {
    const a = getParameters({ x: 2, y: 3, theta: 0 }, still, HUB_WPI, TURRET.mountsWpi[1]);
    const b = getParameters({ x: 2, y: 3, theta: 0.7 }, still, HUB_WPI, TURRET.mountsWpi[1]);
    // the turret mount rotates with the robot, so the bearing shifts a little besides the 0.7 rad
    expect(Math.abs(wrapPi(a.turretAngle - b.turretAngle) - 0.7)).toBeLessThan(0.08);
  });
});

describe('buttons (DefaultModeBindings, IntakeCommand, Intake, Indexer)', () => {
  const mods = (speed = 0, angle = 0) => [0, 1, 2, 3].map(() => ({ speed, angle }));
  const sens = (o: Partial<Sensors> = {}): Sensors => ({ enabled: true, x: 2.27, y: 4.04, theta: 0, modules: mods(), ...o });
  const press = (o: Partial<CodeInput>): CodeInput => ({ ...NO_CODE_INPUT, ...o });

  test('LT: pivot down and rollers at 70 rps + 3 × speed, once; letting go stops the rollers, the pivot stays; RB retracts', () => {
    const c = new RobotCode();
    c.reset(0);
    expect(c.tick(NO_CODE_INPUT, sens()).pivotTarget).toBe(INTAKE.upPos);
    let o = c.tick(press({ lt: true }), sens({ modules: mods(2, 0) }));
    expect(o.pivotTarget).toBe(INTAKE.downPos);
    expect(o.rollerRps).toBeCloseTo(70 + 3 * 2, 9);
    expect(c.isIntaking).toBe(true);
    // the speed was read once: speeding up does not change it
    o = c.tick(press({ lt: true }), sens({ modules: mods(4, 0) }));
    expect(o.rollerRps).toBeCloseTo(76, 9);
    o = c.tick(NO_CODE_INPUT, sens());
    expect(o.rollerRps).toBe(0); expect(o.pivotTarget).toBe(INTAKE.downPos); expect(c.isIntaking).toBe(false);
    o = c.tick(press({ rb: true }), sens());
    expect(o.pivotTarget).toBe(INTAKE.upPos);
    // LT again extends again
    expect(c.tick(press({ lt: true }), sens()).pivotTarget).toBe(INTAKE.downPos);
  });
  test('RB while LT is held: the intake stows, the rollers stop (IntakeCommand is interrupted) and stay stopped while LT stays held', () => {
    const c = new RobotCode();
    c.reset(0);
    c.tick(press({ lt: true }), sens());
    let o = c.tick(press({ lt: true, rb: true }), sens());
    expect(o.pivotTarget).toBe(INTAKE.upPos); expect(o.rollerRps).toBe(0); expect(c.isIntaking).toBe(false);
    o = c.tick(press({ lt: true }), sens());
    expect(o.rollerRps).toBe(0); expect(o.pivotTarget).toBe(INTAKE.upPos);
    o = c.tick(NO_CODE_INPUT, sens());
    o = c.tick(press({ lt: true }), sens());
    expect(o.pivotTarget).toBe(INTAKE.downPos); expect(o.rollerRps).toBeGreaterThan(60);
  });
  test('RT feeds forward (indexer 55, hotdog 70 rps) and caps the drive at 2.5 m/s; B unjams backward; Start + B runs the intake rollers backward at −50', () => {
    const c = new RobotCode();
    c.reset(0);
    expect(c.maxLinearSpeed).toBeCloseTo(4.58, 12);
    let o = c.tick(press({ rt: true }), sens());
    expect([o.indexerRps, o.hotdogRps]).toEqual([55, 70]);
    expect(c.isFeedingForward).toBe(true); expect(c.maxLinearSpeed).toBe(2.5);
    o = c.tick(NO_CODE_INPUT, sens());
    expect([o.indexerRps, o.hotdogRps]).toEqual([0, 0]); expect(c.maxLinearSpeed).toBeCloseTo(4.58, 12);
    o = c.tick(press({ b: true }), sens());
    expect([o.indexerRps, o.hotdogRps]).toEqual([-55, -70]); expect(c.isFeedingForward).toBe(false); expect(c.maxLinearSpeed).toBeCloseTo(4.58, 12);
    o = c.tick(NO_CODE_INPUT, sens());
    expect(o.indexerRps).toBe(0);
    o = c.tick(press({ b: true, start: true }), sens());
    expect(o.rollerRps).toBe(-50); expect([o.indexerRps, o.hotdogRps]).toEqual([0, 0]);
    o = c.tick(NO_CODE_INPUT, sens());
    expect(o.rollerRps).toBe(0);
  });
  test('LT caps the drive at 2.5 m/s too, and full stick drives at it', () => {
    const c = new RobotCode();
    c.reset(0);
    const o = c.tick(press({ lt: true, x: 1 }), sens());
    expect(c.maxLinearSpeed).toBe(2.5);
    expect(Math.max(...o.drive!.map((m) => Math.abs(m.speed)))).toBeCloseTo(2.5, 6);
  });
  test('disabled: nothing runs', () => {
    const c = new RobotCode();
    c.reset(0);
    c.tick(press({ lt: true, rt: true, x: 1 }), sens());
    const o = c.tick(press({ lt: true, rt: true, x: 1 }), sens({ enabled: false }));
    expect(o.drive).toBeNull(); expect(o.turrets).toEqual([null, null]); expect(o.rollerRps).toBe(0); expect(o.indexerRps).toBe(0);
  });
  test('the neutral zone (WPILib x 5.18…11.30) switches both turrets to the closer ferry target', () => {
    const c = new RobotCode();
    c.reset(0);
    c.tick(NO_CODE_INPUT, sens({ x: 2 }));
    expect(c.ferry).toBe(false);
    c.tick(NO_CODE_INPUT, sens({ x: 5.2 }));
    expect(c.ferry).toBe(true);
    c.tick(NO_CODE_INPUT, sens({ x: 11.4 }));
    expect(c.ferry).toBe(false);
    // robot on the left half (+y): the closer ferry target is (1.6256, 6.0346); aim there, not at the HUB
    const left = new RobotCode(); left.reset(0);
    const o = left.tick(NO_CODE_INPUT, sens({ x: 6, y: 6.5 }));
    const toFerry = Math.atan2(6.0346376 - (6.5 + 0.197), 1.6256194 - (6 - 0.184)) / DEG;
    expect(wrapPi(o.turrets[0]!.azimuthDeg * DEG - toFerry * DEG)).toBeCloseTo(0, 1);
  });
  test('D-pad down toggles the turret lock: both turrets go to −174.93°', () => {
    const c = new RobotCode();
    c.reset(0);
    let o = c.tick(press({ povDown: true }), sens());
    expect(c.locked).toBe(true);
    expect(o.turrets[0]!.azimuthDeg).toBeCloseTo(-174.93, 1); expect(o.turrets[1]!.azimuthDeg).toBeCloseTo(-174.93, 1);
    o = c.tick(press({ povDown: true }), sens());
    expect(c.locked).toBe(false);
  });
  test('Y resets the estimated heading to zero; vision pulls the error back with τ = 2 s', () => {
    const c = new RobotCode();
    c.reset(0);
    c.tick(press({ yPress: true }), sens({ theta: 1.0 }));
    expect(c.headingErr).toBeCloseTo(-1.0, 9);
    for (let k = 0; k < 100; k++) c.tick(NO_CODE_INPUT, sens({ theta: 1.0 }));
    expect(Math.abs(c.headingErr)).toBeCloseTo(Math.exp(-1), 2);
  });
  test('the hood servo: channel 2 (left) is inverted, pulses 500…2500 µs over 13…29.85°', () => {
    expect(Hood.pulseFor(13, false)).toBe(500); expect(Hood.pulseFor(29.85, false)).toBe(2500);
    expect(Hood.pulseFor(13, true)).toBe(2500); expect(Hood.pulseFor(29.85, true)).toBe(500);
    expect(Hood.pulseFor(21.425, false)).toBe(1500);
    expect(Hood.pulseFor(5, false)).toBe(500); expect(Hood.pulseFor(40, false)).toBe(2500);
    const h = new Hood(true);
    expect(h.deg).toBeCloseTo(13, 9);
    h.command(29.85);
    for (let k = 0; k < 36; k++) h.step(1 / 120);
    expect(h.deg).toBeGreaterThan(20); expect(h.deg).toBeLessThan(23);   // half of the 0.6 s travel
    for (let k = 0; k < 60; k++) h.step(1 / 120);
    expect(h.deg).toBeCloseTo(29.85, 6);
  });
});

describe('the robot running the code', () => {
  test('flywheels: up to the table\'s speed within 1.5 s of power-up, and spinning with nobody shooting', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2, { clearFuel: true });
    const r = w.robot;
    r.powerCycle();
    expect(r.turrets.every((t) => t.rps === 0)).toBe(true);
    let t95 = -1;
    run(w, 3, NO_INPUT, (t) => { if (t95 < 0 && r.turrets.every((q) => q.rps > 0.97 * q.rpsGoal && q.rpsGoal > 30)) t95 = t; });
    log(`flywheels at power-up: within 3 % of the table speed after ${t95.toFixed(2)} s (goals ${r.turrets.map((t) => t.rpsGoal.toFixed(1))} rps)`);
    expect(t95).toBeGreaterThan(0.5); expect(t95).toBeLessThan(1.5);
    // still spinning ten seconds later, no RT
    run(w, 10, NO_INPUT);
    for (const t of r.turrets) { expect(t.rps).toBeGreaterThan(40); expect(Math.abs(t.rps - t.rpsGoal)).toBeLessThan(1.5); }
    expect(r.shotsFired).toBe(0);
  });

  test('no readiness gate: a ball fed before the wheel is up to speed goes at the wheel\'s speed and falls short', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2);
    const r = w.robot, f = w.fuel;
    // the wheels have dropped to about half speed (a brownout, or RT pressed at power-up)
    const seen: { ball: number; rpsAtLaunch: number; goal: number; speed: number }[] = [];
    r.onShot = (i, side) => { const t = r.turrets[side === 1 ? 0 : 1]; seen.push({ ball: i, rpsAtLaunch: t.rps, goal: t.rpsGoal, speed: 0 }); };
    const before = w.hubs.scoredBy[0];
    // watch launch speeds the step after each shot
    const launched: number[] = [];
    run(w, 1.4, input({ shoot: true }), (t) => {
      // the wheels are held near standstill for the first 0.9 s (power-up, or a brownout), then free to recover
      if (t < 0.9) for (const q of r.turrets) q.flywheel.w = 0.1 * q.rpsGoal * 2 * Math.PI;
      for (const s of seen) if (!launched.includes(s.ball)) { launched.push(s.ball); s.speed = f.speed(s.ball); }
    });
    const first = seen[0];
    log(`first shot with the wheel at ${first.rpsAtLaunch.toFixed(1)} rps of ${first.goal.toFixed(1)}: ball left at ${first.speed.toFixed(2)} m/s (wheel ${(first.rpsAtLaunch * exitFor(1).speed).toFixed(2)} m/s)`);
    expect(first.rpsAtLaunch).toBeLessThan(first.goal * 0.8);
    expect(first.speed).toBeCloseTo(first.rpsAtLaunch * exitFor(1).speed, 0);
    run(w, 3, NO_INPUT);
    // the slow ones did not score: fewer scored than shot
    const slow = seen.filter((s) => s.rpsAtLaunch < s.goal * 0.9).length;
    log(`${seen.length} shots, ${slow} with the wheel under 90 %: scored ${w.hubs.scoredBy[0] - before}`);
    expect(slow).toBeGreaterThan(0);
    expect(w.hubs.scoredBy[0] - before).toBeLessThan(seen.length);
  });

  test('stationary shots from 2…5.2 m at several bearings: at least 90 % score', async () => {
    const hub = { x: HUB_WPI[0] - FIELD_X0, z: -(HUB_WPI[1] - FIELD_Y0) };
    const cases: [number, number][] = [[2, -35], [2, 0], [2.5, 50], [3, -45], [3, 20], [4, -30], [4, 35], [5, 20], [5.2, 38]];
    let shots = 0, scored = 0;
    for (const [d, bearing] of cases) {
      // robot center d from the HUB center, bearing measured from the −x axis (toward our wall), back toward the HUB
      const x = hub.x - d * Math.cos(bearing * DEG), z = hub.z + d * Math.sin(bearing * DEG);
      if (x < -7.8 || Math.abs(z) > 3.5) continue;
      const w = await driverWorld(x, z, Math.atan2(hub.x - x, hub.z - z) + Math.PI);
      const r = w.robot;
      run(w, 2.0, NO_INPUT);
      const before = w.hubs.scoredBy[0], s0 = r.shotsFired;
      run(w, 3.5, input({ shoot: true }));
      run(w, 2.0, NO_INPUT);
      const n = r.shotsFired - s0, sc = w.hubs.scoredBy[0] - before;
      log(`stationary ${d} m @ ${bearing}°: ${sc}/${n} scored (turret distances ${r.turrets.map((t) => t.rpsGoal.toFixed(1))} rps)`);
      expect(n).toBeGreaterThanOrEqual(6);
      shots += n; scored += sc;
    }
    log(`stationary shots overall: ${scored}/${shots}`);
    expect(scored / shots).toBeGreaterThanOrEqual(0.9);
  });

  test('turret wrapping: the robot spinning in place for 2+ turns never takes a turret past its soft limits, and the turret unwinds', async () => {
    const w = await driverWorld(-6.0, 1.0, Math.PI / 2, { clearFuel: true });
    const r = w.robot;
    const lo = [1e9, 1e9], hi = [-1e9, -1e9];
    let unwinds = 0;
    const last = [r.turrets[0].azGoal, r.turrets[1].azGoal];
    let turned = 0, prev = r.th;
    run(w, 4.5, input({ turn: 0.7 }), () => {
      r.turrets.forEach((t, i) => {
        lo[i] = Math.min(lo[i], t.aw); hi[i] = Math.max(hi[i], t.aw);
        if (Math.abs(t.azGoal - last[i]) > 300) unwinds++;
        last[i] = t.azGoal;
      });
      turned += Math.abs(wrapPi(r.th - prev)); prev = r.th;
    });
    log(`spun ${(turned / (2 * Math.PI)).toFixed(1)} turns: left turret ${lo[0].toFixed(0)}…${hi[0].toFixed(0)}°, right ${lo[1].toFixed(0)}…${hi[1].toFixed(0)}°, ${unwinds} unwinds`);
    expect(turned).toBeGreaterThan(2 * 2 * Math.PI);
    for (let i = 0; i < 2; i++) {
      expect(lo[i]).toBeGreaterThanOrEqual(TURRET.min[i] - TURRET.softMargin - 0.5);
      expect(hi[i]).toBeLessThanOrEqual(TURRET.max[i] + TURRET.softMargin + 0.5);
    }
    expect(unwinds).toBeGreaterThanOrEqual(3);
    // the turret ends up pointing at the HUB, whichever way the robot faces
    run(w, 1.5, NO_INPUT);
    for (const t of r.turrets) expect(Math.abs(t.aw - t.azGoal)).toBeLessThan(2);
  });

  test('ferry: only in the neutral zone; the lob carries beyond the ferry target, peaks above the HUB opening, and never scores', async () => {
    // HUB mode in our zone
    const home = await driverWorld(-6.0, 0.5, Math.PI / 2);
    run(home, 0.2, NO_INPUT);
    expect(home.robot.ferry).toBe(false);
    // neutral zone, on the left half: shots go for (−6.64, +2.00) or (−6.64, −2.00)
    const w = await driverWorld(0.5, -1.8, Math.PI / 2);
    const r = w.robot, f = w.fuel;
    run(w, 2, NO_INPUT);
    expect(r.ferry).toBe(true);
    const apex = new Map<number, number>(), landed: { x: number; z: number; apex: number }[] = [];
    r.onShot = (i) => apex.set(i, 0);
    const before = w.hubs.scoredBy[0];
    run(w, 6, input({ shoot: true }), () => {
      for (const [i, a] of apex) {
        if (f.st[i] === BS.FLYING) apex.set(i, Math.max(a, f.pos[i * 3 + 1]));
        else { landed.push({ x: f.pos[i * 3], z: f.pos[i * 3 + 2], apex: a }); apex.delete(i); }
      }
    });
    run(w, 3, NO_INPUT);
    const target = { x: -6.64497, z: -1.99683 };
    log(`ferry from (0.5, −1.8): ${landed.length} landed; first contact x ${landed.slice(0, 6).map((l) => l.x.toFixed(1))}; apex ${landed.slice(0, 6).map((l) => l.apex.toFixed(1))}`);
    expect(landed.length).toBeGreaterThan(3);
    expect(w.hubs.scoredBy[0] - before).toBe(0);
    expect(landed.filter((l) => l.x < -3.2).length / landed.length).toBeGreaterThan(0.6);
    // beyond the target: farther from the robot than the ferry target is
    const beyond = landed.filter((l) => Math.hypot(l.x - 0.5, l.z + 1.8) > Math.hypot(target.x - 0.5, target.z + 1.8) - 0.1).length;
    expect(beyond / landed.length).toBeGreaterThan(0.6);
    for (const l of landed) expect(l.apex).toBeGreaterThan(1.9);
  });

  test('intake: LT puts it down in about 0.65 s, rollers run only while held, it stays down, RB stows it', async () => {
    const w = await driverWorld(-6.0, 0.5, Math.PI / 2, { clearFuel: true, intakeDown: false });
    const r = w.robot;
    expect(r.intakeAngle).toBeCloseTo(INTAKE.upDeg, 0);
    let tDown = -1, rollerWhileHeld = 0;
    run(w, 1.5, input({ intake: true }), (t) => {
      if (tDown < 0 && r.intakeAngle > -2) tDown = t;
      rollerWhileHeld = Math.max(rollerWhileHeld, r.roller.rps);
    });
    log(`LT: arm down (within 2°) after ${tDown.toFixed(2)} s; rollers reached ${rollerWhileHeld.toFixed(0)} rps`);
    expect(tDown).toBeGreaterThan(0.55); expect(tDown).toBeLessThan(0.85);
    expect(rollerWhileHeld).toBeGreaterThan(60); expect(rollerWhileHeld).toBeLessThan(85);
    expect(r.rollers).toBe('in');
    run(w, 1.5, NO_INPUT);
    expect(r.roller.rps).toBeLessThan(2);
    expect(r.rollers).toBe('off');
    expect(r.intakeAngle).toBeGreaterThan(-2);              // stays down
    expect(r.intakeDown).toBe(true);
    run(w, 0.05, input({ retract: true }));
    let tUp = -1;
    run(w, 1.5, NO_INPUT, (t) => { if (tUp < 0 && r.intakeAngle < INTAKE.upDeg + 2) tUp = t; });
    log(`RB: stowed after ${tUp.toFixed(2)} s`);
    expect(tUp).toBeGreaterThan(0.4); expect(tUp).toBeLessThan(1.0);
    // the rollers pull fuel in: unjam (Start + B) pushes it out
    run(w, 0.3, input({ reverseIntake: true }));
    expect(r.roller.rps).toBeLessThan(-30);
  });

  test('drive speed: capped at 2.5 m/s while LT or RT is held, near full speed otherwise (battery sag from the always-on flywheels keeps it under 4.58)', async () => {
    const speedWith = async (extra: Partial<typeof NO_INPUT>) => {
      const w = await driverWorld(-7.0, -3.45, Math.PI / 2, { clearFuel: true });
      let top = 0;
      run(w, 2.4, input({ mz: 1, ...extra }), () => { top = Math.max(top, w.robot.speed); });
      return top;
    };
    const free = await speedWith({});
    const lt = await speedWith({ intake: true });
    const rt = await speedWith({ shoot: true });
    log(`top speed: free ${free.toFixed(2)}, LT held ${lt.toFixed(2)}, RT held ${rt.toFixed(2)} m/s`);
    expect(free).toBeGreaterThan(3.6); expect(free).toBeLessThan(4.62);
    expect(lt).toBeGreaterThan(2.3); expect(lt).toBeLessThan(2.7);
    expect(rt).toBeGreaterThan(2.3); expect(rt).toBeLessThan(2.7);
  });

  test('intake compliance: driving the deployed intake into a wall pushes the arm up, and the motor brings it back down within a second', async () => {
    // the intake end (+z) toward the side guardrail at z = +4.03
    const w = await driverWorld(-5.2, 2.2, 0, { clearFuel: true });
    const r = w.robot;
    expect(r.intakeAngle).toBeCloseTo(0, 1);
    let minArm = 0, v0 = 0;
    run(w, 1.4, input({ mz: 1 }), () => {
      v0 = Math.max(v0, r.speed);
      minArm = Math.min(minArm, r.intakeAngle);
    });
    log(`intake into the wall at ${v0.toFixed(1)} m/s: arm pushed to ${minArm.toFixed(1)}°, robot at z = ${r.z.toFixed(2)}`);
    expect(minArm).toBeLessThan(-10);
    expect(Number.isFinite(r.x + r.z + r.th)).toBe(true);
    // back away: the pivot motor returns it
    let back = -1;
    run(w, 2, input({ mz: -1 }), (t) => { if (back < 0 && t > 0.1 && r.intakeAngle > -3) back = t; });
    log(`back off: arm down again after ${back.toFixed(2)} s`);
    expect(back).toBeGreaterThan(0); expect(back).toBeLessThan(1.0);
    expect(Math.abs(r.pitch)).toBeLessThan(0.3);
  });

  test('A turns the back of the robot to the HUB within a second; Y rotates field-relative driving and the error decays', async () => {
    const w = await driverWorld(-6.1, 1.5, Math.PI / 2, { clearFuel: true });
    const r = w.robot;
    const err = () => { const dx = -3.64497 - r.x, dz = 0.00317 - r.z; return Math.abs(wrapPi(r.th + Math.PI - Math.atan2(dx, dz))); };
    r.body.setRotation({ x: 0, y: Math.sin(0.35), z: 0, w: Math.cos(0.35) }, true);   // facing 0.7 rad, away from the answer
    run(w, 0.05, NO_INPUT);
    const e0 = err();
    let tFace = -1;
    run(w, 1.5, input({ faceHub: true }), (t) => { if (tFace < 0 && err() < 5 * DEG) tFace = t; });
    log(`faceHub: back to the HUB (within 5°) after ${tFace.toFixed(2)} s from ${(e0 / DEG).toFixed(0)}° off`);
    expect(tFace).toBeGreaterThan(0); expect(tFace).toBeLessThan(1.0);
    run(w, 0.5, input({ faceHub: true }));
    expect(err()).toBeLessThan(3 * DEG);

    // Y: the estimate says "facing the opponent" whatever the truth; pushing forward then drives off at the truth's angle off
    const dir = async (wait: number) => {
      const w2 = await driverWorld(-7.0, -3.0, Math.PI / 2 + 0.8, { clearFuel: true });
      run(w2, 0.1, input({ resetHeading: true }));
      run(w2, wait, NO_INPUT);
      const x0 = w2.robot.x, z0 = w2.robot.z;
      run(w2, 0.5, input({ mx: 1 }));
      // direction the robot moved, as an angle from the field +x axis (CCW in WPILib terms: +z is −y)
      return Math.atan2(-(w2.robot.z - z0), w2.robot.x - x0);
    };
    const now = await dir(0), later = await dir(8);
    log(`Y pressed with the robot 0.8 rad off: pushing +x moves it ${now.toFixed(2)} rad off at once, ${later.toFixed(2)} rad after 8 s`);
    expect(now).toBeGreaterThan(0.5); expect(now).toBeLessThan(0.95);
    expect(Math.abs(later)).toBeLessThan(0.12);
  });

  test('feed rate: with a full hopper each turret fires about 4 balls a second', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2, { clearFuel: true });
    const r = w.robot, f = w.fuel;
    const spare = Array.from({ length: f.n }, (_, i) => i).slice(0, 44);
    for (const i of spare) f.place(i, [20, 5, 0]);
    r.preload(spare);
    run(w, 1.5, NO_INPUT);
    expect(r.stored).toBeGreaterThanOrEqual(36);
    const times: number[][] = [[], []];
    r.onShot = (_i, side) => times[side === 1 ? 0 : 1].push(w.clock);
    run(w, 4.5, input({ shoot: true }));
    for (const t of [0, 1]) {
      const ts = times[t];
      const rate = (ts.length - 1) / (ts[ts.length - 1] - ts[0]);
      log(`turret ${t}: ${ts.length} balls in ${(ts[ts.length - 1] - ts[0]).toFixed(1)} s → ${rate.toFixed(2)} balls/s`);
      expect(rate).toBeGreaterThan(3); expect(rate).toBeLessThan(5);
    }
    // hotdog and indexer only run while RT is held
    run(w, 1.5, NO_INPUT);
    expect(Math.abs(r.hotdogMotor.rps)).toBeLessThan(2); expect(Math.abs(r.indexerMotor.rps)).toBeLessThan(2);
  });

  test('nothing moves in the hopper unless RT (or B) is held', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2);
    run(w, 4, input({ intake: true }));
    expect(w.robot.shotsFired).toBe(0);
    expect(w.robot.turrets[0].lane.length + w.robot.turrets[1].lane.length).toBe(0);
    expect(Math.abs(w.robot.hotdogMotor.rps) + Math.abs(w.robot.indexerMotor.rps)).toBeLessThan(1);
  });

  test('B unjams: the lanes run backward and give their balls back to the hopper', async () => {
    const w = await driverWorld(-6.1, 0.4, Math.PI / 2);
    const rb = w.robot;
    // load the lanes without letting the head reach the flywheel
    run(w, 0.7, input({ shoot: true }));
    expect(rb.turrets[0].lane.length + rb.turrets[1].lane.length).toBeGreaterThan(0);
    const stored = rb.stored;
    run(w, 2, input({ unjam: true }));
    expect(rb.hotdogMotor.rps).toBeLessThan(-30); expect(rb.indexerMotor.rps).toBeLessThan(-30);
    expect(rb.turrets[0].lane.length + rb.turrets[1].lane.length).toBe(0);
    expect(rb.stored).toBeGreaterThanOrEqual(stored - rb.shotsFired);
  });

  test('the world builds and the robot powers up like the real one: stowed intake, turrets pointing backward, hood at its minimum', async () => {
    const w = await makeWorld(2);
    w.takeOver();
    const rb = w.robot;
    rb.powerCycle();
    expect(rb.intakeAngle).toBeCloseTo(INTAKE.upDeg, 1);
    expect(rb.turrets.map((t) => t.aw)).toEqual([-180, -180]);
    expect(rb.turrets.map((t) => t.hoodDeg)).toEqual([13, 13]);
    expect(rb.turrets.map((t) => t.pulseUs)).toEqual([2500, 500]);
    expect(rb.turrets.map((t) => t.rps)).toEqual([0, 0]);
  });
});
