// The simulation itself, with no rendering: the field, the fuel, Chuck Norris, whoever is driving
// (the autonomous brains or a person), and the field crew. index.ts draws it; scripts/soak.ts runs
// it for hours of simulated time in Node to prove it never gets stuck.
//   Auto    — the team's autonomous (20 s), scripted from the real routine.
//   Endless — keeps collecting and scoring forever. Touch any control and you drive; 15 s after you
//             let go the robot picks up where you left it.
import { FieldGrid, type GridData, type HubData } from './field';
import { Balls, S, R, hubShape, type HubShape } from './balls';
import { RobotSim } from './robot';
import { SwerveDrive } from './drive';
import { DRIVE, HOPPER, FRONT_WALL } from './geometry';
import { AutoBrain, EndlessBrain, AUTO_SECONDS } from './brain';

export interface FieldData { fuelRadius: number; hubs: HubData[]; fuel: number[][]; grid: GridData }
export type SimMode = 'auto' | 'endless';
export type Control = 'ai' | 'driver';

/** One frame of driver input, already turned into the field frame by whoever reads the controls. */
export interface DriverInput {
  /** desired travel, field frame, each −1…1 (length ≤ 1) */
  mx: number; mz: number;
  /** turn rate −1…1 (positive = clockwise seen from above) */
  turn: number;
  shoot: boolean; intake: boolean; eject: boolean; lock: boolean; slow: boolean;
  /** pressed this frame */
  toggleArm: boolean; handBack: boolean;
}
export const NO_INPUT: DriverInput = { mx: 0, mz: 0, turn: 0, shoot: false, intake: false, eject: false, lock: false, slow: false, toggleArm: false, handBack: false };

export const STEP = 1 / 120;
export const HANDBACK_SECONDS = 15;
export const ALLIANCE_LINE = -4.24;       // red alliance zone is x < this
const SHOOT_X = -4.45;                    // robot center must be this deep for the AI to score
export const CLEAR = 0.5;                 // planning clearance (robot half-diagonal with margin)
// Each alliance's BUMPs and TRENCHes sit on one line across the field (x = ±CROSS_X). A route
// across it enters at CROSS_X ∓ CROSS_OUT and leaves on the other side.
export const CROSS_X = 3.62;
const CROSS_OUT = 1.2;
const BUMP_Z = 1.5;
// the red OUTPOST: its human player rolls fuel back onto the field through the chute
const CHUTE = { x: -7.95, y: 0.55, z: 3.25 };

export interface SimStatus {
  mode: SimMode; phase: string; scored: number; stored: number; capacity: number;
  time: number | null; bps: number; ended: boolean; loading: boolean;
  control: Control; resumeIn: number | null; outOfRange: boolean; intakeDown: boolean;
}

export class SimWorld {
  readonly field: FieldGrid;
  readonly hub: HubShape;
  readonly hubTarget: number[];
  readonly balls: Balls;
  readonly robot: RobotSim;
  brain: AutoBrain | EndlessBrain;
  mode: SimMode = 'auto';
  clock = 0;
  control: Control = 'ai';
  private lastInput = -Infinity;
  /** Trench lane center (|z|), measured from the field grid. */
  trenchZ = 3.38;
  private crossMemo = new Map<number, { kind: 'trench' | 'bump'; z: number; t: number; from: number }>();
  private scoreTimes: number[] = [];
  private chuteClock = 0;
  private sweepIndex = 0;
  private stranded = new Float32Array(0);
  readonly deployedOutline = SwerveDrive.outlineFor(HOPPER.hx, FRONT_WALL.z + 0.005);
  /** the renderer answers whether a point is on screen (fuel is only tidied away off camera) */
  isVisible: (x: number, y: number, z: number) => boolean = () => false;
  /** events for sounds and effects */
  onScore: ((i: number) => void) | null = null;
  onRespawn: (() => void) | null = null;
  onControl: ((c: Control) => void) | null = null;

  constructor(private data: FieldData) {
    const red = [...data.hubs].sort((a, b) => a.center[0] - b.center[0])[0];
    this.field = new FieldGrid(data.grid, data.hubs);
    this.hub = hubShape(red);
    // aim a little past the center of the funnel so near-misses still roll in
    this.hubTarget = [red.center[0] + 0.05, red.center[1] + 0.02, red.center[2]];
    this.balls = new Balls(this.field, this.hub, data.fuel);
    this.stranded = new Float32Array(this.balls.n);
    this.balls.onScore = (i) => { this.scoreTimes.push(this.clock); this.onScore?.(i); };
    this.balls.onLost = (i) => this.park(i);
    this.robot = new RobotSim(this.field, this.balls, this.hubTarget);
    // trench lane: the widest free gap between the side wall and the trench supports
    let best = 0;
    for (let z = 2.4; z < 4.05; z += 0.025) {
      const d = Math.min(this.field.distAt(-CROSS_X, -z), this.field.distAt(-CROSS_X - 0.4, -z), this.field.distAt(-CROSS_X + 0.4, -z));
      if (d > best) { best = d; this.trenchZ = z; }
    }
    this.brain = new AutoBrain(this);
  }

  setMode(mode: SimMode) {
    this.mode = mode;
    this.balls.reset(this.data.fuel);
    this.scoreTimes = [];
    this.stranded.fill(0);
    this.clock = 0;
    this.crossMemo.clear();
    this.control = 'ai';
    this.lastInput = -Infinity;
    // preload: 8 fuel from the red outpost (staged outside the field)
    const pre: number[] = [];
    for (let i = 0; i < this.balls.n && pre.length < 8; i++) if (this.balls.st[i] === S.PARKED && this.balls.pos[i * 3] < -8) pre.push(i);
    const r = this.robot;
    if (mode === 'auto') {
      r.place(ALLIANCE_LINE - 0.445 - 0.03, -this.trenchZ, Math.PI / 2);
      r.setIntakeNow(false);
      this.brain = new AutoBrain(this);
    } else {
      r.place(-5.6, 1.0, Math.PI / 2);
      r.setIntakeNow(true);
      this.brain = new EndlessBrain(this);
    }
    r.preload(pre);
  }

  // ------------------------------------------------------------------ step
  step(dt: number, input: DriverInput = NO_INPUT) {
    this.clock += dt;
    const r = this.robot;
    const active = Math.hypot(input.mx, input.mz) > 0.08 || Math.abs(input.turn) > 0.08 || input.shoot || input.intake || input.eject || input.lock || input.toggleArm;
    if (input.handBack && this.control === 'driver') this.resumeAI();
    else if (active) this.takeOver();
    if (active) this.lastInput = this.clock;
    if (this.control === 'driver' && this.clock - this.lastInput > HANDBACK_SECONDS) this.resumeAI();

    if (this.control === 'driver') this.applyDriver(input);
    else this.brain.update(dt);
    r.update(dt);
    this.balls.step(dt, r);
    if (this.mode === 'endless') { this.humanPlayer(dt); this.sweep(); }
    // a crossing choice lasts until the robot is through it
    for (const [L, m] of this.crossMemo) {
      const across = Math.sign(r.x - L) !== m.from && Math.abs(r.x - L) > CROSS_OUT + 0.3;
      if (across || this.clock - m.t > 20) this.crossMemo.delete(L);
    }
    this.guard();
  }

  /** A person takes the controls (Auto ends early: the robot is theirs, on the same field). */
  takeOver() {
    if (this.control === 'driver') return;
    if (this.mode === 'auto') { this.mode = 'endless'; this.brain = new EndlessBrain(this, true); }
    this.control = 'driver';
    const r = this.robot;
    r.navigate(null);
    r.spinUp = false;
    this.onControl?.('driver');
  }

  resumeAI() {
    if (this.control === 'ai') return;
    this.control = 'ai';
    this.robot.manual = null;
    this.robot.shoot = false;
    this.lastInput = -Infinity;
    if (this.brain instanceof EndlessBrain) this.brain.resume();
    this.onControl?.('ai');
  }

  private applyDriver(input: DriverInput) {
    const r = this.robot;
    const scale = input.slow ? 0.35 : 1;
    // the sticks ask for a velocity; the drivetrain delivers what physics allows
    r.manual = { vx: input.mx * DRIVE.vmax * scale, vz: input.mz * DRIVE.vmax * scale, w: -input.turn * DRIVE.wmax * (input.slow ? 0.4 : 0.75), lock: input.lock };
    if (input.toggleArm) r.intakeDown = !r.intakeDown;
    if (input.intake && !r.intakeDown) r.intakeDown = true;
    if (input.eject && !r.intakeDown) r.intakeDown = true;
    r.rollers = input.eject ? 'out' : input.intake ? 'in' : 'off';
    r.shoot = input.shoot;
    r.spinUp = input.shoot;
  }

  get resumeIn() { return this.control === 'driver' ? Math.max(0, HANDBACK_SECONDS - (this.clock - this.lastInput)) : null; }

  // ------------------------------------------------------------------ the field crew
  /** Parks a ball at the red outpost, for the human player to roll back in. */
  park(i: number) {
    const b = this.balls;
    b.st[i] = S.PARKED;
    b.vel.fill(0, i * 3, i * 3 + 3);
    let n = 0;
    for (let k = 0; k < b.n; k++) if (b.st[k] === S.PARKED && b.pos[k * 3] < -8.2 && b.pos[k * 3 + 2] > 2.9) n++;
    b.pos[i * 3] = -8.4 - (n % 4) * 0.15;
    b.pos[i * 3 + 1] = 0.82 + Math.floor(n / 16) * 0.15;
    b.pos[i * 3 + 2] = 3.1 + (Math.floor(n / 4) % 4) * 0.15;
  }

  /** Outpost fuel the human player can send through the chute. */
  private outpostPool(): number[] {
    const b = this.balls, out: number[] = [];
    for (let k = 0; k < b.n; k++) if (b.st[k] === S.PARKED && b.pos[k * 3] < -8.2 && b.pos[k * 3 + 2] > 2.9) out.push(k);
    return out;
  }

  /** The red human player rolls outpost fuel back in, one every second or so, when it's needed. */
  private humanPlayer(dt: number) {
    this.chuteClock -= dt;
    if (this.chuteClock > 0) return;
    this.chuteClock = 1.3;
    const pool = this.outpostPool();
    if (!pool.length) return;
    const r = this.robot;
    if (Math.hypot(r.x - CHUTE.x, r.z - CHUTE.z) < 1.4) return;
    if (this.reachableCount() > 140 && pool.length < 12) return;
    const i = pool[pool.length - 1];
    this.balls.launch(i, [CHUTE.x, CHUTE.y, CHUTE.z + (Math.random() - 0.5) * 0.25], [1.5 + Math.random() * 0.9, -0.3, -0.5 - Math.random() * 0.6]);
    this.stranded[i] = 0;
  }

  private reachCount = 0;
  private reachT = -1;
  reachableCount() {
    if (this.clock - this.reachT < 3) return this.reachCount;
    this.reachT = this.clock;
    let n = 0;
    for (let i = 0; i < this.balls.n; i++) if (this.balls.available(i) && this.stranded[i] === 0) n++;
    this.reachCount = n;
    return n;
  }

  /**
   * Fuel wedged where no robot could ever pick it up (in a corner, against a field element) for
   * a minute is quietly carried back to the outpost by the field crew, off camera, so hours of
   * Endless never run dry.
   */
  private sweep() {
    const b = this.balls;
    for (let k = 0; k < 6; k++) {
      const i = this.sweepIndex = (this.sweepIndex + 1) % b.n;
      if (b.st[i] !== S.REST) { if (b.st[i] !== S.MOVING) this.stranded[i] = 0; continue; }
      const x = b.pos[i * 3], y = b.pos[i * 3 + 1], z = b.pos[i * 3 + 2];
      if (this.reachable(x, z)) { this.stranded[i] = 0; continue; }
      if (this.stranded[i] === 0) { this.stranded[i] = this.clock; continue; }
      if (this.clock - this.stranded[i] > 60 && !this.isVisible(x, y, z)) { this.park(i); this.stranded[i] = 0; }
    }
  }

  /** Could the robot get its intake roller onto a ball here, from any direction? */
  reachable(x: number, z: number) {
    if (Math.abs(x) > 8.27 || Math.abs(z) > 4.03) return false;
    for (let k = 0; k < 16; k++) {
      const th = (k / 16) * Math.PI * 2, dx = Math.sin(th), dz = Math.cos(th);
      // robot center behind the ball, roller touching it
      const cx = x - dx * 0.71, cz = z - dz * 0.71;
      if (this.field.clearance(cx, cz) < 0.3) continue;
      if (this.robot.drive.clearance(cx, cz, th, this.deployedOutline) > 0.01) return true;
    }
    return false;
  }
  isStranded(i: number) { return this.stranded[i] !== 0; }

  /** Last line of defense for a pit display that runs for hours: nothing may ever go non-finite. */
  private guard() {
    const d = this.robot.drive;
    if (!Number.isFinite(d.x + d.z + d.th + d.vx + d.vz + d.w) || d.clearance(d.x, d.z, d.th) < -0.25) this.respawn();
  }

  /** Put the robot back on its feet in the alliance zone (the renderer covers it with a fade). */
  respawn() {
    const r = this.robot;
    const held: number[] = [];
    r.forEachHeld((i) => held.push(i));
    r.place(-6.2, 0.9, Math.PI / 2);
    r.setIntakeNow(true);
    for (const i of held) this.park(i);
    if (this.brain instanceof EndlessBrain) this.brain.resume();
    this.onRespawn?.();
  }

  // ------------------------------------------------------------------ routing
  /**
   * Plans to (gx, gz). When the trip crosses an alliance's BUMP/TRENCH line it goes through a trench
   * lane or over a bump: `prefer` forces one, otherwise each crossing is chosen at random (and kept
   * until the robot is across, so re-plans don't flip-flop).
   */
  route(gx: number, gz: number, prefer?: 'trench' | 'bump'): { path: [number, number][]; via: string; segHeading?: (number | null)[] } | null {
    const r = this.robot, f = this.field;
    const waypoints: [number, number][] = [];
    let via = '';
    for (const L of [-CROSS_X, CROSS_X]) {
      if ((r.x - L) * (gx - L) >= 0) continue;
      const dir = Math.sign(gx - L);
      let memo = this.crossMemo.get(L);
      if (!memo || prefer && memo.kind !== prefer) {
        const kind = prefer ?? (Math.random() < 0.5 ? 'trench' : 'bump');
        const lane = kind === 'trench' ? this.trenchZ : BUMP_Z;
        const cost = (z: number) => Math.abs(r.z - z) + Math.abs(gz - z);
        const z = cost(lane) <= cost(-lane) ? lane : -lane;
        memo = { kind, z, t: this.clock, from: Math.sign(r.x - L) };
        this.crossMemo.set(L, memo);
      }
      // line up in the open first, so the deployed intake never swings into the trench supports
      waypoints.push([L - dir * (CROSS_OUT + 0.55), memo.z], [L - dir * CROSS_OUT, memo.z], [L + dir * CROSS_OUT, memo.z]);
      via = memo.kind === 'trench' ? 'under the trench' : 'over the bump';
    }
    if (!waypoints.length) {
      const path = f.plan(r.x, r.z, gx, gz, CLEAR);
      return path ? { path, via } : null;
    }
    const out: [number, number][] = [];
    const segHeading: (number | null)[] = [];
    let sx = r.x, sz = r.z;
    for (let k = 0; k < waypoints.length; k += 3) {
      const [pre, entry, exit] = [waypoints[k], waypoints[k + 1], waypoints[k + 2]];
      const dir = Math.sign(exit[0] - pre[0]);
      const lane = dir > 0 ? Math.PI / 2 : -Math.PI / 2;
      // already lined up in (or inside) the lane: carry on from here instead of going back
      const inLane = Math.abs(sz - pre[1]) < 0.4 && (sx - pre[0]) * dir > -0.1;
      if (inLane) {
        if (!out.length) out.push([sx, sz]);
        if ((sx - entry[0]) * dir < -0.05) { out.push(entry); segHeading.push(lane); }
        out.push(exit); segHeading.push(lane);
        [sx, sz] = exit;
        continue;
      }
      const leg = f.plan(sx, sz, pre[0], pre[1], CLEAR, 50);
      if (!leg) return null;
      out.push(...(out.length ? leg.slice(1) : leg));
      while (segHeading.length < out.length - 1) segHeading.push(null);
      // line up with the lane on the approach (intake leading), then drive straight through
      if (out.length >= 2) segHeading[out.length - 2] = lane;
      out.push(entry, exit);
      segHeading.push(lane, lane);
      [sx, sz] = exit;
    }
    const last = f.plan(sx, sz, gx, gz, CLEAR, 50);
    if (!last) return null;
    out.push(...last.slice(1));
    while (segHeading.length < out.length - 1) segHeading.push(null);
    return { path: out, via, segHeading };
  }

  canShoot() { return this.robot.x < SHOOT_X; }

  /** What the robot is doing for the driver, in a few words. */
  private driverPhase() {
    const r = this.robot;
    if (r.shoot) return !r.stored ? 'Shooting · empty' : r.outOfRange ? 'Too far to score' : r.flywheelReady ? 'Shooting' : 'Spinning up';
    if (r.rollers === 'out') return 'Ejecting';
    if (r.rollers === 'in') return r.load >= r.capacity ? 'Hopper full' : 'Intaking';
    if (Math.abs(r.intakeAngle - (r.intakeDown ? 0 : -73)) > 1) return r.intakeDown ? 'Lowering the intake' : r.capacityBlocked ? 'Hopper too full to stow' : 'Raising the intake';
    return r.speed > 0.3 ? 'Driving' : 'Ready';
  }

  status(loading = false): SimStatus {
    const now = this.clock;
    this.scoreTimes = this.scoreTimes.filter((t) => now - t < 3);
    const b = this.brain;
    const driver = this.control === 'driver';
    return {
      mode: this.mode,
      phase: driver ? this.driverPhase() : b.phase,
      scored: this.balls.scored,
      stored: this.robot.load,
      capacity: this.robot.capacity,
      time: this.mode === 'auto' && b instanceof AutoBrain ? Math.max(0, AUTO_SECONDS - b.t) : null,
      bps: this.scoreTimes.length / 3,
      ended: b instanceof AutoBrain && b.ended,
      loading,
      control: this.control,
      resumeIn: this.resumeIn,
      outOfRange: driver && this.robot.outOfRange,
      intakeDown: this.robot.intakeDown,
    };
  }
}

export { R };
