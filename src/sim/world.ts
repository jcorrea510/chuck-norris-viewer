// The simulation without any rendering: Rapier physics (phys/), the field and all 504 FUEL, Chuck
// Norris, and the game around them.
//   Free Drive — relaxed: drive whenever you like; left alone, the robot drives itself (Endless).
//   High Score — 3-2-1, then 60 s from the real pre-match layout; at 0:00 the robot is disabled and
//                fuel still in the air counts if it scores within 3 s (manual 6.5).
// Internally we are always the alliance at −x (the red end of the field model). Playing blue is the
// same match seen from the other end: the field is rotationally symmetric, so the renderer turns
// the whole scene 180° and swaps the alliance colors.
import { FieldGrid, type GridData, type HubData } from './field';
import { Engine, type CollisionIndex } from './phys/engine';
import { Fuel, BS } from './phys/fuel';
import { Hubs, HUB } from './phys/hub';
import { Robot } from './phys/robot';
import { Rng } from './phys/rng';
import { STEP, FUEL, DRIVE } from './phys/config';
import { EndlessBrain } from './brain';
import { CROSS_X, CROSS_OUT, BUMP_Z, footprintClearance, OUTLINE_DOWN, type Crossing, type PlanJob, type PlanResult } from './route';
import { InlinePlanner, type Planner } from './planner';

export interface FieldData { fuelRadius: number; hubs: HubData[]; fuel: number[][]; grid: GridData }
export type SimMode = 'free' | 'highscore';
export type Control = 'ai' | 'driver';
export type HsPhase = 'countdown' | 'play' | 'post' | 'results';

/** One frame of driver input, already in the field frame (+x = toward the opponent's wall). */
export interface DriverInput {
  mx: number; mz: number;
  /** turn rate −1…1 (positive = clockwise seen from above) */
  turn: number;
  shoot: boolean; intake: boolean; eject: boolean; lock: boolean; slow: boolean;
  /** pressed this frame */
  toggleArm: boolean; toggleIntake: boolean; handBack: boolean;
}
export const NO_INPUT: DriverInput = { mx: 0, mz: 0, turn: 0, shoot: false, intake: false, eject: false, lock: false, slow: false, toggleArm: false, toggleIntake: false, handBack: false };

export { STEP };
export const HANDBACK_SECONDS = 15;
export const ALLIANCE_LINE = -4.24;       // our ALLIANCE ZONE is x < this (158.6 in from our wall)
export const HS_SECONDS = 60;
export const HS_COUNTDOWN = 3;
export const HS_ASSESS = 3;
const CHUTE = { x: -7.95, y: 0.55, z: 3.25 };

export interface HsResult { score: number; shots: number; accuracy: number; bestRate: number; best: number; newBest: boolean }
export interface SimStatus {
  mode: SimMode; scored: number; rate: number; shooting: boolean; ferry: boolean; outOfRange: boolean;
  control: Control; resumeIn: number | null; intakeDown: boolean; rollers: boolean; stored: number; capacity: number;
  hs: { phase: HsPhase; t: number; result: HsResult | null } | null;
  loading: boolean;
}

export class SimWorld {
  readonly engine: Engine;
  readonly field: FieldGrid;
  readonly fuel: Fuel;
  readonly hubs: Hubs;
  readonly robot: Robot;
  readonly rng: Rng;
  planner: Planner;
  brain: EndlessBrain;
  mode: SimMode = 'free';
  clock = 0;
  control: Control = 'ai';
  private lastInput = -Infinity;
  trenchZ = 3.38;
  private crossMemo = new Map<number, { kind: 'trench' | 'bump'; z: number; t: number; from: number }>();
  private shotTimes: number[] = [];
  private chuteClock = 0;
  /** staged layout: index → start position, and which balls start parked or preloaded */
  private starts: number[][];
  /** the 8 FUEL preloaded in our robot */
  preloadIdx: number[] = [];
  // High Score
  hsPhase: HsPhase = 'countdown';
  hsT = 0;
  hsScore = 0;
  hsShots = 0;
  hsBestRate = 0;
  hsResult: HsResult | null = null;
  /** step timing (ms) for the debug overlay */
  stepMs = 0;
  /** the renderer answers whether a point is on screen (the field crew only works off camera) */
  isVisible: (x: number, y: number, z: number) => boolean = () => false;
  onScore: ((i: number) => void) | null = null;
  onControl: ((c: Control) => void) | null = null;
  onRespawn: (() => void) | null = null;
  onHsPhase: ((p: HsPhase) => void) | null = null;
  bestStore: { get(): number; set(v: number): void } = { get: () => 0, set: () => {} };

  constructor(private data: FieldData, collision: { bin: ArrayBuffer; index: CollisionIndex }, seed = 1, planner?: Planner) {
    this.rng = new Rng(seed);
    this.engine = new Engine();
    this.engine.addField(collision.bin, collision.index);
    this.field = new FieldGrid(data.grid, data.hubs);
    this.starts = this.layout();
    this.fuel = new Fuel(this.engine, this.starts, this.rng, (x, z) => this.field.floor(x, z));
    this.hubs = new Hubs(this.engine, this.fuel, this.rng);
    this.hubs.onScore = (i, h) => this.hubScored(i, h);
    this.robot = new Robot(this.engine, this.fuel, this.rng, (x, z) => this.field.floor(x, z));
    this.robot.onShot = () => { this.shotTimes.push(this.clock); if (this.mode === 'highscore' && this.hsPhase === 'play') this.hsShots++; };
    this.planner = planner ?? new InlinePlanner(this.field);
    // trench lane: the widest free gap between the side wall and the trench supports
    let best = 0;
    for (let z = 2.4; z < 4.05; z += 0.025) {
      const d = Math.min(this.field.distAt(-CROSS_X, -z), this.field.distAt(-CROSS_X - 0.4, -z), this.field.distAt(-CROSS_X + 0.4, -z));
      if (d > best) { best = d; this.trenchZ = z; }
    }
    this.brain = new EndlessBrain(this);
    this.setMode('free');
  }

  /**
   * The real pre-match layout (manual 6.3.4): the field model's 456 staged FUEL (neutral zone,
   * depots, outpost chutes), our 8 preloads, and the 40 the other five robots would have
   * preloaded, which the manual stages in the NEUTRAL ZONE instead: 504 in all.
   */
  private layout(): number[][] {
    const r = FUEL.radius;
    const base = this.data.fuel.map((p) => [p[0], Math.max(p[1], r + 0.002), p[2]]);
    const extra: number[][] = [];
    const free = (x: number, z: number) => [...base, ...extra].every((p) => Math.hypot(p[0] - x, p[2] - z) > 2 * r + 0.004 || p[1] > 0.3);
    const rng = new Rng(77);
    for (let tries = 0; extra.length < 40 && tries < 20000; tries++) {
      // the NEUTRAL ZONE staging box: 72 in deep (x) by 206 in wide (z), around the CENTER LINE
      const x = rng.range(-1.4, 1.4), z = rng.range(-2.7, 2.7);
      if (free(x, z)) extra.push([x, r + 0.002, z]);
    }
    const pre: number[][] = [];
    for (let k = 0; k < 8; k++) pre.push([-12, r, k * 0.2]);
    const all = [...base, ...extra, ...pre];
    this.preloadIdx = Array.from({ length: 8 }, (_, k) => base.length + extra.length + k);
    return all;
  }

  /** Resets everything to the pre-match layout. */
  setMode(mode: SimMode) {
    this.mode = mode;
    const f = this.fuel;
    this.hubs.clear();
    for (let i = 0; i < f.n; i++) {
      const p = this.starts[i];
      if (this.preloadIdx.includes(i)) continue;
      // outpost chutes and anything staged outside the field wait off the field
      if (Math.abs(p[0]) > 8.2 || Math.abs(p[2]) > 4.0) f.park(i, p);
      else { f.place(i, p); f.bodies[i].sleep(); }
    }
    f.syncAll();
    this.clock = 0;
    this.shotTimes = [];
    this.crossMemo.clear();
    this.control = mode === 'highscore' ? 'driver' : 'ai';
    this.lastInput = mode === 'highscore' ? 0 : -Infinity;
    const r = this.robot;
    r.disabled = false;
    r.shoot = false; r.spinUp = false; r.rollers = 'off';
    r.place(-6.1, mode === 'highscore' ? 0 : 1.0, Math.PI / 2);
    r.setIntakeNow(true);
    r.preload(this.preloadIdx);
    this.hubs.scoredBy[0] = this.hubs.scoredBy[1] = 0;
    this.hsScore = 0; this.hsShots = 0; this.hsBestRate = 0; this.hsResult = null; this.hsT = 0;
    if (mode === 'highscore') { this.hsPhase = 'countdown'; r.disabled = true; this.onHsPhase?.('countdown'); }
    this.brain = new EndlessBrain(this, mode === 'highscore');
    // settle the preloads and the staged fuel
    for (let k = 0; k < 120; k++) this.physicsStep();
  }

  get scored() { return this.mode === 'highscore' ? this.hsScore : this.hubs.scoredBy[0]; }

  private hubScored(i: number, hub: number) {
    if (hub !== 0) return;
    if (this.mode === 'highscore' && (this.hsPhase === 'play' || this.hsPhase === 'post')) this.hsScore++;
    this.onScore?.(i);
  }

  // ------------------------------------------------------------------ step
  private physicsStep() {
    this.robot.preStep();
    this.fuel.preStep();
    this.hubs.preStep();
    this.engine.step();
    this.fuel.postStep((i) => this.lost(i));
    this.hubs.postStep();
    this.robot.postStep();
  }

  step(input: DriverInput = NO_INPUT) {
    const t0 = performance.now();
    this.clock += STEP;
    const r = this.robot;
    if (this.mode === 'free') {
      const active = Math.hypot(input.mx, input.mz) > 0.08 || Math.abs(input.turn) > 0.08 || input.shoot || input.intake || input.eject || input.lock || input.toggleArm || input.toggleIntake;
      if (input.handBack && this.control === 'driver') this.resumeAI();
      else if (active) this.takeOver();
      if (active) this.lastInput = this.clock;
      if (this.control === 'driver' && this.clock - this.lastInput > HANDBACK_SECONDS) this.resumeAI();
      if (this.control === 'driver') this.applyDriver(input); else this.brain.update(STEP);
      this.humanPlayer();
    } else this.highScore(input);
    // a crossing choice lasts until the robot is through it
    for (const [L, m] of this.crossMemo) {
      const across = Math.sign(r.x - L) !== m.from && Math.abs(r.x - L) > CROSS_OUT + 0.3;
      if (across || this.clock - m.t > 20) this.crossMemo.delete(L);
    }
    this.physicsStep();
    this.guard();
    this.stepMs = performance.now() - t0;
  }

  private highScore(input: DriverInput) {
    const r = this.robot;
    this.hsT += STEP;
    if (this.hsPhase === 'countdown') {
      r.disabled = true;
      if (this.hsT >= HS_COUNTDOWN) { this.hsPhase = 'play'; this.hsT = 0; r.disabled = false; this.onHsPhase?.('play'); }
    } else if (this.hsPhase === 'play') {
      this.applyDriver(input);
      const rate = this.rate;
      if (rate > this.hsBestRate) this.hsBestRate = rate;
      if (this.hsT >= HS_SECONDS) {
        // end of the match: the robot is disabled; fuel in the air still counts for 3 s
        this.hsPhase = 'post'; this.hsT = 0;
        r.disabled = true; r.manual = null; r.shoot = false; r.spinUp = false; r.rollers = 'off';
        this.onHsPhase?.('post');
      }
    } else if (this.hsPhase === 'post') {
      if (this.hsT >= HS_ASSESS) {
        this.hsPhase = 'results';
        const best = this.bestStore.get();
        const newBest = this.hsScore > best;
        if (newBest) this.bestStore.set(this.hsScore);
        this.hsResult = { score: this.hsScore, shots: this.hsShots, accuracy: this.hsShots ? this.hsScore / this.hsShots : 0, bestRate: this.hsBestRate, best: Math.max(best, this.hsScore), newBest };
        this.onHsPhase?.('results');
      }
    }
  }

  /** Balls leaving the shooter per second, over the last second. */
  get rate() {
    while (this.shotTimes.length && this.clock - this.shotTimes[0] > 1) this.shotTimes.shift();
    return this.shotTimes.length;
  }

  takeOver() {
    this.lastInput = this.clock;
    if (this.control === 'driver') return;
    this.control = 'driver';
    const r = this.robot;
    r.navigate(null);
    r.spinUp = false;
    r.shoot = false;
    r.rollers = 'off';
    this.onControl?.('driver');
  }

  resumeAI() {
    if (this.control === 'ai' || this.mode !== 'free') return;
    this.control = 'ai';
    this.robot.manual = null;
    this.robot.shoot = false;
    this.lastInput = -Infinity;
    this.brain.resume();
    this.onControl?.('ai');
  }

  private applyDriver(input: DriverInput) {
    const r = this.robot;
    const scale = input.slow ? 0.35 : 1;
    // deadband like the robot code; the drivetrain's slew limits do the rest
    const m = Math.hypot(input.mx, input.mz);
    const k = m < DRIVE.deadband ? 0 : (m - DRIVE.deadband) / (1 - DRIVE.deadband) / m;
    const t = Math.abs(input.turn) < DRIVE.deadband ? 0 : input.turn;
    r.manual = { vx: input.mx * k * DRIVE.vmax * scale, vz: input.mz * k * DRIVE.vmax * scale, w: -t * DRIVE.wmax * (input.slow ? 0.4 : 1), lock: input.lock };
    if (input.toggleArm) r.intakeDown = !r.intakeDown;
    // intake on: deploy and run the rollers; off: stop and stow (the robot is then 0.728 m tall
    // and no longer fits under a TRENCH)
    if (input.toggleIntake) {
      const on = r.rollers !== 'in';
      r.rollers = on ? 'in' : 'off';
      r.intakeDown = on;
    }
    if (input.intake) { r.intakeDown = true; r.rollers = 'in'; }
    if (input.eject) { r.intakeDown = true; r.rollers = 'out'; }
    else if (r.rollers === 'out') r.rollers = 'off';
    r.shoot = input.shoot;
    r.spinUp = input.shoot;
  }

  get resumeIn() { return this.control === 'driver' && this.mode === 'free' ? Math.max(0, HANDBACK_SECONDS - (this.clock - this.lastInput)) : null; }

  // ------------------------------------------------------------------ the field crew
  /** A ball that somehow left the field goes to the outpost (the referees put it back in play). */
  private lost(i: number) {
    let n = 0;
    for (let k = 0; k < this.fuel.n; k++) if (this.fuel.st[k] === BS.PARKED && this.fuel.pos[k * 3] < -8.2) n++;
    this.fuel.park(i, [-8.6 - (n % 4) * 0.15, 0.8 + Math.floor(n / 16) * 0.15, 3.1 + (Math.floor(n / 4) % 4) * 0.15]);
  }

  /** In Free Drive the human player rolls outpost fuel back in through the chute when it's needed. */
  private humanPlayer() {
    this.chuteClock -= STEP;
    if (this.chuteClock > 0) return;
    this.chuteClock = 1.3;
    const f = this.fuel;
    let pick = -1, inPlay = 0;
    for (let k = 0; k < f.n; k++) {
      if (f.st[k] === BS.PARKED && f.pos[k * 3] < -8.2 && f.pos[k * 3 + 2] > 2.0) pick = k;
      else if (f.isFree(k) && f.pos[k * 3 + 1] < 0.3) inPlay++;
    }
    if (pick < 0 || inPlay > 200) return;
    if (Math.hypot(this.robot.x - CHUTE.x, this.robot.z - CHUTE.z) < 1.4) return;
    f.launch(pick, [CHUTE.x, CHUTE.y, CHUTE.z + (this.rng.next() - 0.5) * 0.25], [1.5 + this.rng.next() * 0.9, -0.3, -0.5 - this.rng.next() * 0.6], [0, 0, 0]);
  }

  /** Last line of defense for a pit display that runs for hours: nothing may ever go non-finite. */
  private guard() {
    const r = this.robot;
    if (!Number.isFinite(r.x + r.y + r.z + r.vx + r.vz) || r.y < -0.3 || r.y > 2 || Math.abs(r.x) > 8.6 || Math.abs(r.z) > 4.4 || Math.abs(r.pitch) > 1.2 || Math.abs(r.roll) > 1.2) this.respawn();
  }

  respawn() {
    const r = this.robot;
    r.place(-6.2, 0.9, Math.PI / 2);
    r.setIntakeNow(true);
    this.brain.resume();
    this.onRespawn?.();
  }

  // ------------------------------------------------------------------ routing (for the brain)
  /** Decides each alliance-line crossing (trench or bump, kept until the robot is through). */
  crossings(gx: number, gz: number, prefer?: 'trench' | 'bump'): Crossing[] {
    const r = this.robot, out: Crossing[] = [];
    for (const L of [-CROSS_X, CROSS_X]) {
      if ((r.x - L) * (gx - L) >= 0) continue;
      let memo = this.crossMemo.get(L);
      if (!memo || prefer && memo.kind !== prefer) {
        const kind = prefer ?? (this.rng.next() < 0.5 ? 'trench' : 'bump');
        const lane = kind === 'trench' ? this.trenchZ : BUMP_Z;
        const cost = (z: number) => Math.abs(r.z - z) + Math.abs(gz - z);
        const z = cost(lane) <= cost(-lane) ? lane : -lane;
        memo = { kind, z, t: this.clock, from: Math.sign(r.x - L) };
        this.crossMemo.set(L, memo);
      }
      out.push({ L, kind: memo.kind, z: memo.z });
    }
    return out;
  }

  /** Ask the planner (a Web Worker in the browser) for a route to the first reachable goal. */
  plan(goals: [number, number][], cb: (r: PlanResult) => void, prefer?: 'trench' | 'bump') {
    if (!goals.length) return;
    const r = this.robot;
    const job: PlanJob = { sx: r.x, sz: r.z, goals, cross: this.crossings(goals[0][0], goals[0][1], prefer) };
    this.planner.request(job, cb);
  }

  canShoot() { return this.robot.x < ALLIANCE_LINE - 0.2; }
  footprint(x: number, z: number, th: number) { return footprintClearance(this.field, x, z, th, OUTLINE_DOWN); }
  hubCenter() { return { x: -HUB.cx, z: 0 }; }

  status(loading = false): SimStatus {
    const r = this.robot;
    return {
      mode: this.mode, scored: this.scored, rate: this.rate, shooting: r.shoot && r.stored > 0, ferry: r.ferry,
      outOfRange: r.shoot && r.outOfRange, control: this.control, resumeIn: this.resumeIn,
      intakeDown: r.intakeDown, rollers: r.rollers === 'in', stored: r.stored, capacity: r.capacity,
      hs: this.mode === 'highscore' ? { phase: this.hsPhase, t: this.hsPhase === 'play' ? Math.max(0, HS_SECONDS - this.hsT) : this.hsPhase === 'countdown' ? Math.max(0, HS_COUNTDOWN - this.hsT) : 0, result: this.hsResult } : null,
      loading,
    };
  }

  dispose() { this.planner.dispose(); this.engine.dispose(); }
}
