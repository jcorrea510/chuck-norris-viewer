// The physics on this thread: the fixed-step accumulator shared with the worker, and the backend
// tests use when they step the simulation themselves (?physics=main).
import { SimWorld, NO_INPUT, type DriverInput, type SimMode } from './world';
import { STEP } from './phys/config';
import type { Backend, Frame } from './backend';

/** Everything the renderer needs from the world, after `steps` steps; `copy` for sending to another thread. */
export function frameOf(w: SimWorld, alpha: number, steps: number, physMs: number, impacts: number[], respawned: boolean, copy: boolean, debug: boolean): Frame {
  const r = w.robot, f = w.fuel;
  const c = <T extends Float32Array | Uint8Array>(a: T): T => (copy ? a.slice() as T : a);
  return {
    clock: w.clock, alpha, steps, physMs, status: w.status(), impacts, respawned, plannerWorst: w.planner.worst,
    robot: {
      x: r.x, y: r.y, z: r.z, th: r.th, vx: r.vx, vz: r.vz, speed: r.speed, pitch: r.pitch, roll: r.roll,
      p: [...r.p], prevP: [...r.prevP], q: [r.q.x, r.q.y, r.q.z, r.q.w], prevQ: [r.prevQ.x, r.prevQ.y, r.prevQ.z, r.prevQ.w],
      intakeAngle: r.intakeAngle, rollers: r.rollers, ferry: r.ferry,
      turrets: r.turrets.map((t) => ({ id: t.id, a: t.a, fly: t.fly, flyGoal: t.flyGoal })),
      modules: r.modules.map((m) => ({ steer: m.steer, wheel: m.wheel, current: m.current, N: m.N })),
      battery: r.battery, totalCurrent: r.totalCurrent, brownout: r.brownout, slipping: r.slipping,
    },
    fuel: {
      n: f.n, awake: f.awake, st: c(f.st), pos: c(f.pos), prev: c(f.prev), rot: c(f.rot), prevRot: c(f.prevRot),
      squashT: c(f.squashT), squashA: c(f.squashA), squashN: c(f.squashN), vel: c(f.vel), dragF: c(f.dragF), liftF: c(f.liftF),
    },
    netHits: w.hubs.netHits.map((h) => ({ ...h })),
    debug: debug ? (() => { const d = w.engine.world.debugRender(); return { vertices: d.vertices.slice(), colors: d.colors.slice() }; })() : undefined,
  };
}

/** Steps a world with a fixed-step accumulator; shared by both backends. */
export class Stepper {
  private acc = 0;
  impacts: number[] = [];
  respawned = false;
  constructor(readonly world: SimWorld) {
    world.fuel.onImpact = (_i, sp, x, y, z) => { if (this.impacts.length < 64) this.impacts.push(sp, x, y, z); };
    world.onRespawn = () => { this.respawned = true; };
  }
  advance(dt: number, input: DriverInput, copy: boolean, debug = false): Frame {
    const w = this.world;
    this.acc += Math.min(dt, 0.1);
    let steps = 0, phys = 0;
    while (this.acc >= STEP && steps < 12) {
      this.acc -= STEP;
      w.step(steps === 0 ? input : { ...input, toggleArm: false, toggleIntake: false, handBack: false });
      phys += w.stepMs;
      steps++;
    }
    if (steps === 12) this.acc = 0;
    const fr = frameOf(w, this.acc / STEP, steps, phys, this.impacts, this.respawned, copy, debug);
    this.impacts = [];
    this.respawned = false;
    return fr;
  }
}

export class LocalBackend implements Backend {
  frame: Frame | null = null;
  private stepper: Stepper;
  constructor(readonly world: SimWorld) {
    this.stepper = new Stepper(world);
    this.frame = this.stepper.advance(0, NO_INPUT, false);
  }
  tick(dt: number, input: DriverInput, debug = false) { this.frame = this.stepper.advance(dt, input, false, debug); }
  setMode(mode: SimMode) { this.world.setMode(mode); this.frame = this.stepper.advance(0, NO_INPUT, false); }
  resumeAI() { this.world.resumeAI(); }
  dispose() { this.world.dispose(); }
}

