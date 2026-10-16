// FUEL: 504 foam balls as Rapier rigid bodies. Per step, on top of the engine's contacts:
//   - air drag  F = ½ρC_dAv²  and Magnus lift  F = ½ρC_LAv² (ω̂×v̂), C_L from Nathan (2008)
//   - spin decay from air torque
//   - rolling resistance on carpet (foam pile deformation): balls slow down and stop quickly
//   - restitution that falls with impact speed (foam loses more energy in hard hits)
// Balls in the HUB or out of play are disabled bodies; balls riding the indexer are kinematic.
import type RAPIER from '@dimforge/rapier3d-compat';
import { FUEL, RHO, STEP } from './config';
import { BALL_E, BALL_MU, IG, type Engine } from './engine';
import type { Rng } from './rng';

/** Ball state. */
export const BS = {
  /** in play, on the field or in the robot */
  FREE: 0,
  /** launched by the shooter, not yet touched anything */
  FLYING: 1,
  /** carried by the indexer belts (kinematic) */
  INDEXER: 2,
  /** inside the HUB, rattling down to the exits (disabled) */
  HUB: 3,
  /** out of play: staged outside the field, or waiting for the human player (disabled) */
  PARKED: 4,
} as const;

const AREA = Math.PI * FUEL.radius * FUEL.radius;
const KD = 0.5 * RHO * FUEL.cd * AREA;     // drag: F = KD v²
const KL = 0.5 * RHO * AREA;               // lift: F = KL C_L v²
/** contact half-period for the squash (s) */
export const SQUASH_TIME = Math.PI * Math.sqrt(FUEL.mass / FUEL.springRate);

export interface FloorFn { (x: number, z: number): number }
export interface ImpactFn { (i: number, speed: number, x: number, y: number, z: number): void }

export class Fuel {
  readonly n: number;
  readonly bodies: RAPIER.RigidBody[] = [];
  readonly colliders: RAPIER.Collider[] = [];
  readonly st: Uint8Array;
  readonly mass: Float32Array;
  /** current and previous step poses (for render interpolation) */
  readonly pos: Float32Array; readonly prev: Float32Array;
  readonly rot: Float32Array; readonly prevRot: Float32Array;
  readonly vel: Float32Array;
  readonly spin: Float32Array;
  /** visual squash: remaining time and amount, and the impact direction */
  readonly squashT: Float32Array; readonly squashA: Float32Array; readonly squashN: Float32Array;
  /** restitution bucket currently set on each collider */
  private eBucket: Int16Array;
  /** how long each ball has been (nearly) still on the floor */
  private still: Float32Array;
  /** the robot that launched a ball ignores it briefly (it leaves through the hood) */
  readonly launchT: Float32Array;
  /** forces applied last step, for the debug overlay */
  readonly dragF: Float32Array; readonly liftF: Float32Array;
  onImpact: ImpactFn | null = null;
  /** total rolling-resistance events, etc. for tests */
  awake = 0;

  constructor(eng: Engine, starts: number[][], rng: Rng, private floor: FloorFn) {
    const R = eng.R, w = eng.world;
    this.n = starts.length;
    this.st = new Uint8Array(this.n);
    this.mass = new Float32Array(this.n);
    this.pos = new Float32Array(this.n * 3); this.prev = new Float32Array(this.n * 3);
    this.rot = new Float32Array(this.n * 4); this.prevRot = new Float32Array(this.n * 4);
    this.vel = new Float32Array(this.n * 3); this.spin = new Float32Array(this.n * 3);
    this.squashT = new Float32Array(this.n); this.squashA = new Float32Array(this.n); this.squashN = new Float32Array(this.n * 3);
    this.eBucket = new Int16Array(this.n).fill(-1);
    this.launchT = new Float32Array(this.n);
    this.still = new Float32Array(this.n);
    this.dragF = new Float32Array(this.n * 3); this.liftF = new Float32Array(this.n * 3);
    for (let i = 0; i < this.n; i++) {
      const m = Math.max(0.203, Math.min(0.227, FUEL.mass + rng.gauss() * FUEL.massSpread));
      this.mass[i] = m;
      const [x, y, z] = starts[i];
      const body = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(x, y, z).setCcdEnabled(true)
        .setRotation(quatFromYaw(rng.range(0, Math.PI * 2), rng.range(-0.6, 0.6)))
        .setCanSleep(true));
      const col = w.createCollider(R.ColliderDesc.ball(FUEL.radius).setMass(m)
        .setFriction(BALL_MU).setRestitution(BALL_E)
        .setFrictionCombineRule(R.CoefficientCombineRule.Multiply).setRestitutionCombineRule(R.CoefficientCombineRule.Multiply)
        .setCollisionGroups(IG.ball), body);
      this.bodies.push(body);
      this.colliders.push(col);
    }
    this.syncAll();
  }

  /** Copy every body's pose into the arrays (after a reset or a teleport). */
  syncAll() {
    for (let i = 0; i < this.n; i++) this.read(i, true);
  }

  private read(i: number, snap: boolean) {
    const b = this.bodies[i];
    const t = b.translation(), q = b.rotation();
    const o = i * 3, r = i * 4;
    if (snap) { this.prev[o] = t.x; this.prev[o + 1] = t.y; this.prev[o + 2] = t.z; this.prevRot[r] = q.x; this.prevRot[r + 1] = q.y; this.prevRot[r + 2] = q.z; this.prevRot[r + 3] = q.w; }
    else { this.prev[o] = this.pos[o]; this.prev[o + 1] = this.pos[o + 1]; this.prev[o + 2] = this.pos[o + 2]; this.prevRot.copyWithin(r, r, r); for (let k = 0; k < 4; k++) this.prevRot[r + k] = this.rot[r + k]; }
    this.pos[o] = t.x; this.pos[o + 1] = t.y; this.pos[o + 2] = t.z;
    this.rot[r] = q.x; this.rot[r + 1] = q.y; this.rot[r + 2] = q.z; this.rot[r + 3] = q.w;
  }

  /** Forces before the engine steps. */
  preStep() {
    const g = 9.81;
    let awake = 0;
    for (let i = 0; i < this.n; i++) {
      const s = this.st[i];
      if (s !== BS.FREE && s !== BS.FLYING) continue;
      const b = this.bodies[i];
      if (b.isSleeping()) continue;
      awake++;
      if (this.launchT[i] > 0) { this.launchT[i] -= STEP; if (this.launchT[i] <= 0) this.colliders[i].setCollisionGroups(IG.ball); }
      const v = b.linvel(), w = b.angvel();
      const o = i * 3;
      this.vel[o] = v.x; this.vel[o + 1] = v.y; this.vel[o + 2] = v.z;
      this.spin[o] = w.x; this.spin[o + 1] = w.y; this.spin[o + 2] = w.z;
      const m = this.mass[i];
      const sp = Math.hypot(v.x, v.y, v.z);
      let ix = 0, iy = 0, iz = 0;
      // air: drag against the motion, Magnus lift along ω × v
      if (sp > 0.8) {
        const fd = KD * sp;              // ×v gives KD v² along −v̂
        let dx = -fd * v.x, dy = -fd * v.y, dz = -fd * v.z;
        const wm = Math.hypot(w.x, w.y, w.z);
        let lx = 0, ly = 0, lz = 0;
        if (wm > 1) {
          const S = (FUEL.radius * wm) / sp;
          const cl = FUEL.liftScale / (FUEL.liftA + FUEL.liftB / S);
          // ω̂ × v̂ scaled to KL C_L v²
          const cx = w.y * v.z - w.z * v.y, cy = w.z * v.x - w.x * v.z, cz = w.x * v.y - w.y * v.x;
          const cm = Math.hypot(cx, cy, cz);
          if (cm > 1e-9) { const f = KL * cl * sp * sp / cm; lx = cx * f; ly = cy * f; lz = cz * f; }
        }
        ix += (dx + lx) * STEP; iy += (dy + ly) * STEP; iz += (dz + lz) * STEP;
        this.dragF[o] = dx; this.dragF[o + 1] = dy; this.dragF[o + 2] = dz;
        this.liftF[o] = lx; this.liftF[o + 1] = ly; this.liftF[o + 2] = lz;
      } else {
        this.dragF[o] = this.dragF[o + 1] = this.dragF[o + 2] = 0;
        this.liftF[o] = this.liftF[o + 1] = this.liftF[o + 2] = 0;
      }
      // rolling on the floor: the foam and the carpet pile soak up energy
      const t = b.translation();
      const fh = this.floor(t.x, t.z);
      const onFloor = t.y < fh + FUEL.radius + 0.006 && Math.abs(v.y) < 0.6;
      const hs = Math.hypot(v.x, v.z);
      if (onFloor && hs > 1e-4) {
        const dec = (fh > 0.01 ? FUEL.rollDecelHdpe : FUEL.rollDecel) + FUEL.rollDamp * hs;
        // never reverse the ball: at most what stops it this step (with the rolling inertia)
        const dv = Math.min(hs, dec * STEP * 1.4);
        const k = (m * dv) / hs;
        ix -= v.x * k; iz -= v.z * k;
      }
      if (ix || iy || iz) b.applyImpulse({ x: ix, y: iy, z: iz }, false);
      // spin decays (air torque, and the carpet scrubbing a ball that spins in place)
      const I = FUEL.inertiaFactor * m * FUEL.radius * FUEL.radius;
      const tau = onFloor ? 0.6 : FUEL.spinTau;
      const kw = I * (1 - Math.exp(-STEP / tau));
      if (onFloor) {
        // only the part of the spin that isn't rolling decays fast on the carpet
        const rx = v.z / FUEL.radius, rz = -v.x / FUEL.radius;
        b.applyTorqueImpulse({ x: -(w.x - rx) * kw, y: -w.y * kw, z: -(w.z - rz) * kw }, false);
      } else b.applyTorqueImpulse({ x: -w.x * kw, y: -w.y * kw, z: -w.z * kw }, false);
      // restitution falls with impact speed: 1 − e ∝ v^(1/5)
      const e = speedFactor(sp);
      const bucket = Math.round(e * 50);
      if (bucket !== this.eBucket[i]) { this.eBucket[i] = bucket; this.colliders[i].setRestitution(BALL_E * bucket / 50); }
      void g;
    }
    this.awake = awake;
  }

  /** Read back after the engine stepped: poses, impacts (for squash and sounds), lost balls. */
  postStep(onLost: (i: number) => void) {
    for (let i = 0; i < this.n; i++) {
      const s = this.st[i];
      if (s === BS.HUB || s === BS.PARKED) continue;
      const b = this.bodies[i];
      if (this.squashT[i] > 0) this.squashT[i] = Math.max(0, this.squashT[i] - STEP);
      if (b.isSleeping()) {
        const o = i * 3;
        this.prev[o] = this.pos[o]; this.prev[o + 1] = this.pos[o + 1]; this.prev[o + 2] = this.pos[o + 2];
        continue;
      }
      this.read(i, false);
      if (s === BS.INDEXER) continue;
      const o = i * 3;
      const v = b.linvel();
      const dvx = v.x - this.vel[o], dvy = v.y - this.vel[o + 1] + 9.81 * STEP, dvz = v.z - this.vel[o + 2];
      const dv = Math.hypot(dvx, dvy, dvz);
      if (dv > 0.7) {
        // a contact impulse: the ball hit something this step
        const sp = Math.hypot(this.vel[o], this.vel[o + 1], this.vel[o + 2]);
        if (sp > 1.2) {
          const a = Math.min(1, sp / 14);
          if (a > this.squashA[i] * (this.squashT[i] / SQUASH_TIME)) {
            this.squashA[i] = a; this.squashT[i] = SQUASH_TIME * 1.6;
            this.squashN[o] = dvx / dv; this.squashN[o + 1] = dvy / dv; this.squashN[o + 2] = dvz / dv;
          }
          this.onImpact?.(i, Math.min(sp, dv), this.pos[o], this.pos[o + 1], this.pos[o + 2]);
        }
        if (s === BS.FLYING && this.launchT[i] <= 0) this.st[i] = BS.FREE;
      }
      this.vel[o] = v.x; this.vel[o + 1] = v.y; this.vel[o + 2] = v.z;
      const x = this.pos[o], y = this.pos[o + 1], z = this.pos[o + 2];
      // at rest on the floor for 0.4 s: put it to sleep (Rapier would wait ~2 s); any contact from
      // something moving wakes it again
      if (dv < 0.05 && Math.hypot(v.x, v.y, v.z) < 0.04 && y < this.floor(x, z) + FUEL.radius + 0.01) {
        if ((this.still[i] += STEP) > 0.4) { b.sleep(); this.still[i] = 0; }
      } else this.still[i] = 0;
      if (!Number.isFinite(x + y + z) || Math.abs(x) > 8.45 || Math.abs(z) > 4.25 || y < -0.05 || y > 15) onLost(i);
    }
  }

  // ------------------------------------------------------------------ state changes
  /** Throw a ball: position, velocity and spin (field frame). */
  launch(i: number, p: number[], v: number[], w: number[]) {
    const b = this.bodies[i];
    if (!b.isEnabled()) b.setEnabled(true);
    this.restore(i);
    b.setTranslation({ x: p[0], y: p[1], z: p[2] }, true);
    b.setLinvel({ x: v[0], y: v[1], z: v[2] }, true);
    b.setAngvel({ x: w[0], y: w[1], z: w[2] }, true);
    this.st[i] = BS.FLYING;
    this.launchT[i] = 0.08;
    const o = i * 3;
    this.vel[o] = v[0]; this.vel[o + 1] = v[1]; this.vel[o + 2] = v[2];
    this.read(i, true);
  }

  /** Put a ball somewhere at rest (in play). */
  place(i: number, p: number[], v: number[] = [0, 0, 0]) {
    const b = this.bodies[i];
    if (!b.isEnabled()) b.setEnabled(true);
    this.restore(i);
    b.setTranslation({ x: p[0], y: p[1], z: p[2] }, true);
    b.setLinvel({ x: v[0], y: v[1], z: v[2] }, true);
    b.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.st[i] = BS.FREE;
    const o = i * 3;
    this.vel[o] = v[0]; this.vel[o + 1] = v[1]; this.vel[o + 2] = v[2];
    this.read(i, true);
  }

  /** Take a ball out of play (disabled), parked at a point for drawing. */
  park(i: number, p: number[], state: number = BS.PARKED) {
    const b = this.bodies[i];
    b.setTranslation({ x: p[0], y: p[1], z: p[2] }, false);
    b.setLinvel({ x: 0, y: 0, z: 0 }, false);
    b.setAngvel({ x: 0, y: 0, z: 0 }, false);
    if (b.isEnabled()) b.setEnabled(false);
    this.st[i] = state;
    this.read(i, true);
  }

  /** The indexer takes a ball: it becomes kinematic and follows the belts. */
  toIndexer(i: number) {
    // carried by the belts: a weightless body that touches nothing, moved along the feed path
    // (switching Rapier body types mid-contact is fragile; this is equivalent and robust)
    const b = this.bodies[i];
    b.enableCcd(false);
    b.setGravityScale(0, true);
    this.colliders[i].setCollisionGroups(0);
    this.st[i] = BS.INDEXER;
  }
  moveKinematic(i: number, x: number, y: number, z: number) {
    const b = this.bodies[i];
    const t = b.translation();
    b.setLinvel({ x: (x - t.x) / STEP, y: (y - t.y) / STEP, z: (z - t.z) / STEP }, true);
    b.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  /** back to a normal ball after riding the indexer */
  private restore(i: number) {
    if (this.st[i] !== BS.INDEXER) return;
    const b = this.bodies[i];
    b.setGravityScale(1, true);
    b.enableCcd(true);
    this.colliders[i].setCollisionGroups(IG.ball);
  }

  isFree(i: number) { const s = this.st[i]; return s === BS.FREE || s === BS.FLYING; }
  speed(i: number) { const o = i * 3; return Math.hypot(this.vel[o], this.vel[o + 1], this.vel[o + 2]); }
  sleeping(i: number) { return this.bodies[i].isSleeping(); }
}

/**
 * Ball-side restitution factor for an impact at speed v: 1 − e(v) = (1 − e_ref)(v / v_ref)^(1/5)
 * with the carpet's e_ref, normalized to 1 at v_ref, so every surface keeps its own value at the
 * reference speed and all of them soften the same way with harder hits.
 */
export function speedFactor(v: number) {
  const e0 = FUEL.restitution.carpet;
  const vv = Math.max(0.35, v);
  const e = 1 - (1 - e0) * Math.pow(vv / FUEL.vRef, 0.2);
  return Math.max(0.3, Math.min(1.4, e / e0));
}

function quatFromYaw(yaw: number, tilt: number) {
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2), cx = Math.cos(tilt / 2), sx = Math.sin(tilt / 2);
  return { x: sx * cy, y: sy * cx, z: -sx * sy, w: cx * cy };
}
