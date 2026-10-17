// The cargo net over the hopper: a diamond-mesh cord net as an XPBD cloth, in the world frame.
//   lattice   knots at (a, b) with a + b even; each knot is tied to its four diagonal neighbours by a
//             cord (5.2 cm, like the real polyester net's ~5 cm mesh); N × M = 18 × 18 cells over the
//             0.66 m × 0.67 m opening. Boundary knots are the webbing: pinned to the top edges of the
//             hopper walls (sides, back) and of the intake's front wall, which rides the four-bar.
//   cords     tension-only distance constraints with a little slack (the sag in the photos) and a
//             stiffness from the cord; knots add a weak shear/bending link two cells apart (a pure
//             diamond lattice would scissor freely). Cord damping is the knots' friction.
//   scheme    small-steps XPBD (Macklin et al. 2019): 6 substeps per 120 Hz step, one Gauss-Seidel
//             pass each. World frame, so gravity and the robot's accelerations (the pins move with
//             the robot) sway it with no fictitious forces.
//   FUEL      each ball near the net is a particle that moves with the cloth through the substeps;
//             sphere-vs-triangle contacts (barycentric weights on the three knots) exchange real
//             momentum: the knots get the impulse in the cloth, the ball's body gets the opposite
//             through Rapier, and the robot gets the net's reaction (the pins carry the load).
// The net runs before the engine step (the ball's path is swept through the cloth, so nothing
// tunnels), with the robot pose at the start of the step extrapolated by its velocity.
import { FUEL, STEP } from './config';
import { couplerOffset } from './fourbar';

/** what the net needs of the robot and the FUEL (keeps this file free of Rapier: the CAD viewer drapes the same net) */
export interface NetRobot {
  p: number[]; intakeAngle: number; captured: { i: number }[];
  body: { linvel(): { x: number; y: number; z: number }; angvel(): { x: number; y: number; z: number }; applyImpulseAtPoint(f: { x: number; y: number; z: number }, p: { x: number; y: number; z: number }, wake: boolean): void };
  toWorld(x: number, y: number, z: number, out?: number[]): number[];
  toLocal(x: number, y: number, z: number, out?: number[]): number[];
}
interface Body {
  isSleeping(): boolean; linvel(): { x: number; y: number; z: number }; angvel(): { x: number; y: number; z: number };
  applyImpulse(f: { x: number; y: number; z: number }, wake: boolean): void; applyTorqueImpulse(t: { x: number; y: number; z: number }, wake: boolean): void;
}
/** launchT: a ball just fired by the shooter ignores the robot for 0.08 s (it leaves over the hopper), the net included */
export interface NetFuel { n: number; st: Uint8Array; pos: Float32Array; mass: Float32Array; launchT: Float32Array; bodies: Body[] }
/** BS.FREE and BS.FLYING (phys/fuel.ts): the states a ball can be in the cloth */
const FREE = 0, FLYING = 1;
/** the coupler's displacement (z, y) at a four-bar angle (deg): robot.ts couplerOffset */
export type CouplerFn = (deg: number) => [number, number];

const R = FUEL.radius;
const G = 9.81;
export const NET = {
  /** cells across (x) and along (z); knots exist where a + b is even */
  N: 18, M: 18,
  /** pins, robot frame: back edge (top of the rear panel) and front edge (top of the intake's front wall, arm deployed), CAD */
  half: 0.33, backZ: -0.02, backY: 0.546, frontZ: 0.646, frontY: 0.549,
  /** total mass (kg): polyester cord and webbing (spec: ~0.5 kg). EDIT: weigh the net */
  mass: 0.5,
  /** cord slack at the deployed design length, fraction; gives the ~4.5 cm sag of the photos. EDIT: measure the sag */
  slack: 0.0033,
  /** cord stiffness (N/m per cord) and weak knot link stiffness. EDIT: pull-test a net panel */
  kCord: 6000, kKnot: 120,
  /** relative cord damping rate (1/s): knot friction and cord hysteresis make a cargo net dead */
  dampCord: 70,
  /** air damping on the cloth (1/s) */
  dampAir: 1.2,
  substeps: 6,
  /** net-ball restitution above 0.5 m/s and contact friction */
  e: 0.04, mu: 0.6,
  /** the cloth cannot go below this height above the robot's floor or outside the walls (robot frame) */
  floorY: 0.31,
  /** cord diameter (m), for contact */
  cord: 0.002,
};
const SUB = NET.substeps, H = STEP / SUB;
const BALL_I = FUEL.inertiaFactor;

interface BallCtx { i: number; m: number; c: Float64Array; v: Float64Array; w: Float64Array; dV: Float64Array; dW: Float64Array; tris: number[]; asleep: boolean }

/** The lattice: knots at (a, b), a + b even; cords to the four diagonal neighbours, weak links two cells apart, triangles for contact and drawing. */
export function netTopology() {
  const { N, M } = NET;
  const index = new Int16Array((N + 1) * (M + 1)).fill(-1);
  const abl: number[] = [];
  for (let b = 0; b <= M; b++) for (let a = 0; a <= N; a++) if ((a + b) % 2 === 0) { index[b * (N + 1) + a] = abl.length / 2; abl.push(a, b); }
  const n = abl.length / 2, ab = Int16Array.from(abl), pinned = new Uint8Array(n);
  for (let k = 0; k < n; k++) { const a = ab[2 * k], b = ab[2 * k + 1]; if (a === 0 || a === N || b === 0 || b === M) pinned[k] = 1; }
  const at = (a: number, b: number) => (a < 0 || b < 0 || a > N || b > M ? -1 : index[b * (N + 1) + a]);
  const cords: number[] = [], links: number[] = [];
  for (let k = 0; k < n; k++) {
    const a = ab[2 * k], b = ab[2 * k + 1];
    for (const [da, db] of [[1, 1], [1, -1]]) { const j = at(a + da, b + db); if (j >= 0 && !(pinned[k] && pinned[j])) cords.push(k, j); }
    for (const [da, db] of [[2, 0], [0, 2]]) { const j = at(a + da, b + db); if (j >= 0 && !(pinned[k] && pinned[j])) links.push(k, j); }
  }
  // triangles: a diamond around each odd position, split along its horizontal diagonal
  const tris: number[] = [];
  for (let b = 0; b <= M; b++) for (let a = 0; a <= N; a++) {
    if ((a + b) % 2 === 0) continue;
    const L = at(a - 1, b), Rr = at(a + 1, b), D = at(a, b - 1), U = at(a, b + 1);
    if (L >= 0 && Rr >= 0 && D >= 0 && U >= 0) tris.push(L, D, Rr, L, Rr, U);
    else if (L < 0 && Rr >= 0 && D >= 0 && U >= 0) tris.push(D, Rr, U);
    else if (Rr < 0 && L >= 0 && D >= 0 && U >= 0) tris.push(L, D, U);
    else if (D < 0 && L >= 0 && Rr >= 0 && U >= 0) tris.push(L, Rr, U);
    else if (U < 0 && L >= 0 && Rr >= 0 && D >= 0) tris.push(L, D, Rr);
  }
  return { n, ab, index, pinned, cords: Uint16Array.from(cords), links: Uint16Array.from(links), tris: Uint16Array.from(tris) };
}

export class Net {
  readonly n: number;
  /** knot positions (m), world frame, and velocities */
  readonly x: Float64Array;
  private v: Float64Array;
  private xp: Float64Array;
  private invM: Float64Array;
  readonly pinned: Uint8Array;
  /** robot-frame knot positions now and one step ago, for the renderer (Float32, 3 per knot) */
  readonly local: Float32Array;
  readonly prevLocal: Float32Array;
  readonly cords: Uint16Array;
  private cordL: Float64Array;
  private links: Uint16Array;
  private linkL: Float64Array;
  /** triangles for contact and drawing, knot indices */
  readonly tris: Uint16Array;
  /** knots (a, b) */
  private ab: Int16Array;
  private pinLocal: Float64Array;
  private pinLocalPrev: Float64Array;
  private pinW0: Float64Array;
  private pinW1: Float64Array;
  private pinV: Float64Array;
  private pinList: number[] = [];
  private arm = 0;
  /** knot index by lattice position, −1 where there is no knot */
  readonly index: Int16Array;
  /** reaction impulse on the robot last step (N·s, world), for tests */
  readonly lastReaction = [0, 0, 0];
  private balls: BallCtx[] = [];
  private lastP = [0, 0, 0];
  /** contacts last step (for tests/debug) */
  contacts = 0;

  constructor(private robot: NetRobot, private coupler: CouplerFn = couplerOffset) {
    const T = netTopology();
    this.index = T.index; this.ab = T.ab; this.n = T.n; this.pinned = T.pinned;
    this.cords = T.cords; this.links = T.links; this.tris = T.tris;
    const n = this.n;
    this.x = new Float64Array(n * 3); this.v = new Float64Array(n * 3); this.xp = new Float64Array(n * 3);
    this.invM = new Float64Array(n);
    this.local = new Float32Array(n * 3); this.prevLocal = new Float32Array(n * 3);
    this.pinLocal = new Float64Array(n * 3); this.pinLocalPrev = new Float64Array(n * 3);
    this.pinW0 = new Float64Array(n * 3); this.pinW1 = new Float64Array(n * 3); this.pinV = new Float64Array(n * 3);
    let free = 0;
    for (let k = 0; k < n; k++) { if (this.pinned[k]) this.pinList.push(k); else free++; }
    for (let k = 0; k < n; k++) this.invM[k] = this.pinned[k] ? 0 : free / NET.mass;
    this.cordL = new Float64Array(this.cords.length / 2); this.linkL = new Float64Array(this.links.length / 2);
    // design lengths: the deployed flat lattice plus slack
    const P = new Float64Array(n * 3);
    this.lattice(0, P);
    for (let c = 0; c < this.cordL.length; c++) this.cordL[c] = dist(P, this.cords[2 * c], this.cords[2 * c + 1]) * (1 + NET.slack);
    for (let c = 0; c < this.linkL.length; c++) this.linkL[c] = dist(P, this.links[2 * c], this.links[2 * c + 1]) * (1 + NET.slack);
    this.reset();
  }

  /** the flat lattice between the pins (robot frame) for an arm angle */
  private lattice(arm: number, out: Float64Array) {
    const { N, M, half, backZ, backY, frontZ, frontY } = NET;
    const [dz, dy] = this.coupler(arm);
    for (let k = 0; k < this.n; k++) {
      const a = this.ab[2 * k], b = this.ab[2 * k + 1], t = b / M;
      out[3 * k] = -half + (2 * half * a) / N;
      out[3 * k + 1] = backY + t * (frontY + dy - backY);
      out[3 * k + 2] = backZ + t * (frontZ + dz - backZ);
    }
  }

  /** Back to the rest shape under the robot's present pose and arm. */
  reset() {
    const r = this.robot;
    this.lastP[0] = r.p[0]; this.lastP[1] = r.p[1]; this.lastP[2] = r.p[2];
    this.arm = r.intakeAngle;
    this.lattice(this.arm, this.pinLocal);
    this.pinLocalPrev.set(this.pinLocal);
    const w = [0, 0, 0];
    for (let k = 0; k < this.n; k++) {
      r.toWorld(this.pinLocal[3 * k], this.pinLocal[3 * k + 1], this.pinLocal[3 * k + 2], w);
      this.x[3 * k] = w[0]; this.x[3 * k + 1] = w[1]; this.x[3 * k + 2] = w[2];
    }
    this.v.fill(0);
    for (let s = 0; s < 90; s++) this.advance(null, true);
    this.v.fill(0);
    this.updateLocal(); this.prevLocal.set(this.local);
  }

  /** Before the engine step: moves the cloth through one step with the balls in contact. */
  preStep(fuel: NetFuel) {
    const r = this.robot;
    // teleported (placed, respawned): the cloth starts over under the robot
    if (Math.hypot(r.p[0] - this.lastP[0], r.p[1] - this.lastP[1], r.p[2] - this.lastP[2]) > 0.4) this.reset();
    this.lastP[0] = r.p[0]; this.lastP[1] = r.p[1]; this.lastP[2] = r.p[2];
    this.arm = r.intakeAngle;
    this.pinLocalPrev.set(this.pinLocal);
    this.lattice(this.arm, this.pinLocal);
    this.contacts = 0;
    this.gatherBalls(fuel);
    this.advance(this.balls);
    this.applyToBalls(fuel);
  }

  /** After the engine step and the robot's pose update: the cloth in the robot's frame. */
  postStep() {
    this.prevLocal.set(this.local);
    this.updateLocal();
  }

  private updateLocal() {
    const r = this.robot, l = [0, 0, 0];
    for (let k = 0; k < this.n; k++) {
      r.toLocal(this.x[3 * k], this.x[3 * k + 1], this.x[3 * k + 2], l);
      this.local[3 * k] = l[0]; this.local[3 * k + 1] = l[1]; this.local[3 * k + 2] = l[2];
    }
  }

  // ------------------------------------------------------------------ balls
  private gatherBalls(fuel: NetFuel) {
    this.balls.length = 0;
    const x = this.x, n = this.n;
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, z0 = Infinity, z1 = -Infinity;
    for (let k = 0; k < n; k++) {
      x0 = Math.min(x0, x[3 * k]); x1 = Math.max(x1, x[3 * k]); y0 = Math.min(y0, x[3 * k + 1]); y1 = Math.max(y1, x[3 * k + 1]); z0 = Math.min(z0, x[3 * k + 2]); z1 = Math.max(z1, x[3 * k + 2]);
    }
    const m = R + 0.2;
    const captured = this.robot.captured;
    for (let i = 0; i < fuel.n; i++) {
      const s = fuel.st[i];
      if (s !== FREE && s !== FLYING) continue;
      if (fuel.launchT[i] > 0) continue;
      const o = i * 3, px = fuel.pos[o], py = fuel.pos[o + 1], pz = fuel.pos[o + 2];
      if (px < x0 - m || px > x1 + m || py < y0 - m || py > y1 + m || pz < z0 - m || pz > z1 + m) continue;
      if (captured.length && captured.some((c) => c.i === i)) continue;
      const b = fuel.bodies[i];
      const asleep = b.isSleeping();
      const v = asleep ? { x: 0, y: 0, z: 0 } : b.linvel(), w = asleep ? { x: 0, y: 0, z: 0 } : b.angvel();
      this.balls.push({
        i, m: fuel.mass[i], c: Float64Array.of(px, py, pz), v: Float64Array.of(v.x, v.y, v.z), w: Float64Array.of(w.x, w.y, w.z),
        dV: new Float64Array(3), dW: new Float64Array(3), tris: [], asleep,
      });
    }
    // triangles that can be reached this step
    const T = this.tris;
    for (const bl of this.balls) {
      const reach = R + 0.06 + Math.hypot(bl.v[0], bl.v[1], bl.v[2]) * STEP;
      for (let t = 0; t < T.length; t += 3) {
        const a = T[t] * 3, b = T[t + 1] * 3, c = T[t + 2] * 3;
        const cx = (x[a] + x[b] + x[c]) / 3 - bl.c[0], cy = (x[a + 1] + x[b + 1] + x[c + 1]) / 3 - bl.c[1], cz = (x[a + 2] + x[b + 2] + x[c + 2]) / 3 - bl.c[2];
        if (cx * cx + cy * cy + cz * cz < reach * reach) bl.tris.push(t);
      }
    }
  }

  private applyToBalls(fuel: NetFuel) {
    const rb = this.robot, J = [0, 0, 0];
    this.lastReaction[0] = this.lastReaction[1] = this.lastReaction[2] = 0;
    for (const bl of this.balls) {
      const m = bl.m, dv = bl.dV, dw = bl.dW;
      const mag = Math.hypot(dv[0], dv[1], dv[2]);
      if (mag < 1e-9) continue;
      J[0] = m * dv[0]; J[1] = m * dv[1]; J[2] = m * dv[2];
      // a sleeping ball the cloth only holds up (≈ g·dt) stays asleep; anything more wakes it
      if (!bl.asleep || mag > 1.5 * G * STEP) {
        const b = fuel.bodies[bl.i];
        b.applyImpulse({ x: J[0], y: J[1], z: J[2] }, true);
        const I = BALL_I * m * R * R;
        b.applyTorqueImpulse({ x: dw[0] * I, y: dw[1] * I, z: dw[2] * I }, true);
      }
      // the cloth's reaction goes into the robot through the pins
      rb.body.applyImpulseAtPoint({ x: -J[0], y: -J[1], z: -J[2] }, { x: bl.c[0], y: bl.c[1], z: bl.c[2] }, true);
      this.lastReaction[0] -= J[0]; this.lastReaction[1] -= J[1]; this.lastReaction[2] -= J[2];
    }
  }

  // ------------------------------------------------------------------ the cloth
  private advance(balls: BallCtx[] | null, settle = false) {
    const r = this.robot, n = this.n, x = this.x, v = this.v, xp = this.xp, invM = this.invM;
    // pins this step: the pose at the start, carried along by the robot's velocity (first order)
    const body = r.body, lv = settle ? { x: 0, y: 0, z: 0 } : body.linvel(), av = settle ? { x: 0, y: 0, z: 0 } : body.angvel();
    const P = r.p;
    const w = [0, 0, 0];
    if (!settle) {
      // each pin travels in a straight line over the step: from where the previous step's arm put it (robot at its
      // start pose) to where this step's arm puts it (robot drifted by its velocity)
      for (const k of this.pinList) {
        const o = 3 * k;
        r.toWorld(this.pinLocalPrev[o], this.pinLocalPrev[o + 1], this.pinLocalPrev[o + 2], w);
        this.pinW0[o] = w[0]; this.pinW0[o + 1] = w[1]; this.pinW0[o + 2] = w[2];
        r.toWorld(this.pinLocal[o], this.pinLocal[o + 1], this.pinLocal[o + 2], w);
        const rx = w[0] - P[0], ry = w[1] - P[1], rz = w[2] - P[2];
        this.pinV[o] = lv.x + av.y * rz - av.z * ry; this.pinV[o + 1] = lv.y + av.z * rx - av.x * rz; this.pinV[o + 2] = lv.z + av.x * ry - av.y * rx;
        this.pinW1[o] = w[0] + this.pinV[o] * STEP; this.pinW1[o + 1] = w[1] + this.pinV[o + 1] * STEP; this.pinW1[o + 2] = w[2] + this.pinV[o + 2] * STEP;
      }
    }
    // the robot's axes in the world, for the keep-out box
    const up = r.toWorld(0, 1, 0, [0, 0, 0]), left = r.toWorld(1, 0, 0, [0, 0, 0]);
    const U = [up[0] - P[0], up[1] - P[1], up[2] - P[2]], X = [left[0] - P[0], left[1] - P[1], left[2] - P[2]];
    // settling (reset) uses heavy air damping and almost no knot friction, so the rest shape is found in a few swings
    const aAir = 1 - (settle ? 10 : NET.dampAir) * H;
    const aCord = (settle ? 2 : NET.dampCord) * H;
    const alphaC = 1 / (NET.kCord * H * H), alphaK = 1 / (NET.kKnot * H * H);
    const cords = this.cords, cordL = this.cordL, links = this.links, linkL = this.linkL;
    for (let s = 0; s < SUB; s++) {
      const t = (s + 1) * H;
      // 1. pins move on
      if (!settle) {
        const f = (s + 1) / SUB;
        for (const k of this.pinList) {
          const o = 3 * k;
          const nx = this.pinW0[o] + (this.pinW1[o] - this.pinW0[o]) * f, ny = this.pinW0[o + 1] + (this.pinW1[o + 1] - this.pinW0[o + 1]) * f, nz = this.pinW0[o + 2] + (this.pinW1[o + 2] - this.pinW0[o + 2]) * f;
          v[o] = (this.pinW1[o] - this.pinW0[o]) / STEP; v[o + 1] = (this.pinW1[o + 1] - this.pinW0[o + 1]) / STEP; v[o + 2] = (this.pinW1[o + 2] - this.pinW0[o + 2]) / STEP;
          x[o] = nx; x[o + 1] = ny; x[o + 2] = nz;
        }
      }
      // 2. gravity and air on the free knots; gravity on the balls
      for (let k = 0; k < n; k++) {
        if (!invM[k]) continue;
        v[3 * k] *= aAir; v[3 * k + 1] = v[3 * k + 1] * aAir - G * H; v[3 * k + 2] *= aAir;
      }
      if (balls) for (const bl of balls) bl.v[1] -= G * H;
      // 3. ball contacts at the velocity level
      if (balls) for (const bl of balls) this.contactBall(bl);
      // 4. predict
      for (let k = 0; k < n; k++) {
        if (invM[k]) { xp[3 * k] = x[3 * k] + H * v[3 * k]; xp[3 * k + 1] = x[3 * k + 1] + H * v[3 * k + 1]; xp[3 * k + 2] = x[3 * k + 2] + H * v[3 * k + 2]; }
        else { xp[3 * k] = x[3 * k]; xp[3 * k + 1] = x[3 * k + 1]; xp[3 * k + 2] = x[3 * k + 2]; }
      }
      if (balls) for (const bl of balls) { bl.c[0] += H * bl.v[0]; bl.c[1] += H * bl.v[1]; bl.c[2] += H * bl.v[2]; }
      // 5. constraints: cords (tension only), then the weak knot links
      solve(xp, invM, cords, cordL, alphaC);
      solve(xp, invM, links, linkL, alphaK);
      // keep-out: not below the hopper's floor or through its side walls
      for (let k = 0; k < n; k++) {
        if (!invM[k]) continue;
        const o = 3 * k;
        const dx = xp[o] - P[0] - lv.x * t, dy = xp[o + 1] - P[1] - lv.y * t, dz = xp[o + 2] - P[2] - lv.z * t;
        const hu = dx * U[0] + dy * U[1] + dz * U[2];
        if (hu < NET.floorY) { const d = NET.floorY - hu; xp[o] += U[0] * d; xp[o + 1] += U[1] * d; xp[o + 2] += U[2] * d; }
        const hx = dx * X[0] + dy * X[1] + dz * X[2];
        const lim = NET.half;
        if (hx > lim) { const d = lim - hx; xp[o] += X[0] * d; xp[o + 1] += X[1] * d; xp[o + 2] += X[2] * d; }
        else if (hx < -lim) { const d = -lim - hx; xp[o] += X[0] * d; xp[o + 1] += X[1] * d; xp[o + 2] += X[2] * d; }
      }
      // 6. velocities from the corrected positions; knot friction along the cords
      for (let k = 0; k < n; k++) {
        if (!invM[k]) continue;
        v[3 * k] = (xp[3 * k] - x[3 * k]) / H; v[3 * k + 1] = (xp[3 * k + 1] - x[3 * k + 1]) / H; v[3 * k + 2] = (xp[3 * k + 2] - x[3 * k + 2]) / H;
        x[3 * k] = xp[3 * k]; x[3 * k + 1] = xp[3 * k + 1]; x[3 * k + 2] = xp[3 * k + 2];
      }
      const f = 1 - Math.exp(-aCord);
      for (let c = 0; c < cordL.length; c++) {
        const i = cords[2 * c], j = cords[2 * c + 1], wi = invM[i], wj = invM[j];
        const o = 3 * i, q = 3 * j;
        const dx = x[q] - x[o], dy = x[q + 1] - x[o + 1], dz = x[q + 2] - x[o + 2];
        const d = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-9;
        const nx = dx / d, ny = dy / d, nz = dz / d;
        const vr = (v[q] - v[o]) * nx + (v[q + 1] - v[o + 1]) * ny + (v[q + 2] - v[o + 2]) * nz;
        const jj = (vr * f) / (wi + wj);
        v[o] += wi * jj * nx; v[o + 1] += wi * jj * ny; v[o + 2] += wi * jj * nz;
        v[q] -= wj * jj * nx; v[q + 1] -= wj * jj * ny; v[q + 2] -= wj * jj * nz;
      }
    }
  }

  /** One ball against the triangles it can reach: sequential impulses on the ball and the three knots. */
  private contactBall(bl: BallCtx) {
    const x = this.x, v = this.v, invM = this.invM, T = this.tris;
    const rc = R + NET.cord * 0.5;
    const cp = [0, 0, 0], bc = [0, 0, 0];
    for (const t of bl.tris) {
      const ia = T[t], ib = T[t + 1], ic = T[t + 2];
      closest(bl.c, x, ia, ib, ic, cp, bc);
      let nx = bl.c[0] - cp[0], ny = bl.c[1] - cp[1], nz = bl.c[2] - cp[2];
      const d = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (d >= rc) continue;
      if (d < 1e-7) { nx = 0; ny = 1; nz = 0; } else { nx /= d; ny /= d; nz /= d; }
      this.contacts++;
      const pen = rc - d;
      // knot velocity at the contact point, and the knots' share of the effective mass
      const wa = bc[0], wb = bc[1], wc = bc[2];
      const ux = wa * v[3 * ia] + wb * v[3 * ib] + wc * v[3 * ic], uy = wa * v[3 * ia + 1] + wb * v[3 * ib + 1] + wc * v[3 * ic + 1], uz = wa * v[3 * ia + 2] + wb * v[3 * ib + 2] + wc * v[3 * ic + 2];
      const kInv = wa * wa * invM[ia] + wb * wb * invM[ib] + wc * wc * invM[ic];
      const m = bl.m, wsum = 1 / m + kInv;
      const vn = (bl.v[0] - ux) * nx + (bl.v[1] - uy) * ny + (bl.v[2] - uz) * nz;
      // push-out: gentle, so a resting ball never pops
      const bias = Math.min(0.6, Math.max(0, pen - 0.0015) * 0.25 / H);
      const target = vn < -0.5 ? -NET.e * vn : 0;
      let jn = (target + bias - vn) / wsum;
      if (jn <= 0) continue;
      this.impulse(bl, ia, ib, ic, wa, wb, wc, nx, ny, nz, jn);
      // friction at the contact point (ball surface vs the cloth), with the ball's spin
      const rx = -nx * R, ry = -ny * R, rz = -nz * R;
      const sx = bl.v[0] + bl.w[1] * rz - bl.w[2] * ry - ux, sy = bl.v[1] + bl.w[2] * rx - bl.w[0] * rz - uy, sz = bl.v[2] + bl.w[0] * ry - bl.w[1] * rx - uz;
      const sn = sx * nx + sy * ny + sz * nz;
      const tx = sx - sn * nx, ty = sy - sn * ny, tz = sz - sn * nz;
      const tm = Math.hypot(tx, ty, tz);
      if (tm > 1e-6) {
        const wt = (1 + 1 / BALL_I) / m + kInv;
        const jt = Math.min(NET.mu * jn, tm / wt);
        this.impulse(bl, ia, ib, ic, wa, wb, wc, -tx / tm, -ty / tm, -tz / tm, jt, true, rx, ry, rz);
      }
    }
  }

  /** Impulse j along (nx, ny, nz) on the ball; −j on the knots by barycentric share. */
  private impulse(bl: BallCtx, ia: number, ib: number, ic: number, wa: number, wb: number, wc: number, nx: number, ny: number, nz: number, j: number, spin = false, rx = 0, ry = 0, rz = 0) {
    const v = this.v, invM = this.invM, m = bl.m;
    const px = nx * j, py = ny * j, pz = nz * j;
    bl.v[0] += px / m; bl.v[1] += py / m; bl.v[2] += pz / m;
    bl.dV[0] += px / m; bl.dV[1] += py / m; bl.dV[2] += pz / m;
    if (spin) {
      // torque impulse r × p on the ball: Δω = r × p / I
      const I = BALL_I * m * R * R;
      const tx = (ry * pz - rz * py) / I, ty = (rz * px - rx * pz) / I, tz = (rx * py - ry * px) / I;
      bl.w[0] += tx; bl.w[1] += ty; bl.w[2] += tz;
      bl.dW[0] += tx; bl.dW[1] += ty; bl.dW[2] += tz;
    }
    for (const [k, wk] of [[ia, wa], [ib, wb], [ic, wc]] as const) {
      const s = (wk * invM[k]) * -j;
      v[3 * k] += nx * s; v[3 * k + 1] += ny * s; v[3 * k + 2] += nz * s;
    }
  }

  speedOf(k: number) { return Math.hypot(this.v[3 * k], this.v[3 * k + 1], this.v[3 * k + 2]); }
  restOf(cord: number) { return this.cordL[cord]; }

  /** How far the cloth's middle hangs below the line between the pins (m): the design sag. */
  get sag() {
    const { N, M } = NET;
    const k = this.index[(M / 2) * (N + 1) + N / 2];
    const pinY = (NET.backY + NET.frontY + this.coupler(this.arm)[1]) / 2;
    return pinY - this.local[3 * k + 1];
  }
}

function dist(P: Float64Array, i: number, j: number) {
  return Math.hypot(P[3 * i] - P[3 * j], P[3 * i + 1] - P[3 * j + 1], P[3 * i + 2] - P[3 * j + 2]);
}

/** One XPBD pass over tension-only distance constraints (a cord never pushes). */
function solve(xp: Float64Array, invM: Float64Array, pairs: Uint16Array, L: Float64Array, alpha: number) {
  for (let c = 0; c < L.length; c++) {
    const i = pairs[2 * c], j = pairs[2 * c + 1];
    const o = 3 * i, q = 3 * j;
    const dx = xp[o] - xp[q], dy = xp[o + 1] - xp[q + 1], dz = xp[o + 2] - xp[q + 2];
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const C = d - L[c];
    if (C <= 0 || d < 1e-9) continue;
    const wi = invM[i], wj = invM[j];
    const dl = -C / (wi + wj + alpha);
    const s = dl / d;
    xp[o] += wi * s * dx; xp[o + 1] += wi * s * dy; xp[o + 2] += wi * s * dz;
    xp[q] -= wj * s * dx; xp[q + 1] -= wj * s * dy; xp[q + 2] -= wj * s * dz;
  }
}

/** Closest point on triangle (a, b, c) to p (Ericson, Real-Time Collision Detection 5.1.5); also its barycentric weights. */
function closest(p: Float64Array, x: Float64Array, ia: number, ib: number, ic: number, out: number[], bary: number[]) {
  const ax = x[3 * ia], ay = x[3 * ia + 1], az = x[3 * ia + 2];
  const bx = x[3 * ib], by = x[3 * ib + 1], bz = x[3 * ib + 2];
  const cx = x[3 * ic], cy = x[3 * ic + 1], cz = x[3 * ic + 2];
  const abx = bx - ax, aby = by - ay, abz = bz - az, acx = cx - ax, acy = cy - ay, acz = cz - az;
  const apx = p[0] - ax, apy = p[1] - ay, apz = p[2] - az;
  const d1 = abx * apx + aby * apy + abz * apz, d2 = acx * apx + acy * apy + acz * apz;
  let u: number, v: number, w: number;
  const set = (uu: number, vv: number, ww: number) => { u = uu; v = vv; w = ww; };
  u = v = w = 0;
  if (d1 <= 0 && d2 <= 0) set(1, 0, 0);
  else {
    const bpx = p[0] - bx, bpy = p[1] - by, bpz = p[2] - bz;
    const d3 = abx * bpx + aby * bpy + abz * bpz, d4 = acx * bpx + acy * bpy + acz * bpz;
    if (d3 >= 0 && d4 <= d3) set(0, 1, 0);
    else {
      const vc = d1 * d4 - d3 * d2;
      if (vc <= 0 && d1 >= 0 && d3 <= 0) { const t = d1 / (d1 - d3); set(1 - t, t, 0); }
      else {
        const cpx = p[0] - cx, cpy = p[1] - cy, cpz = p[2] - cz;
        const d5 = abx * cpx + aby * cpy + abz * cpz, d6 = acx * cpx + acy * cpy + acz * cpz;
        if (d6 >= 0 && d5 <= d6) set(0, 0, 1);
        else {
          const vb = d5 * d2 - d1 * d6;
          if (vb <= 0 && d2 >= 0 && d6 <= 0) { const t = d2 / (d2 - d6); set(1 - t, 0, t); }
          else {
            const va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { const t = (d4 - d3) / ((d4 - d3) + (d5 - d6)); set(0, 1 - t, t); }
            else { const den = 1 / (va + vb + vc); const vv = vb * den, ww = vc * den; set(1 - vv - ww, vv, ww); }
          }
        }
      }
    }
  }
  bary[0] = u!; bary[1] = v!; bary[2] = w!;
  out[0] = u! * ax + v! * bx + w! * cx; out[1] = u! * ay + v! * by + w! * cy; out[2] = u! * az + v! * bz + w! * cz;
}
