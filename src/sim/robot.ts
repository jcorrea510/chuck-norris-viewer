// Chuck Norris on the field: holonomic swerve dynamics (acceleration, top speed and turn-rate
// limits), pure-pursuit path following, chassis tilt from the real floor (bumps), the intake
// capture zone, fuel tumbling in the hopper (its own little physics world that sloshes as the robot
// accelerates), the full bumper + intake footprint colliding with walls, and both turrets aiming and
// firing through the shot solver.
import * as THREE from 'three';
import type { Viewer } from '../viewer';
import { Balls, S, R, type RobotContact } from './balls';
import type { FieldGrid } from './field';
import { solveShot } from './shot';

const DEG = Math.PI / 180;
export const HX = 0.445, HZ = 0.435;           // bumper half-size (robot frame, front = +z)
const INTAKE = { zMin: 0.52, zMax: 0.80, hx: 0.31 }; // the rollers are the front edge: first touch = capture
const INTAKE_FRONT = 0.7;                        // front of the deployed intake (robot frame)
const MODULES: [number, number][] = [[0.29, 0.29], [-0.29, 0.29], [0.29, -0.29], [-0.29, -0.29]];
export const VMAX = 4.4;                         // m/s (Kraken X60 on MK4i, ~14.5 ft/s real-world)
const AMAX = 7.5;                                // m/s² traction-limited
const WMAX = 7;                                  // rad/s
const ALPHA = 22;                                // rad/s²
const FIRE_INTERVAL = 0.16;                      // per turret (~6 fuel/s each, indexer-limited)
const INTAKE_LANES = 4;                          // the full-width rollers take fuel side by side…
const INTAKE_RATE = 8;                           // …each lane about 8 fuel/s
const INTAKE_TIME = 0.42;                        // from roller, up and over, into the hopper
// Hopper interior (robot frame). The front wall rides on the four-bar: it moves forward when the intake drops.
const HOP = { x: 0.318, floor: 0.14, back: -0.02, frontUp: 0.335, frontDown: 0.6, top: 0.66 };
const G = 9.81;
const TURRET_W = 10;                             // rad/s turret slew
const TURRET_A = 60;                             // rad/s²

export interface DriveOrder {
  path: [number, number][];
  speed: number;
  heading: 'travel' | number;
  stopAtEnd: boolean;
  /** Per-segment heading override (swerve: line up with a trench or bump lane before entering it). */
  segHeading?: (number | null)[];
}

interface HopperBall { i: number; p: THREE.Vector3; v: THREE.Vector3 }

export class RobotSim implements RobotContact {
  x = 0; z = 0; th = 0;
  vx = 0; vz = 0; w = 0;
  private py = 0; private pitch = 0; private roll = 0;
  stored: number[] = [];
  private intaking: { i: number; t: number; from: THREE.Vector3; to: THREE.Vector3 }[] = [];
  /** fuel in the hopper, in the robot frame */
  private hop: HopperBall[] = [];
  private feedWait = 0;
  intakeDown = true;
  rollers = true;
  shooting = false;
  order: DriveOrder | null = null;
  private seg = 0;
  done = true;
  private turrets: { id: string; pivot: THREE.Vector3; base: number; clock: number; err: number; a: number; w: number }[] = [];
  /** intake four-bar angle (deg): 0 = down, -73 = up; moves at a real deploy speed */
  private intakeAngle = 0;
  private intakeClock = new Float32Array(INTAKE_LANES);
  /** swerve modules: pivot (robot frame), wheel rolling heading in the CAD pose, current steer angle */
  private modules: { id: string; px: number; pz: number; h0: number; steer: number }[] = [];
  private accel = new THREE.Vector3();
  private euler = new THREE.Euler();
  shotsFired = 0;
  onShot: ((i: number) => void) | null = null;
  onIntake: ((i: number) => void) | null = null;

  constructor(private viewer: Viewer, private field: FieldGrid, private balls: Balls, private target: number[]) {
    // Turrets: pivot in the robot frame and the shooting heading at 0° (from the CAD flywheel shaft).
    viewer.stage.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(viewer.stage.matrixWorld).invert();
    for (const id of ['turret-left', 'turret-right']) {
      const mech = viewer.mechanisms.get(id);
      if (!mech) continue;
      const p = mech.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(inv);
      this.turrets.push({ id, pivot: p, base: Math.atan2(Math.sign(p.x) * 0.8, -0.6), clock: 0, err: 1, a: 0, w: 0 });
    }
    for (const [id, mech] of viewer.mechanisms) {
      if (!id.startsWith('module-')) continue;
      const p = mech.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(inv);
      // In the CAD the axle points away from the robot's center, so the wheel rolls tangentially.
      const r = Math.hypot(p.x, p.z) || 1;
      this.modules.push({ id, px: p.x, pz: p.z, h0: Math.atan2(-p.z / r, p.x / r), steer: 0 });
    }
  }

  /** Snap the intake to a position (used when placing the robot). */
  setIntakeNow(down: boolean) {
    this.intakeDown = down;
    this.intakeAngle = down ? 0 : -73;
    this.viewer.setMechanismNow('intake', this.intakeAngle);
  }

  /**
   * Swerve steering: each wheel points along its own velocity (robot velocity + rotation),
   * flipping ±90° like real modules do, and locks into an X when the robot stops.
   */
  private steerModules(dt: number) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const vxL = this.vx * c - this.vz * s, vzL = this.vx * s + this.vz * c;
    const moving = this.speed > 0.06 || Math.abs(this.w) > 0.12;
    for (const m of this.modules) {
      const mx = vxL + this.w * m.pz, mz = vzL - this.w * m.px;
      // X-lock: roll toward the robot's center so it can't be pushed
      const want = moving && Math.hypot(mx, mz) > 0.03 ? Math.atan2(mx, mz) : Math.atan2(m.px, m.pz);
      let d = want - m.h0 - m.steer;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      if (d > Math.PI / 2) d -= Math.PI; else if (d < -Math.PI / 2) d += Math.PI; // reverse the wheel instead
      const maxStep = 18 * dt;
      m.steer += Math.max(-maxStep, Math.min(maxStep, d));
      m.steer = Math.atan2(Math.sin(m.steer), Math.cos(m.steer));
    }
  }

  /** 48 with the hopper expanded (intake down), 32 with it stowed */
  get capacity() { return this.intakeDown ? 48 : 32; }
  get load() { return this.stored.length + this.intaking.length; }
  get speed() { return Math.hypot(this.vx, this.vz); }

  place(x: number, z: number, th: number) {
    this.x = x; this.z = z; this.th = th;
    this.vx = this.vz = this.w = 0;
    this.py = this.terrain().y; this.pitch = 0; this.roll = 0;
    this.order = null; this.done = true;
    this.stored = []; this.intaking = []; this.hop = [];
    this.writePose(0);
  }

  preload(indices: number[]) {
    // preloaded fuel sits settled in the back of the hopper
    indices.forEach((i, k) => {
      this.balls.st[i] = S.STORED;
      this.stored.push(i);
      const p = new THREE.Vector3(-0.24 + (k % 4) * 0.16 + (Math.random() - 0.5) * 0.02, HOP.floor + R + Math.floor(k / 8) * 0.14, 0.06 + (Math.floor(k / 4) % 2) * 0.15);
      this.hop.push({ i, p, v: new THREE.Vector3() });
    });
  }

  /**
   * Smallest gap (m) between the footprint — bumpers plus the deployed intake — and any wall or
   * field element at this pose; negative when overlapping.
   */
  clearance(x: number, z: number, th: number, front = this.frontExtent()) {
    const c = Math.cos(th), s = Math.sin(th), f = this.field;
    let min = Infinity;
    const test = (lx: number, lz: number) => { const d = f.distAt(x + lx * c + lz * s, z - lx * s + lz * c); if (d < min) min = d; };
    for (let lx = -HX; lx <= HX + 1e-6; lx += HX / 6) { test(lx, front); test(lx, -HZ); }
    for (let lz = -HZ; lz <= front + 1e-6; lz += (front + HZ) / 10) { test(-HX, lz); test(HX, lz); }
    return min - 0.035;
  }
  private footprintFree(x: number, z: number, th: number, front = this.frontExtent()) { return this.clearance(x, z, th, front) > 0; }
  /** A move is allowed if it ends clear, or (when already touching something) if it backs out. */
  private canMove(x: number, z: number, th: number, now: number) {
    const c = this.clearance(x, z, th);
    return c > 0 || c > now + 1e-4;
  }
  private frontExtent() { return this.intakeAngle > -45 ? INTAKE_FRONT : HZ; }

  drive(order: DriveOrder | null) {
    this.order = order;
    this.seg = 0;
    this.done = !order;
  }

  // ------------------------------------------------------------------ dynamics
  update(dt: number) {
    let tvx = 0, tvz = 0, th = this.th;
    if (this.order && !this.done) {
      const r = this.follow();
      tvx = r.vx; tvz = r.vz; th = r.heading;
    }
    // acceleration limit on the velocity vector (swerve: any direction)
    let dvx = tvx - this.vx, dvz = tvz - this.vz;
    const dv = Math.hypot(dvx, dvz), maxDv = AMAX * dt;
    if (dv > maxDv) { dvx *= maxDv / dv; dvz *= maxDv / dv; }
    this.accel.set(dvx / dt, 0, dvz / dt);
    this.vx += dvx; this.vz += dvz;
    // heading: rate- and acceleration-limited
    let e = th - this.th;
    e = Math.atan2(Math.sin(e), Math.cos(e));
    const wTarget = Math.max(-WMAX, Math.min(WMAX, e * 5.5));
    const dw = Math.max(-ALPHA * dt, Math.min(ALPHA * dt, wTarget - this.w));
    this.w += dw;
    // move: the whole footprint (bumpers + deployed intake) stays out of walls and field elements;
    // blocked motion slides along the obstacle instead of passing through it
    let now = this.clearance(this.x, this.z, this.th);
    const nth = this.th + this.w * dt;
    if (this.canMove(this.x, this.z, nth, now)) { this.th = nth; now = this.clearance(this.x, this.z, this.th); } else this.w = 0;
    const nx = this.x + this.vx * dt, nz = this.z + this.vz * dt;
    if (this.canMove(nx, this.z, this.th, now)) { this.x = nx; now = this.clearance(this.x, this.z, this.th); } else this.vx *= -0.1;
    if (this.canMove(this.x, nz, this.th, now)) this.z = nz; else this.vz *= -0.1;
    for (let k = 0; k < INTAKE_LANES; k++) this.intakeClock[k] -= dt;
    // intake four-bar: ~0.45 s full travel (it waits to drop if it would hit a wall)
    const goal = this.intakeDown ? 0 : -73;
    if (this.intakeAngle !== goal && (goal < this.intakeAngle || this.intakeAngle <= -45 && this.footprintFree(this.x, this.z, this.th, INTAKE_FRONT) || this.intakeAngle > -45)) {
      const step = 165 * dt;
      this.intakeAngle = Math.abs(goal - this.intakeAngle) <= step ? goal : this.intakeAngle + Math.sign(goal - this.intakeAngle) * step;
    }
    this.steerModules(dt);
    this.writePose(dt);
    this.updateHopper(dt);
  }

  /** Once per rendered frame: mechanisms, the robot model and the fuel riding inside it. */
  present() {
    const v = this.viewer;
    v.setMechanismNow('intake', this.intakeAngle);
    for (const t of this.turrets) v.setMechanismNow(t.id, t.a / DEG);
    for (const m of this.modules) v.setMechanismNow(m.id, m.steer / DEG);
    v.syncPose();
    this.writeHopperWorld();
  }

  /** Pure pursuit with a velocity profile (corners, stops, bumps) and a heading command. */
  private follow(): { vx: number; vz: number; heading: number } {
    const o = this.order!;
    const pts = o.path;
    // advance segment when we pass its end
    while (this.seg < pts.length - 2) {
      const [ax, az] = pts[this.seg], [bx, bz] = pts[this.seg + 1];
      const sx = bx - ax, sz = bz - az, L2 = sx * sx + sz * sz;
      const t = L2 > 0 ? ((this.x - ax) * sx + (this.z - az) * sz) / L2 : 1;
      if (t >= 1) this.seg++; else break;
    }
    const [ax, az] = pts[this.seg], [bx, bz] = pts[Math.min(this.seg + 1, pts.length - 1)];
    const sx = bx - ax, sz = bz - az, L = Math.hypot(sx, sz) || 1e-6;
    const t = Math.max(0, Math.min(1, ((this.x - ax) * sx + (this.z - az) * sz) / (L * L)));
    // remaining distance along the path
    let remaining = (1 - t) * L;
    for (let k = this.seg + 1; k < pts.length - 1; k++) remaining += Math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]);
    // lookahead point
    const sp = this.speed;
    let look = 0.35 + 0.28 * sp;
    let tx = ax + sx * t, tz = az + sz * t;
    let k = this.seg;
    let segLeft = (1 - t) * L;
    while (look > 0 && k < pts.length - 1) {
      const [cx, cz] = k === this.seg ? [tx, tz] : pts[k];
      const [dx2, dz2] = pts[k + 1];
      const l = k === this.seg ? segLeft : Math.hypot(dx2 - cx, dz2 - cz);
      if (look <= l) { const f = look / (l || 1); tx = cx + (dx2 - cx) * f; tz = cz + (dz2 - cz) * f; look = 0; break; }
      look -= l; tx = dx2; tz = dz2; k++;
      segLeft = 0;
    }
    let dx = tx - this.x, dz = tz - this.z;
    const dl = Math.hypot(dx, dz);
    if (dl < 1e-4) { dx = sx / L; dz = sz / L; } else { dx /= dl; dz /= dl; }
    // speed profile
    let v = o.speed;
    if (o.stopAtEnd) v = Math.min(v, Math.sqrt(2 * AMAX * 0.7 * Math.max(0, remaining - 0.02)));
    // slow for the next corner
    if (this.seg < pts.length - 2) {
      const [px, pz] = pts[this.seg + 1], [qx, qz] = pts[this.seg + 2];
      const ux = qx - px, uz = qz - pz, ul = Math.hypot(ux, uz) || 1;
      const turn = Math.acos(Math.max(-1, Math.min(1, (sx * ux + sz * uz) / (L * ul))));
      const vCorner = Math.max(0.9, VMAX * (1 - turn / Math.PI) ** 1.6);
      const dCorner = (1 - t) * L;
      v = Math.min(v, Math.sqrt(vCorner * vCorner + 2 * AMAX * 0.7 * dCorner));
    }
    // climb bumps carefully
    if (this.field.floor(this.x, this.z) > 0.02 || this.field.floor(this.x + dx * 0.6, this.z + dz * 0.6) > 0.02) v = Math.min(v, 1.7);
    if (remaining < 0.05 && (!o.stopAtEnd || sp < 0.15)) { this.done = true; return { vx: 0, vz: 0, heading: this.th }; }
    if (!o.stopAtEnd && remaining < 0.25) this.done = true;
    const heading = o.segHeading?.[this.seg] ?? (o.heading === 'travel' ? Math.atan2(dx, dz) : o.heading);
    return { vx: dx * v, vz: dz * v, heading };
  }

  /** Floor under the four swerve modules → ride height, pitch and roll. */
  private terrain() {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const h = MODULES.map(([mx, mz]) => this.field.floor(this.x + mx * c + mz * s, this.z - mx * s + mz * c));
    const front = (h[0] + h[1]) / 2, back = (h[2] + h[3]) / 2, left = (h[0] + h[2]) / 2, right = (h[1] + h[3]) / 2;
    return { y: Math.max(...h) * 0.5 + (h[0] + h[1] + h[2] + h[3]) / 8, pitch: -Math.atan2(front - back, 0.58), roll: Math.atan2(left - right, 0.58) };
  }

  private writePose(dt: number) {
    const t = this.terrain();
    const k = dt > 0 ? 1 - Math.exp(-dt * 14) : 1; // suspension-like smoothing
    this.py += (t.y - this.py) * k;
    this.pitch += (t.pitch - this.pitch) * k;
    this.roll += (t.roll - this.roll) * k;
    const st = this.viewer.stage;
    st.position.set(this.x, this.py, this.z);
    st.quaternion.setFromEuler(this.euler.set(this.pitch, this.th, this.roll, 'YXZ'));
    // only the stage's own matrix here (physics rate); the renderer updates the robot's children
    st.updateMatrix();
    st.matrixWorld.copy(st.matrix);
  }

  // ------------------------------------------------------------------ contact with fuel
  private toLocal(i: number) {
    const o = i * 3;
    const dx = this.balls.pos[o] - this.x, dz = this.balls.pos[o + 2] - this.z;
    const c = Math.cos(this.th), s = Math.sin(this.th);
    return { lx: dx * c - dz * s, lz: dx * s + dz * c, ly: this.balls.pos[o + 1] - this.py };
  }

  contact(i: number, balls: Balls) {
    const { lx, lz, ly } = this.toLocal(i);
    if (ly > 0.42) return;
    // the intake rollers pull in anything they touch, until the hopper is full
    const lane = Math.min(INTAKE_LANES - 1, Math.max(0, Math.floor(((lx + INTAKE.hx) / (2 * INTAKE.hx)) * INTAKE_LANES)));
    if (this.intakeClock[lane] <= 0 && this.intakeDown && this.intakeAngle > -12 && this.rollers && lz > INTAKE.zMin && lz < INTAKE.zMax && Math.abs(lx) < INTAKE.hx && ly < 0.3 && this.load < this.capacity) {
      balls.st[i] = S.INTAKING;
      this.intakeClock[lane] = (1 / INTAKE_RATE) * (0.85 + Math.random() * 0.3);
      // it rides up over the roller where it touched, then drops into the hopper a little inboard
      const to = new THREE.Vector3(lx * 0.8 + (Math.random() - 0.5) * 0.08, 0.5, 0.5 + Math.random() * 0.06);
      this.intaking.push({ i, t: 0, from: new THREE.Vector3(lx, ly, lz), to });
      this.onIntake?.(i);
      return;
    }
    // otherwise the bumpers (and the intake frame) push it
    const front = this.frontExtent();
    const ex = Math.abs(lx) - (HX + R), ez1 = lz - (front + R), ez0 = -(HZ + R) - lz;
    if (ex >= 0 || ez1 >= 0 || ez0 >= 0) return;
    // shallowest way out
    let nlx = 0, nlz = 0, depth = 0;
    if (-ex < Math.min(-ez1, -ez0)) { nlx = Math.sign(lx) || 1; depth = -ex; }
    else if (-ez1 < -ez0) { nlz = 1; depth = -ez1; } else { nlz = -1; depth = -ez0; }
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const wx = nlx * c + nlz * s, wz = -nlx * s + nlz * c;
    const o = i * 3;
    balls.pos[o] += wx * (depth + 0.002);
    balls.pos[o + 2] += wz * (depth + 0.002);
    // never shove fuel through a wall or field element: slide it along instead
    const d = this.field.distAt(balls.pos[o], balls.pos[o + 2]);
    if (d < R) {
      const [ax, az] = this.field.awayFromWalls(balls.pos[o], balls.pos[o + 2]);
      balls.pos[o] += ax * (R - d); balls.pos[o + 2] += az * (R - d);
    }
    balls.pos[o] = Math.max(-8.27 + R, Math.min(8.27 - R, balls.pos[o]));
    balls.pos[o + 2] = Math.max(-4.03 + R, Math.min(4.03 - R, balls.pos[o + 2]));
    // the ball picks up the bumper's velocity at that point (including rotation) plus a little bounce
    const rx = balls.pos[o] - this.x, rz = balls.pos[o + 2] - this.z;
    const pvx = this.vx + this.w * rz, pvz = this.vz - this.w * rx;
    const vn = pvx * wx + pvz * wz;
    const bvn = balls.vel[o] * wx + balls.vel[o + 2] * wz;
    if (bvn < vn + 0.25) {
      const dvn = vn + 0.25 - bvn;
      balls.vel[o] += wx * dvn;
      balls.vel[o + 2] += wz * dvn;
    }
    if (balls.st[i] === S.REST) balls.st[i] = S.MOVING;
  }

  // ------------------------------------------------------------------ hopper
  /**
   * The fuel inside the robot is a small physics world of its own (robot frame): gravity plus the
   * robot's own acceleration (so it sloshes when the robot brakes or turns), ball-to-ball contacts,
   * the hopper walls (the front wall moves with the four-bar) and the powered floor rollers that
   * carry fuel back to the indexer while shooting.
   */
  private updateHopper(dt: number) {
    for (let k = this.intaking.length - 1; k >= 0; k--) {
      const it = this.intaking[k];
      it.t += dt / INTAKE_TIME;
      if (it.t >= 1) {
        this.intaking.splice(k, 1);
        this.balls.st[it.i] = S.STORED;
        this.stored.push(it.i);
        // it leaves the rollers moving backward and down onto the pile
        this.hop.push({ i: it.i, p: it.to.clone(), v: new THREE.Vector3((Math.random() - 0.5) * 0.3, -0.4, -1.1 - Math.random() * 0.5) });
      }
    }
    const hop = this.hop;
    if (!hop.length) return;
    // gravity and the pseudo-force from the robot accelerating, in the robot frame
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const ax = this.accel.x * c - this.accel.z * s, az = this.accel.x * s + this.accel.z * c;
    const gx = -ax - G * Math.sin(this.roll) * 0.7, gz = -az + G * Math.sin(this.pitch) * 0.7;
    const front = HOP.frontUp + (HOP.frontDown - HOP.frontUp) * (1 - Math.min(1, -this.intakeAngle / 73));
    const conveyor = this.shooting || this.intaking.length > 0;
    const D = 2 * R;
    for (const b of hop) {
      b.v.x += gx * dt; b.v.y -= G * dt; b.v.z += gz * dt;
      // foam on polycarb: heavy damping
      const k = Math.exp(-2.2 * dt);
      b.v.x *= k; b.v.z *= k;
      b.p.addScaledVector(b.v, dt);
    }
    // contacts: a couple of relaxation passes
    for (let pass = 0; pass < 2; pass++) {
      for (let a = 0; a < hop.length; a++) {
        const A = hop[a];
        for (let bI = a + 1; bI < hop.length; bI++) {
          const B = hop[bI];
          const dx = B.p.x - A.p.x, dy = B.p.y - A.p.y, dz = B.p.z - A.p.z;
          const d2 = dx * dx + dy * dy + dz * dz;
          if (d2 >= D * D || d2 < 1e-10) continue;
          const d = Math.sqrt(d2), pen = (D - d) / 2;
          const nx = dx / d, ny = dy / d, nz = dz / d;
          A.p.x -= nx * pen; A.p.y -= ny * pen; A.p.z -= nz * pen;
          B.p.x += nx * pen; B.p.y += ny * pen; B.p.z += nz * pen;
          const rv = (B.v.x - A.v.x) * nx + (B.v.y - A.v.y) * ny + (B.v.z - A.v.z) * nz;
          if (rv < 0) {
            const j = -1.15 * rv / 2;
            A.v.x -= j * nx; A.v.y -= j * ny; A.v.z -= j * nz;
            B.v.x += j * nx; B.v.y += j * ny; B.v.z += j * nz;
          }
        }
      }
      for (const b of hop) {
        const p = b.p, v = b.v;
        if (p.y < HOP.floor + R) {
          p.y = HOP.floor + R;
          if (v.y < 0) v.y = -v.y * 0.15;
          // carpet-like friction on the floor rollers; while feeding they carry fuel to the back
          v.x *= 0.96;
          if (conveyor) v.z += (-0.9 - v.z) * 0.08; else v.z *= 0.96;
        }
        if (p.y > HOP.top) { p.y = HOP.top; if (v.y > 0) v.y = 0; }
        if (p.x < -HOP.x + R) { p.x = -HOP.x + R; if (v.x < 0) v.x = -v.x * 0.2; }
        if (p.x > HOP.x - R) { p.x = HOP.x - R; if (v.x > 0) v.x = -v.x * 0.2; }
        if (p.z < HOP.back + R) { p.z = HOP.back + R; if (v.z < 0) v.z = -v.z * 0.2; }
        if (p.z > front - R) { p.z = front - R; if (v.z > 0) v.z = -v.z * 0.2; }
      }
    }
  }

  /** Writes intaking and stored fuel into world space (once per rendered frame). */
  private writeHopperWorld() {
    const m = this.viewer.stage.matrixWorld;
    const v = new THREE.Vector3(), a = new THREE.Vector3(), b = new THREE.Vector3();
    const pos = this.balls.pos;
    // intaking: pulled up the front of the roller, over the top, and dropped inboard
    for (const it of this.intaking) {
      const t = it.t, u = 1 - t;
      a.set(it.from.x, Math.max(it.from.y, 0.2), 0.74);
      b.set(it.to.x, 0.6, 0.62);
      // cubic Bézier: floor contact → in front of the roller → over the roller → drop point
      v.set(
        u * u * u * it.from.x + 3 * u * u * t * a.x + 3 * u * t * t * b.x + t * t * t * it.to.x,
        u * u * u * it.from.y + 3 * u * u * t * a.y + 3 * u * t * t * b.y + t * t * t * it.to.y,
        u * u * u * it.from.z + 3 * u * u * t * a.z + 3 * u * t * t * b.z + t * t * t * it.to.z,
      ).applyMatrix4(m);
      pos[it.i * 3] = v.x; pos[it.i * 3 + 1] = v.y; pos[it.i * 3 + 2] = v.z;
    }
    for (const h of this.hop) {
      v.copy(h.p).applyMatrix4(m);
      pos[h.i * 3] = v.x; pos[h.i * 3 + 1] = v.y; pos[h.i * 3 + 2] = v.z;
    }
  }

  /** The indexer takes the fuel nearest its mouth (back, bottom of the hopper). */
  private nextToFeed(): number {
    let best = -1, bestD = Infinity;
    for (let k = 0; k < this.hop.length; k++) {
      const p = this.hop[k].p;
      const d = (p.z - HOP.back) + (p.y - HOP.floor) * 0.8 + Math.abs(p.x) * 0.3;
      if (d < bestD) { bestD = d; best = k; }
    }
    // the floor rollers need a moment to bring fuel back from the front of an expanded hopper
    if (best >= 0 && bestD > 0.32 && this.feedWait < 0.5) return -1;
    return best;
  }

  // ------------------------------------------------------------------ turrets
  /** Turrets track the HUB continuously; they fire when on target and `allowFire`. */
  aimAndFire(dt: number, allowFire: boolean) {
    const v = this.viewer;
    const yaw = this.th + v.root.rotation.y;
    const m = v.stage.matrixWorld;
    if (allowFire && this.stored.length) this.feedWait += dt; else this.feedWait = 0;
    for (const t of this.turrets) {
      const pw = t.pivot.clone().applyMatrix4(m);
      const want = Math.atan2(this.target[0] - pw.x, this.target[2] - pw.z);
      // compensate for the robot turning underneath the turret
      let goal = want - yaw - t.base - this.w * 0.03;
      goal = Math.atan2(Math.sin(goal), Math.cos(goal));
      // turret servo: ±180° travel (no wrap through the wiring), rate- and accel-limited
      const e = goal - t.a;
      const wGoal = Math.max(-TURRET_W, Math.min(TURRET_W, e * 14));
      t.w += Math.max(-TURRET_A * dt, Math.min(TURRET_A * dt, wGoal - t.w));
      t.a = Math.max(-Math.PI, Math.min(Math.PI, t.a + t.w * dt));
      const actual = yaw + t.base + t.a;
      const err = Math.atan2(Math.sin(want - actual), Math.cos(want - actual));
      t.err = Math.abs(err);
      t.clock -= dt;
      if (allowFire && this.stored.length && t.clock <= 0 && t.err < 2.5 * DEG) {
        t.clock = FIRE_INTERVAL * (0.9 + Math.random() * 0.2);
        this.fire(pw, actual);
      }
    }
    this.shooting = allowFire && this.stored.length > 0;
  }

  private fire(pivotWorld: THREE.Vector3, heading: number) {
    const k = this.nextToFeed();
    if (k < 0) return;
    const i = this.hop[k].i;
    this.hop.splice(k, 1);
    this.stored.splice(this.stored.indexOf(i), 1);
    this.feedWait = 0;
    const exit = [pivotWorld.x + Math.sin(heading) * 0.11, pivotWorld.y + 0.24, pivotWorld.z + Math.cos(heading) * 0.11];
    // the exit point moves with the robot (translation + rotation)
    const rx = exit[0] - this.x, rz = exit[2] - this.z;
    const rv: [number, number] = [this.vx + this.w * rz, this.vz - this.w * rx];
    const aim = [this.target[0] + (Math.random() - 0.5) * 0.4, this.target[1], this.target[2] + (Math.random() - 0.5) * 0.4];
    const shot = solveShot(exit, aim, rv);
    if (!shot) { this.stored.push(i); this.hop.push({ i, p: new THREE.Vector3(0, HOP.floor + R, HOP.back + R), v: new THREE.Vector3() }); return; }
    // real-world dispersion: flywheel speed, hood and turret error
    const sp = 1 + (Math.random() - 0.5) * 0.045;
    const yawErr = (Math.random() - 0.5) * 2.4 * DEG;
    const [x, y, z] = shot.v;
    const lx = x - rv[0], lz = z - rv[1];
    const c = Math.cos(yawErr), s = Math.sin(yawErr);
    this.balls.launch(i, exit, [(lx * c + lz * s) * sp + rv[0], y * sp, (-lx * s + lz * c) * sp + rv[1]]);
    this.shotsFired++;
    this.onShot?.(i);
  }

  /** World positions of the turret exits (for the aim lines). */
  turretExits(): THREE.Vector3[] {
    const m = this.viewer.stage.matrixWorld;
    const yaw = this.th + this.viewer.root.rotation.y;
    return this.turrets.map((t) => {
      const h = yaw + t.base + t.a;
      return t.pivot.clone().applyMatrix4(m).add(new THREE.Vector3(Math.sin(h) * 0.11, 0.24, Math.cos(h) * 0.11));
    });
  }

  turretsOnTarget() { return this.turrets.every((t) => t.err < 4 * DEG); }
}
