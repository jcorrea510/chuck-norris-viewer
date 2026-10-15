// Chuck Norris on the field: holonomic swerve dynamics (acceleration, top speed and turn-rate
// limits), pure-pursuit path following, chassis tilt from the real floor (bumps), the intake
// capture zone, fuel riding in the hopper, and both turrets aiming and firing through the shot solver.
import * as THREE from 'three';
import type { Viewer } from '../viewer';
import { Balls, S, R, type RobotContact } from './balls';
import type { FieldGrid } from './field';
import { solveShot } from './shot';

const DEG = Math.PI / 180;
export const HX = 0.445, HZ = 0.435;           // bumper half-size (robot frame, front = +z)
const INTAKE = { zMin: 0.40, zMax: 0.79, hx: 0.31 }; // the rollers are the front edge: first touch = capture
const MODULES: [number, number][] = [[0.29, 0.29], [-0.29, 0.29], [0.29, -0.29], [-0.29, -0.29]];
export const VMAX = 4.4;                         // m/s (Kraken X60 on MK4i, ~14.5 ft/s real-world)
const AMAX = 7.5;                                // m/s² traction-limited
const WMAX = 7;                                  // rad/s
const ALPHA = 22;                                // rad/s²
const FIRE_INTERVAL = 0.16;                      // per turret (~6 fuel/s each, indexer-limited)
const INTAKE_RATE = 12;                          // fuel/s the rollers can swallow
const INTAKE_TIME = 0.32;                        // from roller to the hopper
const TURRET_W = 10;                             // rad/s turret slew
const TURRET_A = 60;                             // rad/s²

export interface DriveOrder {
  path: [number, number][];
  speed: number;
  heading: 'travel' | number;
  stopAtEnd: boolean;
}

interface Slot { p: THREE.Vector3; frontRow: boolean }

export class RobotSim implements RobotContact {
  x = 0; z = 0; th = 0;
  vx = 0; vz = 0; w = 0;
  private py = 0; private pitch = 0; private roll = 0;
  stored: number[] = [];
  private intaking: { i: number; t: number; from: THREE.Vector3 }[] = [];
  private localPos = new Map<number, THREE.Vector3>();
  intakeDown = true;
  rollers = true;
  shooting = false;
  order: DriveOrder | null = null;
  private seg = 0;
  done = true;
  private slots: Slot[] = [];
  private turrets: { id: string; pivot: THREE.Vector3; base: number; clock: number; err: number; a: number; w: number }[] = [];
  /** intake four-bar angle (deg): 0 = down, -73 = up; moves at a real deploy speed */
  private intakeAngle = 0;
  private intakeClock = 0;
  private accel = new THREE.Vector3();
  private m = new THREE.Matrix4();
  shotsFired = 0;
  onShot: ((i: number) => void) | null = null;
  onIntake: ((i: number) => void) | null = null;

  constructor(private viewer: Viewer, private field: FieldGrid, private balls: Balls, private target: number[]) {
    // Hopper slots (robot frame): 4 across × 4 deep × 3 high, filled bottom-rear first. The two
    // front rows only exist when the intake is down (the four-bar pulls the hopper wall forward).
    const ys = [0.21, 0.36, 0.51], zs = [0.06, 0.21, 0.36, 0.51], xs = [-0.225, -0.075, 0.075, 0.225];
    for (const y of ys) for (const z of zs) for (const x of xs) this.slots.push({ p: new THREE.Vector3(x, y, z), frontRow: z > 0.3 });
    // Turrets: pivot in the robot frame and the shooting heading at 0° (from the CAD flywheel shaft).
    viewer.stage.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(viewer.stage.matrixWorld).invert();
    for (const id of ['turret-left', 'turret-right']) {
      const mech = viewer.mechanisms.get(id);
      if (!mech) continue;
      const p = mech.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(inv);
      this.turrets.push({ id, pivot: p, base: Math.atan2(Math.sign(p.x) * 0.8, -0.6), clock: 0, err: 1, a: 0, w: 0 });
    }
  }

  /** Snap the intake to a position (used when placing the robot). */
  setIntakeNow(down: boolean) {
    this.intakeDown = down;
    this.intakeAngle = down ? 0 : -73;
    this.viewer.setMechanismNow('intake', this.intakeAngle);
  }

  get capacity() { return this.slots.filter((s) => this.intakeDown || !s.frontRow).length; }
  get load() { return this.stored.length + this.intaking.length; }
  get speed() { return Math.hypot(this.vx, this.vz); }

  place(x: number, z: number, th: number) {
    this.x = x; this.z = z; this.th = th;
    this.vx = this.vz = this.w = 0;
    this.py = this.terrain().y; this.pitch = 0; this.roll = 0;
    this.order = null; this.done = true;
    this.stored = []; this.intaking = []; this.localPos.clear();
    this.writePose(0);
  }

  preload(indices: number[]) {
    for (const i of indices) { this.balls.st[i] = S.STORED; this.stored.push(i); }
  }

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
    // move, never into a wall
    const nx = this.x + this.vx * dt, nz = this.z + this.vz * dt;
    if (this.field.distAt(nx, this.z) > 0.36) this.x = nx; else this.vx *= -0.1;
    if (this.field.distAt(this.x, nz) > 0.36) this.z = nz; else this.vz *= -0.1;
    this.th += this.w * dt;
    this.intakeClock -= dt;
    // intake four-bar: ~0.45 s full travel
    const goal = this.intakeDown ? 0 : -73;
    if (this.intakeAngle !== goal) {
      const step = 165 * dt;
      this.intakeAngle = Math.abs(goal - this.intakeAngle) <= step ? goal : this.intakeAngle + Math.sign(goal - this.intakeAngle) * step;
      this.viewer.setMechanismNow('intake', this.intakeAngle);
    }
    for (const t of this.turrets) this.viewer.setMechanismNow(t.id, t.a / DEG);
    this.viewer.syncPose();
    this.writePose(dt);
    this.updateHopper(dt);
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
    const heading = o.heading === 'travel' ? Math.atan2(dx, dz) : o.heading;
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
    st.quaternion.setFromEuler(new THREE.Euler(this.pitch, this.th, this.roll, 'YXZ'));
    st.updateMatrixWorld(true);
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
    if (this.intakeClock <= 0 && this.intakeDown && this.intakeAngle > -12 && this.rollers && lz > INTAKE.zMin && lz < INTAKE.zMax && Math.abs(lx) < INTAKE.hx && ly < 0.3 && this.load < this.capacity) {
      balls.st[i] = S.INTAKING;
      this.intakeClock = 1 / INTAKE_RATE;
      this.intaking.push({ i, t: 0, from: new THREE.Vector3(lx, ly, lz) });
      this.onIntake?.(i);
      return;
    }
    // otherwise the bumpers (and the intake frame) push it
    const front = this.intakeDown ? 0.7 : HZ;
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
  private slotOrder() {
    return this.slots.filter((s) => this.intakeDown || !s.frontRow);
  }

  private updateHopper(dt: number) {
    const slots = this.slotOrder();
    // finished intakes join the hopper
    for (let k = this.intaking.length - 1; k >= 0; k--) {
      const it = this.intaking[k];
      it.t += dt / INTAKE_TIME;
      if (it.t >= 1) {
        this.intaking.splice(k, 1);
        this.balls.st[it.i] = S.STORED;
        this.stored.push(it.i);
        this.localPos.set(it.i, new THREE.Vector3(0, 0.36, 0.44));
      }
    }
    this.m.copy(this.viewer.stage.matrixWorld);
    const v = new THREE.Vector3();
    const pos = this.balls.pos;
    // intaking: up the kicker ramp and over the roller into the hopper
    for (const it of this.intaking) {
      const t = it.t, a = it.from, b = new THREE.Vector3(0, 0.36, 0.44);
      const mid = new THREE.Vector3(a.x * 0.5, 0.3, 0.6);
      const u = 1 - t;
      v.set(u * u * a.x + 2 * u * t * mid.x + t * t * b.x, u * u * a.y + 2 * u * t * mid.y + t * t * b.y, u * u * a.z + 2 * u * t * mid.z + t * t * b.z);
      v.applyMatrix4(this.m);
      pos.set([v.x, v.y, v.z], it.i * 3);
    }
    // stored fuel settles into its slot and sloshes with the robot's acceleration
    const ax = -this.accel.x, az = -this.accel.z;
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const slx = Math.max(-0.02, Math.min(0.02, (ax * c - az * s) * 0.0025));
    const slz = Math.max(-0.02, Math.min(0.02, (ax * s + az * c) * 0.0025));
    this.stored.forEach((i, k) => {
      const slot = slots[Math.min(k, slots.length - 1)].p;
      const lp = this.localPos.get(i) ?? this.localPos.set(i, slot.clone()).get(i)!;
      const target = v.set(slot.x + slx * (1 + slot.y), slot.y, slot.z + slz * (1 + slot.y));
      lp.lerp(target, 1 - Math.exp(-dt * 10));
      const w = lp.clone().applyMatrix4(this.m);
      pos.set([w.x, w.y, w.z], i * 3);
    });
  }

  // ------------------------------------------------------------------ turrets
  /** Turrets track the HUB continuously; they fire when on target and `allowFire`. */
  aimAndFire(dt: number, allowFire: boolean) {
    const v = this.viewer;
    const yaw = this.th + v.root.rotation.y;
    const m = v.stage.matrixWorld;
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
    const i = this.stored.shift();
    if (i === undefined) return;
    this.localPos.delete(i);
    const exit = [pivotWorld.x + Math.sin(heading) * 0.11, pivotWorld.y + 0.24, pivotWorld.z + Math.cos(heading) * 0.11];
    // the exit point moves with the robot (translation + rotation)
    const rx = exit[0] - this.x, rz = exit[2] - this.z;
    const rv: [number, number] = [this.vx + this.w * rz, this.vz - this.w * rx];
    const aim = [this.target[0] + (Math.random() - 0.5) * 0.4, this.target[1], this.target[2] + (Math.random() - 0.5) * 0.4];
    const shot = solveShot(exit, aim, rv);
    if (!shot) { this.stored.unshift(i); return; }
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

  turretsOnTarget() { return this.turrets.every((t) => t.err < 4 * DEG); }
}
