// The swerve drivetrain as a rigid body: 61 kg and 6.2 kg·m² pushed around by four steered
// modules. Each module turns at a real steering rate, its drive motor follows the Kraken X60 torque
// curve (current limit at low speed, back-EMF near top speed), and each tread holds the carpet only
// up to its share of the friction circle — push too hard and it slides. Walls and field elements
// push back with impulses at the contact point (so a clipped corner spins the robot), the floor's
// slope pulls on it over the BUMPs, and the chassis rocks a little as weight shifts.
import type { FieldGrid } from './field';
import { BUMPER, DRIVE, INERTIA, MASS, MODULES, MU, MU_SLIDE } from './geometry';

const G = 9.81;
const WALL_E = 0.18;          // bumper restitution against field elements
const WALL_MU = 0.3;          // bumper fabric sliding along a wall

/** What the robot is asked to do: chassis velocity in the field frame (m/s) and turn rate (rad/s). */
export interface ChassisCommand { vx: number; vz: number; w: number; lock?: boolean }

export class SwerveDrive {
  x = 0; z = 0; th = 0;
  vx = 0; vz = 0; w = 0;
  /** chassis acceleration in the robot frame (smoothed, for the hopper and the chassis tilt) */
  ax = 0; az = 0; alpha = 0;
  /** module steering (rad, robot frame: 0 = rolling toward the front) and wheel surface speed */
  readonly steer = new Float64Array(4);
  readonly wheel = new Float64Array(4);
  /** strongest wall hit in the last step (N·s), and whether any tread is sliding */
  impact = 0;
  slipping = false;
  /** drive current (0..1) for the motor sound */
  effort = 0;
  /** the front of the footprint: bumpers, or the deployed intake (half-width, reach) */
  front = { hx: BUMPER.hx, z: BUMPER.hz };
  /** body tilt from weight transfer (rad) and its rate */
  tiltP = 0; tiltR = 0; private tiltPv = 0; private tiltRv = 0;
  private samples: [number, number][] = [];
  private sampleKey = '';

  constructor(private field: FieldGrid) {}

  get speed() { return Math.hypot(this.vx, this.vz); }

  place(x: number, z: number, th: number) {
    this.x = x; this.z = z; this.th = th;
    this.vx = this.vz = this.w = 0;
    this.ax = this.az = this.alpha = 0;
    this.tiltP = this.tiltR = this.tiltPv = this.tiltRv = 0;
    this.wheel.fill(0);
    for (let i = 0; i < 4; i++) this.steer[i] = 0;
  }

  /** Outline of the footprint (robot frame), refreshed when the intake moves in or out. */
  outline(): [number, number][] {
    const f = this.front;
    const key = `${f.hx.toFixed(3)}|${f.z.toFixed(3)}`;
    if (key === this.sampleKey) return this.samples;
    this.sampleKey = key;
    this.samples = SwerveDrive.outlineFor(f.hx, f.z);
    return this.samples;
  }

  /** Points every 4 cm around the bumpers, plus the intake out to `reach` when it is deployed. */
  static outlineFor(fhx: number, reach: number): [number, number][] {
    const pts: [number, number][] = [];
    const edge = (ax: number, az: number, bx: number, bz: number) => {
      const n = Math.max(1, Math.ceil(Math.hypot(bx - ax, bz - az) / 0.04));
      for (let k = 0; k < n; k++) pts.push([ax + (bx - ax) * k / n, az + (bz - az) * k / n]);
    };
    const { hx, hz } = BUMPER;
    edge(-hx, -hz, hx, -hz);
    edge(hx, -hz, hx, hz);
    if (reach > hz + 1e-3) {
      edge(hx, hz, fhx, hz);
      edge(fhx, hz, fhx, reach);
      edge(fhx, reach, -fhx, reach);
      edge(-fhx, reach, -fhx, hz);
      edge(-fhx, hz, -hx, hz);
    } else edge(hx, hz, -hx, hz);
    edge(-hx, hz, -hx, -hz);
    return pts;
  }

  /** Smallest gap (m) between the footprint and any obstacle at this pose (negative = overlapping). */
  clearance(x: number, z: number, th: number, pts = this.outline()) {
    const c = Math.cos(th), s = Math.sin(th);
    let min = Infinity;
    for (const [lx, lz] of pts) {
      const d = this.field.clearance(x + lx * c + lz * s, z - lx * s + lz * c);
      if (d < min) min = d;
    }
    return min;
  }

  /**
   * Inverse kinematics: the wheel speed and steering each module wants for a chassis command.
   * A module never turns more than 90°: past that it reverses the wheel instead, like real swerve.
   */
  private targets(cmd: ChassisCommand) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const lx = cmd.vx * c - cmd.vz * s, lz = cmd.vx * s + cmd.vz * c;
    const out: { speed: number; angle: number }[] = [];
    let peak = 0;
    for (const [px, pz] of MODULES) {
      const mx = lx + cmd.w * pz, mz = lz - cmd.w * px;
      const sp = Math.hypot(mx, mz);
      peak = Math.max(peak, sp);
      out.push({ speed: sp, angle: Math.atan2(mx, mz) });
    }
    // too fast for the slowest wheel: scale everything so the robot still drives the same arc
    const k = peak > DRIVE.vmax ? DRIVE.vmax / peak : 1;
    for (const o of out) o.speed *= k;
    return out;
  }

  step(dt: number, cmd: ChassisCommand) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const t = this.targets(cmd);
    const moving = Math.hypot(cmd.vx, cmd.vz) > 0.02 || Math.abs(cmd.w) > 0.03;
    const vlx = this.vx * c - this.vz * s, vlz = this.vx * s + this.vz * c;
    const N = (MASS * G) / 4;
    let fx = 0, fz = 0, tq = 0, slip = false, effort = 0;
    for (let i = 0; i < 4; i++) {
      const [px, pz] = MODULES[i];
      // steering: point along the wanted direction, or into an X when parked (it can't be pushed)
      let want = t[i].speed > 0.03 ? t[i].angle : cmd.lock || !moving && this.speed < 0.05 ? Math.atan2(px, pz) : this.steer[i];
      let d = Math.atan2(Math.sin(want - this.steer[i]), Math.cos(want - this.steer[i]));
      let dir = 1;
      if (d > Math.PI / 2) { d -= Math.PI; dir = -1; } else if (d < -Math.PI / 2) { d += Math.PI; dir = -1; }
      want = this.steer[i] + d;
      const maxStep = DRIVE.steerRate * dt;
      this.steer[i] += Math.max(-maxStep, Math.min(maxStep, d));
      this.steer[i] = Math.atan2(Math.sin(this.steer[i]), Math.cos(this.steer[i]));
      // wheel speed it should run at: less while the module is still turning (cosine compensation)
      const err = want - this.steer[i];
      const vd = dir * t[i].speed * Math.max(0, Math.cos(err));
      const ux = Math.sin(this.steer[i]), uz = Math.cos(this.steer[i]);
      const pvx = vlx + this.w * pz, pvz = vlz - this.w * px;
      const vl = pvx * ux + pvz * uz;       // rolling speed of this wheel
      const vt = pvx * uz - pvz * ux;       // sideways scrub (n = (uz, −ux))
      // drive motor: velocity loop, limited by the stator current limit and by back-EMF near top speed
      let Fl = (MASS / 4) * (vd - vl) / 0.05;
      const driving = Fl * vl >= 0;
      const lim = driving ? Math.max(0, Math.min(DRIVE.currentForce, DRIVE.stallForce * (1 - Math.abs(vl) / DRIVE.freeSpeed))) : DRIVE.currentForce;
      Fl = Math.max(-lim, Math.min(lim, Fl));
      effort += Math.abs(Fl) / DRIVE.currentForce / 4;
      // the tread resists sliding sideways (a stiff constraint: 0.5 keeps the four wheels from
      // over-correcting a spin together), within the friction circle
      let Ft = -(MASS / 4) * vt / dt * 0.5;
      const F = Math.hypot(Fl, Ft);
      if (F > MU * N) { const k = (MU_SLIDE * N) / F; Fl *= k; Ft *= k; slip = true; }
      const Fx = Fl * ux + Ft * uz, Fz = Fl * uz - Ft * ux;
      fx += Fx; fz += Fz;
      tq += Fx * pz - Fz * px;
      this.wheel[i] = vl;
    }
    this.slipping = slip;
    this.effort = effort;
    // to the field frame, plus gravity along the floor's slope (the BUMPs)
    let awx = (fx * c + fz * s) / MASS, awz = (-fx * s + fz * c) / MASS;
    const [sx, sz] = this.field.slope(this.x, this.z);
    awx -= G * sx * 0.92; awz -= G * sz * 0.92;
    const alpha = tq / INERTIA;
    this.vx += awx * dt; this.vz += awz * dt;
    this.w += alpha * dt;
    // smoothed acceleration in the robot frame
    const k = 1 - Math.exp(-dt * 30);
    this.ax += ((awx * c - awz * s) - this.ax) * k;
    this.az += ((awx * s + awz * c) - this.az) * k;
    this.alpha += (alpha - this.alpha) * k;
    this.x += this.vx * dt; this.z += this.vz * dt;
    this.th += this.w * dt;
    this.th = Math.atan2(Math.sin(this.th), Math.cos(this.th));
    this.collide();
    this.tilt(dt);
  }

  /** Bumpers (and the deployed intake) against walls and field elements: push out, then an impulse. */
  private collide() {
    this.impact = 0;
    const pts = this.outline();
    for (let iter = 0; iter < 4; iter++) {
      const c = Math.cos(this.th), s = Math.sin(this.th);
      let worst = 0, wx = 0, wz = 0;
      for (const [lx, lz] of pts) {
        const px = this.x + lx * c + lz * s, pz = this.z - lx * s + lz * c;
        const d = this.field.clearance(px, pz);
        if (d < worst) { worst = d; wx = px; wz = pz; }
      }
      if (worst >= 0) return;
      let [nx, nz] = this.field.normal(wx, wz);
      if (!nx && !nz) { const l = Math.hypot(this.x - wx, this.z - wz) || 1; nx = (this.x - wx) / l; nz = (this.z - wz) / l; }
      // positional correction: straight out of the obstacle
      this.x += nx * (-worst + 0.001);
      this.z += nz * (-worst + 0.001);
      // velocity: bounce off the contact point (translation and spin), with bumper friction
      const rx = wx - this.x, rz = wz - this.z;
      const pvx = this.vx + this.w * rz, pvz = this.vz - this.w * rx;
      const vn = pvx * nx + pvz * nz;
      if (vn >= 0) continue;
      const rn = nx * rz - nz * rx;
      const kn = 1 / MASS + (rn * rn) / INERTIA;
      const J = -(1 + WALL_E) * vn / kn;
      const tx = -nz, tz = nx;
      const vt = pvx * tx + pvz * tz;
      const rt = tx * rz - tz * rx;
      const kt = 1 / MASS + (rt * rt) / INERTIA;
      const Jt = Math.max(-WALL_MU * J, Math.min(WALL_MU * J, -vt / kt));
      this.vx += (J * nx + Jt * tx) / MASS;
      this.vz += (J * nz + Jt * tz) / MASS;
      this.w += (J * rn + Jt * rt) / INERTIA;
      this.impact = Math.max(this.impact, J);
    }
  }

  /** Apply an impulse (N·s) at a world point, e.g. from fuel the bumpers hit. */
  push(jx: number, jz: number, wx: number, wz: number) {
    const rx = wx - this.x, rz = wz - this.z;
    this.vx += jx / MASS; this.vz += jz / MASS;
    this.w += (jx * rz - jz * rx) / INERTIA;
  }

  /** The chassis rocks on its treads as weight shifts: nose dips under braking, leans in turns. */
  private tilt(dt: number) {
    const wn = 22, zeta = 0.32, gain = 0.0028;
    const tp = -this.az * gain, tr = this.ax * gain * 0.8;
    const jolt = Math.min(0.03, this.impact * 0.0004);
    this.tiltPv += ((tp - this.tiltP) * wn * wn - 2 * zeta * wn * this.tiltPv) * dt + (this.impact ? (Math.random() - 0.5) * jolt * wn : 0);
    this.tiltRv += ((tr - this.tiltR) * wn * wn - 2 * zeta * wn * this.tiltRv) * dt;
    this.tiltP += this.tiltPv * dt;
    this.tiltR += this.tiltRv * dt;
    this.tiltP = Math.max(-0.05, Math.min(0.05, this.tiltP));
    this.tiltR = Math.max(-0.05, Math.min(0.05, this.tiltR));
  }
}
