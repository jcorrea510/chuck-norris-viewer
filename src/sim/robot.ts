// Chuck Norris on the field, piece by piece as on the real robot:
//   drivetrain  — the swerve rigid body (drive.ts)
//   intake      — a four-bar that deploys at a real speed; its full-width roller grabs any fuel it
//                 touches and carries it under the roller, up the polycarb kicker ramp and over the
//                 secondary roller into the hopper; it can't stow onto a hopper that's too full
//   hopper      — fuel tumbling in its own physics world (robot frame) on the CAD's inclined bed of
//                 rollers, sloshing as the robot accelerates and spins; the floor rollers carry it
//                 back to the indexer
//   indexer     — two lanes, one per turret, that pull fuel from the back of the hopper, stage it
//                 under each turntable and feed it up through the ring into the shooter
//   turrets     — slew to the HUB, spin their flywheels up to the speed the shot needs, and launch
//                 each ball from the hood; every shot costs the flywheel some speed
// Autopilot: pure pursuit along a planned path with a velocity profile, steering the same drivetrain
// the driver does.
import type { Balls, RobotContact } from './balls';
import { S, R, MASS as BALL_MASS } from './balls';
import type { FieldGrid } from './field';
import { SwerveDrive, type ChassisCommand } from './drive';
import { solveShot } from './shot';
import {
  BARS, BODY, BUMPER, DEG, DRIVE, FEED_FIXED, FEED_TURRET, FLYWHEEL, FRONT_WALL, HOPPER, INTAKE_PATH,
  INTAKE_UP, MODULES, ROLLER, TURRETS, capacityAt, couplerOffset, crankPoint,
} from './geometry';

export const VMAX = DRIVE.vmax;
const G = 9.81;
const AUTO_ACCEL = 4.2;            // m/s² the autopilot plans with (the drivetrain can do more)
const INTAKE_SPEED = 2.6;          // m/s a ball travels along the intake path
const LANE_SPEED = 2.2;            // m/s indexer belts
const SHOOTER_SPEED = 7;           // m/s through the flywheel and hood
const SPACING = 2 * R + 0.006;     // fuel touching in a lane
const ARM_VMAX = 230, ARM_ACCEL = 1700; // deg/s, deg/s² four-bar
const TURRET_W = 10, TURRET_A = 60;
const SHOT_LOSS = 1.6;             // flywheel energy per shot ÷ the ball's kinetic energy

export interface DriveOrder {
  path: [number, number][];
  speed: number;
  heading: 'travel' | number;
  stopAtEnd: boolean;
  /** Per-segment heading override (swerve: line up with a trench or bump lane before entering it). */
  segHeading?: (number | null)[];
}

interface HopBall { i: number; x: number; y: number; z: number; vx: number; vy: number; vz: number }
interface Intaking { i: number; x: number; s: number; path: [number, number][]; len: number; back?: boolean }
interface LaneBall { i: number; s: number; from: [number, number, number]; firing: boolean }
interface Turret {
  id: string; side: number; pivot: [number, number, number]; base: number;
  a: number; w: number; err: number;
  fly: number; flyTarget: number; shot: { v: [number, number, number]; speed: number } | null; solveT: number;
  lane: LaneBall[]; range: boolean;
}

const pathLength = (p: [number, number][]) => { let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); return L; };

export class RobotSim implements RobotContact {
  readonly drive: SwerveDrive;
  /** chassis height and attitude from the floor under the wheels (bumps), smoothed like suspension */
  py = 0; pitch = 0; roll = 0; private pyV = 0;
  // intake
  intakeDown = true;
  intakeAngle = 0;
  private armV = 0;
  /** the intake can't stow: the fuel on board wouldn't fit */
  capacityBlocked = false;
  rollers: 'in' | 'off' | 'out' = 'in';
  intaking: Intaking[] = [];
  hop: HopBall[] = [];
  private ejectClock = 0;
  // shooting
  shoot = false;          // fire whenever a turret is ready
  spinUp = false;         // keep the flywheels at shooting speed
  turrets: Turret[];
  shotsFired = 0;
  /** seconds since either turret last fired */
  lastShotT = 0;
  outOfRange = false;
  onShot: ((i: number, side: number) => void) | null = null;
  onIntake: ((i: number) => void) | null = null;
  // autopilot
  order: DriveOrder | null = null;
  private seg = 0;
  done = true;
  /** last command sent to the drivetrain (for the steering animation and sounds) */
  cmd: ChassisCommand = { vx: 0, vz: 0, w: 0 };
  /** a driver's command replaces the autopilot's while set */
  manual: ChassisCommand | null = null;

  constructor(private field: FieldGrid, private balls: Balls, private target: number[]) {
    this.drive = new SwerveDrive(field);
    this.turrets = TURRETS.map((t) => ({
      id: t.id, side: Math.sign(t.pivot[0]), pivot: t.pivot, base: t.base, a: 0, w: 0, err: 1,
      fly: 0, flyTarget: 0, shot: null, solveT: 0, lane: [], range: true,
    }));
  }

  get x() { return this.drive.x; }
  get z() { return this.drive.z; }
  get th() { return this.drive.th; }
  get vx() { return this.drive.vx; }
  get vz() { return this.drive.vz; }
  get w() { return this.drive.w; }
  get speed() { return this.drive.speed; }
  get capacity() { return capacityAt(this.intakeAngle); }
  /** fuel on board: hopper, indexer lanes and anything on its way in */
  get load() { return this.hop.length + this.intaking.length + this.turrets[0].lane.length + this.turrets[1].lane.length; }
  get stored() { return this.load - this.intaking.length; }
  get shooting() { return this.shoot && this.stored > 0; }
  get flywheelReady() { return this.turrets.every((t) => t.flyTarget > 0 && Math.abs(t.fly - t.flyTarget) < t.flyTarget * 0.04); }

  place(x: number, z: number, th: number) {
    this.drive.place(x, z, th);
    this.order = null; this.done = true; this.manual = null;
    for (const t of [...this.hop, ...this.intaking, ...this.turrets.flatMap((t) => t.lane)]) if (this.balls.st[t.i] === S.STORED || this.balls.st[t.i] === S.INTAKING) this.balls.st[t.i] = S.PARKED;
    this.hop = []; this.intaking = [];
    for (const t of this.turrets) { t.lane = []; t.a = 0; t.w = 0; t.fly = 0; t.flyTarget = 0; t.shot = null; }
    const tr = this.terrain();
    this.py = tr.y; this.pitch = tr.pitch; this.roll = tr.roll; this.pyV = 0;
    this.updateFront();
  }

  /** Snap the intake to a position (used when placing the robot). */
  setIntakeNow(down: boolean) {
    this.intakeDown = down;
    this.intakeAngle = down ? 0 : INTAKE_UP;
    this.armV = 0;
    this.updateFront();
  }

  /** Preloaded fuel, settled in the hopper. */
  preload(indices: number[]) {
    indices.forEach((i, k) => {
      this.balls.st[i] = S.STORED;
      this.hop.push({ i, x: -0.22 + (k % 4) * 0.147 + (Math.random() - 0.5) * 0.01, y: 0.3 + Math.floor(k / 8) * 0.15, z: 0.08 + (Math.floor(k / 4) % 2) * 0.15, vx: 0, vy: 0, vz: 0 });
    });
  }

  /** Follow a planned path (null stops). */
  navigate(order: DriveOrder | null) {
    this.order = order;
    this.seg = 0;
    this.done = !order;
    this.blocked = false;
    this.bestLeft = Infinity; this.bestT = 0;
  }
  /** the last order ended because something was in the way (the robot stopped gaining ground) */
  blocked = false;
  private bestLeft = Infinity;
  private bestT = 0;

  /** Footprint gap at a pose, with the intake deployed or not (for planning). */
  clearance(x: number, z: number, th: number) { return this.drive.clearance(x, z, th); }

  // ------------------------------------------------------------------ step
  update(dt: number) {
    let cmd: ChassisCommand;
    if (this.manual) cmd = this.manual;
    else if (this.order && !this.done) {
      const f = this.follow();
      let e = f.heading - this.th;
      e = Math.atan2(Math.sin(e), Math.cos(e));
      // heading loop with a feedforward from the remaining error, gentler as it closes
      const w = Math.max(-7, Math.min(7, e * 6 - this.w * 0.15));
      cmd = { vx: f.vx, vz: f.vz, w };
    } else cmd = { vx: 0, vz: 0, w: 0 };
    this.cmd = cmd;
    this.drive.step(dt, cmd);
    this.updateArm(dt);
    this.updatePose(dt);
    this.updateIntake(dt);
    this.updateHopper(dt);
    this.updateTurrets(dt);
    this.lastShotT += dt;
  }

  // ------------------------------------------------------------------ intake arm
  private updateArm(dt: number) {
    const goal = this.intakeDown ? 0 : INTAKE_UP;
    const d = goal - this.intakeAngle;
    // trapezoidal motion: accelerate, cruise, brake into the hard stop
    const want = Math.sign(d) * Math.min(ARM_VMAX, Math.sqrt(2 * ARM_ACCEL * Math.abs(d)));
    this.armV += Math.max(-ARM_ACCEL * dt, Math.min(ARM_ACCEL * dt, want - this.armV));
    let next = this.intakeAngle + this.armV * dt;
    // raising squeezes the hopper: it stalls if the fuel on board no longer fits
    this.capacityBlocked = next < this.intakeAngle && capacityAt(next) < this.load;
    if (this.capacityBlocked) { next = this.intakeAngle; this.armV = 0; }
    if ((goal - next) * d <= 0) { next = goal; this.armV = Math.abs(this.armV) > 120 ? -this.armV * 0.12 : 0; }
    this.intakeAngle = Math.max(INTAKE_UP, Math.min(0, next));
    this.updateFront();
  }

  private updateFront() {
    const [dz] = couplerOffset(this.intakeAngle);
    const reach = FRONT_WALL.z + 0.005 + dz;
    this.drive.front = reach > BUMPER.hz ? { hx: HOPPER.hx, z: reach } : { hx: BUMPER.hx, z: BUMPER.hz };
  }

  /** Floor under the four modules → ride height, pitch and roll. */
  private terrain() {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const h = MODULES.map(([mx, mz]) => this.field.floor(this.x + mx * c + mz * s, this.z - mx * s + mz * c));
    const front = (h[0] + h[1]) / 2, back = (h[2] + h[3]) / 2, left = (h[0] + h[2]) / 2, right = (h[1] + h[3]) / 2;
    return { y: Math.max(...h) * 0.5 + (h[0] + h[1] + h[2] + h[3]) / 8, pitch: -Math.atan2(front - back, 0.552), roll: Math.atan2(left - right, 0.578) };
  }

  private updatePose(dt: number) {
    const t = this.terrain();
    // ride height on stiff, well-damped "suspension" (tread + frame flex): a small settle after the BUMP
    this.pyV += ((t.y - this.py) * 900 - this.pyV * 42) * dt;
    this.py += this.pyV * dt;
    if (this.py < t.y - 0.01) { this.py = t.y - 0.01; this.pyV = Math.max(0, this.pyV); }
    const k = 1 - Math.exp(-dt * 16);
    this.pitch += (t.pitch - this.pitch) * k;
    this.roll += (t.roll - this.roll) * k;
  }

  // ------------------------------------------------------------------ frames
  /** World → robot frame (flat: yaw and ride height only). */
  private toLocal(wx: number, wy: number, wz: number) {
    const dx = wx - this.x, dz = wz - this.z;
    const c = Math.cos(this.th), s = Math.sin(this.th);
    return { lx: dx * c - dz * s, ly: wy - this.py, lz: dx * s + dz * c };
  }
  /** Robot frame → world. */
  toWorld(lx: number, ly: number, lz: number, out: number[] = [0, 0, 0]) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    out[0] = this.x + lx * c + lz * s; out[1] = this.py + ly; out[2] = this.z - lx * s + lz * c;
    return out;
  }
  /** Velocity of a robot-frame point, in the world frame. */
  private pointVel(lx: number, lz: number): [number, number] {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const rx = lx * c + lz * s, rz = -lx * s + lz * c;
    return [this.vx + this.w * rz, this.vz - this.w * rx];
  }

  // ------------------------------------------------------------------ contact with fuel on the field
  contact(i: number, balls: Balls) {
    const o = i * 3;
    const { lx, ly, lz } = this.toLocal(balls.pos[o], balls.pos[o + 1], balls.pos[o + 2]);
    if (ly > BODY.top + 0.25 || ly < -0.2) return;
    const [dz, dy] = couplerOffset(this.intakeAngle);
    const deployed = this.drive.front.z > BUMPER.hz;

    // 1. the intake roller grabs whatever it touches
    if (deployed && this.rollers === 'in' && this.intakeAngle > -10 && Math.abs(lx) < ROLLER.hx - 0.01 && ly < 0.3) {
      const rz = ROLLER.z + dz, ry = ROLLER.y + dy;
      if (Math.hypot(lz - rz, ly - ry) < ROLLER.r + R + 0.03 && lz > rz - 0.03 && this.load < this.capacity && this.laneFree(lx)) {
        this.startIntake(i, lx, lz - dz, ly - dy);
        return;
      }
    }

    // 2. fuel falling onto the robot: into the open hopper, or off the top
    const reach = Math.max(BODY.z1, this.drive.front.z);
    if (ly > BUMPER.y1 + R - 0.01) {
      if (Math.abs(lx) > BODY.hx + R || lz < BODY.z0 - R || lz > reach + R || ly > BODY.top + R) return;
      if (balls.st[i] === S.FLYING && balls.vel[o + 1] < 0 && Math.abs(lx) < HOPPER.hx - 0.02 && lz > HOPPER.back + 0.02 && lz < reach - 0.04 && ly > BODY.top - 0.08 && this.load < this.capacity) {
        // dropped into the hopper
        const v = this.velToLocal(balls.vel[o], balls.vel[o + 1], balls.vel[o + 2]);
        balls.st[i] = S.STORED;
        this.hop.push({ i, x: lx, y: Math.min(ly, HOPPER.top - R), z: lz, vx: v[0] * 0.5, vy: v[1] * 0.5, vz: v[2] * 0.5 });
        return;
      }
      // off the top (or the side of the superstructure)
      const out = [ly - (BODY.top + R), Math.abs(lx) - (BODY.hx + R), lz - (reach + R), (BODY.z0 - R) - lz];
      let k = 0;
      for (let q = 1; q < 4; q++) if (out[q] > out[k]) k = q;
      const nl: [number, number, number] = k === 0 ? [0, 1, 0] : k === 1 ? [Math.sign(lx) || 1, 0, 0] : k === 2 ? [0, 0, 1] : [0, 0, -1];
      this.bounce(i, balls, nl, -out[k], lx, lz, 0.35);
      return;
    }

    // 3. bumpers and the intake frame push fuel at floor level
    let best = Infinity, nl: [number, number, number] = [0, 0, 0];
    const test = (hx: number, z0: number, z1: number) => {
      const ex = Math.abs(lx) - (hx + R), e0 = (z0 - R) - lz, e1 = lz - (z1 + R);
      if (ex >= 0 || e0 >= 0 || e1 >= 0) return;
      // shallowest way out
      const opts: [number, [number, number, number]][] = [[-ex, [Math.sign(lx) || 1, 0, 0]], [-e0, [0, 0, -1]], [-e1, [0, 0, 1]]];
      for (const [d, n] of opts) if (d < best) { best = d; nl = n; }
    };
    test(BUMPER.hx, -BUMPER.hz, BUMPER.hz);
    if (deployed) {
      // between the bumper and the roller sits the kicker ramp: the intake is solid at fuel height,
      // up to just behind where the roller grabs
      test(HOPPER.hx, BUMPER.hz - 0.05, ROLLER.z + ROLLER.r + dz - 0.012);
    }
    if (best < Infinity) this.bounce(i, balls, nl, best, lx, lz, 0.3);
  }

  private velToLocal(vx: number, vy: number, vz: number): [number, number, number] {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    return [vx * c - vz * s, vy, vx * s + vz * c];
  }

  /**
   * Push a ball out along a robot-frame normal and trade momentum with the robot (a 0.2 kg ball
   * barely slows a 61 kg robot, but a pile of them does).
   */
  private bounce(i: number, balls: Balls, nl: [number, number, number], depth: number, lx: number, lz: number, e: number) {
    const o = i * 3;
    const c = Math.cos(this.th), s = Math.sin(this.th);
    const nx = nl[0] * c + nl[2] * s, ny = nl[1], nz = -nl[0] * s + nl[2] * c;
    balls.pos[o] += nx * (depth + 0.001);
    balls.pos[o + 1] += ny * (depth + 0.001);
    balls.pos[o + 2] += nz * (depth + 0.001);
    // never shove fuel through a wall or field element: slide it along instead
    if (balls.pos[o + 1] < 0.5) {
      const d = this.field.clearance(balls.pos[o], balls.pos[o + 2]);
      if (d < R) {
        const [ax, az] = this.field.normal(balls.pos[o], balls.pos[o + 2]);
        balls.pos[o] += ax * (R - d); balls.pos[o + 2] += az * (R - d);
      }
    }
    balls.pos[o] = Math.max(-8.27 + R, Math.min(8.27 - R, balls.pos[o]));
    balls.pos[o + 2] = Math.max(-4.03 + R, Math.min(4.03 - R, balls.pos[o + 2]));
    const [pvx, pvz] = this.pointVel(lx, lz);
    const rvx = balls.vel[o] - pvx, rvy = balls.vel[o + 1], rvz = balls.vel[o + 2] - pvz;
    const vn = rvx * nx + rvy * ny + rvz * nz;
    if (vn < 0) {
      const J = -(1 + e) * vn * BALL_MASS; // the robot is ~300× heavier
      balls.vel[o] += (J / BALL_MASS) * nx; balls.vel[o + 1] += (J / BALL_MASS) * ny; balls.vel[o + 2] += (J / BALL_MASS) * nz;
      // bumper fabric drags the ball along a little
      const tvx = rvx - vn * nx, tvz = rvz - vn * nz;
      balls.vel[o] -= tvx * 0.25; balls.vel[o + 2] -= tvz * 0.25;
      this.drive.push(-J * nx, -J * nz, balls.pos[o], balls.pos[o + 2]);
    }
    if (balls.st[i] === S.REST) balls.wake(i);
  }

  // ------------------------------------------------------------------ intake path
  /** No other ball is still in the first stretch of the path at this spot across the roller. */
  private laneFree(x: number) {
    for (const it of this.intaking) if (Math.abs(it.x - x) < 2 * R - 0.01 && it.s < SPACING + 0.01 + (it.back ? 0.15 : 0)) return false;
    return true;
  }

  private startIntake(i: number, x: number, z: number, y: number) {
    this.balls.st[i] = S.INTAKING;
    const path: [number, number][] = [[z, y], ...INTAKE_PATH];
    this.intaking.push({ i, x: Math.max(-0.25, Math.min(0.25, x)), s: 0, path, len: pathLength(path) });
    this.onIntake?.(i);
  }

  private updateIntake(dt: number) {
    const [dz, dy] = couplerOffset(this.intakeAngle);
    for (let k = this.intaking.length - 1; k >= 0; k--) {
      const it = this.intaking[k];
      if (it.back) {
        // ejecting: back out over the roller and onto the floor in front
        it.s -= INTAKE_SPEED * 1.1 * dt;
        if (it.s <= 0) {
          this.intaking.splice(k, 1);
          const [z, y] = it.path[0];
          const p = this.toWorld(it.x, Math.max(R, y + dy), z + dz + 0.03);
          const fwd = this.velToWorld(0, 0, 2.4);
          this.balls.launch(it.i, p, [fwd[0] + this.vx, 0.3, fwd[2] + this.vz]);
        }
        continue;
      }
      // rollers stopped: the ball waits where it is; reversed: it comes back out
      if (this.rollers === 'off') continue;
      if (this.rollers === 'out') { it.back = true; continue; }
      it.s += INTAKE_SPEED * dt;
      if (it.s >= it.len) {
        this.intaking.splice(k, 1);
        this.balls.st[it.i] = S.STORED;
        const [z, y] = it.path[it.path.length - 1];
        // it leaves the secondary roller moving backward and drops onto the pile
        this.hop.push({ i: it.i, x: it.x, y: y + dy, z: z + dz, vx: (Math.random() - 0.5) * 0.3, vy: 0.2, vz: -1.3 - Math.random() * 0.4 });
      }
    }
    // ejecting: fuel at the front of the hopper goes back out over the rollers
    this.ejectClock -= dt;
    if (this.rollers === 'out' && this.intakeAngle > -20 && this.ejectClock <= 0 && this.hop.length) {
      let best = -1, bz = -Infinity;
      for (let k = 0; k < this.hop.length; k++) { const h = this.hop[k]; const sc = h.z - h.y * 0.5; if (sc > bz) { bz = sc; best = k; } }
      if (best >= 0 && this.hop[best].z > 0.2 + dz) {
        const h = this.hop.splice(best, 1)[0];
        this.balls.st[h.i] = S.INTAKING;
        const path: [number, number][] = [[ROLLER.z + ROLLER.r + R, R], ...INTAKE_PATH];
        const len = pathLength(path);
        this.intaking.push({ i: h.i, x: Math.max(-0.25, Math.min(0.25, h.x)), s: len, path, len, back: true });
        this.ejectClock = 0.11;
      }
    }
  }

  private velToWorld(lx: number, ly: number, lz: number) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    return [lx * c + lz * s, ly, -lx * s + lz * c];
  }

  // ------------------------------------------------------------------ hopper
  /**
   * The hopper is its own small physics world in the robot frame: gravity, the robot's own
   * acceleration and spin (so the fuel sloshes forward when it brakes and outward in a spin),
   * ball-to-ball contacts, the walls, and the CAD's rollers and bars as cylinders across the robot,
   * moving with the four-bar. The floor rollers carry fuel back to the indexer while it feeds.
   */
  private updateHopper(dt: number) {
    const hop = this.hop;
    // indexer: pull fuel from the back of the hopper into a lane with room
    this.feedLanes();
    if (!hop.length) return;
    const d = this.drive;
    const [cdz, cdy] = couplerOffset(this.intakeAngle);
    // obstacles for this pose
    const bars = BARS.map((b) => {
      if (b.link === 'fixed') return [b.z, b.y, b.r];
      if (b.link === 'crank') { const [z, y] = crankPoint(b.z, b.y, this.intakeAngle); return [z, y, b.r]; }
      return [b.z + cdz, b.y + cdy, b.r];
    });
    const wallZ = FRONT_WALL.z + cdz, wallY0 = FRONT_WALL.y0 + cdy;
    const feeding = this.turrets.some((t) => t.lane.length < 2) || this.shoot;
    const conveyor = this.rollers === 'out' ? 1.1 : feeding ? -1.0 : 0;
    const gy = -G * Math.cos(this.pitch) * Math.cos(this.roll);
    const gxT = -G * Math.sin(this.roll), gzT = G * Math.sin(this.pitch);
    const damp = Math.exp(-1.6 * dt);
    for (const b of hop) {
      // pseudo-forces: the chassis' acceleration, the centrifugal pull of a spin, and the angular acceleration
      const ax = -d.ax + d.w * d.w * b.x - d.alpha * b.z;
      const az = -d.az + d.w * d.w * b.z + d.alpha * b.x;
      b.vx += (ax + gxT) * dt; b.vy += gy * dt; b.vz += (az + gzT) * dt;
      // powered floor and mecanum side rollers keep low fuel moving toward the indexer (or out, ejecting)
      if (conveyor && b.y < 0.45 + Math.max(0, cdy)) { b.vz += Math.sign(conveyor) * 0.9 * dt; if (conveyor < 0 && b.z < 0.12) b.vx += -Math.sign(b.x) * 0.25 * dt; }
      b.vx *= damp; b.vz *= damp;
      b.x += b.vx * dt; b.y += b.vy * dt; b.z += b.vz * dt;
    }
    const D = 2 * R;
    const moving: number[][] = [];
    const tmp = [0, 0, 0];
    for (const t of this.turrets) for (const lb of t.lane) if (lb.s < 0.12) { this.feedPoint(t, lb, tmp); moving.push([...tmp]); }
    for (const it of this.intaking) if (it.s > 0.12) moving.push(this.intakePoint(it, cdz, cdy));
    for (let pass = 0; pass < 3; pass++) {
      for (let a = 0; a < hop.length; a++) {
        const A = hop[a];
        for (let q = a + 1; q < hop.length; q++) {
          const B = hop[q];
          const dx = B.x - A.x, dy = B.y - A.y, dz = B.z - A.z;
          const d2 = dx * dx + dy * dy + dz * dz;
          if (d2 >= D * D || d2 < 1e-10) continue;
          const dd = Math.sqrt(d2), pen = (D - dd) / 2;
          const nx = dx / dd, ny = dy / dd, nz = dz / dd;
          A.x -= nx * pen; A.y -= ny * pen; A.z -= nz * pen;
          B.x += nx * pen; B.y += ny * pen; B.z += nz * pen;
          const rv = (B.vx - A.vx) * nx + (B.vy - A.vy) * ny + (B.vz - A.vz) * nz;
          if (rv < 0) {
            const j = -1.2 * rv / 2;
            A.vx -= j * nx; A.vy -= j * ny; A.vz -= j * nz;
            B.vx += j * nx; B.vy += j * ny; B.vz += j * nz;
          }
        }
      }
      // fuel on its way into the indexer, or just coming over the secondary roller, shoves the rest aside
      for (const [kx, ky, kz] of moving) for (const b of hop) {
        const dx = b.x - kx, dy = b.y - ky, dz = b.z - kz;
        const d2 = dx * dx + dy * dy + dz * dz;
        if (d2 >= D * D || d2 < 1e-10) continue;
        const dd = Math.sqrt(d2), pen = D - dd;
        b.x += dx / dd * pen; b.y += dy / dd * pen; b.z += dz / dd * pen;
      }
      for (const b of hop) {
        // rollers and bars (cylinders across the robot)
        for (const [bz, by, br] of bars) {
          const ez = b.z - bz, ey = b.y - by, rr = br + R;
          const e2 = ez * ez + ey * ey;
          if (e2 >= rr * rr || e2 < 1e-10) continue;
          const e = Math.sqrt(e2), nz = ez / e, ny = ey / e;
          b.z += nz * (rr - e); b.y += ny * (rr - e);
          const vn = b.vz * nz + b.vy * ny;
          if (vn < 0) { b.vz -= vn * nz * 1.15; b.vy -= vn * ny * 1.15; }
          // resting on a floor roller: it drives the fuel back toward the indexer (or out, ejecting)
          if (ny > 0.4 && conveyor) b.vz += (conveyor - b.vz) * 0.12;
        }
        // the expanded hopper's floor over the intake (rides the coupler): fuel in the hopper never
        // drops into the intake's throat below it
        if (b.z > 0.45 + cdz && b.y < 0.36 + cdy) { b.y = 0.36 + cdy; if (b.vy < 0) b.vy = -b.vy * 0.15; }
        // front wall (rides the coupler)
        // (fuel heaped above it stays in too: the side plates and the pile hold it)
        if (b.y > wallY0 - R * 0.5 && b.z > wallZ - R) { b.z = wallZ - R; if (b.vz > 0) b.vz = -b.vz * 0.2; }
        if (b.y < HOPPER.floor + R) {
          b.y = HOPPER.floor + R;
          if (b.vy < 0) b.vy = -b.vy * 0.15;
          b.vx *= 0.97;
          if (conveyor) b.vz += (conveyor - b.vz) * 0.12; else b.vz *= 0.97;
        }
        if (b.y > HOPPER.top - R) { b.y = HOPPER.top - R; if (b.vy > 0) b.vy = 0; }
        if (b.x < -HOPPER.hx + R) { b.x = -HOPPER.hx + R; if (b.vx < 0) b.vx = -b.vx * 0.2; }
        if (b.x > HOPPER.hx - R) { b.x = HOPPER.hx - R; if (b.vx > 0) b.vx = -b.vx * 0.2; }
        if (b.z < HOPPER.back + R) { b.z = HOPPER.back + R; if (b.vz < 0) b.vz = -b.vz * 0.2; }
      }
    }
    // nothing ever leaves the hopper by tunnelling: clamp the last few millimetres
    for (const b of hop) {
      if (!Number.isFinite(b.x + b.y + b.z)) { b.x = 0; b.y = 0.35; b.z = 0.1; b.vx = b.vy = b.vz = 0; }
      b.z = Math.min(b.z, Math.max(HOPPER.back + R, wallZ - R));
    }
  }

  // ------------------------------------------------------------------ indexer
  /** Indexer: each lane pulls fuel from the back of the hopper while it has room (up to three). */
  private feedLanes() {
    // the lane with fewer balls gets the next one, so both turrets stay fed
    const order = this.turrets[0].lane.length <= this.turrets[1].lane.length ? this.turrets : [this.turrets[1], this.turrets[0]];
    for (const t of order) {
      if (t.lane.length >= 3) continue;
      const mouth = FEED_FIXED(t.side)[0];
      // fuel at the back of the hopper, nearest this lane's mouth
      // from this lane's half of the hopper (fuel from the other half would cross the other lane)
      let best = -1, bd = Infinity;
      for (let k = 0; k < this.hop.length; k++) {
        const h = this.hop[k];
        if (h.z > HOPPER.back + R + 0.07 || h.y > 0.36) continue;
        if (h.x * t.side < -0.04) continue;
        const dd = Math.hypot(h.x - mouth[0], h.y - mouth[1], h.z - mouth[2]);
        if (dd < bd) { bd = dd; best = k; }
      }
      if (best < 0) continue;
      const h = this.hop[best];
      const from: [number, number, number] = [h.x, h.y, h.z];
      // room behind the last ball in the lane?
      const last = t.lane[t.lane.length - 1];
      if (last && last.s + bd < SPACING) continue;
      this.hop.splice(best, 1);
      t.lane.push({ i: h.i, s: -bd, from, firing: false });
    }
  }

  private static fixedLen = (() => { const p = FEED_FIXED(1); let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1], p[k][2] - p[k - 1][2]); return L; })();
  private static turretLen = (() => {
    const p = FEED_TURRET; let L = Math.hypot(p[0][0], p[0][1] - (0.34 - 0.321));
    for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]);
    return L;
  })();

  /** Robot-frame position of a ball at distance `s` along a turret's feed. */
  feedPoint(t: Turret, b: LaneBall, out: number[]) {
    const fixed = FEED_FIXED(t.side);
    if (b.s < 0) {
      const L0 = Math.hypot(b.from[0] - fixed[0][0], b.from[1] - fixed[0][1], b.from[2] - fixed[0][2]) || 1;
      const f = 1 + b.s / L0;
      for (let k = 0; k < 3; k++) out[k] = b.from[k] + (fixed[0][k] - b.from[k]) * Math.max(0, Math.min(1, f));
      return out;
    }
    let s = b.s;
    for (let k = 1; k < fixed.length; k++) {
      const a = fixed[k - 1], c = fixed[k];
      const l = Math.hypot(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
      if (s <= l) { const f = s / l; for (let q = 0; q < 3; q++) out[q] = a[q] + (c[q] - a[q]) * f; return out; }
      s -= l;
    }
    // inside the turret: the path turns with it
    const yaw = t.base + t.a, fx = Math.sin(yaw), fz = Math.cos(yaw);
    const pts: [number, number][] = [[0, 0.34 - t.pivot[1]], ...FEED_TURRET];
    for (let k = 1; k < pts.length; k++) {
      const a = pts[k - 1], c = pts[k];
      const l = Math.hypot(c[0] - a[0], c[1] - a[1]);
      if (s <= l || k === pts.length - 1) {
        const f = Math.min(1, s / l);
        const fw = a[0] + (c[0] - a[0]) * f, up = a[1] + (c[1] - a[1]) * f;
        out[0] = t.pivot[0] + fx * fw; out[1] = t.pivot[1] + up; out[2] = t.pivot[2] + fz * fw;
        return out;
      }
      s -= l;
    }
    return out;
  }

  // ------------------------------------------------------------------ turrets and flywheels
  private updateTurrets(dt: number) {
    const yawR = this.th;
    let outOfRange = false;
    for (const t of this.turrets) {
      const pw = this.toWorld(t.pivot[0], t.pivot[1], t.pivot[2]);
      const want = Math.atan2(this.target[0] - pw[0], this.target[2] - pw[2]);
      // turret servo: ±180° travel (no wrap through the wiring), rate- and acceleration-limited;
      // it leads the robot's own rotation
      let goal = want - yawR - t.base - this.w * 0.03;
      goal = Math.atan2(Math.sin(goal), Math.cos(goal));
      const e = goal - t.a;
      const wGoal = Math.max(-TURRET_W, Math.min(TURRET_W, e * 14));
      t.w += Math.max(-TURRET_A * dt, Math.min(TURRET_A * dt, wGoal - t.w));
      t.a = Math.max(-Math.PI, Math.min(Math.PI, t.a + t.w * dt));
      const actual = yawR + t.base + t.a;
      t.err = Math.abs(Math.atan2(Math.sin(want - actual), Math.cos(want - actual)));
      // the shot this turret would take from here (re-solved ten times a second)
      t.solveT -= dt;
      const exit = this.exitWorld(t);
      if (t.solveT <= 0 && (this.stored > 0 || this.spinUp)) {
        t.solveT = 0.1;
        const [rvx, rvz] = this.pointVel(t.pivot[0], t.pivot[2]);
        const s = solveShot(exit, this.target, [rvx, rvz]);
        t.shot = s ? { v: s.v, speed: s.speed } : null;
        t.range = !!s;
      }
      if (!t.range && this.shoot) outOfRange = true;
      // flywheel: spin to the speed this shot needs while shooting (or asked to be ready), idle otherwise
      const need = t.shot ? t.shot.speed / (FLYWHEEL.slip * FLYWHEEL.radius) : 420;
      t.flyTarget = this.shoot || this.spinUp ? need : this.stored > 0 ? need * 0.55 : 0;
      const tmax = Math.min(FLYWHEEL.torque, Math.max(0, FLYWHEEL.stall * (1 - t.fly / FLYWHEEL.free)));
      const tq = t.fly < t.flyTarget ? Math.min(tmax, (t.flyTarget - t.fly) * 0.02) : -Math.min(0.12, (t.fly - t.flyTarget) * 0.004) - 0.01;
      t.fly = Math.max(0, t.fly + (tq / FLYWHEEL.inertia) * dt);
      // indexer lane: stage up to the turret ring; feed through the shooter when firing
      const stage = RobotSim.fixedLen;
      const end = stage + RobotSim.turretLen;
      const ready = this.shoot && t.err < 2.5 * DEG && t.fly > t.flyTarget * 0.96 && t.flyTarget > 0;
      for (let k = 0; k < t.lane.length; k++) {
        const b = t.lane[k];
        const limit = k === 0 ? (b.firing ? Infinity : stage) : t.lane[k - 1].s - SPACING;
        if (k === 0 && !b.firing && b.s >= stage - 0.002 && ready) b.firing = true;
        const v = b.firing ? SHOOTER_SPEED : LANE_SPEED;
        b.s = Math.min(b.s + v * dt, b.firing ? end : limit);
      }
      const head = t.lane[0];
      if (head && head.firing && head.s >= end) {
        this.lastShotT = 0;
        t.lane.shift();
        this.launch(t, head.i, exit);
      }
    }
    this.outOfRange = outOfRange;
  }

  /** The turret's exit under the hood, in the world. */
  exitWorld(t: Turret) {
    const yaw = t.base + t.a, f = FEED_TURRET[FEED_TURRET.length - 1];
    return this.toWorld(t.pivot[0] + Math.sin(yaw) * f[0], t.pivot[1] + f[1], t.pivot[2] + Math.cos(yaw) * f[0]);
  }

  private launch(t: Turret, i: number, exit: number[]) {
    const [rvx, rvz] = this.pointVel(t.pivot[0], t.pivot[2]);
    const aim = [this.target[0] + (Math.random() - 0.5) * 0.3, this.target[1], this.target[2] + (Math.random() - 0.5) * 0.3];
    const s = solveShot(exit, aim, [rvx, rvz]);
    let v: number[];
    if (s) v = s.v;
    else {
      // out of range: the longest shot the shooter has (it falls short: a pass toward the alliance zone)
      const yaw = this.th + t.base + t.a, sp = 13, pitch = 46 * DEG;
      v = [Math.sin(yaw) * Math.cos(pitch) * sp + rvx, Math.sin(pitch) * sp, Math.cos(yaw) * Math.cos(pitch) * sp + rvz];
    }
    // the flywheel's actual speed sets the ball's speed; plus hood and turret scatter
    const need = s ? s.speed / (FLYWHEEL.slip * FLYWHEEL.radius) : t.flyTarget;
    const ratio = Math.max(0.85, Math.min(1.08, need > 0 ? t.fly / need : 1)) * (1 + (Math.random() - 0.5) * 0.025);
    const yawErr = (Math.random() - 0.5) * 2 * DEG;
    const lx = v[0] - rvx, lz = v[2] - rvz;
    const c = Math.cos(yawErr), sn = Math.sin(yawErr);
    this.balls.launch(i, exit, [(lx * c + lz * sn) * ratio + rvx, v[1] * ratio, (-lx * sn + lz * c) * ratio + rvz]);
    // the shot takes energy out of the flywheel
    const speed = Math.hypot(lx, v[1], lz) * ratio;
    const E = 0.5 * FLYWHEEL.inertia * t.fly * t.fly - SHOT_LOSS * 0.5 * BALL_MASS * speed * speed;
    t.fly = Math.sqrt(Math.max(0, 2 * E / FLYWHEEL.inertia));
    this.shotsFired++;
    this.onShot?.(i, t.side);
  }

  /** World positions of the turret exits (for the aim lines). */
  turretExits() { return this.turrets.map((t) => this.exitWorld(t)); }
  turretsOnTarget() { return this.turrets.every((t) => t.err < 4 * DEG); }

  /** Every ball the robot is holding, in the robot frame (for drawing them where they are). */
  forEachHeld(fn: (i: number, x: number, y: number, z: number, where: string) => void) {
    const [dz, dy] = couplerOffset(this.intakeAngle);
    for (const h of this.hop) fn(h.i, h.x, h.y, h.z, 'hopper');
    for (const it of this.intaking) { const [x, y, z] = this.intakePoint(it, dz, dy); fn(it.i, x, y, z, `intake s=${it.s.toFixed(2)}/${it.len.toFixed(2)}${it.back ? ' back' : ''}`); }
    const out = [0, 0, 0];
    for (const t of this.turrets) for (const b of t.lane) { this.feedPoint(t, b, out); fn(b.i, out[0], out[1], out[2], `lane${t.side} s=${b.s.toFixed(2)}${b.firing ? ' firing' : ''}`); }
  }

  /** Robot-frame position along the intake path (it rides the coupler). */
  private intakePoint(it: Intaking, dz: number, dy: number): number[] {
    let s = Math.max(0, Math.min(it.len, it.s));
    const p = it.path;
    let z = p[0][0], y = p[0][1];
    for (let k = 1; k < p.length; k++) {
      const l = Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]);
      if (s <= l) { const f = s / (l || 1); z = p[k - 1][0] + (p[k][0] - p[k - 1][0]) * f; y = p[k - 1][1] + (p[k][1] - p[k - 1][1]) * f; break; }
      s -= l; z = p[k][0]; y = p[k][1];
    }
    return [it.x, Math.max(R, y + dy), z + dz];
  }

  // ------------------------------------------------------------------ autopilot
  /** Pure pursuit with a velocity profile (corners, stops, bumps) and a heading command. */
  private follow(): { vx: number; vz: number; heading: number } {
    const o = this.order!;
    const pts = o.path;
    while (this.seg < pts.length - 2) {
      const [ax, az] = pts[this.seg], [bx, bz] = pts[this.seg + 1];
      const sx = bx - ax, sz = bz - az, L2 = sx * sx + sz * sz;
      const t = L2 > 0 ? ((this.x - ax) * sx + (this.z - az) * sz) / L2 : 1;
      if (t >= 1) this.seg++; else break;
    }
    const [ax, az] = pts[this.seg], [bx, bz] = pts[Math.min(this.seg + 1, pts.length - 1)];
    const sx = bx - ax, sz = bz - az, L = Math.hypot(sx, sz) || 1e-6;
    const t = Math.max(0, Math.min(1, ((this.x - ax) * sx + (this.z - az) * sz) / (L * L)));
    let remaining = (1 - t) * L;
    for (let k = this.seg + 1; k < pts.length - 1; k++) remaining += Math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]);
    // pinned against something it can't get past: give up on this order and let the brain re-plan
    if (remaining < this.bestLeft - 0.05) { this.bestLeft = remaining; this.bestT = 0; }
    else if ((this.bestT += 1 / 120) > 1.5) { this.done = true; this.blocked = true; return { vx: 0, vz: 0, heading: this.th }; }
    const sp = this.speed;
    let look = 0.35 + 0.3 * sp;
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
    let v = o.speed;
    if (o.stopAtEnd) v = Math.min(v, Math.sqrt(2 * AUTO_ACCEL * Math.max(0, remaining - 0.03)), 0.6 + remaining * 4);
    if (this.seg < pts.length - 2) {
      const [px, pz] = pts[this.seg + 1], [qx, qz] = pts[this.seg + 2];
      const ux = qx - px, uz = qz - pz, ul = Math.hypot(ux, uz) || 1;
      const turn = Math.acos(Math.max(-1, Math.min(1, (sx * ux + sz * uz) / (L * ul))));
      const vCorner = Math.max(0.9, VMAX * (1 - turn / Math.PI) ** 1.6);
      v = Math.min(v, Math.sqrt(vCorner * vCorner + 2 * AUTO_ACCEL * (1 - t) * L));
    }
    // climb bumps carefully
    if (this.field.floor(this.x, this.z) > 0.02 || this.field.floor(this.x + dx * 0.7, this.z + dz * 0.7) > 0.02) v = Math.min(v, 1.6);
    if (remaining < 0.06 && (!o.stopAtEnd || sp < 0.2)) { this.done = true; return { vx: 0, vz: 0, heading: this.th }; }
    if (!o.stopAtEnd && remaining < 0.25) this.done = true;
    const heading = o.segHeading?.[this.seg] ?? (o.heading === 'travel' ? Math.atan2(dx, dz) : o.heading);
    // in tight spots (a trench lane), finish turning before pressing on: a long robot turning
    // across a narrow lane would wedge itself
    const he = Math.abs(Math.atan2(Math.sin(heading - this.th), Math.cos(heading - this.th)));
    if (he > 0.25 && this.drive.clearance(this.x, this.z, this.th) < 0.35) v = Math.min(v, Math.max(0.35, 2.4 * (1 - (he - 0.25) / 1.2)));
    return { vx: dx * v, vz: dz * v, heading };
  }
}

