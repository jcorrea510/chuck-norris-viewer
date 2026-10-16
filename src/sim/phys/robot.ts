// Chuck Norris as a Rapier rigid body.
//   chassis   61 kg body on four swerve modules. Each module raycasts the floor (a stiff, damped
//             spring: tread + frame flex), so the robot pitches and rolls over the BUMPs and weight
//             transfers between wheels under acceleration. Each wheel's drive force comes from the
//             Kraken X60 curve (current limit, back-EMF, battery voltage) and is capped by its own
//             friction circle μN: push too hard and it slides.
//   hull      convex pieces measured from the CAD; the intake's piece moves with the four-bar, so
//             the robot is 0.549 m tall deployed and 0.728 m stowed (TRENCH: 0.565 m).
//   intake    the roller pulls a ball in with a friction-limited force; captured balls ride the
//             kicker ramp over the secondary roller and drop into the hopper.
//   hopper    real ball bodies inside the robot's hopper walls: they jostle with the chassis.
//   indexer   only while shoot is held: floor rollers walk fuel back to two lane mouths and the
//             belts carry one ball at a time up through each turret ring to its flywheel.
//   shooter   flywheel with motor curve, inertia and a velocity loop; each shot takes energy out
//             of it; the ball leaves at ~half the wheel's surface speed with backspin.
//   turrets   rate/acceleration-limited second-order servos; aim from distance lookup tables solved
//             with the same drag + Magnus model, leading the target when moving. From our ALLIANCE
//             ZONE they shoot at our HUB; anywhere else they lob fuel back home (ferry).
import type RAPIER from '@dimforge/rapier3d-compat';
import { BATTERY, DRIVE, FUEL, G, INDEXER, INTAKE, KRAKEN_X60, ROBOT, SHOOTER, STEP, TURRET } from './config';
import { IG, groups, GROUP, type Engine } from './engine';
import { BS, type Fuel } from './fuel';
import type { Rng } from './rng';
import { AimTable } from './aim';

const DEG = Math.PI / 180;
const R = FUEL.radius;
const KT = KRAKEN_X60.stallTorque / KRAKEN_X60.stallCurrent;           // N·m/A
const RM = KRAKEN_X60.nominalV / KRAKEN_X60.stallCurrent;              // Ω
const KE = (KRAKEN_X60.nominalV - KRAKEN_X60.freeCurrent * RM) / KRAKEN_X60.free; // V·s/rad

// ---------------------------------------------------------------- geometry (robot frame: +z front, y up)
export const PIVOT: [number, number] = [0.2094, 0.2164];               // four-bar crank pivot (z, y)
const JOINT: [number, number] = [0.4521, 0.2913];
export function couplerOffset(deg: number): [number, number] {
  const a = deg * DEG, c = Math.cos(a), s = Math.sin(a);
  const dz = JOINT[0] - PIVOT[0], dy = JOINT[1] - PIVOT[1];
  return [PIVOT[0] + dy * s + dz * c - JOINT[0], PIVOT[1] + dy * c - dz * s - JOINT[1]];
}
export const ROLLER = { z: 0.608, y: 0.1675, r: 0.031, hx: 0.295 };
/** the path a captured ball takes (side view, coupler frame): under the roller, up the kicker ramp, over the secondary roller */
export const INTAKE_PATH: [number, number][] = [[0.60, 0.10], [0.56, 0.20], [0.52, 0.34], [0.47, 0.43], [0.38, 0.47]];
export const TURRETS = [
  { id: 'turret-left', pivot: [0.197, 0.321, -0.184] as [number, number, number], base: Math.atan2(0.8, -0.6), side: 1 },
  { id: 'turret-right', pivot: [-0.197, 0.321, -0.184] as [number, number, number], base: Math.atan2(-0.8, -0.6), side: -1 },
];
/** indexer: lane mouth in the hopper, then back under the turret and up through the ring (fixed),
 *  then over the flywheel and out under the hood (turns with the turret: [forward, up] from the pivot) */
const FEED_FIXED = (side: number): [number, number, number][] => [
  [side * 0.13, 0.215, 0.045], [side * 0.16, 0.225, -0.09], [side * 0.197, 0.24, -0.184], [side * 0.197, 0.34, -0.184],
];
const FEED_TURRET: [number, number][] = [[-0.02, 0.11], [0.075, 0.20], [0.145, 0.225]];
export const CAPACITY_DOWN = 48, CAPACITY_UP = 32;
export const capacityAt = (deg: number) => Math.round(CAPACITY_DOWN + (CAPACITY_UP - CAPACITY_DOWN) * Math.min(1, Math.max(0, deg / INTAKE.upDeg)));
/** hopper interior (for counting what's on board) */
const HOP = { hx: 0.33, back: -0.03, floorY: 0.1, top: 0.62 };

export interface ChassisCommand { vx: number; vz: number; w: number; lock?: boolean }
export interface DriveOrder {
  path: [number, number][]; speed: number; heading: 'travel' | number; stopAtEnd: boolean;
  segHeading?: (number | null)[];
}

interface Module { px: number; pz: number; steer: number; wheel: number; current: number; supply: number; N: number; slip: boolean; contact: boolean; comp: number }
interface Captured { i: number; s: number; x: number; phase: 'pinch' | 'ramp' | 'back' }
interface LaneBall { i: number; s: number; from: [number, number, number] }
export interface Turret {
  id: string; side: number; pivot: [number, number, number]; base: number;
  a: number; w: number; goal: number; err: number;
  hood: number; hoodGoal: number;
  fly: number; flyGoal: number; current: number; ready: boolean;
  lane: LaneBall[]; range: boolean; tof: number;
}

const ACC_PLAN = 4.2;

export class Robot {
  readonly body: RAPIER.RigidBody;
  private intakeHull!: RAPIER.Collider;
  private intakeBall: { c: RAPIER.Collider; x: number; z: number; y: number }[] = [];
  private crankBed!: RAPIER.Collider;
  readonly modules: Module[];
  // pose (current and previous step, for interpolation)
  x = 0; y = 0; z = 0; th = 0; vx = 0; vz = 0; w = 0;
  readonly q = { x: 0, y: 0, z: 0, w: 1 };
  readonly prevQ = { x: 0, y: 0, z: 0, w: 1 };
  readonly prevP = [0, 0, 0];
  readonly p = [0, 0, 0];
  pitch = 0; roll = 0;
  /** robot-frame acceleration (smoothed), for the camera and sounds */
  ax = 0; az = 0;
  battery = BATTERY.restV;
  totalCurrent = 0;
  brownout = false;
  impact = 0;
  slipping = false;
  effort = 0;
  // intake
  intakeDown = true; intakeAngle = 0; private armV = 0; capacityBlocked = false;
  rollers: 'in' | 'off' | 'out' = 'off';
  captured: Captured[] = [];
  // shooting
  shoot = false; spinUp = false;
  turrets: Turret[];
  ferry = false;
  outOfRange = false;
  shotsFired = 0;
  lastShotT = 99;
  /** set when the match ends: everything stops */
  disabled = false;
  onShot: ((i: number, side: number) => void) | null = null;
  onIntake: ((i: number) => void) | null = null;
  // commands
  manual: ChassisCommand | null = null;
  cmd: ChassisCommand = { vx: 0, vz: 0, w: 0 };
  private slewed = { vx: 0, vz: 0, w: 0 };
  order: DriveOrder | null = null;
  done = true;
  blocked = false;
  private seg = 0; private bestLeft = Infinity; private bestT = 0;
  /** balls counted on board last step */
  onboard: number[] = [];
  /** our scoring target and ferry landing (field frame) */
  hubTarget: [number, number, number] = [-3.645, 1.83, 0];
  allianceLine = -4.24;
  ferryX = -6.3;
  readonly aim: AimTable;

  constructor(private eng: Engine, private fuel: Fuel, private rng: Rng, private floorAt: (x: number, z: number) => number) {
    const Rp = eng.R;
    this.aim = AimTable.shared();
    const bd = Rp.RigidBodyDesc.dynamic().setCcdEnabled(true).setCanSleep(false)
      .setAdditionalMassProperties(ROBOT.mass, { x: ROBOT.com[0], y: ROBOT.com[1], z: ROBOT.com[2] },
        { x: ROBOT.inertia[0], y: ROBOT.inertia[1], z: ROBOT.inertia[2] }, { x: 0, y: 0, z: 0, w: 1 })
      .setLinearDamping(0.02).setAngularDamping(0.05);
    this.body = eng.world.createRigidBody(bd);
    const add = (desc: RAPIER.ColliderDesc, ig: number, surface: 'bumper' | 'robot' = 'robot') => {
      desc.setDensity(0).setCollisionGroups(ig)
        .setFriction(surface === 'bumper' ? 0.5 : 0.3).setRestitution(surface === 'bumper' ? 0.2 : 0.12)
        .setFrictionCombineRule(Rp.CoefficientCombineRule.Multiply).setRestitutionCombineRule(Rp.CoefficientCombineRule.Multiply);
      const c = eng.world.createCollider(desc, this.body);
      eng.surfaceOf.set(c.handle, surface);
      return c;
    };
    const B = ROBOT.bumper, bh = (B.y1 - B.y0) / 2, by = (B.y0 + B.y1) / 2, d = B.depth / 2;
    // bumpers: four slabs around the frame (they hit the field and push fuel)
    add(Rp.ColliderDesc.cuboid(B.hx, bh, d).setTranslation(0, by, B.hz - d), IG.bumper, 'bumper');
    add(Rp.ColliderDesc.cuboid(B.hx, bh, d).setTranslation(0, by, -B.hz + d), IG.bumper, 'bumper');
    add(Rp.ColliderDesc.cuboid(d, bh, B.hz - 2 * d).setTranslation(B.hx - d, by, 0), IG.bumper, 'bumper');
    add(Rp.ColliderDesc.cuboid(d, bh, B.hz - 2 * d).setTranslation(-B.hx + d, by, 0), IG.bumper, 'bumper');
    // frame and belly pan (field only: fuel can't reach inside the bumpers)
    add(Rp.ColliderDesc.cuboid(0.356, 0.11, 0.343).setTranslation(0, 0.158, 0), IG.hull);
    // superstructure: hopper walls and turrets, up to 0.549 m (field only)
    add(Rp.ColliderDesc.cuboid(0.36, 0.14, 0.39).setTranslation(0, 0.409, -0.0), IG.hull);
    // the intake's piece: moves with the four-bar (deployed: out past the front bumper; stowed: up to 0.728 m)
    this.intakeHull = add(Rp.ColliderDesc.cuboid(0.327, 0.215, 0.11).setTranslation(0, 0.335, 0.54), IG.hull);
    // ---- what fuel touches inside and on top of the robot
    const BALL = groups(GROUP.ROBOT_BALL, GROUP.BALL);
    // turret/indexer block behind the hopper
    add(Rp.ColliderDesc.cuboid(0.36, 0.14, 0.16).setTranslation(0, 0.409, -0.188), BALL);
    // hopper side walls (fixed part)
    for (const s of [-1, 1]) add(Rp.ColliderDesc.cuboid(0.012, 0.22, 0.24).setTranslation(s * (HOP.hx + 0.012), 0.36, 0.20), BALL);
    // back wall over the indexer
    add(Rp.ColliderDesc.cuboid(0.35, 0.18, 0.012).setTranslation(0, 0.40, HOP.back - 0.012), BALL);
    // inclined bed of floor rollers: 25° up toward the front, so fuel settles at the indexer mouths
    const bedA = Math.atan2(0.09, 0.17);
    add(Rp.ColliderDesc.cuboid(0.34, 0.02, 0.15).setTranslation(0, 0.113 + Math.sin(bedA) * 0.12, 0.10)
      .setRotation(rotX(-bedA)), BALL);
    // under the bed: closed (fuel never falls into the electronics)
    add(Rp.ColliderDesc.cuboid(0.34, 0.04, 0.2).setTranslation(0, 0.06, 0.12), BALL);
    // crank bed (the two hopper rollers on the powered link): continues the floor deployed, stands up stowed
    this.crankBed = add(Rp.ColliderDesc.cuboid(0.32, 0.016, 0.075).setTranslation(0, 0.25, 0.32), BALL);
    // coupler: front wall, side plates over the intake, intake roller and crash bar
    const coupler = (desc: RAPIER.ColliderDesc, z: number, y: number) => {
      const c = add(desc.setTranslation(0, y, z), BALL);
      this.intakeBall.push({ c, x: 0, z, y });
      return c;
    };
    coupler(Rp.ColliderDesc.cuboid(0.33, 0.155, 0.012), 0.654, 0.396);                    // front wall
    for (const s of [-1, 1]) {
      const c = add(Rp.ColliderDesc.cuboid(0.012, 0.2, 0.12).setTranslation(s * (HOP.hx + 0.012), 0.34, 0.54), BALL);
      this.intakeBall.push({ c, x: s * (HOP.hx + 0.012), z: 0.54, y: 0.34 });
    }
    coupler(Rp.ColliderDesc.cylinder(ROLLER.hx, ROLLER.r).setRotation(rotZ(Math.PI / 2)), ROLLER.z, ROLLER.y); // intake roller
    coupler(Rp.ColliderDesc.cuboid(0.3, 0.06, 0.01).setRotation(rotX(0.35)), 0.63, 0.29);  // kicker frame / crash bar
    // the hopper floor over the intake (polycarb kicker ramp, back side): slopes down toward the
    // crank rollers so fuel on it rolls back into the hopper (captured fuel passes under it)
    coupler(Rp.ColliderDesc.cuboid(0.32, 0.008, 0.13).setRotation(rotX(-Math.atan2(0.07, 0.24))), 0.52, 0.345);
    this.modules = ROBOT.modules.map(([px, pz]) => ({ px, pz, steer: 0, wheel: 0, current: 0, supply: 0, N: 0, slip: false, contact: true, comp: 0 }));
    this.turrets = TURRETS.map((t) => ({
      id: t.id, side: t.side, pivot: t.pivot, base: t.base, a: 0, w: 0, goal: 0, err: 1,
      hood: 60 * DEG, hoodGoal: 60 * DEG, fly: 0, flyGoal: 0, current: 0, ready: false, lane: [], range: true, tof: 1,
    }));
    this.place(-6, 0, Math.PI / 2);
  }

  // ------------------------------------------------------------------ placement
  place(x: number, z: number, th: number) {
    const y = this.floorAt(x, z);
    const b = this.body;
    b.setTranslation({ x, y: y + 0.004, z }, true);
    b.setRotation(yawQuat(th), true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true);
    b.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const m of this.modules) { m.steer = 0; m.wheel = 0; m.current = 0; m.supply = 0; }
    this.slewed = { vx: 0, vz: 0, w: 0 };
    this.manual = null; this.order = null; this.done = true;
    for (const c of this.captured) this.releaseCapture(c);
    this.captured = [];
    for (const t of this.turrets) { for (const lb of t.lane) this.fuel.place(lb.i, this.toWorld(lb.from[0], Math.max(lb.from[1], 0.3), lb.from[2])); t.lane = []; t.a = 0; t.w = 0; t.fly = 0; t.ready = false; }
    this.readPose(true);
    this.updateMechanismColliders();
  }

  setIntakeNow(down: boolean) {
    this.intakeDown = down;
    this.intakeAngle = down ? 0 : INTAKE.upDeg;
    this.armV = 0;
    this.updateMechanismColliders();
  }

  /** Preload: balls dropped into the hopper, settled. */
  preload(indices: number[]) {
    indices.forEach((i, k) => {
      const lx = -0.2 + (k % 4) * 0.135, lz = 0.06 + (Math.floor(k / 4) % 2) * 0.15, ly = 0.3 + Math.floor(k / 8) * 0.15;
      this.fuel.place(i, this.toWorld(lx, ly, lz));
    });
  }

  // ------------------------------------------------------------------ frames
  toWorld(lx: number, ly: number, lz: number, out: number[] = [0, 0, 0]) {
    const { x: qx, y: qy, z: qz, w: qw } = this.q;
    // rotate (lx, ly, lz) by q
    const ix = qw * lx + qy * lz - qz * ly, iy = qw * ly + qz * lx - qx * lz, iz = qw * lz + qx * ly - qy * lx, iw = -qx * lx - qy * ly - qz * lz;
    out[0] = ix * qw + iw * -qx + iy * -qz - iz * -qy + this.p[0];
    out[1] = iy * qw + iw * -qy + iz * -qx - ix * -qz + this.p[1];
    out[2] = iz * qw + iw * -qz + ix * -qy - iy * -qx + this.p[2];
    return out;
  }
  toLocal(wx: number, wy: number, wz: number, out: number[] = [0, 0, 0]) {
    const { x: qx, y: qy, z: qz, w: qw } = this.q;
    const lx = wx - this.p[0], ly = wy - this.p[1], lz = wz - this.p[2];
    // rotate by q⁻¹
    const ix = qw * lx - qy * lz + qz * ly, iy = qw * ly - qz * lx + qx * lz, iz = qw * lz - qx * ly + qy * lx, iw = qx * lx + qy * ly + qz * lz;
    out[0] = ix * qw + iw * qx + iy * qz - iz * qy;
    out[1] = iy * qw + iw * qy + iz * qx - ix * qz;
    out[2] = iz * qw + iw * qz + ix * qy - iy * qx;
    return out;
  }
  /** world velocity of a robot-frame point */
  pointVel(lx: number, ly: number, lz: number): [number, number, number] {
    const wp = this.toWorld(lx, ly, lz);
    const v = this.body.linvel(), w = this.body.angvel();
    const rx = wp[0] - this.p[0], ry = wp[1] - this.p[1], rz = wp[2] - this.p[2];
    return [v.x + w.y * rz - w.z * ry, v.y + w.z * rx - w.x * rz, v.z + w.x * ry - w.y * rx];
  }

  private readPose(snap: boolean) {
    const b = this.body;
    const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel();
    if (snap) { this.prevP[0] = t.x; this.prevP[1] = t.y; this.prevP[2] = t.z; Object.assign(this.prevQ, { x: q.x, y: q.y, z: q.z, w: q.w }); }
    else { this.prevP[0] = this.p[0]; this.prevP[1] = this.p[1]; this.prevP[2] = this.p[2]; Object.assign(this.prevQ, this.q); }
    this.p[0] = t.x; this.p[1] = t.y; this.p[2] = t.z;
    this.q.x = q.x; this.q.y = q.y; this.q.z = q.z; this.q.w = q.w;
    this.x = t.x; this.y = t.y; this.z = t.z;
    // heading: where the front (+z) points, projected on the floor
    const f = this.toWorld(0, 0, 1); const fx = f[0] - t.x, fz = f[2] - t.z;
    this.th = Math.atan2(fx, fz);
    const up = this.toWorld(0, 1, 0);
    const ux = up[0] - t.x, uz = up[2] - t.z;
    this.pitch = Math.asin(Math.max(-1, Math.min(1, ux * Math.sin(this.th) + uz * Math.cos(this.th))));
    this.roll = Math.asin(Math.max(-1, Math.min(1, ux * Math.cos(this.th) - uz * Math.sin(this.th))));
    const ax = (v.x - this.vx) / STEP, az = (v.z - this.vz) / STEP;
    const k = 1 - Math.exp(-STEP * 20);
    const c = Math.cos(this.th), s = Math.sin(this.th);
    this.ax += ((ax * c - az * s) - this.ax) * k;
    this.az += ((ax * s + az * c) - this.az) * k;
    this.vx = v.x; this.vz = v.z; this.w = w.y;
  }

  get speed() { return Math.hypot(this.vx, this.vz); }
  get capacity() { return capacityAt(this.intakeAngle); }
  /** fuel on board: hopper + indexer + on its way in */
  get load() { return this.onboard.length + this.captured.length + this.turrets[0].lane.length + this.turrets[1].lane.length; }
  get stored() { return this.onboard.length + this.turrets[0].lane.length + this.turrets[1].lane.length; }
  get shooting() { return this.shoot && this.stored > 0; }
  get flywheelReady() { return this.turrets.every((t) => t.ready); }

  // ------------------------------------------------------------------ step (before the engine)
  preStep() {
    this.updateArm();
    this.updateMechanismColliders();
    let cmd: ChassisCommand;
    if (this.disabled) cmd = { vx: 0, vz: 0, w: 0 };
    else if (this.manual) cmd = this.manual;
    else if (this.order && !this.done) {
      const f = this.follow();
      let e = f.heading - this.th;
      e = Math.atan2(Math.sin(e), Math.cos(e));
      cmd = { vx: f.vx, vz: f.vz, w: Math.max(-6, Math.min(6, e * 5 - this.w * 0.12)) };
    } else cmd = { vx: 0, vz: 0, w: 0 };
    this.cmd = cmd;
    // slew limits like the robot code: the stick never asks for an instant change
    const s = this.slewed, dl = DRIVE.slewLinear * STEP, da = DRIVE.slewAngular * STEP;
    const ex = cmd.vx - s.vx, ez = cmd.vz - s.vz, el = Math.hypot(ex, ez);
    if (el > dl) { s.vx += ex * dl / el; s.vz += ez * dl / el; } else { s.vx = cmd.vx; s.vz = cmd.vz; }
    s.w += Math.max(-da, Math.min(da, cmd.w - s.w));
    this.drive(s, !!cmd.lock);
    this.intakeForces();
    this.indexer();
    this.turretsStep();
    this.lastShotT += STEP;
  }

  // ------------------------------------------------------------------ swerve
  private drive(cmd: ChassisCommand, lock: boolean) {
    const b = this.body;
    const c = Math.cos(this.th), s = Math.sin(this.th);
    // chassis command in the robot frame
    const lx = cmd.vx * c - cmd.vz * s, lz = cmd.vx * s + cmd.vz * c;
    const moving = Math.hypot(cmd.vx, cmd.vz) > 0.02 || Math.abs(cmd.w) > 0.03;
    // inverse kinematics, desaturated so every wheel stays under free speed
    const want = this.modules.map((m) => { const mx = lx + cmd.w * m.pz, mz = lz - cmd.w * m.px; return { sp: Math.hypot(mx, mz), a: Math.atan2(mx, mz) }; });
    const peak = Math.max(...want.map((x) => x.sp));
    const vcap = DRIVE.vmax;
    if (peak > vcap) for (const x of want) x.sp *= vcap / peak;
    const v = b.linvel(), w = b.angvel();
    let total = 0, slip = false, effort = 0;
    const freeV = (KRAKEN_X60.free / DRIVE.gear) * ROBOT.wheelRadius;
    for (let k = 0; k < 4; k++) {
      const m = this.modules[k];
      // ---- suspension: ray down from the module mount
      const mount = this.toWorld(m.px, ROBOT.moduleMountY, m.pz);
      const down = this.toWorld(m.px, ROBOT.moduleMountY - 1, m.pz);
      const dir = { x: down[0] - mount[0], y: down[1] - mount[1], z: down[2] - mount[2] };
      const hit = this.eng.castDown(mount[0], mount[1], mount[2], 0.3, dir);
      const rest = ROBOT.moduleMountY + 0.004;
      m.contact = !!hit && hit.timeOfImpact < rest;
      if (!m.contact || !hit) { m.N = 0; m.comp = 0; m.wheel *= 0.98; m.slip = false; continue; }
      const comp = rest - hit.timeOfImpact;
      const cp = [mount[0] + dir.x * hit.timeOfImpact, mount[1] + dir.y * hit.timeOfImpact, mount[2] + dir.z * hit.timeOfImpact];
      const nrm = hit.normal;
      // velocity of the contact point
      const rx = cp[0] - this.p[0], ry = cp[1] - this.p[1], rz = cp[2] - this.p[2];
      const pvx = v.x + w.y * rz - w.z * ry, pvy = v.y + w.z * rx - w.x * rz, pvz = v.z + w.x * ry - w.y * rx;
      const vn = pvx * nrm.x + pvy * nrm.y + pvz * nrm.z;
      const N = Math.max(0, ROBOT.springK * comp - ROBOT.springC * vn);
      m.N = N; m.comp = comp;
      // ---- steering: toward the wanted direction (never more than 90°: reverse the wheel instead), X when locked
      let target = want[k].sp > 0.03 ? want[k].a : lock || (!moving && this.speed < 0.05) ? Math.atan2(m.px, m.pz) : m.steer;
      let d = Math.atan2(Math.sin(target - m.steer), Math.cos(target - m.steer));
      let flip = 1;
      if (d > Math.PI / 2) { d -= Math.PI; flip = -1; } else if (d < -Math.PI / 2) { d += Math.PI; flip = -1; }
      target = m.steer + d;
      const rate = Math.max(-DRIVE.steerRate, Math.min(DRIVE.steerRate, d * DRIVE.steerKp));
      m.steer += rate * STEP;
      m.steer = Math.atan2(Math.sin(m.steer), Math.cos(m.steer));
      const err = target - m.steer;
      const vd = flip * want[k].sp * Math.max(0, Math.cos(err));
      // ---- wheel axes on the floor: rolling direction and sideways
      const hw = this.toWorld(m.px + Math.sin(m.steer), ROBOT.moduleMountY, m.pz + Math.cos(m.steer));
      let ux = hw[0] - mount[0], uy = hw[1] - mount[1], uz = hw[2] - mount[2];
      const un = ux * nrm.x + uy * nrm.y + uz * nrm.z;
      ux -= un * nrm.x; uy -= un * nrm.y; uz -= un * nrm.z;
      const ul = Math.hypot(ux, uy, uz) || 1; ux /= ul; uy /= ul; uz /= ul;
      // lateral = n × u
      const tx = nrm.y * uz - nrm.z * uy, ty = nrm.z * ux - nrm.x * uz, tz = nrm.x * uy - nrm.y * ux;
      const vl = pvx * ux + pvy * uy + pvz * uz;
      const vt = pvx * tx + pvy * ty + pvz * tz;
      // ---- drive motor: velocity loop → stator current, limited by the current limit and by the
      // battery voltage against back-EMF
      const omegaM = (vl / ROBOT.wheelRadius) * DRIVE.gear;
      let I = (vd - vl) * 140;
      if (this.disabled) I = 0;
      I = Math.max(-DRIVE.statorLimit, Math.min(DRIVE.statorLimit, I));
      const Imax = (this.battery - KE * omegaM) / RM, Imin = (-this.battery - KE * omegaM) / RM;
      I = Math.max(Imin, Math.min(Imax, I));
      const Vapp = KE * omegaM + I * RM;
      m.current = I;
      m.supply = Math.abs(I * Vapp / Math.max(1, this.battery));
      total += m.supply;
      let Fl = (KT * I * DRIVE.gear * DRIVE.efficiency) / ROBOT.wheelRadius;
      // tread scrub resists sideways sliding: the impulse that would stop this contact point's
      // sideways motion (its effective mass includes the arm to the center of mass, which sits
      // above the floor), shared by the four wheels, all within this wheel's friction circle
      const com = this.toWorld(ROBOT.com[0], ROBOT.com[1], ROBOT.com[2]);
      const ax = cp[0] - com[0], ay = cp[1] - com[1], az = cp[2] - com[2];
      const cx = ay * tz - az * ty, cy = az * tx - ax * tz, cz = ax * ty - ay * tx;
      const kInv = 1 / ROBOT.mass + cx * cx / ROBOT.inertia[0] + cy * cy / ROBOT.inertia[1] + cz * cz / ROBOT.inertia[2];
      let Ft = -(vt / kInv) / STEP * 0.22;
      const onHdpe = this.floorAt(cp[0], cp[2]) > 0.01;
      const mu = onHdpe ? ROBOT.muHdpe : ROBOT.mu;
      const F = Math.hypot(Fl, Ft);
      m.slip = false;
      if (F > mu * N) { const kk = ((onHdpe ? ROBOT.muHdpe * 0.88 : ROBOT.muSlide) * N) / F; Fl *= kk; Ft *= kk; m.slip = true; slip = true; }
      m.wheel = m.slip ? vd : vl;
      effort += Math.abs(I) / DRIVE.statorLimit / 4;
      const fx = Fl * ux + Ft * tx + N * nrm.x, fy = Fl * uy + Ft * ty + N * nrm.y, fz = Fl * uz + Ft * tz + N * nrm.z;
      b.applyImpulseAtPoint({ x: fx * STEP, y: fy * STEP, z: fz * STEP }, { x: cp[0], y: cp[1], z: cp[2] }, true);
      void freeV;
    }
    this.slipping = slip;
    this.effort = effort;
    // battery: everything draws from one 12 V battery through its internal resistance
    let shooter = 0;
    for (const t of this.turrets) shooter += Math.abs(t.current) * Math.min(1, (KE * t.fly + Math.abs(t.current) * RM) / Math.max(1, this.battery));
    this.totalCurrent = total + shooter + BATTERY.baseCurrent;
    const vb = BATTERY.restV - BATTERY.resistance * this.totalCurrent;
    this.battery += (vb - this.battery) * 0.5;
    this.brownout = this.battery < BATTERY.brownout;
  }

  // ------------------------------------------------------------------ four-bar
  private updateArm() {
    const goal = this.intakeDown ? 0 : INTAKE.upDeg;
    const d = goal - this.intakeAngle;
    const want = Math.sign(d) * Math.min(INTAKE.vmax, Math.sqrt(2 * INTAKE.accel * Math.abs(d)));
    this.armV += Math.max(-INTAKE.accel * STEP, Math.min(INTAKE.accel * STEP, want - this.armV));
    let next = this.intakeAngle + this.armV * STEP;
    this.capacityBlocked = next < this.intakeAngle && capacityAt(next) < this.load;
    if (this.capacityBlocked) { next = this.intakeAngle; this.armV = 0; }
    if ((goal - next) * d <= 0) { next = goal; this.armV = 0; }
    this.intakeAngle = Math.max(INTAKE.upDeg, Math.min(0, next));
  }

  private lastArm = NaN;
  private updateMechanismColliders() {
    if (this.intakeAngle === this.lastArm) return;
    this.lastArm = this.intakeAngle;
    const [dz, dy] = couplerOffset(this.intakeAngle);
    // field-facing piece: the coupler block (deployed: z 0.43–0.65, up to 0.549 m)
    this.intakeHull.setTranslationWrtParent({ x: 0, y: 0.335 + dy, z: 0.54 + dz });
    for (const p of this.intakeBall) p.c.setTranslationWrtParent({ x: p.x, y: p.y + dy, z: p.z + dz });
    // crank bed rotates about the crank pivot
    const a = this.intakeAngle * DEG, c = Math.cos(a), s = Math.sin(a);
    const z0 = 0.32 - PIVOT[0], y0 = 0.25 - PIVOT[1];
    const bedA = Math.atan2(0.09, 0.17);
    this.crankBed.setTranslationWrtParent({ x: 0, y: PIVOT[1] + y0 * c - z0 * s, z: PIVOT[0] + y0 * s + z0 * c });
    this.crankBed.setRotationWrtParent(rotX(-bedA - a));
  }

  // ------------------------------------------------------------------ intake
  private tmp = [0, 0, 0];
  private intakeForces() {
    const f = this.fuel;
    const [dz, dy] = couplerOffset(this.intakeAngle);
    const deployed = this.intakeAngle > -12;
    // count what's on board and find balls at the roller
    const onboard: number[] = [];
    const L = this.tmp;
    const pull = this.rollers === 'in' && deployed && !this.disabled;
    for (let i = 0; i < f.n; i++) {
      const st = f.st[i];
      if (st !== BS.FREE && st !== BS.FLYING) continue;
      const o = i * 3;
      const ddx = f.pos[o] - this.p[0], ddz = f.pos[o + 2] - this.p[2];
      if (ddx * ddx + ddz * ddz > 1.4) continue;
      this.toLocal(f.pos[o], f.pos[o + 1], f.pos[o + 2], L);
      if (Math.abs(L[0]) < HOP.hx && L[2] > HOP.back && L[2] < 0.66 + dz && L[1] > HOP.floorY && L[1] < HOP.top + 0.1 && !this.isCaptured(i)) {
        if (L[2] < 0.48 + dz || L[1] > 0.3 + dy) onboard.push(i);
      }
      // the roller: friction-limited grab of anything it touches
      if (!pull || this.isCaptured(i)) continue;
      const rz = ROLLER.z + dz, ry = ROLLER.y + dy;
      if (Math.abs(L[0]) > ROLLER.hx - 0.02 || L[1] > 0.26 + dy) continue;
      const gap = Math.hypot(L[2] - rz, L[1] - ry) - (ROLLER.r + R);
      if (gap > 0.012 || L[2] < rz - 0.04) continue;
      if (this.load >= this.capacity || !this.laneFree(L[0])) continue;
      // a ball being shoved away faster than the roller can pull (robot backing off) is missed
      const pv = this.pointVel(L[0], L[1], L[2]);
      const b = f.bodies[i];
      const bv = b.linvel();
      const rel = this.velToLocal(bv.x - pv[0], bv.y - pv[1], bv.z - pv[2]);
      if (rel[2] > 1.2) continue;
      this.captured.push({ i, s: 0, x: Math.max(-0.26, Math.min(0.26, L[0])), phase: 'pinch' });
      f.colliders[i].setCollisionGroups(groups(GROUP.BALL, GROUP.FIELD | GROUP.BALL));
      this.onIntake?.(i);
    }
    this.onboard = onboard;
    // captured balls ride the path: spring to the path point + drive along it, friction-limited
    const path = INTAKE_PATH;
    const len = pathLen(path);
    for (let k = this.captured.length - 1; k >= 0; k--) {
      const c = this.captured[k];
      const b = f.bodies[c.i];
      if (f.st[c.i] !== BS.FREE && f.st[c.i] !== BS.FLYING) { this.captured.splice(k, 1); continue; }
      const speed = this.rollers === 'in' ? INTAKE.feedSpeed : this.rollers === 'out' ? -INTAKE.feedSpeed * 1.1 : 0;
      // the next ball can't overtake the one ahead
      const ahead = this.captured.find((o) => o !== c && o.s > c.s && Math.abs(o.x - c.x) < 2 * R);
      let ds = speed * STEP;
      if (ahead && c.s + ds > ahead.s - 2 * R) ds = Math.max(0, ahead.s - 2 * R - c.s);
      // the pile in the hopper pushes back: if the ball can't follow, it stalls
      const pt = pathAt(path, c.s);
      const want = this.toWorld(c.x, pt[1] + dy, pt[0] + dz);
      const t = b.translation();
      const lag = Math.hypot(want[0] - t.x, want[1] - t.y, want[2] - t.z);
      if (lag < 0.05) c.s = Math.max(0, Math.min(len, c.s + ds));
      const nt = pathAt(path, c.s);
      const goal = this.toWorld(c.x, nt[1] + dy, nt[0] + dz);
      const pv = this.pointVel(c.x, nt[1] + dy, nt[0] + dz);
      const v = b.linvel();
      const m = f.mass[c.i];
      // spring-damper toward the path point (the roller, ramp and side plates holding it), capped
      // at what the rollers' grip can do
      const kx = 900, cd = 40;
      let jx = (kx * (goal[0] - t.x) + cd * (pv[0] - v.x)) * STEP, jy = (kx * (goal[1] - t.y) + cd * (pv[1] - v.y) + m * G) * STEP, jz = (kx * (goal[2] - t.z) + cd * (pv[2] - v.z)) * STEP;
      const jmax = 60 * STEP;
      const jl = Math.hypot(jx, jy, jz);
      if (jl > jmax) { jx *= jmax / jl; jy *= jmax / jl; jz *= jmax / jl; }
      b.applyImpulse({ x: jx, y: jy, z: jz }, true);
      if (lag > 0.25) { this.releaseCapture(c); this.captured.splice(k, 1); continue; }
      if (c.s >= len - 1e-3 && this.rollers === 'in') {
        // over the secondary roller: it drops into the hopper moving backward
        this.releaseCapture(c);
        const back = this.velToWorld(0, 0.2, -1.4);
        b.setLinvel({ x: pv[0] + back[0], y: pv[1] + back[1], z: pv[2] + back[2] }, true);
        this.captured.splice(k, 1);
      } else if (c.s <= 0 && this.rollers === 'out') {
        this.releaseCapture(c);
        const fwd = this.velToWorld(0, 0.3, 2.4);
        b.setLinvel({ x: pv[0] + fwd[0], y: pv[1] + fwd[1], z: pv[2] + fwd[2] }, true);
        this.captured.splice(k, 1);
      }
    }
  }
  private isCaptured(i: number) { for (const c of this.captured) if (c.i === i) return true; return false; }
  private releaseCapture(c: Captured) { this.fuel.colliders[c.i].setCollisionGroups(IG.ball); }
  private laneFree(x: number) { for (const c of this.captured) if (Math.abs(c.x - x) < 2 * R - 0.01 && c.s < 2 * R + 0.02) return false; return true; }
  velToLocal(vx: number, vy: number, vz: number) { const a = this.toLocal(this.p[0] + vx, this.p[1] + vy, this.p[2] + vz); return a; }
  velToWorld(lx: number, ly: number, lz: number) { const a = this.toWorld(lx, ly, lz); return [a[0] - this.p[0], a[1] - this.p[1], a[2] - this.p[2]]; }

  // ------------------------------------------------------------------ indexer
  private static fixedLen = (() => { const p = FEED_FIXED(1); let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1], p[k][2] - p[k - 1][2]); return L; })();
  private static turretLen = (() => { const p: [number, number][] = [[0, 0.34 - 0.321], ...FEED_TURRET]; let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); return L; })();

  private indexer() {
    const f = this.fuel;
    const feeding = this.shoot && !this.disabled;
    // floor rollers walk the pile back toward the lane mouths while feeding
    if (feeding) {
      for (const i of this.onboard) {
        const o = i * 3;
        const L = this.toLocal(f.pos[o], f.pos[o + 1], f.pos[o + 2]);
        if (L[1] > 0.36) continue;
        const b = f.bodies[i];
        const lv = this.velToLocal(b.linvel().x, b.linvel().y, b.linvel().z);
        const pv = this.pointVel(L[0], L[1], L[2]);
        // mecanum side rollers at the back walk fuel toward the nearer lane mouth (a ball dead
        // center goes to the lane with fewer balls)
        const toward = Math.abs(L[0]) > 0.13 ? -Math.sign(L[0]) : L[0] > 0.01 ? 1 : L[0] < -0.01 ? -1 : this.turrets[0].lane.length <= this.turrets[1].lane.length ? 1 : -1;
        const want = this.velToWorld(toward * 0.3 * (L[2] < 0.14 ? 1 : 0), 0, -INDEXER.floorSpeed);
        const m = f.mass[i];
        const k = 6 * m * STEP;
        b.applyImpulse({ x: (want[0] + pv[0] - b.linvel().x) * k, y: 0, z: (want[2] + pv[2] - b.linvel().z) * k }, true);
        void lv;
      }
    }
    const belt = INDEXER.beltSpeed / 2;
    const end = Robot.fixedLen + Robot.turretLen;
    for (const t of this.turrets) {
      // take the next ball at this lane's mouth
      if (feeding && (t.lane.length === 0 || t.lane[t.lane.length - 1].s > 2 * R + 0.01) && t.lane.length < 3) {
        const mouth = FEED_FIXED(t.side)[0];
        let best = -1, bd = 0.15;
        for (const i of this.onboard) {
          const o = i * 3;
          const L = this.toLocal(f.pos[o], f.pos[o + 1], f.pos[o + 2]);
          if (L[0] * t.side < -0.02) continue;
          const d = Math.hypot(L[0] - mouth[0], L[1] - mouth[1], L[2] - mouth[2]);
          if (d < bd) { bd = d; best = i; }
        }
        if (best >= 0) {
          const o = best * 3;
          const L = this.toLocal(f.pos[o], f.pos[o + 1], f.pos[o + 2]);
          f.toIndexer(best);
          t.lane.push({ i: best, s: -bd, from: [L[0], L[1], L[2]] });
          this.onboard = this.onboard.filter((x) => x !== best);
        }
      }
      // belts run only while feeding; the head ball waits at the flywheel until the shooter is ready
      for (let k = 0; k < t.lane.length; k++) {
        const lb = t.lane[k];
        let limit = k === 0 ? end : t.lane[k - 1].s - 2 * R - 0.004;
        if (k === 0 && !t.ready) limit = Math.min(limit, Robot.fixedLen);
        if (feeding) lb.s = Math.min(lb.s + belt * STEP * (lb.s > Robot.fixedLen ? 3 : 1), limit);
        const p = this.feedPoint(t, lb, this.tmp);
        const wp = this.toWorld(p[0], p[1], p[2]);
        f.moveKinematic(lb.i, wp[0], wp[1], wp[2]);
      }
      const head = t.lane[0];
      if (head && head.s >= end - 1e-4) { t.lane.shift(); this.launch(t, head.i); }
    }
  }

  /** Robot-frame point at distance s along a turret's feed (s < 0: from where it was picked up to the mouth). */
  feedPoint(t: Turret, b: LaneBall, out: number[]) {
    const fixed = FEED_FIXED(t.side);
    if (b.s < 0) {
      const L0 = Math.hypot(b.from[0] - fixed[0][0], b.from[1] - fixed[0][1], b.from[2] - fixed[0][2]) || 1;
      const k = Math.max(0, Math.min(1, 1 + b.s / L0));
      for (let q = 0; q < 3; q++) out[q] = b.from[q] + (fixed[0][q] - b.from[q]) * k;
      return out;
    }
    let s = b.s;
    for (let k = 1; k < fixed.length; k++) {
      const a = fixed[k - 1], c = fixed[k];
      const l = Math.hypot(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
      if (s <= l) { const u = s / l; for (let q = 0; q < 3; q++) out[q] = a[q] + (c[q] - a[q]) * u; return out; }
      s -= l;
    }
    const yaw = t.base + t.a, fx = Math.sin(yaw), fz = Math.cos(yaw);
    const pts: [number, number][] = [[0, 0.34 - t.pivot[1]], ...FEED_TURRET];
    for (let k = 1; k < pts.length; k++) {
      const a = pts[k - 1], c = pts[k];
      const l = Math.hypot(c[0] - a[0], c[1] - a[1]);
      if (s <= l || k === pts.length - 1) {
        const u = Math.min(1, s / l);
        const fw = a[0] + (c[0] - a[0]) * u, up = a[1] + (c[1] - a[1]) * u;
        out[0] = t.pivot[0] + fx * fw; out[1] = t.pivot[1] + up; out[2] = t.pivot[2] + fz * fw;
        return out;
      }
      s -= l;
    }
    return out;
  }

  /** Turret exit under the hood, robot frame. */
  exitLocal(t: Turret) {
    const yaw = t.base + t.a, f = FEED_TURRET[FEED_TURRET.length - 1];
    return [t.pivot[0] + Math.sin(yaw) * f[0], t.pivot[1] + f[1] + 0.02, t.pivot[2] + Math.cos(yaw) * f[0]];
  }
  exitWorld(t: Turret) { const e = this.exitLocal(t); return this.toWorld(e[0], e[1], e[2]); }

  // ------------------------------------------------------------------ turrets, hood, flywheels
  /** Where shots go from here: our HUB from our ALLIANCE ZONE, otherwise a lob back home. */
  private target(): { x: number; y: number; z: number; ferry: boolean } {
    if (this.x < this.allianceLine) return { x: this.hubTarget[0], y: this.hubTarget[1], z: this.hubTarget[2], ferry: false };
    const z = Math.max(-2.6, Math.min(2.6, this.z * 0.8));
    return { x: this.ferryX, y: 0, z, ferry: true };
  }

  private turretsStep() {
    const tg = this.target();
    this.ferry = tg.ferry;
    let outOfRange = false;
    for (const t of this.turrets) {
      const exit = this.exitWorld(t);
      const piv = this.toWorld(t.pivot[0], t.pivot[1], t.pivot[2]);
      const pv = this.pointVel(t.pivot[0], t.pivot[1], t.pivot[2]);
      // shooting on the move: aim at where the target will appear to be after the time of flight
      let ax = tg.x, az = tg.z, sol = null as ReturnType<AimTable['lookup']>;
      for (let it = 0; it < 3; it++) {
        const d = Math.hypot(ax - exit[0], az - exit[2]);
        sol = this.aim.lookup(tg.ferry ? 'ferry' : 'hub', d, exit[1] - tg.y);
        const tof = sol?.tof ?? t.tof;
        ax = tg.x - pv[0] * tof; az = tg.z - pv[2] * tof;
      }
      t.range = !!sol;
      if (!sol && this.shoot) outOfRange = true;
      if (sol) { t.tof = sol.tof; t.hoodGoal = sol.hood; }
      // turret: second-order servo with rate and acceleration limits, ±180° (no wrap through the wiring)
      const want = Math.atan2(ax - piv[0], az - piv[2]);
      let goal = want - this.th - t.base - this.w * 0.02;
      goal = Math.atan2(Math.sin(goal), Math.cos(goal));
      goal = Math.max(-TURRET.limit, Math.min(TURRET.limit, goal));
      t.goal = goal;
      const accCmd = TURRET.wn * TURRET.wn * (goal - t.a) - 2 * TURRET.zeta * TURRET.wn * t.w;
      const acc = Math.max(-TURRET.accel, Math.min(TURRET.accel, accCmd));
      t.w = Math.max(-TURRET.vmax, Math.min(TURRET.vmax, t.w + acc * STEP));
      t.a = Math.max(-TURRET.limit, Math.min(TURRET.limit, t.a + t.w * STEP));
      const actual = this.th + t.base + t.a;
      t.err = Math.abs(Math.atan2(Math.sin(want - actual), Math.cos(want - actual)));
      // hood servo
      const hr = SHOOTER.hoodRate * DEG * STEP;
      t.hood += Math.max(-hr, Math.min(hr, t.hoodGoal - t.hood));
      // flywheel: velocity loop → current (limited), against the motor's back-EMF and the battery
      const active = (this.shoot || this.spinUp) && !this.disabled;
      // idle a little below shooting speed while holding fuel (EDIT: the team's idle speed)
      t.flyGoal = active ? (sol?.omega ?? this.aim.maxOmega * 0.9) : this.stored > 0 && !this.disabled ? this.aim.maxOmega * 0.35 : 0;
      const omegaM = t.fly * SHOOTER.gear;
      let I = t.flyGoal > 0 ? (t.flyGoal - t.fly) * SHOOTER.kP * 60 + 2.5 : 0;
      I = Math.max(-5, Math.min(SHOOTER.statorLimit, I));
      const Imax = (this.battery - KE * omegaM) / RM;
      I = Math.min(I, Imax);
      t.current = I;
      const fric = 0.00012 * t.fly + 0.03;   // bearings + windage (N·m)
      const tq = KT * I * SHOOTER.gear - (t.fly > 0 ? fric : 0);
      t.fly = Math.max(0, t.fly + (tq / SHOOTER.inertia) * STEP);
      // ready: within tolerance once; stays ready through the dips of a burst (like most teams' code)
      const near = t.flyGoal > 0 && Math.abs(t.fly - t.flyGoal) < t.flyGoal * SHOOTER.readyTol;
      const onTarget = t.err < TURRET.fireTolDeg * DEG && Math.abs(t.hood - t.hoodGoal) < 1.5 * DEG;
      if (!this.shoot || t.flyGoal === 0) t.ready = false;
      else if (near && onTarget && t.range) t.ready = true;
      else if (t.fly < t.flyGoal * 0.8 || t.err > 6 * DEG) t.ready = false;
    }
    this.outOfRange = outOfRange;
  }

  /** The flywheel throws the ball: speed and backspin from the wheel's actual surface speed. */
  private launch(t: Turret, i: number) {
    const f = this.fuel;
    const exit = this.exitWorld(t);
    const u = t.fly * SHOOTER.wheelRadius;                   // wheel surface speed
    const sp = SHOOTER.slip * u / 2 * (1 + this.rng.gauss() * SHOOTER.speedJitter) * Math.sqrt(FUEL.mass / f.mass[i]);
    const yaw = this.th + t.base + t.a + this.rng.gauss() * SHOOTER.yawJitterDeg * DEG;
    const pitch = t.hood + this.rng.gauss() * SHOOTER.pitchJitterDeg * DEG;
    const dx = Math.sin(yaw) * Math.cos(pitch), dy = Math.sin(pitch), dz = Math.cos(yaw) * Math.cos(pitch);
    const pv = this.pointVel(t.pivot[0], t.pivot[1], t.pivot[2]);
    const v = [dx * sp + pv[0], dy * sp + pv[1], dz * sp + pv[2]];
    // backspin: rolling on the hood, axis horizontal and perpendicular to the shot
    const spin = SHOOTER.slip * u / (2 * R);
    const ax = Math.cos(yaw), az = -Math.sin(yaw);           // (dir × up) normalized → backspin axis
    f.launch(i, exit, v, [-ax * spin, 0, -az * spin]);
    // it leaves through the hood: clear of the robot's own panels for its first instants
    f.colliders[i].setCollisionGroups(groups(GROUP.BALL, GROUP.FIELD | GROUP.BALL));
    f.launchT[i] = 0.25;
    // the shot takes energy out of the wheel
    const ke = 0.5 * f.mass[i] * sp * sp + 0.5 * FUEL.inertiaFactor * f.mass[i] * R * R * spin * spin;
    const E = 0.5 * SHOOTER.inertia * t.fly * t.fly - ke / SHOOTER.energyEfficiency;
    t.fly = Math.sqrt(Math.max(0, (2 * E) / SHOOTER.inertia));
    this.shotsFired++;
    this.lastShotT = 0;
    this.onShot?.(i, t.side);
  }

  postStep() {
    this.readPose(false);
  }

  // ------------------------------------------------------------------ autopilot
  navigate(order: DriveOrder | null) {
    this.order = order; this.seg = 0; this.done = !order; this.blocked = false;
    this.bestLeft = Infinity; this.bestT = 0;
  }

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
    if (remaining < this.bestLeft - 0.05) { this.bestLeft = remaining; this.bestT = 0; }
    else if ((this.bestT += STEP) > 1.5) { this.done = true; this.blocked = true; return { vx: 0, vz: 0, heading: this.th }; }
    const sp = this.speed;
    let look = 0.35 + 0.3 * sp;
    let tx = ax + sx * t, tz = az + sz * t;
    let k = this.seg, segLeft = (1 - t) * L;
    while (look > 0 && k < pts.length - 1) {
      const [cx, cz] = k === this.seg ? [tx, tz] : pts[k];
      const [dx2, dz2] = pts[k + 1];
      const l = k === this.seg ? segLeft : Math.hypot(dx2 - cx, dz2 - cz);
      if (look <= l) { const u = look / (l || 1); tx = cx + (dx2 - cx) * u; tz = cz + (dz2 - cz) * u; look = 0; break; }
      look -= l; tx = dx2; tz = dz2; k++; segLeft = 0;
    }
    let dx = tx - this.x, dz = tz - this.z;
    const dl = Math.hypot(dx, dz);
    if (dl < 1e-4) { dx = sx / L; dz = sz / L; } else { dx /= dl; dz /= dl; }
    let v = o.speed;
    if (o.stopAtEnd) v = Math.min(v, Math.sqrt(2 * ACC_PLAN * Math.max(0, remaining - 0.03)), 0.6 + remaining * 4);
    if (this.seg < pts.length - 2) {
      const [px, pz] = pts[this.seg + 1], [qx, qz] = pts[this.seg + 2];
      const ux = qx - px, uz = qz - pz, ul = Math.hypot(ux, uz) || 1;
      const turn = Math.acos(Math.max(-1, Math.min(1, (sx * ux + sz * uz) / (L * ul))));
      const vCorner = Math.max(0.9, DRIVE.vmax * (1 - turn / Math.PI) ** 1.6);
      v = Math.min(v, Math.sqrt(vCorner * vCorner + 2 * ACC_PLAN * (1 - t) * L));
    }
    // over a BUMP with momentum: the belly (4.8 cm, CAD) scrapes the 0.16 m peak, and a robot that
    // crawls over can beach itself on it, like a real one
    if (this.floorAt(this.x, this.z) > 0.02 || this.floorAt(this.x + dx * 0.7, this.z + dz * 0.7) > 0.02) v = Math.max(Math.min(v, 3.4), Math.min(o.speed, 3.0));
    if (remaining < 0.06 && (!o.stopAtEnd || sp < 0.2)) { this.done = true; return { vx: 0, vz: 0, heading: this.th }; }
    if (!o.stopAtEnd && remaining < 0.25) this.done = true;
    const heading = o.segHeading?.[this.seg] ?? (o.heading === 'travel' ? Math.atan2(dx, dz) : o.heading);
    const he = Math.abs(Math.atan2(Math.sin(heading - this.th), Math.cos(heading - this.th)));
    if (he > 0.25) v = Math.min(v, Math.max(0.6, 3.2 * (1 - (he - 0.25) / 1.4)));
    return { vx: dx * v, vz: dz * v, heading };
  }
}

// ---------------------------------------------------------------- helpers
function pathLen(p: [number, number][]) { let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); return L; }
function pathAt(p: [number, number][], s: number): [number, number] {
  for (let k = 1; k < p.length; k++) {
    const l = Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]);
    if (s <= l) { const u = s / l; return [p[k - 1][0] + (p[k][0] - p[k - 1][0]) * u, p[k - 1][1] + (p[k][1] - p[k - 1][1]) * u]; }
    s -= l;
  }
  return p[p.length - 1];
}
export function yawQuat(th: number) { return { x: 0, y: Math.sin(th / 2), z: 0, w: Math.cos(th / 2) }; }
function rotX(a: number) { return { x: Math.sin(a / 2), y: 0, z: 0, w: Math.cos(a / 2) }; }
function rotZ(a: number) { return { x: 0, y: 0, z: Math.sin(a / 2), w: Math.cos(a / 2) }; }
