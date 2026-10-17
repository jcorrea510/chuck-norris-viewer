// Chuck Norris as a Rapier rigid body running the team's robot code.
//   code      teamcode.ts, the Java ported to the roboRIO's 50 Hz: sticks and buttons in, setpoints out
//             (swerve module states, turret azimuth/hood/flywheel, intake pivot and rollers, indexer).
//   chassis   74 kg body on four swerve modules. Each module raycasts the floor (a stiff, damped
//             spring: tread + frame flex), so the robot pitches and rolls over the BUMPs and weight
//             transfers between wheels under acceleration. Each wheel is a Kraken X60 under the Talon's
//             VelocityVoltage loop (current limits, back-EMF, battery) on a wheel with inertia; its
//             steering is a Kraken X60 under PositionVoltage. Tread force is capped by the friction
//             circle μN: push too hard and the wheel spins.
//   hull      convex pieces measured from the CAD; the intake's piece moves with the four-bar, so
//             the robot is 0.549 m tall deployed and 0.728 m stowed (TRENCH: 0.565 m).
//   intake    the four-bar is a real DOF: the pivot motor (Motion Magic) against gravity and the contact
//             forces on the intake's hull, so a hit pushes it up. The rollers pull a ball in with a
//             friction-limited force; captured balls ride the kicker ramp over the secondary roller.
//   hopper    real ball bodies inside the robot's hopper walls: they jostle with the chassis.
//   indexer   the hotdog rollers (the hopper floor) walk fuel to two lane mouths and the indexer
//             belts carry it up through each turret ring to the flywheel, only while their motors run.
//   shooter   the flywheels run all the time; a ball leaves at the wheel's speed when it reaches it
//             (no readiness gate), and each shot takes energy out of the wheel.
//   turrets   Talon position loops on a geared turret with soft limits; the targeting and the
//             wrapping are the team's code.
import type RAPIER from '@dimforge/rapier3d-compat';
import { BATTERY, CODE_PERIOD, DRIVE, FIELD_X0, FIELD_Y0, FUEL, G, INDEXER, INTAKE, KRAKEN_X60, ROBOT, SHOOTER, STEP, TURRET } from './config';
import { IG, groups, GROUP, surfaceMu, surfaceE, type Engine } from './engine';
import { BS, type Fuel } from './fuel';
import { PIVOT, JOINT, couplerOffset } from './fourbar';
import { Azimuth, Flywheel, Hood, Pivot, VelocityMotor } from './mechanisms';
import { motorCurrent, motorOf, stepMotor } from './motors';
import type { Rng } from './rng';
import { RobotCode, wrapPi, type CodeInput, type Outputs } from './teamcode';
import type { DriverInput } from '../world';

const DEG = Math.PI / 180;
const TAU = Math.PI * 2;
const R = FUEL.radius;

// ---------------------------------------------------------------- geometry (robot frame: +z front, y up)
export { PIVOT, couplerOffset };
const JOINT_DZ = JOINT[0] - PIVOT[0], JOINT_DY = JOINT[1] - PIVOT[1];
/** how the four-bar's crank angle (rad) maps onto the motor shaft (rad): the stowed→deployed travel is downPos rotations */
const DPHI_DTHETA = (-INTAKE.upDeg * DEG) / INTAKE.downPos / TAU;
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

export interface DriveOrder {
  path: [number, number][]; speed: number; heading: 'travel' | number; stopAtEnd: boolean;
  segHeading?: (number | null)[];
}

interface Module {
  px: number; pz: number;
  /** steering angle (rad, continuous) and speed; wheel spin (rad/s) and its surface speed along the wheel (m/s) */
  steer: number; steerW: number; ww: number; wheel: number;
  /** drive stator current (A) and supply current (A); steering supply */
  current: number; supply: number; steerSupply: number;
  N: number; slip: boolean; contact: boolean; comp: number;
}
interface Captured { i: number; s: number; x: number; phase: 'pinch' | 'ramp' | 'back'; idle: number }
interface LaneBall { i: number; s: number; from: [number, number, number] }

/** One turret: azimuth, hood and flywheel hardware plus the fuel riding its lane. */
export class TurretUnit {
  readonly az: Azimuth;
  readonly flywheel = new Flywheel();
  readonly hood: Hood;
  /** the CAD mesh's rotation (rad): the turret's yaw relative to the robot minus the mesh's base direction */
  a = 0;
  /** setpoints the code last sent (null while disabled) */
  azGoal = TURRET.startupDeg; hoodGoal = SHOOTER.hoodMin; rpsGoal = 0;
  lane: LaneBall[] = [];
  constructor(readonly index: number, readonly id: string, readonly side: number, readonly pivot: [number, number, number], readonly base: number) {
    this.az = new Azimuth(index);
    // channel 2 (the left turret) has its pulse direction inverted
    this.hood = new Hood(index === 0);
    this.a = this.az.deg * DEG - base;
  }
  /** azimuth, deg (WPILib, unwrapped) */
  get aw() { return this.az.deg; }
  get hoodDeg() { return this.hood.deg; }
  get pulseUs() { return this.hood.pos; }
  get rps() { return this.flywheel.rps; }
  /** flywheel speed and goal, rad/s */
  get fly() { return this.flywheel.w; }
  get flyGoal() { return this.rpsGoal * TAU; }
  get current() { return this.flywheel.current; }
}

const idleInput = (): DriverInput => ({ mx: 0, mz: 0, turn: 0, intake: false, shoot: false, unjam: false, reverseIntake: false, faceHub: false, retract: false, resetHeading: false, turretLock: false, handBack: false });

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
  // the team's code and what it drives
  readonly code = new RobotCode();
  private out!: Outputs;
  private codeClock = 0;
  readonly pivot: Pivot;
  readonly roller: VelocityMotor;
  readonly indexerMotor: VelocityMotor;
  readonly hotdogMotor: VelocityMotor;
  intakeAngle = INTAKE.upDeg; capacityBlocked = false;
  captured: Captured[] = [];
  // shooting
  readonly turrets: TurretUnit[];
  shotsFired = 0;
  lastShotT = 99;
  /** set when the match ends: everything stops */
  disabled = false;
  onShot: ((i: number, side: number) => void) | null = null;
  onIntake: ((i: number) => void) | null = null;
  // the controller: a human's (the world copies it in each step) or the autopilot's (the brain sets
  // the buttons, the path follower the sticks)
  readonly input: DriverInput = idleInput();
  autopilot = false;
  private press = { retract: false, resetHeading: false, turretLock: false };
  order: DriveOrder | null = null;
  done = true;
  blocked = false;
  private seg = 0; private bestLeft = Infinity; private bestT = 0;
  /** balls counted on board last step */
  onboard: number[] = [];
  private armInertia = 0;
  private steerMotor = motorOf(KRAKEN_X60);
  private driveMotor = motorOf(KRAKEN_X60);

  constructor(private eng: Engine, private fuel: Fuel, private rng: Rng, private floorAt: (x: number, z: number) => number) {
    const Rp = eng.R;
    // the robot's contacts start 6 cm (one 120 Hz step at full speed) before it touches anything (soft CCD prediction), so at full
    // speed into a wall it overlaps by at most ~1 cm for one step (tests/phys/realism.test.ts)
    const bd = Rp.RigidBodyDesc.dynamic().setCcdEnabled(true).setSoftCcdPrediction(0.06).setCanSleep(false)
      .setAdditionalMassProperties(ROBOT.mass, { x: ROBOT.com[0], y: ROBOT.com[1], z: ROBOT.com[2] },
        { x: ROBOT.inertia[0], y: ROBOT.inertia[1], z: ROBOT.inertia[2] }, { x: 0, y: 0, z: 0, w: 1 })
      .setLinearDamping(0.02).setAngularDamping(0.05);
    this.body = eng.world.createRigidBody(bd);
    // field contacts use the hull/bumper values; fuel-only colliders use the FUEL table (÷ the ball's share)
    const add = (desc: RAPIER.ColliderDesc, ig: number, surface: 'bumper' | 'robot' = 'robot') => {
      const fuelOnly = (ig & 0xffff) === GROUP.BALL;
      desc.setDensity(0).setCollisionGroups(ig)
        .setFriction(fuelOnly ? surfaceMu(surface) : surface === 'bumper' ? ROBOT.bumperFriction : ROBOT.hullFriction)
        .setRestitution(fuelOnly ? surfaceE(surface) : surface === 'bumper' ? ROBOT.bumperRestitution : ROBOT.hullRestitution)
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
    // for fuel, the bumpers reach the carpet (3.7 cm under them: a 15 cm ball can never get in) and
    // the space under the frame is closed, so the robot can never drive over or swallow a ball
    const BALLONLY = groups(GROUP.ROBOT_BALL, GROUP.BALL);
    add(Rp.ColliderDesc.cuboid(B.hx, 0.03, B.hz).setTranslation(0, 0.03, 0), BALLONLY, 'bumper');
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
    // inclined bed of floor rollers (the hotdog): 25° up toward the front, so fuel settles at the indexer mouths
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
    this.modules = ROBOT.modules.map(([px, pz]) => ({ px, pz, steer: 0, steerW: 0, ww: 0, wheel: 0, current: 0, supply: 0, steerSupply: 0, N: 0, slip: false, contact: true, comp: 0 }));
    this.turrets = TURRETS.map((t, i) => new TurretUnit(i, t.id, t.side, t.pivot, t.base));
    // the arm's inertia about the crank pivot (moving mass at the joint's radius), seen from the motor shaft
    this.armInertia = INTAKE.mass * (JOINT_DZ ** 2 + JOINT_DY ** 2) * DPHI_DTHETA ** 2;
    this.pivot = new Pivot(this.armInertia, INTAKE.armFriction * DPHI_DTHETA);
    this.roller = new VelocityMotor({ kP: INTAKE.rollerKp, kV: INTAKE.rollerKv, stator: INTAKE.rollerStator, supply: INTAKE.rollerSupply, inertia: INTAKE.rollerInertia, friction: 0.02, eta: 0.9 }, INTAKE.motor);
    this.indexerMotor = new VelocityMotor({ kP: 0, kV: INDEXER.indexerKv, stator: INDEXER.indexerStator, supply: INDEXER.indexerSupply, inertia: INDEXER.inertia, friction: INDEXER.indexerFriction, eta: 0.9 }, INDEXER.motor);
    this.hotdogMotor = new VelocityMotor({ kP: 0, kV: INDEXER.hotdogKv, stator: INDEXER.hotdogStator, supply: INDEXER.hotdogSupply, inertia: INDEXER.inertia, friction: INDEXER.hotdogFriction, eta: 0.9 }, INDEXER.motor);
    this.place(-6, 0, Math.PI / 2);
    this.powerCycle();
  }

  // ------------------------------------------------------------------ placement
  place(x: number, z: number, th: number) {
    const y = this.floorAt(x, z);
    const b = this.body;
    b.setTranslation({ x, y: y + 0.004, z }, true);
    b.setRotation(yawQuat(th), true);
    b.setLinvel({ x: 0, y: 0, z: 0 }, true);
    b.setAngvel({ x: 0, y: 0, z: 0 }, true);
    for (const m of this.modules) { m.ww = 0; m.wheel = 0; m.current = 0; m.supply = 0; m.steerW = 0; }
    this.order = null; this.done = true;
    for (const c of this.captured) this.releaseCapture(c);
    this.captured = [];
    for (const t of this.turrets) { for (const lb of t.lane) this.fuel.place(lb.i, this.toWorld(lb.from[0], Math.max(lb.from[1], 0.3), lb.from[2])); t.lane = []; }
    this.code.headingErr = 0;
    this.readPose(true);
    this.updateMechanismColliders();
  }

  /** The robot powers up: code restarted, intake stowed, turrets at their startup pose pointing backward, flywheels stopped. */
  powerCycle() {
    this.code.reset(this.th - Math.PI / 2);
    this.codeClock = CODE_PERIOD;
    this.pivot.place(false);
    this.intakeAngle = this.pivot.arm;
    for (const t of this.turrets) {
      t.az.deg = TURRET.startupDeg; t.az.w = 0; t.flywheel.w = 0; t.flywheel.step(null, BATTERY.restV, STEP);
      t.hood.pos = t.hood.pulseCmd = Hood.pulseFor(SHOOTER.hoodMin, t.index === 0);
      t.a = t.az.deg * DEG - t.base; t.azGoal = TURRET.startupDeg; t.hoodGoal = SHOOTER.hoodMin; t.rpsGoal = 0;
    }
    for (const mo of [this.roller, this.indexerMotor, this.hotdogMotor]) mo.w = 0;
    this.releaseControls();
    this.battery = BATTERY.restV;
    this.out = { drive: null, turrets: [null, null], pivotTarget: INTAKE.upPos, rollerRps: 0, indexerRps: 0, hotdogRps: 0 };
    this.updateMechanismColliders();
  }

  /** Put the intake in a pose directly (tests, respawn): the pivot and the code's setpoint follow. */
  setIntakeNow(down: boolean) {
    this.pivot.place(down);
    this.code.setPivot(down);
    this.intakeAngle = this.pivot.arm;
    this.updateMechanismColliders();
  }

  /** Preload: balls dropped into the hopper, settled. */
  preload(indices: number[]) {
    indices.forEach((i, k) => {
      const lx = -0.2 + (k % 4) * 0.135, lz = 0.06 + (Math.floor(k / 4) % 2) * 0.15, ly = 0.3 + Math.floor(k / 8) * 0.15;
      this.fuel.place(i, this.toWorld(lx, ly, lz));
    });
  }

  // ------------------------------------------------------------------ the controller
  /** The human's controller this step: held buttons replace the old state, presses are latched until the next 50 Hz tick. */
  setInput(inp: DriverInput) {
    Object.assign(this.input, inp);
    this.pulse('retract', inp.retract); this.pulse('resetHeading', inp.resetHeading); this.pulse('turretLock', inp.turretLock);
  }
  /** nobody is holding anything */
  releaseControls() { Object.assign(this.input, idleInput()); this.press = { retract: false, resetHeading: false, turretLock: false }; }
  /** a button pressed this step (latched until the code sees it) */
  pulse(k: 'retract' | 'resetHeading' | 'turretLock', on = true) { if (on) this.press[k] = true; }

  /** The rollers' surface speed (m/s, + pulls fuel in), the hotdog's floor speed and the indexer's belt speed, from the motors' actual speeds. */
  get rollerSurface() { return this.roller.rps * INTAKE.rollerSurface; }
  get rollers(): 'in' | 'off' | 'out' { const s = this.rollerSurface; return s > 0.25 ? 'in' : s < -0.25 ? 'out' : 'off'; }
  get intakeDown() { return this.code.pivotTarget >= INTAKE.downPos - 1e-6; }
  get ferry() { return this.code.ferry; }

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
  /** RT is held (or the indexer is otherwise feeding forward) with fuel on board */
  get shooting() { return this.code.isFeedingForward && this.stored > 0; }
  /** the pose estimate's heading (sim yaw, rad): the truth plus the estimate's error */
  get headingEstimate() { return wrapPi(this.th + this.code.headingErr); }

  // ------------------------------------------------------------------ step (before the engine)
  preStep() {
    this.codeClock += STEP;
    if (this.codeClock >= CODE_PERIOD - 1e-9) { this.codeClock -= CODE_PERIOD; this.runCode(); }
    const out = this.out;
    this.updateArm(out);
    this.updateMechanismColliders();
    const supply = this.drive(out) + this.mechanisms(out);
    this.intakeForces();
    this.indexer();
    this.lastShotT += STEP;
    // battery: everything draws from one 12 V battery through its internal resistance
    this.totalCurrent = supply + BATTERY.baseCurrent;
    const vb = BATTERY.restV - BATTERY.resistance * this.totalCurrent;
    this.battery += (vb - this.battery) * 0.5;
    this.brownout = this.battery < BATTERY.brownout;
  }

  /** One scheduler iteration of the robot code. */
  private runCode() {
    const inp = this.input;
    let mx = inp.mx, mz = inp.mz, turn = inp.turn;
    if (this.autopilot) [mx, mz, turn] = this.autopilotSticks();
    const ci: CodeInput = {
      x: mx, y: -mz, rightX: turn,
      lt: inp.intake, rt: inp.shoot, b: inp.unjam || inp.reverseIntake, start: inp.reverseIntake, a: inp.faceHub,
      rb: this.press.retract, yPress: this.press.resetHeading, povDown: this.press.turretLock,
    };
    this.press = { retract: false, resetHeading: false, turretLock: false };
    this.out = this.code.tick(ci, {
      enabled: !this.disabled,
      x: this.x + FIELD_X0, y: -this.z + FIELD_Y0, theta: wrapPi(this.th - Math.PI / 2),
      modules: this.modules.map((m) => ({ speed: m.wheel, angle: wrapPi(m.steer) })),
    });
  }

  /** The path follower's desired motion as the stick positions that make the team's joystick shaping produce it. */
  private autopilotSticks(): [number, number, number] {
    if (!this.order || this.done) return [0, 0, 0];
    const f = this.follow();
    let e = f.heading - this.th;
    e = Math.atan2(Math.sin(e), Math.cos(e));
    const w = Math.max(-6, Math.min(6, e * 5 - this.w * 0.12));
    const inv = (frac: number) => frac <= 0 ? 0 : DRIVE.deadband + Math.sqrt(Math.min(1, frac)) * (1 - DRIVE.deadband);
    const speed = Math.hypot(f.vx, f.vz);
    const raw = inv(speed / this.code.maxLinearSpeed);
    // the code rotates the field-frame stick by its (possibly wrong) heading estimate: pre-rotate
    // by the estimate's error so the robot still goes where the path says
    const c = Math.cos(this.code.headingErr), s = Math.sin(this.code.headingErr);
    const ux = speed > 1e-6 ? f.vx / speed : 0, uy = speed > 1e-6 ? -f.vz / speed : 0;
    const sx = (ux * c - uy * s) * raw, sy = (ux * s + uy * c) * raw;
    return [sx, -sy, -Math.sign(w) * inv(Math.abs(w) / this.code.maxAngularSpeed)];
  }

  // ------------------------------------------------------------------ swerve
  private steerStep(m: Module, target: number | null, vb: number) {
    const h = STEP / 4;
    let supply = 0;
    const friction = (this.steerMotor.kt * DRIVE.steerGear * DRIVE.steerKs) / this.steerMotor.r;
    for (let k = 0; k < 4; k++) {
      const err = target === null ? 0 : wrapPi(target - m.steer) / TAU;
      const open = DRIVE.steerKp * err;
      const r = stepMotor({
        motor: this.steerMotor, n: 1, gear: DRIVE.steerGear, eta: DRIVE.efficiency, inertia: DRIVE.steerInertia, w: m.steerW,
        v0: target === null ? 0 : open + DRIVE.steerKs * Math.sign(open), kDamp: target === null ? 0 : DRIVE.steerKd / TAU,
        vLo: -vb, vHi: vb, stator: DRIVE.steerStator, supply: DRIVE.steerSupply, vBus: vb, tau: 0, friction, dt: h,
      });
      m.steerW = r.w; m.steer += r.w * h; supply = Math.max(supply, r.supplyA);
    }
    m.steer = wrapPi(m.steer);
    return supply;
  }

  /** The drive motor's torque at the wheel and the supply current, from the Talon's VelocityVoltage loop. */
  private wheelDrive(m: Module, speed: number | null, vb: number) {
    const spRps = speed === null ? 0 : speed / (TAU * ROBOT.wheelRadius);
    const v0 = speed === null ? 0 : DRIVE.kS * Math.sign(spRps) + DRIVE.kV * spRps + DRIVE.kP * (spRps - m.ww / TAU);
    const c = motorCurrent(this.driveMotor, DRIVE.gear, v0, m.ww, vb, DRIVE.statorLimit, DRIVE.supplyLimit);
    const tau = this.driveMotor.kt * DRIVE.gear * c.i * (c.i * m.ww >= 0 ? DRIVE.efficiency : 1 / DRIVE.efficiency);
    return { i: c.i, supplyA: c.supplyA, tau };
  }

  /** Returns the supply current drawn by the drive and steering motors. */
  private drive(out: Outputs) {
    const b = this.body;
    const v = b.linvel(), w = b.angvel();
    const vb = Math.max(6, this.battery);
    const r = ROBOT.wheelRadius;
    const Jw = DRIVE.wheelInertia + this.driveMotor.spec.rotor * DRIVE.gear * DRIVE.gear;
    // the wheels' rotational inertia slows the chassis' acceleration as extra mass at the tread: J/r² each
    const kRot = (4 * Jw) / (r * r * ROBOT.mass);
    // the team's kS is the voltage that overcomes the drivetrain's static friction: the same as a torque at the wheel
    const tauFric = (this.driveMotor.kt * DRIVE.gear * DRIVE.kS) / this.driveMotor.r;
    let total = 0, slip = false, effort = 0;
    const com = this.toWorld(ROBOT.com[0], ROBOT.com[1], ROBOT.com[2]);
    for (let k = 0; k < 4; k++) {
      const m = this.modules[k];
      const sp = out.drive ? out.drive[k] : null;
      total += this.steerStep(m, sp ? sp.angle : null, vb);
      // ---- suspension: ray down from the module mount
      const mount = this.toWorld(m.px, ROBOT.moduleMountY, m.pz);
      const down = this.toWorld(m.px, ROBOT.moduleMountY - 1, m.pz);
      const dir = { x: down[0] - mount[0], y: down[1] - mount[1], z: down[2] - mount[2] };
      const hit = this.eng.castDown(mount[0], mount[1], mount[2], 0.3, dir);
      const rest = ROBOT.moduleMountY + 0.004;
      m.contact = !!hit && hit.timeOfImpact < rest;
      const friction = tauFric * Math.max(-1, Math.min(1, m.ww / 0.5));
      if (!m.contact || !hit) {
        // airborne: the wheel spins freely under its motor
        const dr = this.wheelDrive(m, sp ? sp.speed : null, vb);
        m.ww += ((dr.tau - friction) / Jw) * STEP;
        m.wheel = m.ww * r; m.N = 0; m.comp = 0; m.slip = false; m.current = dr.i; m.supply = dr.supplyA; total += dr.supplyA;
        continue;
      }
      const comp = rest - hit.timeOfImpact;
      const cp = [mount[0] + dir.x * hit.timeOfImpact, mount[1] + dir.y * hit.timeOfImpact, mount[2] + dir.z * hit.timeOfImpact];
      const nrm = hit.normal;
      // velocity of the contact point
      const rx = cp[0] - this.p[0], ry = cp[1] - this.p[1], rz = cp[2] - this.p[2];
      const pvx = v.x + w.y * rz - w.z * ry, pvy = v.y + w.z * rx - w.x * rz, pvz = v.z + w.x * ry - w.y * rx;
      const vn = pvx * nrm.x + pvy * nrm.y + pvz * nrm.z;
      const N = Math.max(0, ROBOT.springK * comp - ROBOT.springC * vn);
      m.N = N; m.comp = comp;
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
      // a rolling wheel turns with the ground; a slipping one keeps its own speed
      if (!m.slip) m.ww = vl / r;
      const dr = this.wheelDrive(m, sp ? sp.speed : null, vb);
      m.current = dr.i; m.supply = dr.supplyA; total += dr.supplyA;
      let Fl = (dr.tau - tauFric * Math.max(-1, Math.min(1, m.ww / 0.5))) / r / (1 + kRot);
      // tread scrub resists sideways sliding: the impulse that would stop this contact point's
      // sideways motion (its effective mass includes the arm to the center of mass, which sits
      // above the floor), shared by the four wheels, all within this wheel's friction circle
      const ax = cp[0] - com[0], ay = cp[1] - com[1], az = cp[2] - com[2];
      const cx = ay * tz - az * ty, cy = az * tx - ax * tz, cz = ax * ty - ay * tx;
      const kInv = 1 / ROBOT.mass + cx * cx / ROBOT.inertia[0] + cy * cy / ROBOT.inertia[1] + cz * cz / ROBOT.inertia[2];
      let Ft = -(vt / kInv) / STEP * 0.22;
      const onHdpe = this.floorAt(cp[0], cp[2]) > 0.01;
      const mu = onHdpe ? ROBOT.muHdpe : ROBOT.mu;
      const muK = onHdpe ? ROBOT.muHdpe * 0.88 : ROBOT.muSlide;
      const F = Math.hypot(Fl, Ft);
      if (!m.slip) {
        if (F > mu * N) { m.slip = true; slip = true; }
      } else if (Math.hypot(m.ww * r - vl, vt) < 0.15 && F <= mu * N) { m.slip = false; m.ww = vl / r; }
      else slip = true;
      if (m.slip) {
        // kinetic friction opposes the sliding of the tread over the carpet (along the demand while it has barely started)
        const sl = m.ww * r - vl, sm = Math.hypot(sl, vt);
        if (sm > 0.15) { Fl = (muK * N * sl) / sm; Ft = (-muK * N * vt) / sm; }
        else if (F > 0) { Fl *= (muK * N) / F; Ft *= (muK * N) / F; }
        m.ww += ((dr.tau - tauFric * Math.max(-1, Math.min(1, m.ww / 0.5)) - Fl * r) / Jw) * STEP;
      }
      m.wheel = m.slip ? m.ww * r : vl;
      effort += Math.abs(dr.i) / DRIVE.statorLimit / 4;
      const fx = Fl * ux + Ft * tx + N * nrm.x, fy = Fl * uy + Ft * ty + N * nrm.y, fz = Fl * uz + Ft * tz + N * nrm.z;
      b.applyImpulseAtPoint({ x: fx * STEP, y: fy * STEP, z: fz * STEP }, { x: cp[0], y: cp[1], z: cp[2] }, true);
    }
    this.slipping = slip;
    this.effort = effort;
    return total;
  }

  // ------------------------------------------------------------------ four-bar
  /** world→robot frame rotation of a vector (no translation) */
  private rotateInv(x: number, y: number, z: number): [number, number, number] {
    const a = this.toLocal(this.p[0] + x, this.p[1] + y, this.p[2] + z);
    return [a[0], a[1], a[2]];
  }

  /**
   * Torque on the crank (N·m about the pivot, + toward deployed) from gravity on the intake's mass
   * and from the contact forces the field puts on the intake's hull (last step's solver impulses).
   * The coupler translates along the joint's circle, so a force F acting on it does F·∂P/∂φ.
   */
  private armTorque() {
    const a = this.intakeAngle * DEG, c = Math.cos(a), s = Math.sin(a);
    const dpz = JOINT_DY * c - JOINT_DZ * s, dpy = -JOINT_DY * s - JOINT_DZ * c;
    const g = this.rotateInv(0, -INTAKE.mass * G, 0);
    let q = g[2] * dpz + g[1] * dpy;
    const W = this.eng.world, hull = this.intakeHull;
    W.contactPairsWith(hull, (o) => {
      const surf = this.eng.surfaceOf.get(o.handle);
      if (!surf || surf === 'fuel' || o.parent()?.handle === this.body.handle) return;
      W.contactPair(hull, o, (man, flipped) => {
        let imp = 0;
        for (let i = 0; i < man.numContacts(); i++) imp += man.contactImpulse(i);
        if (imp <= 0) return;
        // the manifold's normal points from its first collider to its second; the force on the hull is along it or against it
        const n = man.normal(), f = ((flipped ? 1 : -1) * imp) / STEP;
        const l = this.rotateInv(n.x * f, n.y * f, n.z * f);
        q += l[2] * dpz + l[1] * dpy;
      });
    });
    return q;
  }

  /** The pivot motor against gravity and contacts; the rollers. */
  private updateArm(out: Outputs) {
    const off = this.disabled;
    const tau = this.armTorque() * DPHI_DTHETA;
    const before = this.pivot.pos;
    for (let k = 0; k < 4; k++) this.pivot.step(off ? null : out.pivotTarget, tau, Math.max(6, this.battery), STEP / 4);
    // the hopper can't fold up around more fuel than it holds at the stowed pose
    this.capacityBlocked = this.pivot.pos < before && capacityAt(this.pivot.arm) < this.load;
    if (this.capacityBlocked) { this.pivot.pos = before; this.pivot.vel = 0; }
    this.intakeAngle = this.pivot.arm;
  }

  /** The other motors; returns their supply current. */
  private mechanisms(out: Outputs) {
    const vb = Math.max(6, this.battery), off = this.disabled;
    let total = this.pivot.supplyA;
    this.roller.step(off ? null : out.rollerRps, vb, STEP);
    this.indexerMotor.step(off ? null : out.indexerRps, vb, STEP);
    this.hotdogMotor.step(off ? null : out.hotdogRps, vb, STEP);
    total += this.roller.supplyA + this.indexerMotor.supplyA + this.hotdogMotor.supplyA;
    for (const t of this.turrets) {
      const sp = off ? null : out.turrets[t.index];
      if (sp) { t.azGoal = sp.azimuthDeg; t.hoodGoal = sp.hoodDeg; t.rpsGoal = sp.rps; t.hood.command(sp.hoodDeg); } else t.rpsGoal = 0;
      t.az.step(sp ? sp.azimuthDeg : null, vb, STEP);
      t.hood.step(STEP);
      t.flywheel.step(sp ? sp.rps : null, vb, STEP);
      t.a = t.az.deg * DEG - t.base;
      total += t.az.supplyA + t.flywheel.supplyA;
    }
    return total;
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
    const surface = this.rollerSurface;
    // count what's on board and find balls at the roller
    const onboard: number[] = [];
    const L = this.tmp;
    const pull = surface > 1 && deployed && !this.disabled;
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
      if (rel[2] > surface * 0.25) continue;
      this.captured.push({ i, s: 0, x: Math.max(-0.26, Math.min(0.26, L[0])), phase: 'pinch', idle: 0 });
      f.colliders[i].setCollisionGroups(groups(GROUP.BALL, GROUP.FIELD | GROUP.BALL));
      this.onIntake?.(i);
    }
    this.onboard = onboard;
    // captured balls ride the path: spring to the path point + drive along it, friction-limited
    const path = INTAKE_PATH;
    const len = pathLen(path);
    const ride = surface * INTAKE.feedRatio;
    for (let k = this.captured.length - 1; k >= 0; k--) {
      const c = this.captured[k];
      const b = f.bodies[c.i];
      if (f.st[c.i] !== BS.FREE && f.st[c.i] !== BS.FLYING) { this.captured.splice(k, 1); continue; }
      // rollers stopped (trigger released) with a ball between them: it drops out
      c.idle = Math.abs(surface) < 0.5 ? c.idle + STEP : 0;
      if (c.idle > 0.4) { this.releaseCapture(c); this.captured.splice(k, 1); continue; }
      // the next ball can't overtake the one ahead
      const ahead = this.captured.find((o) => o !== c && o.s > c.s && Math.abs(o.x - c.x) < 2 * R);
      let ds = ride * STEP;
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
      const jmax = INTAKE.rollerGrip * INTAKE.pinchForce * STEP;
      const jl = Math.hypot(jx, jy, jz);
      if (jl > jmax) { jx *= jmax / jl; jy *= jmax / jl; jz *= jmax / jl; }
      b.applyImpulse({ x: jx, y: jy, z: jz }, true);
      if (lag > 0.25) { this.releaseCapture(c); this.captured.splice(k, 1); continue; }
      if (c.s >= len - 1e-3 && surface > 0.5) {
        // over the secondary roller: it drops into the hopper moving backward
        this.releaseCapture(c);
        const back = this.velToWorld(0, 0.2, -1.4);
        b.setLinvel({ x: pv[0] + back[0], y: pv[1] + back[1], z: pv[2] + back[2] }, true);
        this.captured.splice(k, 1);
      } else if (c.s <= 0 && surface < -0.5) {
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

  // ------------------------------------------------------------------ hotdog and indexer
  private static fixedLen = (() => { const p = FEED_FIXED(1); let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1], p[k][2] - p[k - 1][2]); return L; })();
  private static turretLen = (() => { const p: [number, number][] = [[0, 0.34 - 0.321], ...FEED_TURRET]; let L = 0; for (let k = 1; k < p.length; k++) L += Math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1]); return L; })();

  private indexer() {
    const f = this.fuel;
    const floor = (INDEXER.floorSpeed * this.hotdogMotor.rps) / INDEXER.hotdogRps;
    const belt = (INDEXER.beltSpeed * this.indexerMotor.rps) / INDEXER.indexerRps / 2;
    // the hotdog (the floor rollers) walks the pile toward the lane mouths, or away from them when it runs backward
    if (Math.abs(floor) > 0.02) {
      for (const i of this.onboard) {
        const o = i * 3;
        const L = this.toLocal(f.pos[o], f.pos[o + 1], f.pos[o + 2]);
        if (L[1] > 0.36) continue;
        const b = f.bodies[i];
        const pv = this.pointVel(L[0], L[1], L[2]);
        // mecanum side rollers at the back walk fuel toward the nearer lane mouth (a ball dead
        // center goes to the lane with fewer balls)
        const toward = Math.abs(L[0]) > 0.13 ? -Math.sign(L[0]) : L[0] > 0.01 ? 1 : L[0] < -0.01 ? -1 : this.turrets[0].lane.length <= this.turrets[1].lane.length ? 1 : -1;
        const want = this.velToWorld(toward * 0.3 * (L[2] < 0.14 ? 1 : 0) * Math.sign(floor), 0, -floor);
        const m = f.mass[i];
        const k = (L[2] > 0.3 ? 25 : 6) * m * STEP;
        b.applyImpulse({ x: (want[0] + pv[0] - b.linvel().x) * k, y: 0, z: (want[2] + pv[2] - b.linvel().z) * k }, true);
      }
    }
    const end = Robot.fixedLen + Robot.turretLen;
    for (const t of this.turrets) {
      // take the next ball at this lane's mouth
      if (belt > 0.01 && (t.lane.length === 0 || t.lane[t.lane.length - 1].s > 2 * R + 0.01) && t.lane.length < 3) {
        const mouth = FEED_FIXED(t.side)[0];
        let best = -1, bd = 0.2;
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
      // the belts carry the lane's balls at the indexer motor's actual speed (backward when it unjams)
      for (let k = 0; k < t.lane.length; k++) {
        const lb = t.lane[k];
        const adv = belt * STEP * (lb.s > Robot.fixedLen ? 3 : 1);
        lb.s = adv >= 0 ? Math.min(lb.s + adv, k === 0 ? end : t.lane[k - 1].s - 2 * R - 0.004) : lb.s + adv;
        const p = this.feedPoint(t, lb, this.tmp);
        const wp = this.toWorld(p[0], p[1], p[2]);
        if (lb.s < -0.12) {
          // pushed back out of the lane's mouth: a free ball in the hopper again
          f.place(lb.i, wp);
          t.lane.splice(k--, 1);
          continue;
        }
        f.moveKinematic(lb.i, wp[0], wp[1], wp[2]);
      }
      const head = t.lane[0];
      if (head && head.s >= end - 1e-4) { t.lane.shift(); this.launch(t, head.i); }
    }
  }

  /** Robot-frame point at distance s along a turret's feed (s < 0: from where it was picked up to the mouth). */
  feedPoint(t: TurretUnit, b: LaneBall, out: number[]) {
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
  exitLocal(t: TurretUnit) {
    const yaw = t.base + t.a, f = FEED_TURRET[FEED_TURRET.length - 1];
    return [t.pivot[0] + Math.sin(yaw) * f[0], t.pivot[1] + f[1] + 0.02, t.pivot[2] + Math.cos(yaw) * f[0]];
  }
  exitWorld(t: TurretUnit) { const e = this.exitLocal(t); return this.toWorld(e[0], e[1], e[2]); }

  /** The flywheel throws the ball: speed and backspin from the wheel's actual surface speed when the ball reaches it. */
  private launch(t: TurretUnit, i: number) {
    const f = this.fuel;
    const exit = this.exitWorld(t);
    const u = t.flywheel.w * SHOOTER.wheelRadius;            // wheel surface speed
    const sp = SHOOTER.slip * u / 2 * (1 + this.rng.gauss() * SHOOTER.speedJitter) * Math.sqrt(FUEL.mass / f.mass[i]);
    const yaw = this.th + t.az.deg * DEG + this.rng.gauss() * SHOOTER.yawJitterDeg * DEG;
    // the hood's actual position (the servo may still be moving), elevation = 82° − hood angle
    const pitch = (SHOOTER.elevation - t.hood.deg + this.rng.gauss() * SHOOTER.pitchJitterDeg) * DEG;
    const dx = Math.sin(yaw) * Math.cos(pitch), dy = Math.sin(pitch), dz = Math.cos(yaw) * Math.cos(pitch);
    // the ball leaves with the exit point's velocity: the chassis' (and its rotation) plus the turning turret's
    const e = this.exitLocal(t);
    const pv = this.pointVel(e[0], e[1], e[2]);
    const pw = this.toWorld(t.pivot[0], t.pivot[1], t.pivot[2]);
    const rx = exit[0] - pw[0], rz = exit[2] - pw[2];
    const v = [dx * sp + pv[0] + t.az.w * rz, dy * sp + pv[1], dz * sp + pv[2] - t.az.w * rx];
    // backspin: rolling on the hood, axis horizontal and perpendicular to the shot
    const spin = SHOOTER.slip * u / (2 * R);
    const ax = Math.cos(yaw), az = -Math.sin(yaw);           // (dir × up) normalized → backspin axis
    f.launch(i, exit, v, [-ax * spin, 0, -az * spin]);
    // it leaves through the hood: clear of the robot's own panels for its first instants
    f.colliders[i].setCollisionGroups(groups(GROUP.BALL, GROUP.FIELD | GROUP.BALL));
    f.launchT[i] = 0.25;
    // the shot takes energy out of the wheel
    const ke = 0.5 * f.mass[i] * sp * sp + 0.5 * FUEL.inertiaFactor * f.mass[i] * R * R * spin * spin;
    t.flywheel.takeEnergy(ke / SHOOTER.energyEfficiency);
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
      const vCorner = Math.max(0.9, DRIVE.speedAt12V * (1 - turn / Math.PI) ** 1.6);
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
