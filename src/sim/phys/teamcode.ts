// The team's robot code (repo Rebuilt26, branch insert-name-here), ported function by function to
// run at the roboRIO's 50 Hz. It sees the robot only through sensors (module states, gyro) and
// answers with setpoints (module velocity/angle, turret azimuth/hood/flywheel, intake pivot and
// rollers, indexer and hotdog), exactly like the Java: the motors, Talon loops and mechanisms that
// carry them out live in robot.ts and mechanisms.ts. Everything here is in the WPILib frame (blue
// origin, x toward the opponent, y left, angles CCW-positive, robot x forward), like the Java.
import { CODE_PERIOD, DRIVE, FERRY_WPI, HEADING_TAU, HUB_WPI, INDEXER, INTAKE, LOOKAHEAD, NEUTRAL_ZONE_WPI, ROBOT, SHOT_TABLE, SHOOTER, TURRET } from './config';
import { trapezoid, type ProfileState } from './motors';

const DEG = Math.PI / 180;
export const wrapPi = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** MathUtil.inputModulus */
export function inputModulus(input: number, min: number, max: number) {
  const modulus = max - min;
  const numMax = Math.trunc((input - min) / modulus);
  input -= numMax * modulus;
  const numMin = Math.trunc((input - max) / modulus);
  input -= numMin * modulus;
  return input;
}
/** MathUtil.applyDeadband (maxMagnitude 1) */
export function applyDeadband(value: number, deadband: number) {
  if (Math.abs(value) <= deadband) return 0;
  return value > 0 ? (value - deadband) / (1 - deadband) : (value + deadband) / (1 - deadband);
}

// ------------------------------------------------------------------ shot table: GetAdjustedShot.java
export interface Shot { hoodDeg: number; rps: number; tof: number }
const rows = SHOT_TABLE;
const lerpShot = (a: Shot, b: Shot, t: number): Shot => ({ hoodDeg: a.hoodDeg + (b.hoodDeg - a.hoodDeg) * t, rps: a.rps + (b.rps - a.rps) * t, tof: a.tof + (b.tof - a.tof) * t });

/**
 * GetAdjustedShot.getShotForDistance: linear between neighboring rows inside the table
 * (InterpolatingTreeMap), linear extrapolation from the first two or last two rows outside it.
 */
export function getShotForDistance(distance: number): Shot {
  const n = rows.length;
  const between = (a: number, b: number): Shot => lerpShot(rows[a], rows[b], (distance - rows[a].d) / (rows[b].d - rows[a].d));
  if (distance < rows[0].d) return between(0, 1);
  if (distance > rows[n - 1].d) return between(n - 2, n - 1);
  let k = 0;
  while (k < n - 2 && rows[k + 1].d <= distance) k++;
  return between(k, k + 1);
}

export interface ShootingParameters { turretAngle: number; hoodDeg: number; rps: number }

/**
 * GetAdjustedShot.getParameters: the turret's pose and velocity from the robot's, then 10
 * lookahead iterations of "where will the turret be when the ball arrives" against the table's
 * time of flight (+ 0.05 s of latency). Returns the robot-relative turret angle (rad) and the
 * table row at the lookahead distance.
 */
export function getParameters(pose: { x: number; y: number; theta: number }, vel: { vx: number; vy: number; omega: number }, target: [number, number], mount: [number, number]): ShootingParameters {
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  const offX = mount[0] * c - mount[1] * s, offY = mount[0] * s + mount[1] * c;
  const tx = pose.x + offX, ty = pose.y + offY;
  // getTurretFieldVelocity
  const tvx = vel.vx - vel.omega * offY, tvy = vel.vy + vel.omega * offX;
  let lx = tx, ly = ty;
  let dist = Math.hypot(target[0] - tx, target[1] - ty);
  for (let i = 0; i < LOOKAHEAD.iterations; i++) {
    const tof = getShotForDistance(dist).tof + LOOKAHEAD.extraLatency;
    lx = tx + tvx * tof; ly = ty + tvy * tof;
    dist = Math.hypot(target[0] - lx, target[1] - ly);
  }
  const shot = getShotForDistance(dist);
  const turretAngleField = Math.atan2(target[1] - ly, target[0] - lx);
  return { turretAngle: wrapPi(turretAngleField - pose.theta), hoodDeg: shot.hoodDeg, rps: shot.rps };
}

/**
 * Turret.selectSafeSetpointDegrees: the target angle plus a multiple of 360° that lies inside
 * [min, max] and is closest to the last setpoint (clamped into the range); the turret unwinds a
 * full turn when the target crosses the seam. With no candidate in range, the target is clamped.
 */
export function selectSafeSetpointDegrees(target: number, reference: number, min: number, max: number) {
  const ref = Math.max(min, Math.min(max, reference));
  let best = NaN, bestErr = Infinity;
  const lo = Math.ceil((min - target) / 360), hi = Math.floor((max - target) / 360);
  for (let k = lo; k <= hi; k++) {
    const cand = target + 360 * k;
    const err = Math.abs(cand - ref);
    if (err < bestErr) { best = cand; bestErr = err; }
  }
  return Number.isNaN(best) ? Math.max(min, Math.min(max, target)) : best;
}

// ------------------------------------------------------------------ swerve: WPILib kinematics + Module.java
export interface ModuleState { speed: number; angle: number }
/** module translations in the WPILib robot frame (x forward, y left), order FL, FR, BL, BR */
export const MODULE_XY: [number, number][] = ROBOT.modules.map(([left, front]) => [front, left]);
/** Drive.DRIVE_BASE_RADIUS */
export const DRIVE_BASE_RADIUS = Math.max(...MODULE_XY.map(([x, y]) => Math.hypot(x, y)));
const SUM_R2 = MODULE_XY.reduce((a, [x, y]) => a + x * x + y * y, 0);

/** ChassisSpeeds.discretize */
export function discretize(vx: number, vy: number, omega: number, dt: number) {
  const dx = vx * dt, dy = vy * dt, dth = omega * dt;
  const cosMinusOne = Math.cos(dth) - 1;
  const halfThetaByTan = Math.abs(cosMinusOne) < 1e-9 ? 1 - (dth * dth) / 12 : -((dth / 2) * Math.sin(dth)) / cosMinusOne;
  // translation part: (dx, dy) rotated by atan2(−dθ/2, halfThetaByTan), scaled by hypot(halfThetaByTan, dθ/2)
  const n = Math.hypot(halfThetaByTan, dth / 2);
  const cr = halfThetaByTan / n, sr = -(dth / 2) / n;
  const tx = (dx * cr - dy * sr) * n, ty = (dx * sr + dy * cr) * n;
  return { vx: tx / dt, vy: ty / dt, omega: dth / dt };
}

/** SwerveDriveKinematics: toSwerveModuleStates (a zero request keeps the last headings), desaturateWheelSpeeds, toChassisSpeeds */
export class Kinematics {
  readonly headings = [0, 0, 0, 0];
  toStates(vx: number, vy: number, omega: number): ModuleState[] {
    if (vx === 0 && vy === 0 && omega === 0) return this.headings.map((angle) => ({ speed: 0, angle }));
    return MODULE_XY.map(([x, y], i) => {
      const mx = vx - omega * y, my = vy + omega * x;
      this.headings[i] = Math.atan2(my, mx);
      return { speed: Math.hypot(mx, my), angle: this.headings[i] };
    });
  }
  static desaturate(states: ModuleState[], attainable: number) {
    const real = Math.max(...states.map((s) => Math.abs(s.speed)));
    if (real > attainable) for (const s of states) s.speed = (s.speed / real) * attainable;
  }
  static toChassis(states: ModuleState[]) {
    let vx = 0, vy = 0, w = 0;
    states.forEach((s, i) => {
      const x = s.speed * Math.cos(s.angle), y = s.speed * Math.sin(s.angle);
      vx += x / 4; vy += y / 4;
      w += (-MODULE_XY[i][1] * x + MODULE_XY[i][0] * y) / SUM_R2;
    });
    return { vx, vy, omega: w };
  }
}

/** SwerveModuleState.optimize, then cosineScale (Module.runSetpoint) */
export function optimizeAndScale(state: ModuleState, current: number): ModuleState {
  let { speed, angle } = state;
  if (Math.abs(wrapPi(angle - current)) > 90 * DEG) { speed = -speed; angle = wrapPi(angle + Math.PI); }
  speed *= Math.cos(wrapPi(angle - current));
  return { speed, angle };
}

/** DriveCommands.getLinearVelocityFromJoysticks: deadband on the magnitude, then squared */
export function stickToLinear(x: number, y: number) {
  const mag = applyDeadband(Math.hypot(x, y), DRIVE.deadband);
  const m = mag * mag, dir = Math.atan2(y, x);
  return { x: m * Math.cos(dir), y: m * Math.sin(dir) };
}
/** DriveCommands.joystickDrive's omega shaping: deadband, then squared with the sign kept */
export const stickToOmega = (w: number) => { const o = applyDeadband(w, DRIVE.deadband); return Math.sign(o) * o * o; };

/** ProfiledPIDController(kP 5, 0, kD 0.4, (8 rad/s, 20 rad/s²)) with continuous input, as in DriveCommands.joystickDriveAtAngle */
export class AngleController {
  private sp: ProfileState = { pos: 0, vel: 0 };
  private err = 0;
  reset(measurement: number) { this.sp = { pos: measurement, vel: 0 }; this.err = 0; }
  calculate(measurement: number, goal: number) {
    const g = inputModulus(goal - measurement, -Math.PI, Math.PI) + measurement;
    this.sp = { pos: inputModulus(this.sp.pos - measurement, -Math.PI, Math.PI) + measurement, vel: this.sp.vel };
    this.sp = trapezoid(CODE_PERIOD, this.sp, g, DRIVE.angleMaxVel, DRIVE.angleMaxAccel);
    const prev = this.err;
    this.err = inputModulus(this.sp.pos - measurement, -Math.PI, Math.PI);
    return DRIVE.angleKp * this.err + (DRIVE.angleKd * (this.err - prev)) / CODE_PERIOD;
  }
}

// ------------------------------------------------------------------ the robot code
/** WPILib ↔ sim frame (x_sim = x_wpi − FIELD_X0 …) lives in robot.ts; this layer only sees WPILib. */
export interface CodeInput {
  /** left stick in the WPILib field frame (x toward the opponent, y left), raw, length ≤ 1 */
  x: number; y: number;
  /** right stick X as the Java reads it (positive = clockwise), raw */
  rightX: number;
  /** held buttons */
  lt: boolean; rt: boolean; b: boolean; start: boolean; a: boolean;
  /** pressed since the last tick */
  rb: boolean; yPress: boolean; povDown: boolean;
}
export const NO_CODE_INPUT: CodeInput = { x: 0, y: 0, rightX: 0, lt: false, rt: false, b: false, start: false, a: false, rb: false, yPress: false, povDown: false };

export interface Sensors {
  enabled: boolean;
  /** true pose, WPILib frame; the estimate adds the heading error */
  x: number; y: number; theta: number;
  /** measured module states: wheel speed (m/s, signed) and module angle (rad) */
  modules: ModuleState[];
}
export interface TurretSetpoint { azimuthDeg: number; hoodDeg: number; rps: number }
export interface Outputs {
  /** per-module velocity/angle setpoints (after optimize and cosineScale); null = drive stopped (disabled) */
  drive: ModuleState[] | null;
  /** null = Turret.stop() (disabled) */
  turrets: (TurretSetpoint | null)[];
  /** Intake: pivot Motion Magic target (motor rotations) and roller velocity (rps) */
  pivotTarget: number; rollerRps: number;
  indexerRps: number; hotdogRps: number;
}

/** Runs the Java at 50 Hz: RobotContainer + DefaultModeBindings + the subsystems' commands. */
export class RobotCode {
  /** Drive.getRotation() = the pose estimate's heading minus the truth (rad) */
  headingErr = 0;
  /** Turret.lockedIn (D-pad down toggles it) */
  locked = false;
  /** the neutral-zone Trigger is active: both turrets target the closer ferry target */
  ferry = false;
  /** Intake.isIntaking / Indexer.isFeedingForward → Drive.getMaxLinearSpeedMetersPerSec */
  isIntaking = false;
  private pivotSetpoint = INTAKE.upPos;
  private rollerSpeed = 0;
  private indexerSpeed = 0;
  private hotdogSpeed = 0;
  /** Azimuth.lastSetpointDeg per turret: they start where the operator placed them (−180°, pointing backward) */
  private lastSetpoint = [TURRET.startupDeg, TURRET.startupDeg];
  private prev = NO_CODE_INPUT;
  private kin = new Kinematics();
  private angle = new AngleController();
  private ltCommand = false;
  private indexerOwner: 'rt' | 'b' | null = null;
  private reverseCmd = false;

  get isFeedingForward() { return this.indexerSpeed > 0 && this.hotdogSpeed > 0; }
  /** Drive.getMaxLinearSpeedMetersPerSec (teleop is always enabled in the sim) */
  get maxLinearSpeed() { return this.isIntaking || this.isFeedingForward ? DRIVE.reducedSpeed : DRIVE.speedAt12V; }
  get maxAngularSpeed() { return this.maxLinearSpeed / DRIVE_BASE_RADIUS; }
  get pivotTarget() { return this.pivotSetpoint; }

  /** Robot code restart: the intake is stowed (upPos), the turrets sit at their startup pose, nothing runs. */
  reset(theta: number) {
    this.headingErr = 0; this.locked = false; this.ferry = false; this.isIntaking = false;
    this.pivotSetpoint = INTAKE.upPos; this.rollerSpeed = 0; this.indexerSpeed = 0; this.hotdogSpeed = 0;
    this.lastSetpoint = [TURRET.startupDeg, TURRET.startupDeg];
    this.prev = NO_CODE_INPUT; this.kin = new Kinematics(); this.angle.reset(theta);
    this.ltCommand = false; this.indexerOwner = null; this.reverseCmd = false;
  }
  /** The intake is set to a pose directly (tests, respawn): the Talon's setpoint follows. */
  setPivot(down: boolean) { this.pivotSetpoint = down ? INTAKE.downPos : INTAKE.upPos; }

  /** One scheduler iteration (button loop, then the commands' execute()). */
  tick(inp: CodeInput, s: Sensors): Outputs {
    const prev = this.prev;
    this.prev = inp;
    this.headingErr *= Math.exp(-CODE_PERIOD / HEADING_TAU);
    if (!s.enabled) {
      // the scheduler cancels the commands when the robot disables: Intake.stopIntake runs, the rest just stops
      if (this.ltCommand) { this.ltCommand = false; this.isIntaking = false; this.rollerSpeed = 0; }
      this.indexerOwner = null; this.reverseCmd = false;
      this.indexerSpeed = 0; this.hotdogSpeed = 0; this.rollerSpeed = 0;
      this.ferry = false;
      return { drive: null, turrets: [null, null], pivotTarget: this.pivotSetpoint, rollerRps: 0, indexerRps: 0, hotdogRps: 0 };
    }
    const theta = wrapPi(s.theta + this.headingErr);
    // measured robot-relative speeds, as Drive.getRobotRelativeSpeeds()
    const meas = Kinematics.toChassis(s.modules);
    const ct = Math.cos(theta), st = Math.sin(theta);
    const field = { vx: meas.vx * ct - meas.vy * st, vy: meas.vx * st + meas.vy * ct, omega: meas.omega };

    // ---- buttons (EventLoop poll)
    // LT: IntakeCommand (requires the intake): extend + rollers at mintakeSpeed + 3 × speed, once
    if (inp.lt && !prev.lt) {
      this.ltCommand = true; this.reverseCmd = false;
      this.pivotSetpoint = INTAKE.downPos;
      this.rollerSpeed = INTAKE.rollerBase + 3 * Math.hypot(meas.vx, meas.vy);
      this.isIntaking = this.rollerSpeed > 0;
    }
    if (!inp.lt && prev.lt && this.ltCommand) { this.ltCommand = false; this.isIntaking = false; this.rollerSpeed = 0; }
    // RB: intake.retractCommand (runOnce, requires the intake): it interrupts a running IntakeCommand
    // (end() → stopIntake) or reverse command, which do not restart until their trigger is pressed again
    if (inp.rb) {
      if (this.ltCommand || this.reverseCmd) { this.ltCommand = false; this.reverseCmd = false; this.isIntaking = false; this.rollerSpeed = 0; }
      this.pivotSetpoint = INTAKE.upPos;
    }
    // Start + B: intake.reverseIntake while held (requires the intake, so it cancels the LT command)
    const rev = inp.start && inp.b, revPrev = prev.start && prev.b;
    if (rev && !revPrev) { this.reverseCmd = true; this.ltCommand = false; this.isIntaking = false; this.rollerSpeed = INTAKE.rollerUnjam; }
    if (!rev && revPrev && this.reverseCmd) { this.reverseCmd = false; this.rollerSpeed = 0; }
    // RT: startIndexer + startHotdog while held; B (without Start): both backward (both require the indexer)
    const bUnjam = inp.b && !inp.start, bUnjamPrev = prev.b && !prev.start;
    if (inp.rt && !prev.rt) { this.indexerOwner = 'rt'; this.indexerSpeed = INDEXER.indexerRps; this.hotdogSpeed = INDEXER.hotdogRps; }
    if (bUnjam && !bUnjamPrev) { this.indexerOwner = 'b'; this.indexerSpeed = -INDEXER.indexerRps; this.hotdogSpeed = -INDEXER.hotdogRps; }
    if (this.indexerOwner === 'rt' && !inp.rt) { this.indexerOwner = null; this.indexerSpeed = 0; this.hotdogSpeed = 0; }
    if (this.indexerOwner === 'b' && !bUnjam) { this.indexerOwner = null; this.indexerSpeed = 0; this.hotdogSpeed = 0; }
    // Y: drive.setPose(translation, Rotation2d.kZero): the estimate's heading becomes 0
    if (inp.yPress) this.headingErr = wrapPi(0 - s.theta);
    // D-pad down: Turret.toggleTurretMode
    if (inp.povDown) this.locked = !this.locked;
    // A: joystickDriveAtAngle starts: the angle controller resets to the estimated heading
    if (inp.a && !prev.a) this.angle.reset(wrapPi(s.theta + this.headingErr));

    // ---- the drive command: joystickDrive, or joystickDriveAtAngle while A is held
    const lin = stickToLinear(inp.x, inp.y);
    const maxLin = this.maxLinearSpeed;
    let omega: number;
    const est = wrapPi(s.theta + this.headingErr);
    if (inp.a) {
      const toHub = [HUB_WPI[0] - s.x, HUB_WPI[1] - s.y];
      // new Rotation2d(robotToHub).rotateBy(π): the BACK of the robot faces the HUB
      omega = this.angle.calculate(est, wrapPi(Math.atan2(toHub[1], toHub[0]) + Math.PI));
    } else omega = stickToOmega(-inp.rightX) * this.maxAngularSpeed;
    const sp = { vx: lin.x * maxLin, vy: lin.y * maxLin, omega };
    // fromFieldRelativeSpeeds(speeds, estimated rotation), runVelocity
    const c = Math.cos(est), sn = Math.sin(est);
    const rvx = sp.vx * c + sp.vy * sn, rvy = -sp.vx * sn + sp.vy * c;
    const d = discretize(rvx, rvy, sp.omega, CODE_PERIOD);
    const states = this.kin.toStates(d.vx, d.vy, d.omega);
    Kinematics.desaturate(states, DRIVE.speedAt12V);
    const drive = states.map((st2, i) => optimizeAndScale(st2, wrapPi(s.modules[i].angle)));

    // ---- turret commands (TurretTargeting): HUB by default, the closer ferry target in the neutral zone
    const inNeutral = s.x >= NEUTRAL_ZONE_WPI[0] && s.x <= NEUTRAL_ZONE_WPI[1];
    this.ferry = inNeutral;
    const pose = { x: s.x, y: s.y, theta: est };
    let target: [number, number] = HUB_WPI;
    if (inNeutral) {
      const [l, r] = FERRY_WPI;
      target = Math.hypot(s.x - l[0], s.y - l[1]) <= Math.hypot(s.x - r[0], s.y - r[1]) ? l : r;
    }
    const turrets = TURRET.mountsWpi.map((mount, i): TurretSetpoint => {
      const p = getParameters(pose, field, target, mount);
      const targetDeg = this.locked ? TURRET.lockedAngleRad / DEG : p.turretAngle / DEG;
      const setpoint = selectSafeSetpointDegrees(targetDeg, this.lastSetpoint[i], TURRET.min[i], TURRET.max[i]);
      this.lastSetpoint[i] = setpoint;
      return {
        azimuthDeg: setpoint,
        hoodDeg: Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, p.hoodDeg)),
        rps: Math.max(SHOOTER.minRps, Math.min(SHOOTER.maxRps, p.rps)),
      };
    });
    return { drive, turrets, pivotTarget: this.pivotSetpoint, rollerRps: this.rollerSpeed, indexerRps: this.indexerSpeed, hotdogRps: this.hotdogSpeed };
  }
}
