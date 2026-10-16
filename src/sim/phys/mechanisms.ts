// The robot's motors and Talon FX loops (no Rapier): turret azimuth, flywheels, hood servo, intake
// pivot and rollers, indexer and hotdog. Each takes the setpoint the robot code left in the
// Talon (held between the 50 Hz code ticks), runs the Talon's closed loop every step against the
// battery voltage and current limits, and integrates the mechanism.
import { INTAKE, SHOOTER, TURRET, type MotorSpec } from './config';
import { motorOf, stepMotor, trapezoid, type Motor, type ProfileState } from './motors';

const DEG = Math.PI / 180;
const TAU = Math.PI * 2;
/** Talon loops run at 1 kHz; the azimuth is stiff enough to want several sub-steps per 120 Hz step */
const AZ_SUBSTEPS = 4;

/**
 * One turret's azimuth: a Talon FX in PositionVoltage on CANcoder rotations (= turret rotations ×
 * gear), Slot0 kP and kD only, brake, current limits, soft limits 10° beyond the control range.
 */
export class Azimuth {
  /** turret angle relative to the robot (deg, unwrapped: WPILib convention, CCW positive) and speed (rad/s) */
  deg = TURRET.startupDeg;
  w = 0;
  current = 0;
  supplyA = 0;
  private motor: Motor = motorOf(TURRET.motor);
  constructor(readonly index: number) {}

  /** `target` in degrees, or null when the Talon is stopped (disabled, Turret.stop()). */
  step(target: number | null, vBus: number, dt: number) {
    const h = dt / AZ_SUBSTEPS;
    let supply = 0;
    for (let k = 0; k < AZ_SUBSTEPS; k++) {
      const fwd = this.deg >= TURRET.max[this.index] + TURRET.softMargin, rev = this.deg <= TURRET.min[this.index] - TURRET.softMargin;
      // PositionVoltage: kP × error (CANcoder rotations); the derivative term acts on the velocity
      const errRot = target === null ? 0 : ((target - this.deg) / 360) * TURRET.gear;
      const r = stepMotor({
        motor: this.motor, n: 1, gear: TURRET.motorGear, eta: 0.9, inertia: TURRET.inertia, w: this.w,
        v0: target === null ? 0 : TURRET.kP * errRot,
        kDamp: target === null ? 0 : (TURRET.kD * TURRET.gear) / TAU,
        // a soft limit disables the output in that direction
        vLo: rev ? 0 : -vBus, vHi: fwd ? 0 : vBus,
        stator: TURRET.stator, supply: TURRET.supply, vBus, tau: 0, friction: TURRET.friction, dt: h,
      });
      this.w = r.w; this.current = r.i; supply = Math.max(supply, r.supplyA);
      this.deg += (r.w / DEG) * h;
    }
    this.supplyA = supply;
  }
}

/**
 * One turret's flywheel: two Kraken X44 (leader + opposed follower) on a brass wheel, Talon
 * VelocityVoltage (kV feedforward + kP, kD on the error's derivative) with a 0.1 s closed-loop
 * voltage ramp, stator limit, coast when stopped.
 */
export class Flywheel {
  /** wheel speed (rad/s) */
  w = 0;
  current = 0;
  supplyA = 0;
  private v = 0;
  private motor: Motor = motorOf(SHOOTER.motor);

  /** `rps` setpoint, or null when stopped (coast). */
  step(rps: number | null, vBus: number, dt: number) {
    if (rps === null) {
      const r = stepMotor({ motor: this.motor, n: SHOOTER.motors, gear: SHOOTER.gear, eta: 1, inertia: SHOOTER.inertia, w: this.w, v0: 0, vLo: 0, vHi: 0, stator: SHOOTER.stator, supply: 1e3, vBus, tau: 0, friction: 0, coast: true, dt });
      this.w = Math.max(0, r.w); this.current = 0; this.supplyA = 0; this.v = 0;
      return;
    }
    const meas = this.w / TAU;
    const ramp = (12 / SHOOTER.rampPeriod) * dt;
    const r = stepMotor({
      motor: this.motor, n: SHOOTER.motors, gear: SHOOTER.gear, eta: 1, inertia: SHOOTER.inertia, w: this.w,
      v0: SHOOTER.kV * rps + SHOOTER.kP * (rps - meas), kAcc: SHOOTER.kD / TAU,
      vLo: Math.max(-vBus, this.v - ramp), vHi: Math.min(vBus, this.v + ramp),
      stator: SHOOTER.stator, supply: 1e3, vBus, tau: 0, friction: 0, dt,
    });
    this.w = Math.max(0, r.w); this.current = r.i; this.supplyA = r.supplyA; this.v = r.v;
  }
  get rps() { return this.w / TAU; }
  /** a shot takes energy out of the wheel */
  takeEnergy(joules: number) {
    const J = SHOOTER.inertia + SHOOTER.motors * this.motor.spec.rotor;
    this.w = Math.sqrt(Math.max(0, this.w * this.w - (2 * joules) / J));
  }
}

/**
 * The hood: a servo on the REV Servo Hub. The robot code writes a pulse width; the servo moves
 * toward it at a fixed rate and the hood angle follows its position (channel 2, the left turret,
 * has its pulse direction inverted).
 */
export class Hood {
  pulseCmd: number;
  /** servo position in µs */
  pos: number;
  constructor(private inverted: boolean) {
    this.pulseCmd = Hood.pulseFor(SHOOTER.hoodMin, inverted);
    this.pos = this.pulseCmd;
  }
  /** Hood.hoodAngleToPulseWidthUs */
  static pulseFor(deg: number, inverted: boolean) {
    const d = Math.max(SHOOTER.hoodMin, Math.min(SHOOTER.hoodMax, deg));
    let t = (d - SHOOTER.hoodMin) / (SHOOTER.hoodMax - SHOOTER.hoodMin);
    if (inverted) t = 1 - t;
    const p = Math.round(SHOOTER.pulseMin + t * (SHOOTER.pulseMax - SHOOTER.pulseMin));
    return Math.max(SHOOTER.pulseMin, Math.min(SHOOTER.pulseMax, p));
  }
  command(deg: number) { this.pulseCmd = Hood.pulseFor(deg, this.inverted); }
  step(dt: number) {
    const rate = ((SHOOTER.pulseMax - SHOOTER.pulseMin) / SHOOTER.servoTravel) * dt;
    this.pos += Math.max(-rate, Math.min(rate, this.pulseCmd - this.pos));
  }
  get deg() {
    let t = (this.pos - SHOOTER.pulseMin) / (SHOOTER.pulseMax - SHOOTER.pulseMin);
    if (this.inverted) t = 1 - t;
    return SHOOTER.hoodMin + t * (SHOOTER.hoodMax - SHOOTER.hoodMin);
  }
}

/** A Talon FX in VelocityVoltage on a roller, indexer or hotdog: kV + kP, current limits, coast. */
export class VelocityMotor {
  /** shaft speed (rad/s) */
  w = 0;
  current = 0;
  supplyA = 0;
  private motor: Motor;
  constructor(private cfg: { kP: number; kV: number; stator: number; supply: number; inertia: number; friction: number; eta: number }, spec: MotorSpec) {
    this.motor = motorOf(spec);
  }
  step(rps: number | null, vBus: number, dt: number) {
    const stopped = rps === null;
    const r = stepMotor({
      motor: this.motor, n: 1, gear: 1, eta: this.cfg.eta, inertia: this.cfg.inertia, w: this.w,
      v0: stopped ? 0 : this.cfg.kV * rps + this.cfg.kP * (rps - this.w / TAU),
      vLo: -vBus, vHi: vBus, stator: this.cfg.stator, supply: this.cfg.supply, vBus, tau: 0, friction: this.cfg.friction, dt,
    });
    this.w = r.w; this.current = r.i; this.supplyA = r.supplyA;
  }
  get rps() { return this.w / TAU; }
}

/**
 * The intake pivot: a Kraken X60 in MotionMagicVoltage on the four-bar's crank. Position is in
 * motor rotations; `arm` (deg) maps rotations linearly onto the CAD mechanism's −73…0°. The
 * caller supplies the load (gravity and contact forces, as a torque on the motor shaft) and gets
 * the new position back; the hard stops are at the stowed (0 rot) and deployed (downPos) ends.
 */
export class Pivot {
  /** motor position (rotations) and velocity (rot/s) */
  pos: number;
  vel = 0;
  current = 0;
  supplyA = 0;
  private profile: ProfileState;
  private motor: Motor = motorOf(INTAKE.motor);
  /** stowed at power-up */
  constructor(private inertia: number, private friction: number, startDown = false) {
    this.pos = startDown ? INTAKE.downPos : INTAKE.upPos;
    this.profile = { pos: this.pos, vel: 0 };
  }
  get arm() { return INTAKE.upDeg * (1 - this.pos / INTAKE.downPos); }
  /** set the pose directly (teleport) */
  place(down: boolean) { this.pos = down ? INTAKE.downPos : INTAKE.upPos; this.vel = 0; this.profile = { pos: this.pos, vel: 0 }; }

  /** `target` in motor rotations, or null when the Talon is disabled (brake, zero volts). `tau`: external torque on the shaft (N·m, + toward deployed). */
  step(target: number | null, tau: number, vBus: number, dt: number) {
    let v0 = 0;
    if (target === null) this.profile = { pos: this.pos, vel: this.vel };
    else {
      this.profile = trapezoid(dt, this.profile, target, INTAKE.mmVelocity, INTAKE.mmAccel);
      // MotionMagicVoltage: kS in the direction of the profile's velocity, kV × profile velocity, kP × position error, kG (Elevator_Static: constant)
      v0 = INTAKE.kS * Math.sign(this.profile.vel) + INTAKE.kV * this.profile.vel + INTAKE.kP * (this.profile.pos - this.pos) + INTAKE.kG;
    }
    const r = stepMotor({
      motor: this.motor, n: 1, gear: 1, eta: 0.85, inertia: this.inertia, w: this.vel * TAU,
      v0, vLo: -vBus, vHi: vBus, stator: INTAKE.pivotStator, supply: INTAKE.pivotSupply, vBus, tau, friction: this.friction, dt,
    });
    this.vel = r.w / TAU; this.current = r.i; this.supplyA = r.supplyA;
    this.pos += this.vel * dt;
    if (this.pos < INTAKE.upPos) { this.pos = INTAKE.upPos; this.vel = Math.max(0, this.vel); }
    if (this.pos > INTAKE.downPos) { this.pos = INTAKE.downPos; this.vel = Math.min(0, this.vel); }
  }
}
