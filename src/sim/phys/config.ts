// Every physical constant in the simulation, in SI units (m, kg, s, N, rad). Each value is
// listed in overnight/PHYSICS.md with its source, confidence and the test that checks it.
// `EDIT:` marks an estimate the team should replace with a measurement from the real robot.
// Values that cite a Java file come from the team's robot code (repo Rebuilt26, branch
// insert-name-here, src/main/java/frc/robot/...).

export const G = 9.81;
/** air density at sea level, 20 °C (kg/m³) */
export const RHO = 1.2;

/** 120 Hz with CCD on every ball: chosen by benchmark (240 Hz cost twice the time for no visible
 *  difference; every spring in the model has ω·dt < 0.6, comfortably stable). See PHYSICS.md. */
export const STEP_HZ = 120;
export const STEP = 1 / STEP_HZ;

// ------------------------------------------------------------------ FUEL
export const FUEL = {
  radius: 0.075,                 // 5.91 in, game manual 5.10
  mass: 0.2155,                  // kg, whitepaper production boxes average 215.1–218.0 g
  massSpread: 0.0035,            // kg, 1σ (whitepaper Table 4 spread)
  /** solid foam sphere: I = 2/5 m r² */
  inertiaFactor: 0.4,
  /** drag coefficient, smooth sphere at Re ≈ 1e5 (FUEL at 10 m/s: Re = ρvd/μ ≈ 1.0e5) */
  cd: 0.49,
  /**
   * Magnus lift: C_L = 1 / (a + b / S), S = rω/v. Nathan (2008), "The effect of spin on the flight
   * of a baseball", Am. J. Phys. 76(2), a = 2.32, b = 0.4. EDIT: no published fit for this foam
   * ball; scale factor 0.85 because its surface is smoother than a seamed baseball.
   */
  liftA: 2.32, liftB: 0.4, liftScale: 0.85,
  /** spin decays with time constant τ (s) from air torque. EDIT: estimate, "a few seconds" */
  spinTau: 2.2,
  /**
   * Restitution against each surface at the reference impact speed vRef (a 1 m drop = 4.43 m/s),
   * falling with impact speed as 1 − e(v) ∝ v^(1/5) (viscoelastic contact, Kuwabara & Kono 1987).
   * Carpet: a 1 m drop rebounds ~0.25 m (e = 0.50). EDIT: drop-test a FUEL on the practice carpet.
   */
  vRef: 4.43,
  restitution: { carpet: 0.50, hdpe: 0.56, polycarb: 0.60, steel: 0.58, fuel: 0.45, bumper: 0.30, robot: 0.40, net: 0.08 },
  friction: { carpet: 0.85, hdpe: 0.35, polycarb: 0.30, steel: 0.35, fuel: 0.75, bumper: 0.70, robot: 0.45, net: 0.6 },
  /**
   * Rolling resistance on carpet (foam pile deformation): an extra deceleration while rolling, a
   * constant (Coulomb-like, foam pile deformation) plus a speed-proportional term. Calibrated so a
   * 2 m/s roll stops in ~2.0 m (EDIT: measure the roll-out on the practice carpet).
   */
  rollDecel: 0.72,               // m/s²
  rollDamp: 0.28,                // 1/s
  /** HDPE (BUMP tops) is much slicker */
  rollDecelHdpe: 0.18,
  /** contact time (s) for the visual squash: half-period of m on the ≈20 lbf/in spring rate (whitepaper fig. 7) */
  springRate: 3500,              // N/m (20 lbf/in)
};

// ------------------------------------------------------------------ motors
/**
 * DC motors from the WCP datasheets (docs.wcproducts.com, "trapezoidal" = non-FOC commutation,
 * which the drive requests use: ModuleIOTalonFX withEnableFOC(false)). `rotor` is the rotor
 * inertia (kg·m²). EDIT: WCP does not publish it; these are estimates from the rotor's size.
 */
export interface MotorSpec { free: number; stallTorque: number; stallCurrent: number; freeCurrent: number; rotor: number }
/** Kraken X60: 6000 rpm, 7.09 N·m, 366 A, 2 A */
export const KRAKEN_X60: MotorSpec = { free: 628.3, stallTorque: 7.09, stallCurrent: 366, freeCurrent: 2, rotor: 8e-5 };
/** Kraken X44: 7758 rpm, 4.11 N·m, 279 A, 3 A */
export const KRAKEN_X44: MotorSpec = { free: 812.4, stallTorque: 4.11, stallCurrent: 279, freeCurrent: 3, rotor: 3e-5 };

/** Constants.java loopPeriodSecs: the roboRIO runs the robot code at 50 Hz */
export const CODE_PERIOD = 0.02;

// ------------------------------------------------------------------ robot (Chuck Norris)
export const ROBOT = {
  /** Drive.java ROBOT_MASS_KG (PathPlanner config; includes battery and bumpers) */
  mass: 74.088,
  /** center of mass above the floor, robot frame. EDIT: estimate from the CAD subsystem layout */
  com: [0, 0.17, -0.03] as [number, number, number],
  /**
   * principal inertia (pitch about x, yaw about y, roll about z) kg·m². Yaw is Drive.java ROBOT_MOI
   * (6.883). EDIT: pitch and roll are the old box-of-the-frame estimates (4.6 and 4.4 at 61 kg)
   * scaled by the mass ratio 74.088/61.
   */
  inertia: [5.59, 6.883, 5.34] as [number, number, number],
  /** module contact points (x = robot left, z = robot front): TunerConstants kFrontLeftXPos/YPos = 10.875 in forward, 11.375 in left; order FL, FR, BL, BR */
  modules: [[0.288925, 0.276225], [-0.288925, 0.276225], [0.288925, -0.276225], [-0.288925, -0.276225]] as [number, number][],
  moduleMountY: 0.135,
  /** TunerConstants kWheelRadius = 1.94 in */
  wheelRadius: 0.049276,
  /** suspension: tread + frame flex. ~4 mm static sag at 74 kg, well damped. EDIT: estimate */
  springK: 46000,                // N/m per wheel
  springC: 1820,                 // N·s/m per wheel
  /** tread on carpet: static is Drive.java WHEEL_COF = 1.2; sliding is EDIT (0.865 of static, the old estimate's ratio) */
  mu: 1.2, muSlide: 1.04,
  /** tread on the HDPE BUMP ramps (orange-peel texture) */
  muHdpe: 0.75,
  /** heights from the CAD (scripts/measure.mjs), intake deployed / stowed */
  heightDown: 0.549, heightUp: 0.728,
  bumper: { hx: 0.441, hz: 0.429, y0: 0.037, y1: 0.164, depth: 0.085 },
  /** frame/superstructure against field elements: painted aluminum on steel/polycarb */
  hullFriction: 0.3, hullRestitution: 0.12,
  /** bumper cordura against walls and carpet */
  bumperFriction: 0.5, bumperRestitution: 0.2,
};

export const DRIVE = {
  /** TunerConstants kDriveGearRatio (SDS MK4i L2) and kSteerGearRatio */
  gear: 6.746031746031747,
  steerGear: 21.428571428571427,
  /** drive stage efficiency. EDIT: typical 90 % */
  efficiency: 0.9,
  /** TunerConstants driveInitialConfigs: stator 80 A, supply 30 A (kSlipCurrent 100 A is not applied by ModuleIOTalonFX) */
  statorLimit: 80, supplyLimit: 30,
  /** TunerConstants driveGains (VelocityVoltage, V per wheel rps): kP, kS, kV */
  kP: 0.1, kS: 0.2225, kV: 0.84381,
  /** TunerConstants kSpeedAt12Volts (m/s): desaturateWheelSpeeds and the full-stick speed */
  speedAt12V: 4.58,
  /** Constants.SubsystemConstants.Drive.reducedSpeed: the teleop cap while intaking or feeding forward */
  reducedSpeed: 2.5,
  /** TunerConstants steerGains (PositionVoltage, V per module rotation): kP, kD, kS (UseClosedLoopSign) */
  steerKp: 100, steerKd: 0.5, steerKs: 0.1,
  /** TunerConstants steerInitialConfigs: stator 40 A, supply 20 A */
  steerStator: 40, steerSupply: 20,
  /** TunerConstants kSteerInertia (kg·m² at the module) and kSteerFrictionVoltage = kDriveFrictionVoltage (V) */
  steerInertia: 0.01, frictionV: 0.2,
  /** DriveCommands.DEADBAND (stick magnitude and omega) */
  deadband: 0.1,
  /** DriveCommands.joystickDriveAtAngle: ANGLE_KP, ANGLE_KD, ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION */
  angleKp: 5.0, angleKd: 0.4, angleMaxVel: 8.0, angleMaxAccel: 20.0,
  /** wheel + tread inertia about the wheel axle (kg·m²), without the rotor. EDIT: weigh a wheel (0.15 kg, r 0.05 m, plus the bevel stage) */
  wheelInertia: 0.0015,
};

/** The pose estimate's heading error decays with this time constant (s) as the Limelights pull it back (Vision.java). EDIT: measure */
export const HEADING_TAU = 2.0;

export const BATTERY = {
  /** MK ES17-12 resting voltage, charged. EDIT */
  restV: 12.6,
  /** battery + main breaker + PDH wiring (Ω). EDIT */
  resistance: 0.018,
  /** roboRIO 2 brownout threshold (V) */
  brownout: 6.75,
  /** everything else (radio, roboRIO, compressorless) (A) */
  baseCurrent: 3,
};

// ------------------------------------------------------------------ field targets (WPILib frame)
/** WPILib (blue origin) ↔ sim: x_sim = x_wpi − FIELD_X0, z_sim = −(y_wpi − FIELD_Y0), th_sim = θ_wpi + π/2 (FieldConstants fieldLength / 2, fieldWidth / 2) */
export const FIELD_X0 = 16.5411785 / 2;
export const FIELD_Y0 = 8.0756125 / 2;
/** FieldConstants.hubGuy */
export const HUB_WPI: [number, number] = [4.6256194, 4.0346376];
/** getLeftFerryTarget / getRightFerryTarget: (hub.x − 3, hub.y ± 2) */
export const FERRY_WPI: [number, number][] = [[HUB_WPI[0] - 3, HUB_WPI[1] + 2], [HUB_WPI[0] - 3, HUB_WPI[1] - 2]];
/** isRobotInNeutralZone: neutralZoneMinX … neutralZoneMaxX (WPILib x, m) */
export const NEUTRAL_ZONE_WPI: [number, number] = [5.1816, 11.303];

// ------------------------------------------------------------------ turrets, shooter
export const TURRET = {
  /** robotToTurret1 (left) and robotToTurret2 (right) in the WPILib robot frame (x forward, y left): (−7.25 in, ±7.75 in) */
  mountsWpi: [[-7.25 * 0.0254, 7.75 * 0.0254], [-7.25 * 0.0254, -7.75 * 0.0254]] as [number, number][],
  /** azimuthGearRatio: CANcoder rotations per turret rotation (EDIT: assumed to be the motor reduction too) */
  gear: 10,
  /**
   * motor revolutions per turret revolution. EDIT: the code only gives the CANcoder's ratio (10).
   * At 10:1 a Kraken X44 turret with the team's gains overshoots a full unwind by 100° (through
   * its soft limit), which their robot evidently doesn't do; 20:1 is the lowest ratio that settles
   * a 360° unwind in 0.35 s without passing the target.
   */
  motorGear: 20,
  /** PositionVoltage Slot0, V per CANcoder rotation: azimuthKP, azimuthKD (azimuthKV multiplies a velocity setpoint of 0) */
  kP: 8, kD: 0.35,
  /** azimuth config: stator 60 A, supply 25 A */
  stator: 60, supply: 25,
  /** left/right control limits (deg, WPILib robot-relative): left|rightMin|MaxAzimuthControlAngle; the Talon soft limits sit azimuthSoftLimitMarginDeg beyond */
  min: [-380, -365], max: [1, 10], softMargin: 10,
  /** left|rightAzimuthStartupCenterDeg: where the turrets start (physically pointing backward) */
  startupDeg: -180,
  /** Turret.runZeroAzimuthTarget: new Rotation2d(-135) is −135 RADIANS (their code, reproduced) */
  lockedAngleRad: -135,
  /** turret rotating inertia about its axis (kg·m²). EDIT: estimate from the CAD assembly */
  inertia: 0.05,
  /** bearing friction (N·m). EDIT */
  friction: 0.03,
  /** azimuth motor: assumed Kraken X44 (photos show X44s on the turrets). EDIT */
  motor: KRAKEN_X44,
};

export const SHOOTER = {
  /** SimConstants.Fuel.EFFECTIVE_FLYWHEEL_RADIUS_METERS = 2 in */
  wheelRadius: 0.0508,
  /** flywheel + both rotors, per turret (kg·m²). EDIT: brass flywheel + two compliant wheels; weigh it */
  inertia: 0.004,
  /** Constants.flywheelGearRatio */
  gear: 1,
  /** flywheelConfig Slot0 (VelocityVoltage, V per rps): kP, kD (on the error's derivative), kV; VoltageClosedLoopRampPeriod 0.1 s; stator 60 A each */
  kP: 1.2, kD: 0.25, kV: 0.0925, rampPeriod: 0.1, stator: 60,
  /** two motors per turret (leader + opposed follower), assumed Kraken X44. EDIT */
  motors: 2, motor: KRAKEN_X44,
  /** min|maxFlywheelRps */
  minRps: 0, maxRps: 100,
  /**
   * The ball leaves at slip × (wheel surface speed)/2 with backspin v/R. slip is fitted: flying every
   * shot-table row through the sim's drag + Magnus model from the CAD exit height 0.566 m lands
   * them at their distances, rms 0.14 m (tests/phys/teamcode.test.ts). EDIT: confirm on the robot
   */
  slip: 0.91,
  /** launch elevation = this − hood angle (deg), fitted together with slip. EDIT: the hood's real exit angle */
  elevation: 82,
  /** fraction of the wheel energy lost per shot that ends up in the ball (rest is foam hysteresis) */
  energyEfficiency: 0.55,
  /** shot-to-shot scatter from ball mass, compression and the release point */
  speedJitter: 0.012,            // 1σ, fraction
  yawJitterDeg: 0.6,             // 1σ
  pitchJitterDeg: 0.6,           // 1σ
  /** min|maxHoodAngle (deg) */
  hoodMin: 13, hoodMax: 29.85,
  /** REV Servo Hub pulse range (µs), HOOD_SERVO_MIN|MAX_PULSE_US, and the servo's full-travel time (s). EDIT: measure the hood */
  pulseMin: 500, pulseMax: 2500, servoTravel: 0.6,
};

/** The team's shot table, src/main/deploy/shot_table.json: distance (m) from the turret to the target → hood (deg), flywheel (rps), time of flight (s) */
export const SHOT_TABLE: { d: number; hoodDeg: number; rps: number; tof: number }[] = [
  { d: 1.3, hoodDeg: 13, rps: 40.5, tof: 0.81 },
  { d: 2, hoodDeg: 13, rps: 44, tof: 1.01 },
  { d: 2.25, hoodDeg: 13, rps: 46.5, tof: 1.065 },
  { d: 2.5, hoodDeg: 13, rps: 48, tof: 1.155 },
  { d: 2.75, hoodDeg: 14, rps: 50, tof: 1.184 },
  { d: 3, hoodDeg: 15, rps: 51.5, tof: 1.163 },
  { d: 3.25, hoodDeg: 16, rps: 52.5, tof: 1.1767 },
  { d: 3.5, hoodDeg: 17, rps: 53, tof: 1.1483 },
  { d: 3.75, hoodDeg: 17, rps: 54, tof: 1.18 },
  { d: 4, hoodDeg: 18, rps: 55, tof: 1.245 },
  { d: 4.25, hoodDeg: 19.5, rps: 56, tof: 1.21 },
  { d: 4.5, hoodDeg: 21, rps: 58, tof: 1.25 },
  { d: 4.75, hoodDeg: 22.5, rps: 59, tof: 1.265 },
  { d: 5, hoodDeg: 23, rps: 60, tof: 1.332 },
  { d: 5.25, hoodDeg: 24, rps: 61.5, tof: 1.35 },
  { d: 5.5, hoodDeg: 24.5, rps: 62, tof: 1.315 },
  { d: 6, hoodDeg: 25.5, rps: 64.75, tof: 1.415 },
];
/** GetAdjustedShot: LOOKAHEAD_ITERATIONS, SHOT_EXTRA_LATENCY_SECS */
export const LOOKAHEAD = { iterations: 10, extraLatency: 0.05 };

// ------------------------------------------------------------------ intake, hopper, indexer
export const INTAKE = {
  /** Constants.Intake: pivot positions in motor rotations (downPos, upPos); mintakeSpeed and unjamSpeed (rps) */
  downPos: 12.75, upPos: 0, rollerBase: 70, rollerUnjam: -50,
  /** pivotConfig: Motion Magic cruise 60 rps and 120 rps²; Slot0 kP 15, kS 0.5, kV 0.12, kG 1 (Elevator_Static: a constant); stator 40 A, supply 20 A */
  mmVelocity: 60, mmAccel: 120, kP: 15, kS: 0.5, kV: 0.12, kG: 1, pivotStator: 40, pivotSupply: 20,
  /** intakeConfig: Slot0 kP 0.35, kV 0.105 (V per rps); stator 60 A, supply 30 A */
  rollerKp: 0.35, rollerKv: 0.105, rollerStator: 60, rollerSupply: 30,
  /** four-bar crank angle (deg) stowed (0 rot): deployed (downPos) is 0°, the CAD mechanism's convention */
  upDeg: -73,
  /** motor rotations per crank revolution = downPos × 360 / 73. EDIT: the real reduction */
  motorPerArm: 12.75 * 360 / 73,
  /** pivot and roller motors: Kraken X60. EDIT */
  motor: KRAKEN_X60,
  /** moving mass of the intake assembly at the coupler (frame, rollers, motor, kg) and pivot friction at the arm (N·m). EDIT: weigh it */
  mass: 4.5, armFriction: 0.4,
  /** roller surface speed per motor rps (m/s): 70 rps ≈ 5 m/s. EDIT: roller gearing */
  rollerSurface: 5.0 / 70,
  /** roller assembly inertia at the motor, without the rotor (kg·m²). EDIT */
  rollerInertia: 0.0006,
  /** a ball between the rollers rides the kicker ramp at this fraction of the roller surface speed; the rollers pull with μ × pinch force (FUEL spring rate × ~1.4 cm compression ≈ 50 N) */
  feedRatio: 0.48, rollerGrip: 0.9, pinchForce: 50,
};

export const INDEXER = {
  /** Constants.Indexer: indexerSpeed 55 rps, hotdogSpeed 70 rps (B unjams both backward: DefaultModeBindings) */
  indexerRps: 55, hotdogRps: 70,
  /** indexerConfig / hotdogConfig: Slot0 kP 0, kV 0.1285 / 0.11 (V per rps); stator 80 / 50 A, supply 40 / 30 A */
  indexerKv: 0.1285, hotdogKv: 0.11, indexerStator: 80, indexerSupply: 40, hotdogStator: 50, hotdogSupply: 30,
  /** Kraken X60 each. EDIT */
  motor: KRAKEN_X60,
  /**
   * load inertia at the motor (belts, rollers; kg·m²) and friction torque (N·m). EDIT: with kP 0
   * the speed under load is set by kV alone, so the team's kV values (0.1285 and 0.11) imply the
   * load: the friction is chosen so each motor runs at its commanded rps (Kraken X60 alone would
   * run 0.1177 V per rps)
   */
  inertia: 0.0008, indexerFriction: 0.35, hotdogFriction: 0.05,
  /** lane belt surface speed at indexerRps (m/s; the ball rides at half of it, 3× that inside the turret). EDIT: tuned to SimConstants.Fuel.SHOOT_BALLS_PER_SECOND_PER_TURRET = 4 balls/s with a full hopper */
  beltSpeed: 1.25,
  /** the hopper floor (hotdog rollers) walks fuel toward the lane mouths at this speed at hotdogRps (m/s). EDIT */
  floorSpeed: 0.9,
};

export const NET = {
  /** spring-damper barrier (per ball): stiff enough to stop a 20 m/s ball within ~0.2 m, damped near critical */
  k: 600, c: 21, maxDepth: 0.3,
  /** fraction of the stiffness left while the net springs back (hysteresis). EDIT: film a net catch */
  unload: 0.08,
};
