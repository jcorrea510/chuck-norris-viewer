// Every physical constant in the simulation, in SI units (m, kg, s, N, rad). Each value is
// listed in overnight/PHYSICS.md with its source, confidence and the test that checks it.
// `EDIT:` marks an estimate the team should replace with a measurement from the real robot.

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
   * Rolling resistance on carpet: an extra deceleration while rolling, a constant (Coulomb-like,
   * foam pile deformation) plus a speed-proportional term. Calibrated so a 2 m/s roll stops in
   * ~2.0 m (EDIT: measure the roll-out on the practice carpet).
   */
  rollDecel: 0.72,               // m/s²
  rollDamp: 0.28,                // 1/s
  /** HDPE (BUMP tops) is much slicker */
  rollDecelHdpe: 0.18,
  /** contact time (s) for the visual squash: half-period of m on the ≈20 lbf/in spring rate (whitepaper fig. 7) */
  springRate: 3500,              // N/m (20 lbf/in)
};

// ------------------------------------------------------------------ robot (Chuck Norris)
export const ROBOT = {
  /** with battery and bumpers. EDIT: weigh the robot (R408 limit 61.2 kg with bumpers, plus battery) */
  mass: 61,
  /** center of mass above the floor, robot frame. EDIT: estimate from the CAD subsystem layout */
  com: [0, 0.17, -0.03] as [number, number, number],
  /** principal inertia (pitch about x, yaw about y, roll about z) kg·m². Box of the frame, mass low and central */
  inertia: [4.6, 6.2, 4.4] as [number, number, number],
  /** module contact points (x, z), from the CAD swerve module pivots */
  modules: [[0.289, 0.276], [-0.289, 0.276], [0.289, -0.276], [-0.289, -0.276]] as [number, number][],
  moduleMountY: 0.135,
  wheelRadius: 0.0508,           // 4 in tread
  /** suspension: tread + frame flex. ~4 mm static sag, well damped */
  springK: 38000,                // N/m per wheel
  springC: 1500,                 // N·s/m per wheel
  /** tread on carpet (static / sliding). EDIT: vendor blue nitrile ~1.1–1.2 */
  mu: 1.1, muSlide: 0.95,
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

// Kraken X60 (WCP datasheet, non-FOC): 6000 rpm free, 7.09 N·m stall, 366 A stall, 2 A free.
export const KRAKEN_X60 = { free: 628.3, stallTorque: 7.09, stallCurrent: 366, freeCurrent: 2, nominalV: 12 };

export const DRIVE = {
  /** SDS MK4i L3. EDIT: confirm the ratio on the robot */
  gear: 6.12,
  efficiency: 0.9,
  /** stator current limit per drive motor (A), like typical 2026 code. EDIT: from the team's code */
  statorLimit: 60,
  /** steering: max rate (rad/s) and P gain of the module angle loop */
  steerRate: 18, steerKp: 30,
  /** what the driver asks for at full stick (m/s, rad/s) */
  vmax: 4.7, wmax: 7.5,
  /** driver input shaping like robot code: deadband, slew limits (per second) */
  deadband: 0.08, slewLinear: 9, slewAngular: 30,
};

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

export const INTAKE = {
  /** four-bar stow angle (deg) and motion limits */
  upDeg: -73, vmax: 230, accel: 1700,
  /** roller surface speed (m/s), and how hard it can pull a ball: friction μ × the pinch force
   *  (FUEL spring rate × ~1.4 cm compression under the roller ≈ 50 N) */
  rollerSpeed: 5.0, rollerGrip: 0.9, pinchForce: 50,
  /** speed a ball travels up the kicker ramp into the hopper */
  feedSpeed: 2.4,
};

export const INDEXER = {
  /** belt surface speed (m/s); the ball rolls between belt and guide at half of it. EDIT */
  beltSpeed: 2.4,
  /** floor rollers carry hopper fuel back to the indexer mouths (m/s) while shooting */
  floorSpeed: 0.9,
};

export const SHOOTER = {
  wheelRadius: 0.0508,           // 4 in flywheel. EDIT
  /** flywheel + shaft + motor rotor inertia (kg·m²). EDIT */
  inertia: 0.004,
  gear: 1,
  statorLimit: 60,
  /** a ball between a wheel (surface speed u) and a fixed hood leaves at u/2 with backspin u/(2r);
   *  foam compression and slip take a further share (calibrated to ~10–14 m/s exits at 4000–5500 rpm). EDIT */
  slip: 0.82,
  /** fraction of the wheel energy lost per shot that ends up in the ball (rest is foam hysteresis) */
  energyEfficiency: 0.55,
  /** shot-to-shot scatter from ball mass, compression and the release point */
  speedJitter: 0.012,            // 1σ, fraction
  yawJitterDeg: 0.6,             // 1σ
  pitchJitterDeg: 0.6,           // 1σ
  /** adjustable hood range (deg above horizontal) and its servo rate. EDIT */
  hoodMin: 45, hoodMax: 72, hoodRate: 120,
  /** velocity loop: proportional gain on wheel speed error (A per rad/s) and ready tolerance */
  kP: 0.6, readyTol: 0.03,
};

export const TURRET = {
  /** travel limits (rad from center), max speed (rad/s) and acceleration (rad/s²). EDIT */
  limit: Math.PI, vmax: 9, accel: 45,
  /** position loop natural frequency (rad/s) and damping: settles in ~0.25 s */
  wn: 22, zeta: 0.85,
  /** the code only feeds when within this aim error */
  fireTolDeg: 2.5,
};

export const NET = {
  /** spring-damper barrier (per ball): stiff enough to stop a 20 m/s ball within ~0.2 m, damped near critical */
  k: 600, c: 21, maxDepth: 0.3,
  /** fraction of the stiffness left while the net springs back (hysteresis). EDIT: film a net catch */
  unload: 0.08,
};
