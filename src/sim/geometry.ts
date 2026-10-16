// Chuck Norris as the simulation sees it, measured from the CAD (scripts/measure.mjs prints the
// boxes). Robot frame: drivetrain center at the origin, floor at y = 0, front (intake) = +z, meters.

export const DEG = Math.PI / 180;

/** Bumpers: the robot's outline at floor level. */
export const BUMPER = { hx: 0.442, hz: 0.43, y0: 0.037, y1: 0.164 };
/** Everything above the bumpers (frame, hopper walls, turrets): what falling fuel lands on. */
export const BODY = { hx: 0.36, z0: -0.35, z1: 0.43, top: 0.55 };

export const MASS = 61;           // kg: 115 lb robot + battery + bumpers
export const INERTIA = 6.2;       // kg·m² about the vertical axis
export const WHEEL_R = 0.0508;    // 4" tread
export const MU = 1.15;           // tread on carpet (static), sliding a bit less
export const MU_SLIDE = 0.95;
/** Swerve module contact points (x, z). */
export const MODULES: [number, number][] = [[0.289, 0.276], [-0.289, 0.276], [0.289, -0.276], [-0.289, -0.276]];

// Kraken X60 on an SDS MK4i (L3, 6.12:1): 5.2 m/s free speed, 60 A stator limit.
export const DRIVE = {
  freeSpeed: 5.21,                // m/s at the wheel, 12 V
  stallForce: 786,                // N per module at 12 V
  currentForce: 129,              // N per module at the stator current limit
  vmax: 4.7,                      // m/s the driver / autopilot asks for (15.4 ft/s)
  wmax: 10,                       // rad/s
  steerRate: 20,                  // rad/s module steering
};

// ---------------------------------------------------------------- intake four-bar
// In the robot's side view: (z, y). Crank pivot and the crank's far joint (coupler), from the CAD.
export const PIVOT: [number, number] = [0.2094, 0.2164];
const JOINT: [number, number] = [0.4521, 0.2913];
export const INTAKE_UP = -73;     // degrees (0 = deployed on the floor)

/** Rotation of a side-view point about the crank pivot by the intake angle (deg). */
export function crankPoint(z: number, y: number, deg: number): [number, number] {
  const a = deg * DEG, c = Math.cos(a), s = Math.sin(a);
  const dz = z - PIVOT[0], dy = y - PIVOT[1];
  // same rotation as the viewer's (about +x): y' = y·c − z·s, z' = y·s + z·c
  return [PIVOT[0] + dy * s + dz * c, PIVOT[1] + dy * c - dz * s];
}
/** The coupler (rollers, kicker, front wall) translates with the crank's far joint. */
export function couplerOffset(deg: number): [number, number] {
  const [z, y] = crankPoint(JOINT[0], JOINT[1], deg);
  return [z - JOINT[0], y - JOINT[1]];
}

export interface Bar { z: number; y: number; r: number; link: 'fixed' | 'crank' | 'coupler' }
/**
 * Rollers, shafts and bars that shape the hopper, as cylinders across the robot (side view):
 * the inclined bed of floor rollers, the two hopper rollers on the powered pivot bar, and the
 * secondary roller, bars and front wall that ride the coupler. When the intake stows, the crank's
 * rollers stand up into a wall and the coupler lifts the front of the hopper.
 */
export const BARS: Bar[] = [
  { z: 0.040, y: 0.126, r: 0.026, link: 'fixed' },   // feed wheels into the indexer
  { z: 0.101, y: 0.158, r: 0.016, link: 'fixed' },   // floor rollers…
  { z: 0.155, y: 0.187, r: 0.016, link: 'fixed' },
  { z: 0.209, y: 0.216, r: 0.016, link: 'fixed' },
  { z: 0.2825, y: 0.239, r: 0.016, link: 'crank' },  // second hopper roller
  { z: 0.356, y: 0.2615, r: 0.016, link: 'crank' },  // first hopper roller
  { z: 0.4395, y: 0.252, r: 0.007, link: 'coupler' },
  { z: 0.452, y: 0.2915, r: 0.035, link: 'coupler' }, // secondary intake roller
  { z: 0.598, y: 0.355, r: 0.008, link: 'coupler' },
  { z: 0.630, y: 0.256, r: 0.013, link: 'coupler' },  // crash bar
  { z: 0.608, y: 0.1675, r: 0.031, link: 'coupler' }, // intake roller
];
/** The intake's front roller (side view, deployed) and its width. */
export const ROLLER = { z: 0.608, y: 0.1675, r: 0.031, hx: 0.295 };
/** The hopper's front wall rides the coupler: z, and the y range it covers (deployed). */
export const FRONT_WALL = { z: 0.642, y0: 0.244, y1: 0.549 };
export const HOPPER = { hx: 0.327, back: -0.018, floor: 0.12, top: 0.72 };
/** Fuel the hopper holds with the intake deployed / stowed. */
export const CAPACITY_DOWN = 48, CAPACITY_UP = 32;
export const capacityAt = (deg: number) => Math.round(CAPACITY_DOWN + (CAPACITY_UP - CAPACITY_DOWN) * Math.min(1, Math.max(0, deg / INTAKE_UP)));

/**
 * The path a ball takes once the intake roller grabs it (side view, relative to the coupler):
 * pinched under the roller, up the polycarb kicker ramp, over the secondary roller, and dropped
 * into the hopper.
 */
export const INTAKE_PATH: [number, number][] = [[0.585, 0.12], [0.54, 0.235], [0.49, 0.40], [0.40, 0.45]];

// ---------------------------------------------------------------- indexer and turrets
/** Turret pivots (x, y, z) and the direction each shoots at 0° (robot frame, from the flywheel shaft). */
export const TURRETS = [
  { id: 'turret-left', pivot: [0.197, 0.321, -0.184] as [number, number, number], base: Math.atan2(0.8, -0.6) },
  { id: 'turret-right', pivot: [-0.197, 0.321, -0.184] as [number, number, number], base: Math.atan2(-0.8, -0.6) },
];
/**
 * Each turret's feed: from the indexer mouth at the back of the hopper, back under the turret,
 * up through the turntable ring (fixed to the robot), then over the flywheel and out under the
 * hood (these last points turn with the turret: [forward, up] from the pivot).
 */
export const FEED_FIXED = (side: number): [number, number, number][] => [
  [side * 0.13, 0.215, 0.045],
  [side * 0.16, 0.225, -0.09],
  [side * 0.197, 0.24, -0.184],
  [side * 0.197, 0.34, -0.184],
];
export const FEED_TURRET: [number, number][] = [[-0.02, 0.11], [0.075, 0.20], [0.145, 0.225]];
/** Where a fed ball waits for the shot (distance along the feed). */
export const FLYWHEEL = {
  radius: 0.0508, inertia: 0.0012,
  /** motor torque at the current limit (Kraken X60, 40 A) and its free speed (rad/s) */
  torque: 0.78, stall: 7.09, free: 628,
  /** ball exit speed ≈ this × wheel surface speed (single wheel against a hood) */
  slip: 0.5,
};
