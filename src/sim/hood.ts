// Turret hood, servo gear and flywheel visuals, driven from the physics Frame (see backend.ts).
// The CAD hood is a sector of toothed plate + curved shell that pivots on the flywheel shaft; the
// servo's 10T pinion (10 DP) drives the sector's teeth. Mechanism angles are in degrees, positive =
// the shell's far end rotating up (a flatter shot), measured from the CAD pose.

/**
 * Hood angle (deg, the team's scale, 13–29.85) at which the CAD hood is modelled. From the CAD: the
 * shell ends 25.2° above the shaft-to-shell axis, the ball leaves tangent to it, i.e. 64.8° above
 * horizontal, and the launch elevation is 82° − hood (config), so hood = 17.2°.
 * EDIT: measure the real hood against its servo pulse to confirm the zero.
 */
export const HOOD_CAD_DEG = 17.2;
/** sector pitch radius / pinion pitch radius: (0.2160 m shaft-to-pinion − 0.0127 m) / 0.0127 m, from the CAD */
export const HOOD_GEAR_RATIO = 16;
/** Visual flywheel speed = real speed / this: at 60 fps the real 300+ rad/s would strobe */
export const FLY_VIS_DIV = 16;

/** Hood mechanism angle (deg about its axis) for the hood's actual angle */
export const hoodMechDeg = (hoodDeg: number) => hoodDeg - HOOD_CAD_DEG;
/** The servo pinion turns the opposite way to the sector it drives */
export const gearMechDeg = (hoodDeg: number) => -(hoodDeg - HOOD_CAD_DEG) * HOOD_GEAR_RATIO;
/** Any angle (rad) as degrees in (−180, 180]: the turret mesh yaw is unwrapped (−380°…10°) but the rotation is periodic */
export const wrapDeg = (rad: number) => (Math.atan2(Math.sin(rad), Math.cos(rad)) * 180) / Math.PI;
