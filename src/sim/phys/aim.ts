// What a shot does once it leaves the shooter: the exit speed and backspin from the flywheel's
// speed, and a standalone flight of the ball through the same model the simulation uses (gravity,
// drag, Magnus lift from the backspin, spin decay). The team's shot table (config.ts SHOT_TABLE)
// is checked against it in tests/phys/teamcode.test.ts; the robot itself never calls `fly`.
import { FUEL, G, RHO, SHOOTER } from './config';

const R = FUEL.radius;
const KD = (0.5 * RHO * FUEL.cd * Math.PI * R * R) / FUEL.mass;
const KL = (0.5 * RHO * Math.PI * R * R) / FUEL.mass;

/** Exit speed and backspin for a flywheel speed (rps): a ball between the wheel and a fixed hood leaves at slip × (surface speed)/2, rolling on the hood. */
export function exitFor(rps: number) {
  const u = rps * 2 * Math.PI * SHOOTER.wheelRadius;
  return { speed: (SHOOTER.slip * u) / 2, spin: (SHOOTER.slip * u) / (2 * R) };
}

/**
 * Flies a shot in the vertical plane: returns horizontal distance and time each time the ball
 * crosses `yTarget` going down, or null if it never does.
 */
export function fly(speed: number, pitch: number, spin: number, y0: number, yTarget: number, dt = 1 / 240): { d: number; t: number; apex: number; path: [number, number][] } | null {
  let x = 0, y = y0, vx = speed * Math.cos(pitch), vy = speed * Math.sin(pitch), w = spin, t = 0, apex = y0;
  const path: [number, number][] = [[0, y0]];
  const decay = Math.exp(-dt / FUEL.spinTau);
  while (t < 6) {
    const sp = Math.hypot(vx, vy);
    let ax = -KD * sp * vx, ay = -G - KD * sp * vy;
    if (w > 1 && sp > 0.5) {
      const S = (R * w) / sp;
      const cl = FUEL.liftScale / (FUEL.liftA + FUEL.liftB / S);
      // backspin: lift is perpendicular to v, rotated +90° (up for a forward shot)
      ax += KL * cl * sp * -vy;
      ay += KL * cl * sp * vx;
    }
    const nvy = vy + ay * dt;
    const ny = y + nvy * dt;
    const nx = x + (vx + ax * dt) * dt;
    if (nvy < 0 && y >= yTarget && ny < yTarget) {
      const f = (y - yTarget) / (y - ny);
      path.push([x + (nx - x) * f, yTarget]);
      return { d: x + (nx - x) * f, t: t + dt * f, apex, path };
    }
    vx += ax * dt; vy = nvy; x = nx; y = ny; w *= decay; t += dt;
    apex = Math.max(apex, y);
    if ((t * 240) % 6 < 1) path.push([x, y]);
    if (y < -1) return null;
  }
  return null;
}
