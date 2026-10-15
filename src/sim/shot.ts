// Ballistic solver for a hooded shooter: integrates the same flight model as the ball physics
// (gravity, quadratic drag, backspin lift) and finds the launch speed that drops the ball into
// the HUB funnel. Shooting on the move: the robot's velocity is added to the launch velocity and
// the aim point is led by it.
import { K_DRAG, K_LIFT } from './balls';

const G = 9.81;

export interface Shot { v: [number, number, number]; tof: number; speed: number; pitch: number }

/** Integrates a flight; returns the height when the ball reaches horizontal distance `d` (descending), or null. */
function heightAt(p0: number[], v0: number[], dirX: number, dirZ: number, d: number): { y: number; t: number; descending: boolean } | null {
  let x = 0, y = p0[1];
  let vx = v0[0] * dirX + v0[2] * dirZ; // speed along the aim direction
  let vy = v0[1];
  const dt = 1 / 240;
  for (let t = 0; t < 3; t += dt) {
    const sp = Math.hypot(vx, vy);
    const ux = vx / sp, uy = vy / sp;
    let lx = -uy * ux, ly = 1 - uy * uy;
    const ll = Math.hypot(lx, ly) || 1;
    lx /= ll; ly /= ll;
    vx += (-K_DRAG * sp * vx + K_LIFT * sp * sp * lx) * dt;
    vy += (-G - K_DRAG * sp * vy + K_LIFT * sp * sp * ly) * dt;
    const nx = x + vx * dt;
    if (nx >= d) {
      const f = (d - x) / (nx - x);
      return { y: y + vy * dt * f, t: t + dt * f, descending: vy < 0 };
    }
    x = nx;
    y += vy * dt;
    if (y < 0) return null;
  }
  return null;
}

/**
 * Launch velocity (field frame) from exit point `p` so the ball drops through `target`.
 * `robotV` is the robot's velocity, inherited by the ball. Hood angle comes from distance
 * (steeper up close), like a real adjustable hood.
 */
export function solveShot(p: number[], target: number[], robotV: [number, number]): Shot | null {
  let aimX = target[0], aimZ = target[2];
  let best: Shot | null = null;
  for (let iter = 0; iter < 3; iter++) {
    // relative to the moving robot the target slides by −v·t, so aim ahead of it
    const tof = best?.tof ?? 1;
    aimX = target[0] - robotV[0] * tof;
    aimZ = target[2] - robotV[1] * tof;
    const dx = aimX - p[0], dz = aimZ - p[2];
    const d = Math.hypot(dx, dz);
    if (d < 0.6) return null;
    const dirX = dx / d, dirZ = dz / d;
    const pitch = Math.max(46, Math.min(72, 72 - (d - 1.4) * 6.5)) * (Math.PI / 180);
    // bisection on launch speed: too slow falls short (or is still climbing), too fast flies over
    let lo = 3, hi = 16, shot: Shot | null = null;
    for (let k = 0; k < 18; k++) {
      const s = (lo + hi) / 2;
      const v0 = [Math.cos(pitch) * s * dirX, Math.sin(pitch) * s, Math.cos(pitch) * s * dirZ];
      const r = heightAt(p, v0, dirX, dirZ, d);
      // falls short (or below the rim): faster
      if (!r || (r.descending && r.y < target[1])) { lo = s; continue; }
      // above the target, or still climbing when it gets there: slower
      hi = s;
      if (r.descending) shot = { v: [v0[0] + robotV[0], v0[1], v0[2] + robotV[1]], tof: r.t, speed: s, pitch };
    }
    if (!shot) return best;
    best = shot;
  }
  return best;
}
