// Route planning on the field's navigation grid (field.ts, built from the real field model and
// inflated by the robot's footprint). Pure functions: they run in the planner Web Worker in the
// browser and inline in Node tests. Nothing here ever runs on the render thread.
import type { FieldGrid } from './field';

export const CLEAR = 0.5;                 // planning clearance (robot half-diagonal with margin)
export const CROSS_X = 3.62;              // each alliance's BUMPs and TRENCHes sit on x = ±CROSS_X
export const CROSS_OUT = 1.2;
export const BUMP_Z = 1.5;
export const BUMPER = { hx: 0.442, hz: 0.43 };

export interface Crossing { L: number; kind: 'trench' | 'bump'; z: number }
export interface RouteResult { path: [number, number][]; via: string; segHeading?: (number | null)[] }

/**
 * Plans from (sx, sz) to (gx, gz). Each alliance line it crosses goes through the lane chosen in
 * `cross` (a trench lane or over a bump), lined up in the open first so the deployed intake never
 * swings into the trench supports.
 */
export function computeRoute(f: FieldGrid, sx: number, sz: number, gx: number, gz: number, cross: Crossing[]): RouteResult | null {
  const lines = cross.filter((c) => (sx - c.L) * (gx - c.L) < 0).sort((a, b) => Math.abs(a.L - sx) - Math.abs(b.L - sx));
  if (!lines.length) {
    const path = f.plan(sx, sz, gx, gz, CLEAR);
    return path ? { path, via: '' } : null;
  }
  const out: [number, number][] = [];
  const segHeading: (number | null)[] = [];
  let x = sx, z = sz, via = '';
  for (const c of lines) {
    const dir = Math.sign(gx - c.L);
    const pre: [number, number] = [c.L - dir * (CROSS_OUT + 0.55), c.z], entry: [number, number] = [c.L - dir * CROSS_OUT, c.z], exit: [number, number] = [c.L + dir * CROSS_OUT, c.z];
    const lane = dir > 0 ? Math.PI / 2 : -Math.PI / 2;
    via = c.kind === 'trench' ? 'under the trench' : 'over the bump';
    const inLane = Math.abs(z - pre[1]) < 0.4 && (x - pre[0]) * dir > -0.1;
    if (inLane) {
      if (!out.length) out.push([x, z]);
      if ((x - entry[0]) * dir < -0.05) { out.push(entry); segHeading.push(lane); }
      out.push(exit); segHeading.push(lane);
      [x, z] = exit;
      continue;
    }
    const leg = f.plan(x, z, pre[0], pre[1], CLEAR, 50);
    if (!leg) return null;
    out.push(...(out.length ? leg.slice(1) : leg));
    while (segHeading.length < out.length - 1) segHeading.push(null);
    if (out.length >= 2) segHeading[out.length - 2] = lane;
    out.push(entry, exit);
    segHeading.push(lane, lane);
    [x, z] = exit;
  }
  const last = f.plan(x, z, gx, gz, CLEAR, 50);
  if (!last) return null;
  out.push(...last.slice(1));
  while (segHeading.length < out.length - 1) segHeading.push(null);
  return { path: out, via, segHeading };
}

/** Points around the bumpers every 4 cm, plus the intake out to `reach` when deployed (robot frame). */
export function outlineFor(fhx: number, reach: number): [number, number][] {
  const pts: [number, number][] = [];
  const edge = (ax: number, az: number, bx: number, bz: number) => {
    const n = Math.max(1, Math.ceil(Math.hypot(bx - ax, bz - az) / 0.04));
    for (let k = 0; k < n; k++) pts.push([ax + (bx - ax) * k / n, az + (bz - az) * k / n]);
  };
  const { hx, hz } = BUMPER;
  edge(-hx, -hz, hx, -hz);
  edge(hx, -hz, hx, hz);
  if (reach > hz + 1e-3) {
    edge(hx, hz, fhx, hz); edge(fhx, hz, fhx, reach); edge(fhx, reach, -fhx, reach); edge(-fhx, reach, -fhx, hz); edge(-fhx, hz, -hx, hz);
  } else edge(hx, hz, -hx, hz);
  edge(-hx, hz, -hx, -hz);
  return pts;
}
export const OUTLINE_DOWN = outlineFor(0.327, 0.66);
export const OUTLINE_UP = outlineFor(0.442, 0.43);

/** Smallest gap between the footprint and any obstacle at a pose (negative = overlapping). */
export function footprintClearance(f: FieldGrid, x: number, z: number, th: number, pts = OUTLINE_DOWN) {
  const c = Math.cos(th), s = Math.sin(th);
  let min = Infinity;
  for (const [lx, lz] of pts) {
    const d = f.clearance(x + lx * c + lz * s, z - lx * s + lz * c);
    if (d < min) min = d;
  }
  return min;
}

// ------------------------------------------------------------------ jobs (what the brain asks the planner)
export interface PlanJob {
  sx: number; sz: number;
  /** try these goals in order; the first that routes wins */
  goals: [number, number][];
  cross: Crossing[];
}
export interface PlanResult { route: RouteResult | null; goal: number }

export function runJob(f: FieldGrid, job: PlanJob): PlanResult {
  for (let k = 0; k < job.goals.length; k++) {
    const [gx, gz] = job.goals[k];
    const r = computeRoute(f, job.sx, job.sz, gx, gz, job.cross);
    if (r) return { route: r, goal: k };
  }
  return { route: null, goal: -1 };
}
