// The two HUBs, built from the game manual's dimensions (5.4) checked against the field model:
//   body      47 × 47 in box up to the base of the funnel (1.43 m)
//   funnel    hexagonal opening 41.7 in across the flats at 72 in (1.83 m), six sloped panels down
//             to the throat; fuel rattles down them physically
//   sensor    a ball whose center drops into the throat has passed the top opening and the sensor
//             array: it scores, exactly once
//   exits     scored fuel rides the internal ramps (~1 s) and leaves through one of four exits at
//             the base facing the NEUTRAL ZONE (manual figure 5-8)
//   net       behind the opening (neutral-zone side): a spring-damper barrier that catches a ball,
//             kills its energy and drops it, deforming visibly; no tunneling at any speed (it's a
//             swept plane test, not a thin collider)
import { FUEL, NET, STEP } from './config';
import { Fuel, BS } from './fuel';
import type { Engine } from './engine';
import type { Rng } from './rng';

const IN = 0.0254;
export const HUB = {
  half: (47 * IN) / 2,            // 0.597 m
  bodyTop: 1.43,
  rim: 72 * IN,                   // 1.829 m
  /** hexagon across the flats (along z) → circumradius along x */
  flats: 41.7 * IN,               // 1.059 m
  throatR: 0.26,
  /** distance from field center to each HUB center (field model) */
  cx: 3.645,
  net: { off: 0.85, y0: 1.70, y1: 3.02, hz: 0.70 },
};
const CIRCUM = HUB.flats / 2 / Math.cos(Math.PI / 6);   // 0.611 m

/** a ball in the net: where, how deep, which way it pushes the net (±1 along x, field frame), age */
export interface NetHit { hub: number; y: number; z: number; depth: number; dir: number; t: number }

export class Hubs {
  /** red (index 0, x < 0) and blue (index 1) */
  readonly centers: [number, number][] = [[-HUB.cx, 0], [HUB.cx, 0]];
  /** which side of each net a ball is on (−1/+1, 0 = far away) */
  private netSide: [Int8Array, Int8Array];
  /** per-hub queue of balls inside: ball, time left */
  private inside: { i: number; t: number; hub: number }[] = [];
  readonly netHits: NetHit[] = [];
  onScore: ((i: number, hub: number) => void) | null = null;
  scoredBy = [0, 0];

  constructor(private eng: Engine, private fuel: Fuel, private rng: Rng) {
    this.netSide = [new Int8Array(fuel.n), new Int8Array(fuel.n)];
    for (let h = 0; h < 2; h++) this.build(h);
  }

  /** Direction from a HUB toward the NEUTRAL ZONE (+1 = +x). */
  static toNeutral(h: number) { return h === 0 ? 1 : -1; }

  private build(h: number) {
    const R = this.eng.R;
    const [cx, cz] = this.centers[h];
    const s = Hubs.toNeutral(h);
    const H = HUB.half;
    // body
    this.eng.fixed(R.ColliderDesc.cuboid(H, HUB.bodyTop / 2, H).setTranslation(cx, HUB.bodyTop / 2, cz), 'steel');
    // funnel: six wedge panels (inner face sloped from the rim hexagon to the throat, outer face
    // out to the body's square outline, so no ledge can hold a ball)
    const sq = (a: number) => { const c = Math.cos(a), sn = Math.sin(a); const k = H / Math.max(Math.abs(c), Math.abs(sn)); return [c * k, sn * k]; };
    for (let k = 0; k < 6; k++) {
      const a0 = (k * Math.PI) / 3, a1 = ((k + 1) * Math.PI) / 3;
      const pts: number[] = [];
      const push = (x: number, y: number, z: number) => pts.push(cx + x, y, cz + z);
      for (const a of [a0, a1]) {
        push(Math.cos(a) * CIRCUM, HUB.rim, Math.sin(a) * CIRCUM);
        push(Math.cos(a) * HUB.throatR, HUB.bodyTop, Math.sin(a) * HUB.throatR);
        const [ox, oz] = sq(a);
        const r = Math.max(Math.hypot(ox, oz), CIRCUM + 0.01);
        push(Math.cos(a) * r, HUB.rim, Math.sin(a) * r);
        push(Math.cos(a) * r, HUB.bodyTop, Math.sin(a) * r);
      }
      // the square's corner between these two hexagon corners belongs to this panel
      for (const c of [Math.PI / 4, (3 * Math.PI) / 4, (5 * Math.PI) / 4, (7 * Math.PI) / 4]) {
        if (c > a0 && c < a1) { const [ox, oz] = sq(c); push(ox, HUB.rim, oz); push(ox, HUB.bodyTop, oz); }
      }
      const desc = R.ColliderDesc.convexHull(new Float32Array(pts));
      if (desc) this.eng.fixed(desc, 'polycarb');
    }
    // net frame: two uprights at each side and the top bar
    const nx = cx + s * HUB.net.off;
    for (const z of [-0.725, 0.725]) {
      this.eng.fixed(R.ColliderDesc.cuboid(0.14, HUB.net.y1 / 2 + 0.02, 0.03).setTranslation(nx - s * 0.12, HUB.net.y1 / 2 + 0.02, cz + z), 'steel');
    }
    this.eng.fixed(R.ColliderDesc.cuboid(0.025, 0.03, 0.75).setTranslation(nx, HUB.net.y1 + 0.03, cz), 'steel');
  }

  /** Net forces, before the engine steps. */
  preStep() {
    const f = this.fuel;
    const r = FUEL.radius;
    for (let h = 0; h < 2; h++) {
      const [cx, cz] = this.centers[h];
      const xn = cx + Hubs.toNeutral(h) * HUB.net.off;
      const netSide = this.netSide[h];
      for (let i = 0; i < f.n; i++) {
        const st = f.st[i];
        if (st !== BS.FREE && st !== BS.FLYING) continue;
        const o = i * 3;
        const x = f.pos[o], y = f.pos[o + 1], z = f.pos[o + 2];
        if (Math.abs(x - xn) > 0.6 || y < HUB.net.y0 - r || y > HUB.net.y1 + r || Math.abs(z - cz) > HUB.net.hz + r) { netSide[i] = 0; continue; }
        const b = f.bodies[i];
        if (b.isSleeping()) continue;
        const dx = x - xn;
        // which side the ball came from: set while it's clear of the net, kept while it's in the
        // net (a fast ball can pass the net's plane by more than its radius within a few steps)
        if (netSide[i] === 0 || netSide[i] * dx > r) { netSide[i] = dx >= 0 ? 1 : -1; if (Math.abs(dx) > r) continue; }
        const side = netSide[i];
        // penetration past the net (it gives like a stretched net, then holds)
        const depth = r - side * dx;
        if (depth <= 0) continue;
        const v = b.linvel();
        const vn = v.x * side;
        // a net is hysteretic: it stretches stiffly, but gives back little (the mesh slackens and the
        // cords rub), so on the way out it pushes only weakly: the ball drops out of it
        const k = vn < 0 ? NET.k : NET.k * NET.unload;
        let F = k * depth - NET.c * vn;               // toward the ball's side
        if (F < 0) F = 0;
        const m = f.mass[i];
        let jx = side * F * STEP;
        // while the ball presses in, the mesh wraps around it and drags its sideways motion
        const grab = vn < 0 ? 0.04 : 0;
        b.applyImpulse({ x: jx, y: v.y > 0 ? -v.y * m * grab : 0, z: -v.z * m * grab }, true);
        if (depth > NET.maxDepth) {
          // fully stretched: the net is a wall
          b.setTranslation({ x: xn + side * (r - NET.maxDepth), y, z }, true);
          if (vn < 0) b.setLinvel({ x: 0, y: v.y, z: v.z }, true);
        }
        if (depth > 0.02 && -vn > 1) this.netHits.push({ hub: h, y, z: z - cz, depth: Math.min(depth, NET.maxDepth), dir: -side, t: 0 });
        jx = 0;
      }
    }
    for (const n of this.netHits) n.t += STEP;
    while (this.netHits.length && this.netHits[0].t > 0.6) this.netHits.shift();
    if (this.netHits.length > 24) this.netHits.splice(0, this.netHits.length - 24);
  }

  /** Scoring and the trip through the HUB, after the engine steps. */
  postStep() {
    const f = this.fuel;
    for (let h = 0; h < 2; h++) {
      const [cx, cz] = this.centers[h];
      for (let i = 0; i < f.n; i++) {
        const st = f.st[i];
        if (st !== BS.FREE && st !== BS.FLYING) continue;
        const o = i * 3;
        const y = f.pos[o + 1];
        if (y > HUB.bodyTop + FUEL.radius + 0.05 || y < HUB.bodyTop - 0.2) continue;
        const dx = f.pos[o] - cx, dz = f.pos[o + 2] - cz;
        if (dx * dx + dz * dz > (HUB.throatR + 0.02) ** 2) continue;
        // through the opening and the sensor array: scored
        f.park(i, [cx, HUB.bodyTop - 0.4, cz], BS.HUB);
        this.inside.push({ i, t: 0.7 + this.rng.next() * 0.6, hub: h });
        this.scoredBy[h]++;
        this.onScore?.(i, h);
      }
    }
    // out through the exits at the base, facing the NEUTRAL ZONE
    for (let k = this.inside.length - 1; k >= 0; k--) {
      const q = this.inside[k];
      q.t -= STEP;
      if (q.t > 0) continue;
      this.inside.splice(k, 1);
      const [cx, cz] = this.centers[q.hub];
      const s = Hubs.toNeutral(q.hub);
      const exit = Math.floor(this.rng.next() * 4);
      const ez = cz + (exit - 1.5) * 0.3;
      const sp = 1.6 + this.rng.next() * 1.4;
      const fan = (exit - 1.5) * 0.25 + (this.rng.next() - 0.5) * 0.3;
      f.place(q.i, [cx + s * (HUB.half + FUEL.radius + 0.02), FUEL.radius + 0.06, ez], [s * sp * Math.cos(fan), 0, sp * Math.sin(fan)]);
    }
  }

  /** Balls still inside (for resets). */
  clear() { this.inside.length = 0; this.netHits.length = 0; this.netSide[0].fill(0); this.netSide[1].fill(0); }
  get processing() { return this.inside.length; }
}
