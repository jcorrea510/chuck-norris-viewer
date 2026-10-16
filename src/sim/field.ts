// Field physics grid (built by scripts/optimize-field.mjs): floor height for every 5 cm cell plus a
// solid mask, a distance field to the nearest solid, and an A* planner for the robot.

export interface GridData { x0: number; z0: number; cell: number; nx: number; nz: number; heights: string }
export interface HubData { min: number[]; max: number[]; center: number[]; opening: number[] }

export class FieldGrid {
  readonly x0: number; readonly z0: number; readonly cell: number; readonly nx: number; readonly nz: number;
  /** floor height in meters (solid cells hold 0) */
  readonly floorH: Float32Array;
  readonly solid: Uint8Array;
  /** distance (m) from each cell center to the nearest solid cell */
  readonly dist: Float32Array;
  /** signed distance (m) to the nearest solid surface: positive in the open, negative inside */
  readonly sdf: Float32Array;

  constructor(g: GridData, hubs: HubData[]) {
    this.x0 = g.x0; this.z0 = g.z0; this.cell = g.cell; this.nx = g.nx; this.nz = g.nz;
    const raw = Uint8Array.from(atob(g.heights), (c) => c.charCodeAt(0));
    const n = this.nx * this.nz;
    this.floorH = new Float32Array(n);
    this.solid = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (raw[i] === 255) this.solid[i] = 1;
      else this.floorH[i] = raw[i] / 1000;
    }
    // The HUB is a closed structure at robot height even where its internal ramps sit higher than the rays.
    for (const h of hubs) this.fillSolid(h.min[0], h.min[2], h.max[0], h.max[2]);
    this.dist = this.distanceField(this.solid);
    const free = new Uint8Array(n);
    for (let i = 0; i < n; i++) free[i] = this.solid[i] ? 0 : 1;
    const inside = this.distanceField(free);
    this.sdf = new Float32Array(n);
    const h = this.cell / 2;
    for (let i = 0; i < n; i++) this.sdf[i] = this.solid[i] ? -(inside[i] - h) : this.dist[i] - h;
  }

  private fillSolid(xa: number, za: number, xb: number, zb: number) {
    const [ia, ka] = this.cellOf(xa, za), [ib, kb] = this.cellOf(xb, zb);
    for (let k = Math.max(0, ka); k <= Math.min(this.nz - 1, kb); k++) for (let i = Math.max(0, ia); i <= Math.min(this.nx - 1, ib); i++) this.solid[k * this.nx + i] = 1;
  }

  /** Two-pass chamfer distance transform, in meters: distance from each cell to the nearest `src` cell. */
  private distanceField(src: Uint8Array): Float32Array {
    const { nx, nz } = this;
    const INF = 1e9;
    const d = new Float32Array(nx * nz);
    for (let i = 0; i < d.length; i++) d[i] = src[i] ? 0 : INF;
    const a = 1, b = Math.SQRT2;
    for (let k = 0; k < nz; k++) for (let i = 0; i < nx; i++) {
      const o = k * nx + i;
      let v = d[o];
      if (i > 0) v = Math.min(v, d[o - 1] + a);
      if (k > 0) {
        v = Math.min(v, d[o - nx] + a);
        if (i > 0) v = Math.min(v, d[o - nx - 1] + b);
        if (i < nx - 1) v = Math.min(v, d[o - nx + 1] + b);
      }
      d[o] = v;
    }
    for (let k = nz - 1; k >= 0; k--) for (let i = nx - 1; i >= 0; i--) {
      const o = k * nx + i;
      let v = d[o];
      if (i < nx - 1) v = Math.min(v, d[o + 1] + a);
      if (k < nz - 1) {
        v = Math.min(v, d[o + nx] + a);
        if (i < nx - 1) v = Math.min(v, d[o + nx + 1] + b);
        if (i > 0) v = Math.min(v, d[o + nx - 1] + b);
      }
      d[o] = v;
    }
    for (let i = 0; i < d.length; i++) d[i] *= this.cell;
    return d;
  }

  cellOf(x: number, z: number): [number, number] {
    return [Math.floor((x - this.x0) / this.cell), Math.floor((z - this.z0) / this.cell)];
  }
  private idx(x: number, z: number) {
    const i = Math.floor((x - this.x0) / this.cell), k = Math.floor((z - this.z0) / this.cell);
    if (i < 0 || k < 0 || i >= this.nx || k >= this.nz) return -1;
    return k * this.nx + i;
  }
  isSolid(x: number, z: number) { const o = this.idx(x, z); return o < 0 || this.solid[o] === 1; }
  distAt(x: number, z: number) { const o = this.idx(x, z); return o < 0 ? 0 : this.dist[o]; }

  /**
   * Signed distance to the nearest wall or field element (bilinear, so contacts are smooth):
   * positive in the open, negative when overlapping. Outside the grid counts as solid.
   */
  clearance(x: number, z: number): number {
    const fx = (x - this.x0) / this.cell - 0.5, fz = (z - this.z0) / this.cell - 0.5;
    const i = Math.floor(fx), k = Math.floor(fz);
    const tx = fx - i, tz = fz - k;
    const d = (ii: number, kk: number) => (ii < 0 || kk < 0 || ii >= this.nx || kk >= this.nz) ? -0.05 : this.sdf[kk * this.nx + ii];
    return (d(i, k) * (1 - tx) + d(i + 1, k) * tx) * (1 - tz) + (d(i, k + 1) * (1 - tx) + d(i + 1, k + 1) * tx) * tz;
  }

  /** Unit normal pointing out of the nearest obstacle (gradient of the signed distance). */
  normal(x: number, z: number): [number, number] {
    const e = this.cell;
    const gx = this.clearance(x + e, z) - this.clearance(x - e, z), gz = this.clearance(x, z + e) - this.clearance(x, z - e);
    const l = Math.hypot(gx, gz);
    return l < 1e-6 ? [0, 0] : [gx / l, gz / l];
  }

  /** Smooth floor height (bilinear over cell centers; solid cells use their free neighbours' level). */
  floor(x: number, z: number): number {
    const fx = (x - this.x0) / this.cell - 0.5, fz = (z - this.z0) / this.cell - 0.5;
    const i = Math.floor(fx), k = Math.floor(fz);
    const tx = fx - i, tz = fz - k;
    const h = (ii: number, kk: number) => {
      if (ii < 0 || kk < 0 || ii >= this.nx || kk >= this.nz) return 0;
      return this.floorH[kk * this.nx + ii];
    };
    return (h(i, k) * (1 - tx) + h(i + 1, k) * tx) * (1 - tz) + (h(i, k + 1) * (1 - tx) + h(i + 1, k + 1) * tx) * tz;
  }

  /** Floor slope (dh/dx, dh/dz). */
  slope(x: number, z: number): [number, number] {
    const e = this.cell;
    return [(this.floor(x + e, z) - this.floor(x - e, z)) / (2 * e), (this.floor(x, z + e) - this.floor(x, z - e)) / (2 * e)];
  }

  /** Unit vector pointing away from the nearest obstacle. */
  awayFromWalls(x: number, z: number): [number, number] {
    const e = this.cell * 2;
    const gx = this.distAt(x + e, z) - this.distAt(x - e, z), gz = this.distAt(x, z + e) - this.distAt(x, z - e);
    const l = Math.hypot(gx, gz);
    return l < 1e-6 ? [0, 0] : [gx / l, gz / l];
  }

  /** True when a disc of radius `clear` can slide straight from a to b. */
  lineFree(ax: number, az: number, bx: number, bz: number, clear: number): boolean {
    const len = Math.hypot(bx - ax, bz - az);
    const steps = Math.max(1, Math.ceil(len / (this.cell * 0.7)));
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      if (this.distAt(ax + (bx - ax) * t, az + (bz - az) * t) < clear) return false;
    }
    return true;
  }

  /** Nearest point with at least `clear` clearance (spiral search). */
  nearestFree(x: number, z: number, clear: number): [number, number] | null {
    if (this.distAt(x, z) >= clear) return [x, z];
    for (let r = this.cell; r < 2.5; r += this.cell) {
      for (let a = 0; a < 16; a++) {
        const px = x + Math.cos((a / 16) * Math.PI * 2) * r, pz = z + Math.sin((a / 16) * Math.PI * 2) * r;
        if (this.distAt(px, pz) >= clear) return [px, pz];
      }
    }
    return null;
  }

  /**
   * A* over the grid for a robot of clearance radius `clear`. Bump cells cost more (the robot slows
   * down to climb them). Returns a string-pulled polyline (world coords) or null.
   */
  plan(sx: number, sz: number, gx: number, gz: number, clear: number, bumpCost = 2.4): [number, number][] | null {
    const start = this.nearestFree(sx, sz, clear * 0.9);
    const goal = this.nearestFree(gx, gz, clear);
    if (!start || !goal) return null;
    const { nx, nz } = this;
    const [si, sk] = this.cellOf(start[0], start[1]);
    const [ei, ek] = this.cellOf(goal[0], goal[1]);
    const S = sk * nx + si, E = ek * nx + ei;
    const g = new Float32Array(nx * nz).fill(Infinity);
    const came = new Int32Array(nx * nz).fill(-1);
    const closed = new Uint8Array(nx * nz);
    const heap = new MinHeap();
    g[S] = 0;
    heap.push(S, 0);
    const hfun = (o: number) => Math.hypot((o % nx) - ei, Math.floor(o / nx) - ek);
    const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
    let found = false, iter = 0;
    while (heap.size && iter++ < 200000) {
      const o = heap.pop();
      if (o === E) { found = true; break; }
      if (closed[o]) continue;
      closed[o] = 1;
      const i = o % nx, k = Math.floor(o / nx);
      for (const [di, dk, w] of nb) {
        const ii = i + di, kk = k + dk;
        if (ii < 0 || kk < 0 || ii >= nx || kk >= nz) continue;
        const q = kk * nx + ii;
        if (closed[q] || this.dist[q] < clear) continue;
        const bump = this.floorH[q] > 0.03 ? bumpCost : 1;
        const ng = g[o] + w * bump;
        if (ng < g[q]) { g[q] = ng; came[q] = o; heap.push(q, ng + hfun(q)); }
      }
    }
    if (!found) return null;
    const cells: number[] = [];
    for (let o = E; o !== -1; o = came[o]) cells.push(o);
    cells.reverse();
    const pts = cells.map((o) => [this.x0 + ((o % nx) + 0.5) * this.cell, this.z0 + (Math.floor(o / nx) + 0.5) * this.cell] as [number, number]);
    pts[0] = start; pts[pts.length - 1] = goal;
    // string pulling, but keep a vertex where the floor changes between flat and bump
    const out: [number, number][] = [pts[0]];
    let a = 0;
    while (a < pts.length - 1) {
      let b = pts.length - 1;
      while (b > a + 1 && !(this.lineFree(pts[a][0], pts[a][1], pts[b][0], pts[b][1], clear) && this.sameTerrain(pts, a, b))) b--;
      out.push(pts[b]);
      a = b;
    }
    return out;
  }

  private sameTerrain(pts: [number, number][], a: number, b: number) {
    const bumpA = this.floor(pts[a][0], pts[a][1]) > 0.03;
    for (let i = a; i <= b; i += 3) if ((this.floor(pts[i][0], pts[i][1]) > 0.03) !== bumpA) return false;
    return true;
  }
}

class MinHeap {
  private k: number[] = [];
  private p: number[] = [];
  get size() { return this.k.length; }
  push(key: number, pri: number) {
    const k = this.k, p = this.p;
    k.push(key); p.push(pri);
    let i = k.length - 1;
    while (i > 0) {
      const j = (i - 1) >> 1;
      if (p[j] <= p[i]) break;
      [k[i], k[j]] = [k[j], k[i]]; [p[i], p[j]] = [p[j], p[i]];
      i = j;
    }
  }
  pop(): number {
    const k = this.k, p = this.p;
    const top = k[0];
    const lk = k.pop()!, lp = p.pop()!;
    if (k.length) {
      k[0] = lk; p[0] = lp;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < k.length && p[l] < p[m]) m = l;
        if (r < k.length && p[r] < p[m]) m = r;
        if (m === i) break;
        [k[i], k[m]] = [k[m], k[i]]; [p[i], p[m]] = [p[m], p[i]];
        i = m;
      }
    }
    return top;
  }
}
