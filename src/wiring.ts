// Procedural wiring for the CAD robot: the harness under the brain pan (the electronics plate:
// PDH, MPM, roboRIO, CANivore, Pigeon between the 2"x1" walls) and the main runs from there to the
// motors. The CAD has the devices but no wires; the routing follows the team's photos. Everything
// is built once at load into two merged meshes (insulation / braided sleeving), model frame
// (+z front, +y up, ground at y = 0), no per-frame work.
import * as THREE from 'three';
import type { PartIndex } from './parts';

type P3 = [number, number, number];
const V = (p: P3) => new THREE.Vector3(p[0], p[1], p[2]);
const UP = new THREE.Vector3(0, 1, 0);

const col = (hex: number) => new THREE.Color(hex);
const C = {
  red: col(0xd8301e), black: col(0x17171a), yellow: col(0xe8c21a), green: col(0x2e8b3a), white: col(0xe4e4e0),
  blue: col(0x2a5fc4), orange: col(0xf07a1e), body: col(0xc8ccce), tie: col(0x0e0e10), grey: col(0x55585c),
};

/** Parallel-transport frames along a dense polyline (n = normal, b = binormal), so twisted / paired wires never flip. */
function frames(s: THREE.Vector3[], ref: THREE.Vector3 = UP) {
  const m = s.length;
  const t: THREE.Vector3[] = [], n: THREE.Vector3[] = [], b: THREE.Vector3[] = [];
  for (let i = 0; i < m; i++) t.push(s[Math.min(m - 1, i + 1)].clone().sub(s[Math.max(0, i - 1)]).normalize());
  let N = ref.clone().addScaledVector(t[0], -ref.dot(t[0]));
  if (N.lengthSq() < 1e-4) N = new THREE.Vector3(1, 0, 0).addScaledVector(t[0], -t[0].x);
  N.normalize();
  for (let i = 0; i < m; i++) {
    if (i > 0) {
      const axis = new THREE.Vector3().crossVectors(t[i - 1], t[i]);
      if (axis.lengthSq() > 1e-12) N.applyAxisAngle(axis.normalize(), Math.acos(THREE.MathUtils.clamp(t[i - 1].dot(t[i]), -1, 1)));
      N.addScaledVector(t[i], -N.dot(t[i])).normalize();
    }
    n.push(N.clone());
    b.push(new THREE.Vector3().crossVectors(t[i], N));
  }
  return { t, n, b };
}

/** Smooth centripetal Catmull-Rom through the points, resampled about every `step` metres. */
function path(points: P3[] | THREE.Vector3[], step: number): THREE.Vector3[] {
  const pts = (points as (P3 | THREE.Vector3)[]).map((p) => (Array.isArray(p) ? V(p) : (p as THREE.Vector3).clone()));
  const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
  return curve.getSpacedPoints(Math.max(3, Math.ceil(curve.getLength() / step)));
}

class Soup {
  private pos: number[] = [];
  private nrm: number[] = [];
  private colr: number[] = [];
  private pid: number[] = [];
  private idx: number[] = [];
  tris = 0;
  constructor(private partId: number) {}

  private vert(p: THREE.Vector3, n: THREE.Vector3, c: THREE.Color) {
    this.pos.push(p.x, p.y, p.z); this.nrm.push(n.x, n.y, n.z); this.colr.push(c.r, c.g, c.b); this.pid.push(this.partId);
    return this.pos.length / 3 - 1;
  }

  /** Round tube along dense samples. `banded` alternates the shade per ring (braided sleeving). */
  sweep(s: THREE.Vector3[], r: number, color: THREE.Color, radial = 5, opts: { caps?: boolean; banded?: boolean; ref?: THREE.Vector3 } = {}) {
    if (s.length < 2) return;
    const { n, b } = frames(s, opts.ref);
    const base = this.pos.length / 3;
    const tmp = new THREE.Vector3(), nn = new THREE.Vector3(), c2 = new THREE.Color();
    for (let i = 0; i < s.length; i++) {
      c2.copy(color);
      if (opts.banded) c2.multiplyScalar(i % 2 ? 1.5 : 0.8);
      for (let k = 0; k < radial; k++) {
        const a = (k / radial) * Math.PI * 2;
        nn.copy(n[i]).multiplyScalar(Math.cos(a)).addScaledVector(b[i], Math.sin(a));
        tmp.copy(s[i]).addScaledVector(nn, r);
        this.vert(tmp, nn, c2);
      }
    }
    for (let i = 0; i < s.length - 1; i++) {
      for (let k = 0; k < radial; k++) {
        const a = base + i * radial + k, bb = base + i * radial + ((k + 1) % radial);
        const c = a + radial, d = bb + radial;
        this.idx.push(a, c, bb, bb, c, d);
        this.tris += 2;
      }
    }
    if (opts.caps) {
      for (const end of [0, s.length - 1]) {
        const ring = base + end * radial;
        const t = end === 0 ? new THREE.Vector3().subVectors(s[0], s[1]).normalize() : new THREE.Vector3().subVectors(s[end], s[end - 1]).normalize();
        const ci = this.vert(s[end], t, color);
        for (let k = 0; k < radial; k++) {
          const a = ring + k, bb = ring + ((k + 1) % radial);
          // flat-shaded cap: own copies of the ring so the normal is the axis
          const va = this.vert(new THREE.Vector3(this.pos[a * 3], this.pos[a * 3 + 1], this.pos[a * 3 + 2]), t, color);
          const vb = this.vert(new THREE.Vector3(this.pos[bb * 3], this.pos[bb * 3 + 1], this.pos[bb * 3 + 2]), t, color);
          if (end === 0) this.idx.push(ci, vb, va); else this.idx.push(ci, va, vb);
          this.tris++;
        }
      }
    }
  }

  box(center: THREE.Vector3, size: P3, q: THREE.Quaternion, color: THREE.Color) {
    const base = this.pos.length / 3;
    const hx = size[0] / 2, hy = size[1] / 2, hz = size[2] / 2;
    const faces: [P3, P3, P3, P3][] = [
      [[1, 0, 0], [hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz]], [[-1, 0, 0], [-hx, -hy, hz], [-hx, hy, hz], [-hx, hy, -hz]],
      [[0, 1, 0], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz]], [[0, -1, 0], [-hx, -hy, hz], [-hx, -hy, -hz], [hx, -hy, -hz]],
      [[0, 0, 1], [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz]], [[0, 0, -1], [hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz]],
    ];
    let k = 0;
    for (const [nrm, a, b2, c] of faces) {
      const nv = V(nrm).applyQuaternion(q);
      const pa = V(a), pb = V(b2), pc = V(c);
      const pd = pa.clone().add(pc).sub(pb);
      for (const p of [pa, pb, pc, pd]) this.vert(p.applyQuaternion(q).add(center), nv, color);
      const o = base + k * 4;
      this.idx.push(o, o + 1, o + 2, o, o + 2, o + 3);
      this.tris += 2;
      k++;
    }
  }

  ring(center: THREE.Vector3, axis: THREE.Vector3, R: number, r: number, color: THREE.Color, tubular = 10, radial = 5) {
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis.clone().normalize());
    const pts: THREE.Vector3[] = [];
    for (let i = 0; i <= tubular; i++) {
      const a = (i / tubular) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(a) * R, Math.sin(a) * R, 0).applyQuaternion(q).add(center));
    }
    // closed loop: the sweep's first and last rings coincide
    this.sweep(pts, r, color, radial, { ref: axis });
  }

  geometry(): THREE.BufferGeometry {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(this.pos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(this.nrm, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(this.colr, 3));
    g.setAttribute('_partid', new THREE.Float32BufferAttribute(this.pid, 1));
    g.setIndex(this.idx);
    g.computeBoundingSphere();
    g.computeBoundingBox();
    return g;
  }
}

interface WireSpec { from: P3; to: P3; color: THREE.Color; r: number; stubFrom?: P3; stubTo?: P3 }

export interface WiringGeometry { insulation: THREE.BufferGeometry; sleeving: THREE.BufferGeometry; triangles: number }

export function buildWiring(parts: PartIndex | null): WiringGeometry {
  const pdh = parts?.partsMatching(['Simplified PDH'])[0] ?? 0;
  const ins = new Soup(pdh), slv = new Soup(pdh);

  /** One wire: stubs leave the terminals straight (strain relief) and the middle is a smooth curve through `via`. */
  const smooth = (a: number, b: number, x: number) => { const t = THREE.MathUtils.clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
  let slackN = 0;
  /** Loose slack: the run sags off the plate by up to ~2 cm and wanders sideways, except where it threads a wall or the hinge rail. */
  const slack = (s: THREE.Vector3[], amp = 0.006 + 0.012 * ((slackN++ * 0.618) % 1)) => {
    const u = [0];
    for (let i = 1; i < s.length; i++) u.push(u[i - 1] + s[i].distanceTo(s[i - 1]));
    const len = u[u.length - 1];
    if (len < 0.08) return s;
    const ph = slackN * 1.7;
    return s.map((p, i) => {
      const t = u[i] / len;
      const w = Math.sin(Math.PI * t) * smooth(0.017, 0.03, Math.abs(Math.abs(p.x) - 0.051)) * smooth(0.01, 0.03, Math.abs(p.z + 0.0075));
      return new THREE.Vector3(p.x + 0.003 * Math.sin(t * 9 + ph) * w, Math.max(0.058, p.y - amp * w), p.z + 0.003 * Math.cos(t * 7 + ph) * w);
    });
  };

  const wire = (from: P3, via: P3[], to: P3, color: THREE.Color, r: number, stubFrom?: P3, stubTo?: P3) => {
    const pts: P3[] = [from];
    if (stubFrom) pts.push([from[0] + stubFrom[0], from[1] + stubFrom[1], from[2] + stubFrom[2]]);
    pts.push(...via);
    if (stubTo) pts.push([to[0] + stubTo[0], to[1] + stubTo[1], to[2] + stubTo[2]]);
    pts.push(to);
    ins.sweep(slack(path(pts, r > 0.0016 ? 0.008 : 0.008)), r, color, r > 0.0016 ? 6 : 5);
  };

  /** Red/black pair side by side (flat to the plate unless `ref` says otherwise). */
  const pair = (pts: P3[], r: number, ref: THREE.Vector3 = UP, a: THREE.Color = C.red, b: THREE.Color = C.black) => {
    const s = path(pts, 0.01);
    const f = frames(s, ref);
    const o = r * 1.05;
    for (const [sign, c] of [[1, a], [-1, b]] as const) {
      ins.sweep(s.map((p, i) => p.clone().addScaledVector(f.b[i], sign * o)), r, c, 6);
    }
  };

  /** Twisted CAN pair: yellow / green, about 22 mm lay. */
  const can = (pts: P3[], ref: THREE.Vector3 = UP, lay = 0.012) => {
    const s = path(pts, 0.002);
    const f = frames(s, ref);
    const r = 0.0009;
    let acc = 0;
    const off: THREE.Vector3[] = [];
    for (let i = 0; i < s.length; i++) {
      if (i) acc += s[i].distanceTo(s[i - 1]);
      const ph = (acc / lay) * Math.PI * 2;
      off.push(f.n[i].clone().multiplyScalar(Math.cos(ph) * r).addScaledVector(f.b[i], Math.sin(ph) * r));
    }
    ins.sweep(s.map((p, i) => p.clone().add(off[i])), r, C.yellow, 4);
    ins.sweep(s.map((p, i) => p.clone().sub(off[i])), r, C.green, 4);
  };

  /** Spare-wire coil: `turns` loops of radius r, rising slightly each turn. */
  const coil = (c: P3, r: number, turns: number): P3[] => {
    const out: P3[] = [];
    for (let a = 0; a <= turns * Math.PI * 2 + 1e-6; a += Math.PI / 3) out.push([c[0] + r * Math.cos(a), c[1] + 0.003 * (a / (Math.PI * 2)), c[2] + r * Math.sin(a)]);
    return out;
  };

  /** WAGO 221 lever splice lying on the plate: `dir` is along the conductors. */
  const wago = (c: P3, dir: THREE.Vector3, n: number, down = -1) => {
    const L = 0.0063 * n + 0.003, W = 0.0125, H = 0.008;
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), dir.clone().normalize());
    const ctr = V(c);
    ins.box(ctr, [L, H, W], q, C.orange);
    for (let i = 0; i < n; i++) {
      const off = new THREE.Vector3((i - (n - 1) / 2) * 0.0063, down * (H / 2 + 0.0022), 0).applyQuaternion(q);
      ins.box(ctr.clone().add(off), [0.0052, 0.0044, W * 0.82], q, C.white);
    }
  };

  /** Zip tie round a bundle of radius R at p (axis along the bundle), with its trimmed tail. */
  const tie = (p: P3, axis: P3, R: number) => {
    const a = V(axis).normalize();
    ins.ring(V(p), a, R + 0.0009, 0.0008, C.tie, 8, 4);
    const side = new THREE.Vector3().crossVectors(a, UP).normalize();
    ins.box(V(p).addScaledVector(side, R + 0.004).addScaledVector(UP, 0.0006), [0.0016, 0.0016, 0.007], new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), side), C.tie);
  };

  /** Rubber grommet ring in a wall / plate hole. */
  const grommet = (p: P3, axis: P3, R = 0.0085) => {
    slv.ring(V(p), V(axis), R, 0.0028, C.tie, 12, 5);
  };

  /**
   * A bundle of wires along `route`, braided sleeving over the middle (t0..t1 of the route length);
   * wires keep their own offsets inside the sleeve and fan out to their terminals at the ends.
   */
  const harness = (route: P3[], wires: WireSpec[], opts: { sleeve?: [number, number]; sleeveR?: number; ref?: THREE.Vector3; ties?: number[] } = {}) => {
    const s = path(route, 0.02);
    const f = frames(s, opts.ref ?? UP);
    const total = wires.reduce((a, w) => a + w.r * w.r, 0);
    const R = Math.sqrt(total) * 1.25;
    // sleeve's inner radius: the packed bundle
    wires.forEach((w, i) => {
      const ang = i * 2.399963, rad = Math.sqrt((i + 0.5) / wires.length) * R * 0.78;
      const ox = Math.cos(ang) * rad, oy = Math.sin(ang) * rad;
      const mid: THREE.Vector3[] = s.map((p, j) => p.clone().addScaledVector(f.n[j], ox).addScaledVector(f.b[j], oy));
      const pts: THREE.Vector3[] = [V(w.from)];
      if (w.stubFrom) pts.push(V(w.from).add(V(w.stubFrom)));
      for (let j = 0; j < mid.length; j++) pts.push(mid[j]);
      if (w.stubTo) pts.push(V(w.to).add(V(w.stubTo)));
      pts.push(V(w.to));
      ins.sweep(path(pts, w.r > 0.0016 ? 0.01 : 0.008), w.r, w.color, w.r > 0.0016 ? 6 : 5);
    });
    if (opts.sleeve) {
      const [t0, t1] = opts.sleeve;
      const i0 = Math.floor(t0 * (s.length - 1)), i1 = Math.ceil(t1 * (s.length - 1));
      const rs = opts.sleeveR ?? R * 1.05;
      const sub = path(s.slice(i0, i1 + 1), 0.005);
      slv.sweep(sub, rs, col(0x1d1d20), 10, { caps: false, banded: true, ref: opts.ref });
      slv.sweep(path(s.slice(i0, i0 + 2), 0.002), rs * 1.04, C.tie, 10);
    }
    for (const t of opts.ties ?? []) {
      const i = Math.round(t * (s.length - 1));
      tie([s[i].x, s[i].y, s[i].z], [f.t[i].x, f.t[i].y, f.t[i].z], R * 1.05);
    }
    return { s, R };
  };

  // ------------------------------------------------------------------ layout (model frame)
  // The CAD's electronics hang under the 'Brain Pan' plate (underside y = 0.095) between the
  // 2"x1" walls at x = +-(0.038..0.064); the polycarbonate door sits just below them (y 0.041).
  // Photo layout: PDH on the +x side, MPM / roboRIO / CANivore on the -x side, Pigeon in the middle.
  const PDH_Z = (i: number) => -0.182 + 0.0148 * i;
  const A = (i: number): P3 => [0.1595, 0.074, PDH_Z(i)]; // PDH lever rows: inboard (red) ...
  const B = (i: number): P3 => [0.2725, 0.074, PDH_Z(i)]; // ... and outboard (black)
  const L = (n: number) => 0.0875 - 0.0042 * (n % 3); // wire height, layered so crossings don't intersect
  const RAIL = -0.0075; // z of the 26" hinge rail the wires pass through (grommets)
  const kt = (x: number, z: number, away: number) => ({ red: [x - 0.0045, 0.062, z] as P3, blk: [x + 0.0045, 0.062, z] as P3, stub: [0, 0.002, away] as P3 });
  const SR: P3 = [-0.006, 0, 0], SB: P3 = [0.006, 0, 0];
  const R12 = 0.0024; // 12 AWG silicone leads

  // Kraken power-lead terminals (the face toward the pan centre)
  const D3 = kt(0.198, -0.362, 0.012), S3 = kt(0.298, -0.263, 0.012); // rear +x module: drive, steer
  const D4 = kt(-0.198, -0.362, 0.012), S4 = kt(-0.298, -0.263, 0.012); // rear -x
  const D1 = kt(0.198, 0.144, -0.012), S1 = kt(0.298, 0.045, -0.012); // front +x
  const D2 = kt(-0.198, 0.144, -0.012), S2 = kt(-0.298, 0.045, -0.012); // front -x

  // ---- rear +x module: drive (ch0), steer (ch1); the blacks round the PDH's rear end
  wire(A(0), [[0.152, L(0), -0.19], [0.1465, L(0), -0.262], [0.15, L(0), -0.33], [0.185, 0.072, -0.352]], D3.red, C.red, R12, SR, D3.stub);
  wire(B(0), [[0.283, L(1), -0.19], [0.283, L(1), -0.2525], [0.262, L(1), -0.2565], [0.235, L(1), -0.285], [0.215, 0.074, -0.34]], D3.blk, C.black, R12, SB, D3.stub);
  wire(A(1), [[0.1445, L(1), -0.2], [0.1445, L(1), -0.2605], [0.2, L(2), -0.2605], [0.27, L(2), -0.2605], [0.2935, 0.07, -0.255]], S3.red, C.red, R12, SR, S3.stub);
  wire(B(1), [[0.287, L(2), -0.2], [0.3025, 0.072, -0.25]], S3.blk, C.black, R12, SB, S3.stub);

  // ---- west corridor between the roboRIO and the MPM: -x modules (ch2, 3 rear; ch6, 7 front) and the MPM feed
  const wc = (k: number) => -0.208 + (Math.floor(k / 3) - 1.5) * 0.0046;
  const west = (k: number): P3[] => [[0.13, L(k), wc(k)], [0.064, L(k), wc(k)], [-0.064, L(k), wc(k)], [-0.11, L(k), wc(k)]];
  const redWest = (k: number, i: number, lane: number, then: P3[], to: P3, stubTo: P3) =>
    wire(A(i), [[lane, L(k), PDH_Z(i) - 0.004], [lane, L(k), wc(k)], ...west(k), ...then], to, C.red, R12, SR, stubTo);
  const blackWest = (k: number, i: number, wrapZ: number, then: P3[], to: P3, stubTo: P3) =>
    wire(B(i), [[0.2835 + 0.003 * (k % 3), L(k), PDH_Z(i) - 0.01], [0.2835 + 0.003 * (k % 3), L(k), wrapZ], [0.24, L(k), wrapZ], [0.16, L(k), wrapZ], ...west(k), ...then], to, C.black, R12, SB, stubTo);
  const xN = (k: number) => -0.2315 - 0.003 * (k % 4), xS = (k: number) => -0.2495 - 0.003 * (k % 3);
  const toRear = (k: number): P3[] => [[xS(k), L(k), wc(k)], [xS(k), L(k), -0.25]];
  const toFront = (k: number): P3[] => [[xN(k), L(k), wc(k)], [xN(k), L(k), -0.05], [xN(k), L(k), RAIL], [xN(k), L(k), 0.025]];
  // rear -x drive / steer
  redWest(0, 2, 0.1425, [...toRear(0), [-0.245, L(0), -0.32], [-0.215, 0.072, -0.348]], D4.red, D4.stub);
  blackWest(1, 2, -0.2545, [...toRear(1), [-0.245, L(1), -0.322], [-0.185, 0.072, -0.35]], D4.blk, D4.stub);
  redWest(2, 3, 0.139, [...toRear(2), [-0.275, 0.078, -0.245]], S4.red, S4.stub);
  blackWest(3, 3, -0.2585, [...toRear(3), [-0.3, 0.076, -0.242]], S4.blk, S4.stub);
  // front -x drive / steer: up the corridor lane, through the hinge rail
  redWest(4, 6, 0.1355, [...toFront(4), [-0.21, 0.075, 0.12]], D2.red, D2.stub);
  blackWest(5, 6, -0.2625, [...toFront(5), [-0.19, 0.075, 0.122]], D2.blk, D2.stub);
  redWest(6, 7, 0.132, [...toFront(6), [-0.285, 0.078, 0.03]], S2.red, S2.stub);
  blackWest(7, 7, -0.2505, [...toFront(7), [-0.31, 0.078, 0.03]], S2.blk, S2.stub);
  // MPM feed (PDH low-current block)
  const MPM_IN = kt(-0.1295, -0.239, 0.012);
  wire([0.1595, 0.074, -0.1968], [[0.1405, L(8), -0.1968], [0.1405, L(8), wc(8)], ...west(8), [-0.115, L(8), wc(8)], [-0.121, 0.078, -0.226]], [MPM_IN.red[0], 0.079, MPM_IN.red[2]], C.red, R12, SR, [0, 0, 0.012]);
  wire([0.2725, 0.074, -0.1968], [[0.2835, L(9), -0.21], [0.2835, L(9), -0.2565], [0.24, L(9), -0.2565], [0.16, L(9), -0.2565], ...west(9), [-0.11, L(9), wc(9)], [-0.1165, 0.078, -0.226]], [MPM_IN.blk[0], 0.079, MPM_IN.blk[2]], C.black, R12, SB, [0, 0, 0.012]);

  // ---- front +x module: reds up the lane beside the PDH and through the rail, blacks up the outboard lane
  wire(A(4), [[0.1385, L(0), PDH_Z(4)], [0.1385, L(0), -0.05], [0.1385, L(0), RAIL], [0.1385, L(0), 0.1], [0.175, 0.074, 0.125]], D1.red, C.red, R12, SR, D1.stub);
  wire(B(4), [[0.2865, L(1), PDH_Z(4)], [0.2865, L(1), RAIL], [0.2865, L(1), 0.02], [0.24, L(1), 0.03], [0.215, 0.075, 0.1], [0.205, 0.074, 0.125]], D1.blk, C.black, R12, SB, D1.stub);
  wire(A(5), [[0.1415, L(1), PDH_Z(5)], [0.1415, L(1), RAIL], [0.1415, L(1), 0.025], [0.22, L(1), 0.025], [0.285, 0.078, 0.025]], S1.red, C.red, R12, SR, S1.stub);
  wire(B(5), [[0.2895, L(2), PDH_Z(5)], [0.2895, L(2), RAIL], [0.2895, L(2), 0.02], [0.305, 0.076, 0.026]], S1.blk, C.black, R12, SB, S1.stub);

  // ---- CAN: yellow/green twisted pairs, daisy-chained from the roboRIO to the CANivore, Pigeon and the modules
  const T = 0.0745; // CAN lane height, under the power wires
  can([[-0.205, 0.0805, -0.033], [-0.205, 0.079, -0.02], [-0.245, T, -0.024], [-0.29, T, -0.04], [-0.288, T, -0.07], [-0.288, 0.083, -0.084]]); // roboRIO -> CANivore
  // CANivore -> front -x module (through the rail)
  can([[-0.268, 0.083, -0.084], [-0.268, 0.083, -0.072], [-0.262, T, -0.05], [-0.262, T, -0.02], [-0.265, T, RAIL], [-0.262, T, 0.015], [-0.262, 0.0835, 0.03]]);
  can([[-0.262, 0.0835, 0.03], [-0.25, 0.083, 0.03], [-0.23, 0.078, 0.07], [-0.222, 0.07, 0.12], [-0.215, 0.062, 0.132], [-0.215, 0.062, 0.144]]);
  can([[-0.262, 0.0835, 0.03], [-0.285, 0.08, 0.026], [-0.305, 0.072, 0.03], [-0.315, 0.062, 0.033], [-0.315, 0.062, 0.045]]);
  // CANivore -> rear -x module
  can([[-0.288, 0.083, -0.141], [-0.288, 0.083, -0.153], [-0.28, T, -0.18], [-0.262, T, -0.215], [-0.245, T, -0.26], [-0.235, 0.0835, -0.29]]);
  can([[-0.235, 0.0835, -0.29], [-0.25, 0.08, -0.262], [-0.3, 0.075, -0.25], [-0.315, 0.062, -0.25], [-0.315, 0.062, -0.263]]);
  can([[-0.235, 0.0835, -0.29], [-0.235, 0.083, -0.31], [-0.235, 0.078, -0.335], [-0.22, 0.07, -0.35], [-0.215, 0.062, -0.35], [-0.215, 0.062, -0.362]]);
  // CANivore -> Pigeon (under the roboRIO's rear edge, through both walls)
  can([[-0.268, 0.083, -0.141], [-0.268, 0.083, -0.153], [-0.255, T, -0.178], [-0.2, T, -0.187], [-0.1, T, -0.187], [-0.05, T, -0.185], [-0.03, T, -0.17], ...coil([0, 0.074, -0.205], 0.022, 2), [-0.012, 0.086, -0.15], [-0.008, 0.089, -0.134]]);
  // Pigeon -> +x modules
  can([[0.008, 0.089, -0.134], [0.008, 0.089, -0.148], [0.045, T, -0.172], ...coil([0.105, 0.075, -0.12], 0.019, 2), [0.1, 0.0835, -0.175]]);
  can([[0.1, 0.0835, -0.175], [0.1, T, -0.2], [0.105, T, -0.26], [0.15, T, -0.285], [0.2, T, -0.287], [0.245, 0.0835, -0.29]]);
  can([[0.245, 0.0835, -0.29], [0.245, 0.083, -0.3], [0.235, 0.078, -0.33], [0.215, 0.07, -0.348], [0.21, 0.062, -0.348], [0.21, 0.062, -0.362]]);
  can([[0.245, 0.0835, -0.29], [0.245, 0.083, -0.28], [0.255, T, -0.268], [0.3, T, -0.255], [0.315, 0.062, -0.255], [0.315, 0.062, -0.263]]);
  can([[0.1, 0.0835, -0.175], [0.1, T, -0.14], [0.1, T, -0.09], [0.14, T, -0.06], [0.1435, T, -0.03], [0.1435, T, RAIL], [0.15, T, 0.01], [0.2, 0.0835, 0.03]]);
  can([[0.2, 0.0835, 0.03], [0.21, 0.083, 0.03], [0.215, 0.078, 0.08], [0.215, 0.07, 0.125], [0.215, 0.062, 0.132], [0.215, 0.062, 0.144]]);
  can([[0.2, 0.0835, 0.03], [0.23, 0.083, 0.03], [0.29, 0.078, 0.03], [0.315, 0.07, 0.03], [0.315, 0.062, 0.033], [0.315, 0.062, 0.045]]);

  // ---- MPM outputs (18 AWG pairs) to the roboRIO and CANivore
  pair([[-0.125, 0.079, -0.239], [-0.125, 0.079, -0.231], [-0.09, T, -0.222], [-0.071, T, -0.19], [-0.071, T, -0.06], [-0.075, T, -0.04], [-0.092, 0.0755, -0.033]], 0.0013);
  pair([[-0.17, 0.079, -0.239], [-0.17, 0.079, -0.231], [-0.2, T, -0.217], [-0.226, T, -0.19], [-0.226, T, -0.1], [-0.262, T, -0.078], [-0.278, 0.083, -0.072], [-0.278, 0.083, -0.084]], 0.0013);

  // ---- WAGO 221 splices on the CAN daisy chain and the 18 AWG feeds
  const X = new THREE.Vector3(1, 0, 0), Zv = new THREE.Vector3(0, 0, 1);
  wago([-0.262, 0.0835, 0.03], X, 3); wago([0.2, 0.0835, 0.03], X, 3);
  wago([-0.235, 0.0835, -0.29], Zv, 3); wago([0.245, 0.0835, -0.29], Zv, 3); wago([0.1, 0.0835, -0.175], Zv, 3);
  wago([-0.226, 0.0835, -0.12], Zv, 2); wago([-0.071, 0.0835, -0.15], Zv, 2); wago([-0.2, 0.0835, -0.225], X, 2);

  // ---- grommets where the bundles pass through the walls and the hinge rail
  for (const x of [0.0645, 0.0375, -0.0375, -0.0645]) grommet([x, 0.0835, -0.208], [1, 0, 0], 0.0105);
  for (const x of [0.0645, 0.0375, -0.0375, -0.0645]) grommet([x, T, -0.187], [1, 0, 0], 0.006);
  for (const [x, R] of [[0.1435, 0.0105], [0.2885, 0.0085], [-0.2335, 0.0095], [-0.265, 0.0055]] as const) for (const z of [-0.0207, 0.0057]) grommet([x, 0.0825, z], [0, 0, 1], R);

  // ---- zip ties on the main bundles
  for (const z of [-0.205, -0.211]) { tie([-0.16, 0.0833, z], [1, 0, 0], 0.0105); }
  tie([-0.17, 0.0833, -0.208], [1, 0, 0], 0.0105); tie([0.1, 0.0833, -0.208], [1, 0, 0], 0.0105); tie([0.09, 0.0833, -0.208], [1, 0, 0], 0.0105);
  for (const z of [-0.23, -0.14, -0.08]) tie([0.141, 0.0833, z], [0, 0, 1], 0.0065);
  for (const z of [-0.15, -0.08]) tie([0.2865, 0.0833, z], [0, 0, 1], 0.0065);
  for (const z of [-0.12, -0.06]) tie([-0.2335, 0.0833, z], [0, 0, 1], 0.0065);
  for (const x of [0.2, 0.23]) tie([x, 0.0833, -0.2565], [1, 0, 0], 0.0095);

  // ---- turret rotation motors: PDH pairs through the plate holes at the rear, braided sleeving up to the motors
  const HN: P3 = [-0.12, 0.088, -0.395], HP: P3 = [0.1, 0.088, -0.395];
  wire(A(8), [[0.1325, L(2), PDH_Z(8)], [0.1325, L(2), wc(2)], ...west(2), [-0.08, L(2), wc(2)], [-0.085, L(2), -0.3], [-0.095, L(2), -0.385]], HN, C.red, R12, SR);
  wire(B(8), [[0.2865, L(0), PDH_Z(8)], [0.2865, L(0), -0.2555], [0.24, L(0), -0.2555], [0.16, L(0), -0.2555], ...west(0).slice(0, 4), [-0.075, L(0), wc(0)], [-0.082, L(0), -0.3], [-0.1, L(0), -0.39]], HN, C.black, R12, SB);
  wire(A(9), [[0.1305, L(1), PDH_Z(9)], [0.1305, L(1), -0.2], [0.125, L(1), -0.3], [0.112, L(1), -0.385]], HP, C.red, R12, SR);
  wire(B(9), [[0.2895, L(1), PDH_Z(9)], [0.2895, L(1), -0.2575], [0.2, L(1), -0.2575], [0.13, L(1), -0.28], [0.123, L(1), -0.34], [0.108, L(1), -0.39]], HP, C.black, R12, SB);
  for (const [h, sx] of [[HN, -1], [HP, 1]] as const) {
    const mx = sx * 0.098;
    harness([h, [h[0], 0.12, h[2]], [h[0] + sx * 0.005, 0.16, -0.396], [mx - sx * 0.002, 0.2, -0.396]], [
      { from: h, to: [mx - 0.005, 0.222, -0.394], color: C.red, r: R12, stubTo: [0, 0.01, 0] },
      { from: h, to: [mx + 0.005, 0.222, -0.394], color: C.black, r: R12, stubTo: [0, 0.01, 0] },
    ], { sleeve: [0.1, 0.9], sleeveR: 0.0075 });
    grommet([h[0], 0.0945, h[2]], [0, 1, 0], 0.0145);
  }

  // ---- battery -> 120 A breaker -> PDH (6 AWG)
  const R6 = 0.0037;
  wire([0.131, 0.09, 0.0425], [[0.16, 0.088, 0.0425], [0.25, 0.088, 0.04], [0.3, 0.088, 0.03]], [0.3, 0.1, 0.0], C.red, R6);
  wire([0.3, 0.1, 0.0], [[0.3, 0.12, -0.05], [0.31, 0.14, -0.12]], [0.332, 0.15, -0.158], C.red, R6, undefined, [0, 0, 0.012]);
  wire([0.332, 0.14, -0.206], [[0.31, 0.12, -0.215]], [0.3, 0.1, -0.222], C.red, R6, [0, 0, -0.01]);
  wire([0.3, 0.1, -0.222], [[0.3, 0.075, -0.222], [0.25, 0.07, -0.222]], [0.19, 0.07, -0.215], C.red, R6);
  wire([0.131, 0.09, 0.0305], [[0.16, 0.086, 0.0305], [0.25, 0.086, 0.018], [0.2985, 0.086, 0.0], [0.2985, 0.086, -0.15], [0.2985, 0.084, -0.22], [0.27, 0.07, -0.222]], [0.245, 0.07, -0.215], C.black, R6);
  grommet([0.3, 0.0945, 0.0], [0, 1, 0], 0.008); grommet([0.3, 0.0945, -0.222], [0, 1, 0], 0.008);
  wago([0.215, 0.083, -0.268], X, 2); wago([0.26, 0.083, -0.268], X, 2);

  // ---- every other motor controller: PDH pairs ending at the plate holes / a front exit (grommets); the moving motors are not wired
  const Ax = (i: number): P3 => [0.1595, 0.074, PDH_Z(i) + 0.0074], Bx = (i: number): P3 => [0.2725, 0.074, PDH_Z(i) + 0.0074];
  const CX: P3 = [0, 0.088, -0.3], FX: P3 = [0.2, 0.088, 0.19];
  for (let j = 0; j < 2; j++) { // flywheels, +x pair of motors
    const i = 2 + j, z = Ax(i)[2], lx = 0.1285 - 0.003 * j;
    wire(Ax(i), [[lx, L(j), z], [lx, L(j + 1), -0.2], [0.122, L(j), -0.31], [0.113 - 0.003 * j, L(j), -0.386]], HP, C.red, R12, SR);
    wire(Bx(i), [[0.2925 + 0.003 * j, L(j + 1), z], [0.2925 + 0.003 * j, L(j + 1), -0.2575 - 0.004 * j], [0.2, L(j + 1), -0.2625], [0.135, L(j), -0.285], [0.126, L(j), -0.34], [0.108 + 0.003 * j, L(j), -0.39]], HP, C.black, R12, SB);
  }
  for (let j = 0; j < 2; j++) { // flywheels, -x pair of motors, through the corridor
    const i = 4 + j, k = 10 + j, z = Ax(i)[2], lx = 0.1295 - 0.003 * j;
    wire(Ax(i), [[lx, L(k), z], [lx, L(k), wc(k)], ...west(k), [-0.078, L(k), wc(k)], [-0.088, L(k), -0.3], [-0.1, L(k), -0.385]], HN, C.red, R12, SR);
    wire(Bx(i), [[0.2955 + 0.003 * j, L(k), z], [0.2955 + 0.003 * j, L(k), -0.2535 - 0.004 * j], [0.24, L(k), -0.2535 - 0.004 * j], [0.16, L(k), -0.2535 - 0.004 * j], ...west(k), [-0.072, L(k), wc(k)], [-0.085, L(k), -0.3], [-0.104, L(k), -0.39]], HN, C.black, R12, SB);
  }
  for (let j = 0; j < 2; j++) { // indexer motors, into the centre channel
    const i = j, z = Ax(i)[2], k = 12 + j;
    wire(Ax(i), [[0.1275, L(k), z], [0.1275, L(k), -0.3 + 0.006 * j], [0.064, L(k), -0.3 + 0.006 * j], [0.038, L(k), -0.3 + 0.006 * j], [0.012, L(k), -0.3 + 0.006 * j]], [CX[0], CX[1], CX[2] + 0.006 * j], C.red, R12, SR);
    wire(Bx(i), [[0.2975, L(k), z], [0.2975, L(k), -0.2565 + 0.002 * j], [0.2, L(k), -0.2585 + 0.002 * j], [0.13, L(k), -0.2745], [0.1, L(k), -0.3 + 0.006 * j], [0.064, L(k), -0.3 + 0.006 * j], [0.012, L(k), -0.3 + 0.006 * j]], [CX[0] - 0.006, CX[1], CX[2] + 0.006 * j], C.black, R12, SB);
  }
  for (let j = 0; j < 3; j++) { // intake pivot, intake roller, hotdog: up the lanes and through the rail
    const i = 6 + j, z = Ax(i)[2], rx = 0.1345 + 0.0035 * (j % 2), bx = 0.3015 + 0.003 * j;
    wire(Ax(i), [[rx, L(j), z], [rx, L(j), -0.05], [rx, L(j), RAIL], [rx, L(j), 0.1], [0.16 + 0.01 * j, L(j + 1), 0.16]], [FX[0] - 0.006 + 0.006 * j, FX[1], FX[2]], C.red, R12, SR);
    wire(Bx(i), [[bx, L(j + 1), z], [bx, L(j + 1), RAIL], [bx, L(j + 1), 0.015], [0.26, L(j + 1), 0.04], [0.22, L(j + 1), 0.12], [0.205 + 0.006 * j, L(j + 1), 0.17]], [FX[0] + 0.004 + 0.004 * j, FX[1], FX[2]], C.black, R12, SB);
  }
  grommet([FX[0], 0.0945, FX[2]], [0, 1, 0], 0.012); grommet([CX[0], 0.0945, CX[2] + 0.003], [0, 1, 0], 0.012);

  // ---- doubled MPM outputs (14 AWG pairs) toward the rear exit
  pair([[-0.152, 0.079, -0.239], [-0.152, 0.079, -0.23], [-0.1, 0.07, -0.221], [-0.085, 0.066, -0.25], [-0.09, 0.066, -0.33], [-0.105, 0.072, -0.39]], 0.0019);
  pair([[-0.135, 0.079, -0.239], [-0.135, 0.079, -0.229], [-0.095, 0.064, -0.224], [-0.08, 0.062, -0.26], [-0.083, 0.062, -0.33], [-0.1, 0.07, -0.392]], 0.0019);

  // ---- roboRIO cables: ethernet arcs off the plate to the rear exit, USB loops to a plug, three short signal leads
  const ETH: P3 = [-0.165, 0.0775, -0.033];
  wire(ETH, [[-0.165, 0.0775, -0.014], [-0.13, 0.062, -0.011], [-0.088, 0.066, -0.03], [-0.082, 0.066, -0.1], [-0.084, 0.066, -0.2], [-0.09, 0.066, -0.3]], [-0.1, 0.075, -0.39], C.black, 0.0027);
  wire([-0.185, 0.0775, -0.033], [[-0.185, 0.0775, -0.018], [-0.215, 0.064, -0.012], [-0.2, 0.061, -0.04], [-0.16, 0.063, -0.05]], [-0.15, 0.078, -0.06], C.black, 0.002);
  ins.box(V([-0.14, 0.085, -0.062]), [0.012, 0.009, 0.019], new THREE.Quaternion(), C.black);
  for (const [x, ex, ez] of [[-0.21, -0.255, -0.215], [-0.195, -0.28, -0.2], [-0.2, -0.234, -0.25]] as const) {
    wire([x, 0.0765, -0.033], [[x, 0.0765, -0.017], [x - 0.02, 0.066, -0.02], [ex, 0.066, ez + 0.02]], [ex, 0.075, ez], C.black, 0.0009);
  }

  // ---- long loops of spare CAN in the centre channel and beside the PDH
  can([[-0.262, T, -0.05], ...coil([-0.248, 0.075, -0.075], 0.012, 1.5), [-0.262, T, -0.02]]);

  // ---- the PDH's lever ports are orange
  for (let i = 0; i < 10; i++) for (const x of [0.158, 0.2742]) ins.box(V([x, 0.0735, PDH_Z(i) + 0.0005]), [0.0035, 0.026, 0.0108], new THREE.Quaternion(), C.orange);

  const insGeo = ins.geometry();
  const slvGeo = slv.geometry();
  return { insulation: insGeo, sleeving: slvGeo, triangles: ins.tris + slv.tris };
}
