// The cargo net as geometry: every cord is two crossed ribbons (so it reads as a round cord from any
// side), the webbing along the edges is a flat strap. One dynamic mesh; the positions are the cloth's
// knots in the robot frame (phys/net.ts), written straight into the vertex buffer.
import * as THREE from 'three';
import { netTopology } from './phys/net';

const CORD = 0.0022;           // half width of a cord ribbon (m): 4.4 mm (a 2 mm cord with its knots reads thin at robot-viewing distances)
const STRAP = 0.03;            // webbing width (m): 1.2 in
const COLOR = 0x0c0c0d;

export class NetMesh {
  readonly mesh: THREE.Mesh;
  private topo = netTopology();
  private pos: Float32Array;
  private nor: Float32Array;
  private strap: number[][] = [];
  private cordVerts: number;
  constructor() {
    const t = this.topo;
    const cords = t.cords.length / 2;
    // the boundary knots in order around the net: back (b = 0), right, front, left
    const N = 18, M = 18;
    const at = (a: number, b: number) => t.index[b * (N + 1) + a];
    const loop: number[] = [];
    for (let a = 0; a < N; a += 2) loop.push(at(a, 0));
    for (let b = 0; b < M; b += 2) loop.push(at(N, b));
    for (let a = N; a > 0; a -= 2) loop.push(at(a, M));
    for (let b = M; b > 0; b -= 2) loop.push(at(0, b));
    for (let k = 0; k < loop.length; k++) this.strap.push([loop[k], loop[(k + 1) % loop.length]]);
    this.cordVerts = cords * 8;
    const verts = this.cordVerts + this.strap.length * 4;
    this.pos = new Float32Array(verts * 3);
    this.nor = new Float32Array(verts * 3);
    const idx: number[] = [];
    for (let c = 0; c < cords * 2; c++) idx.push(4 * c, 4 * c + 1, 4 * c + 2, 4 * c + 2, 4 * c + 1, 4 * c + 3);
    for (let s = 0; s < this.strap.length; s++) { const o = this.cordVerts + 4 * s; idx.push(o, o + 1, o + 2, o + 2, o + 1, o + 3); }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('normal', new THREE.BufferAttribute(this.nor, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setIndex(idx);
    const mat = new THREE.MeshStandardMaterial({ color: COLOR, roughness: 0.88, metalness: 0, side: THREE.DoubleSide });
    this.mesh = new THREE.Mesh(geo, mat);
    this.mesh.frustumCulled = false;
    this.mesh.name = 'cargo-net';
    this.mesh.castShadow = false;
    this.mesh.receiveShadow = true;
  }

  /** Writes the knots (robot frame), interpolated between `prev` and `cur` by alpha. */
  update(cur: Float32Array, prev: Float32Array | null, alpha: number) {
    const t = this.topo, P = this.pos, Nn = this.nor;
    const k = (i: number, c: number) => (prev ? prev[3 * i + c] + (cur[3 * i + c] - prev[3 * i + c]) * alpha : cur[3 * i + c]);
    let w = 0;
    for (let c = 0; c < t.cords.length; c += 2) {
      const i = t.cords[c], j = t.cords[c + 1];
      const ax = k(i, 0), ay = k(i, 1), az = k(i, 2), bx = k(j, 0), by = k(j, 1), bz = k(j, 2);
      let dx = bx - ax, dy = by - ay, dz = bz - az;
      const L = Math.hypot(dx, dy, dz) || 1;
      dx /= L; dy /= L; dz /= L;
      // ribbon 1 lies flat (width along d × up = (−dz, 0, dx)), ribbon 2 stands (width along d × that)
      let sx = -dz, sy = 0, sz = dx;
      let sl = Math.hypot(sx, sy, sz);
      if (sl < 1e-4) { sx = 1; sy = 0; sz = 0; sl = 1; }
      sx /= sl; sy /= sl; sz /= sl;
      const ux = dy * sz - dz * sy, uy = dz * sx - dx * sz, uz = dx * sy - dy * sx;
      for (const [wx, wy, wz, nx, ny, nz] of [[sx, sy, sz, ux, uy, uz], [ux, uy, uz, sx, sy, sz]]) {
        const h = CORD;
        for (const [px, py, pz, e] of [[ax, ay, az, -1], [ax, ay, az, 1], [bx, by, bz, -1], [bx, by, bz, 1]] as const) {
          P[w] = px + wx * h * e; P[w + 1] = py + wy * h * e; P[w + 2] = pz + wz * h * e;
          Nn[w] = nx; Nn[w + 1] = ny; Nn[w + 2] = nz;
          w += 3;
        }
      }
    }
    // webbing: each strap segment is a quad from the edge knots inward by STRAP (toward the net's middle, in plan)
    let cx = 0, cz = 0, n = 0;
    for (let i = 0; i < t.n; i++) if (t.pinned[i]) { cx += k(i, 0); cz += k(i, 2); n++; }
    cx /= n; cz /= n;
    for (const [i, j] of this.strap) {
      for (const q of [i, j]) {
        const x = k(q, 0), y = k(q, 1), z = k(q, 2);
        let ix = cx - x, iz = cz - z;
        const l = Math.hypot(ix, iz) || 1;
        ix /= l; iz /= l;
        P[w] = x; P[w + 1] = y; P[w + 2] = z; Nn[w] = 0; Nn[w + 1] = 1; Nn[w + 2] = 0; w += 3;
        P[w] = x + ix * STRAP; P[w + 1] = y; P[w + 2] = z + iz * STRAP; Nn[w] = 0; Nn[w + 1] = 1; Nn[w + 2] = 0; w += 3;
      }
    }
    (this.mesh.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
    (this.mesh.geometry.getAttribute('normal') as THREE.BufferAttribute).needsUpdate = true;
  }

  dispose() { this.mesh.geometry.dispose(); (this.mesh.material as THREE.Material).dispose(); }
}
