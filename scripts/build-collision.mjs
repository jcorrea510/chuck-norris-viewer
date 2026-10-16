#!/usr/bin/env node
// Builds the simulation's collision geometry from the real 2026 field model (public/field/field-2026.glb,
// AdvantageScope "2026 FRC Field", BSD-3): every triangle of the walls, guardrails, TRENCHes, TOWERs,
// BUMPs, DEPOTs and OUTPOSTs, sorted by surface (HDPE, polycarbonate, steel/aluminum) and simplified to
// ~1 cm. The carpet is a flat plane in the engine; the HUBs are built from the manual's dimensions in
// src/sim/phys/hub.ts (their internal ramps and the net need real behavior, not the visual mesh).
//
//   node scripts/build-collision.mjs      → public/field/field-collision.bin (+ .json index)
import fs from 'node:fs';
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';

await MeshoptDecoder.ready; await MeshoptSimplifier.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('public/field/field-2026.glb');

/** surface → triangle soup */
const soup = { hdpe: [], polycarb: [], steel: [] };
let dropped = 0;
for (const n of doc.getRoot().listNodes()) {
  const mesh = n.getMesh();
  if (!mesh) continue;
  if (/Hub/i.test(n.getName())) continue;               // HUBs: analytic (hub.ts)
  const W = n.getWorldMatrix();
  for (const p of mesh.listPrimitives()) {
    const mat = p.getMaterial();
    const c = mat?.getBaseColorFactor() ?? [0.5, 0.5, 0.5, 1];
    const A = p.getAttribute('POSITION');
    const I = p.getIndices();
    const el = [0, 0, 0];
    const v = (i) => { A.getElement(i, el); return [W[0] * el[0] + W[4] * el[1] + W[8] * el[2] + W[12], W[1] * el[0] + W[5] * el[1] + W[9] * el[2] + W[13], W[2] * el[0] + W[6] * el[1] + W[10] * el[2] + W[14]]; };
    const alliance = (c[0] > 0.9 && c[1] < 0.1) || (c[2] > 0.9 && c[0] < 0.1);
    const clear = c[3] < 0.5;
    const count = I ? I.getCount() : A.getCount();
    for (let t = 0; t < count; t += 3) {
      const a = v(I ? I.getScalar(t) : t), b = v(I ? I.getScalar(t + 1) : t + 1), d = v(I ? I.getScalar(t + 2) : t + 2);
      const ymax = Math.max(a[1], b[1], d[1]);
      // the carpet, tape lines and anything flush with it: the engine's ground plane covers them
      if (ymax < 0.004) { dropped++; continue; }
      // driver stations and everything outside the field perimeter never touch a robot or FUEL in play
      const xs = Math.min(Math.abs(a[0]), Math.abs(b[0]), Math.abs(d[0]));
      if (xs > 8.9) { dropped++; continue; }
      // the TRENCH arms over the lanes are solid boxes in the engine (Engine.addTrenchArms): a thin
      // triangle bar gives bogus contacts to a robot passing just under it
      const cx = (a[0] + b[0] + d[0]) / 3, cy = (a[1] + b[1] + d[1]) / 3, cz = (a[2] + b[2] + d[2]) / 3;
      if (Math.abs(Math.abs(cx) - 3.64) < 0.11 && cy > 0.54 && cy < 0.69 && Math.abs(cz) > 2.75 && Math.abs(cz) < 4.05) { dropped++; continue; }
      // the TOWER uprights and rungs are solid in the engine too (Engine.addTowers)
      if (Math.abs(Math.abs(cx) - 7.205) < 0.07 && cy > 0.03 && ((cx < 0 && cz > -0.33 && cz < 0.92) || (cx > 0 && cz < 0.33 && cz > -0.92))) { dropped++; continue; }
      const kind = clear ? 'polycarb' : alliance && ymax < 0.2 ? 'hdpe' : 'steel';
      soup[kind].push(a, b, d);
    }
  }
}

const out = [];
const index = { source: 'public/field/field-2026.glb (AdvantageScope 2026 FRC Field, BSD-3-Clause)', surfaces: [] };
let offset = 0;
for (const [kind, tris] of Object.entries(soup)) {
  // weld identical vertices (1 mm grid), then simplify to ~1 cm
  const map = new Map(), pos = [], idx = [];
  for (const p of tris) {
    const key = p.map((x) => Math.round(x * 1000)).join(',');
    let k = map.get(key);
    if (k === undefined) { k = pos.length / 3; map.set(key, k); pos.push(...p); }
    idx.push(k);
  }
  const P = new Float32Array(pos);
  let Ix = new Uint32Array(idx);
  // drop degenerate triangles
  const keep = [];
  for (let t = 0; t < Ix.length; t += 3) if (Ix[t] !== Ix[t + 1] && Ix[t + 1] !== Ix[t + 2] && Ix[t] !== Ix[t + 2]) keep.push(Ix[t], Ix[t + 1], Ix[t + 2]);
  Ix = new Uint32Array(keep);
  const before = Ix.length / 3;
  // error is relative to the mesh extent (~18 m): 0.0006 ≈ 1 cm
  const [simp] = MeshoptSimplifier.simplify(Ix, P, 3, 0, 0.0006, ['LockBorder']);
  // compact vertices
  const remap = new Map(), vp = [], vi = [];
  for (const i of simp) {
    let k = remap.get(i);
    if (k === undefined) { k = vp.length / 3; remap.set(i, k); vp.push(P[i * 3], P[i * 3 + 1], P[i * 3 + 2]); }
    vi.push(k);
  }
  const vbuf = Buffer.from(new Float32Array(vp).buffer), ibuf = Buffer.from(new Uint32Array(vi).buffer);
  index.surfaces.push({ kind, vertexOffset: offset, vertexCount: vp.length / 3, indexOffset: offset + vbuf.length, indexCount: vi.length });
  out.push(vbuf, ibuf);
  offset += vbuf.length + ibuf.length;
  console.log(`${kind.padEnd(9)} ${before} → ${vi.length / 3} triangles`);
}
fs.writeFileSync('public/field/field-collision.bin', Buffer.concat(out));
fs.writeFileSync('public/field/field-collision.json', JSON.stringify(index, null, 1));
console.log(`dropped ${dropped} carpet/outside triangles; wrote ${(offset / 1e6).toFixed(2)} MB`);
