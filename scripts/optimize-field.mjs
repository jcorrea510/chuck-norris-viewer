#!/usr/bin/env node
// Builds the simulation field from the 2026 REBUILT field model in cad/field/ (from the
// AdvantageScope assets: github.com/Mechanical-Advantage/AdvantageScopeAssets, "Field3d_2026FRCFieldV2").
//
//   npm run field
//
// Output (public/field/):
//   field-2026.glb   field, hubs, bumps, trenches and towers, merged per material, meshopt-compressed
//   fuel.glb         one FUEL ball (the app instances it)
//   field-2026.json  staged FUEL positions and the HUB scoring geometry
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { NodeIO, Document } from '@gltf-transform/core';
import { ALL_EXTENSIONS, EXTMeshoptCompression } from '@gltf-transform/extensions';
import { weld, compactPrimitive, meshopt, prune, dedup } from '@gltf-transform/functions';
import { MeshoptEncoder, MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const SRC = path.join(ROOT, 'cad', 'field');
const OUT = path.join(ROOT, 'public', 'field');
const SKIP = /\bPEM\b|rivet|screw|bolt|\bnut\b|washer|bearing|connector pin|gas spring|thread|standoff|shoulder|spacer|e-clip|zip tie/i;
const SIMPLIFY_M = 0.006; // fields are viewed from meters away; 6 mm is invisible
const MIN_PART_M = 0.04;  // parts smaller than this vanish at field scale

await MeshoptDecoder.ready; await MeshoptEncoder.ready; await MeshoptSimplifier.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder, 'meshopt.encoder': MeshoptEncoder });
const src = await io.read(path.join(SRC, 'model.glb'));
const tp = (m, x, y, z) => [m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]];

const buckets = new Map(); // key -> { color, alpha, pos[], nrm[], idx[] , hub }
const fuel = [];
const hubs = new Map(); // hub name -> { lo, hi, funnelLo, funnelHi }
let kept = 0, skipped = 0, srcTris = 0;

function walk(node, names) {
  const here = [...names, node.getName()];
  const joined = here.join(' / ');
  const mesh = node.getMesh();
  if (mesh) {
    const m = node.getWorldMatrix();
    const isFuel = /GE-26900|Fuel/i.test(node.getName()) || /GE-26900/.test(joined);
    const hubName = here.find((s) => /GE-26300: Hub/i.test(s)) ?? null;
    for (const prim of mesh.listPrimitives()) {
      const P = prim.getAttribute('POSITION').getArray();
      const N = prim.getAttribute('NORMAL')?.getArray();
      const I = prim.getIndices()?.getArray();
      srcTris += (I ? I.length : P.length / 3) / 3;
      if (isFuel) {
        const mn = prim.getAttribute('POSITION').getMin([]), mx = prim.getAttribute('POSITION').getMax([]);
        const c = tp(m, (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2);
        fuel.push(c.map((v) => Math.round(v * 1000) / 1000));
        continue;
      }
      if (SKIP.test(here.slice(-2).join(' '))) { skipped++; continue; }
      {
        const mn = prim.getAttribute('POSITION').getMin([]), mx = prim.getAttribute('POSITION').getMax([]);
        const a = tp(m, mn[0], mn[1], mn[2]), c = tp(m, mx[0], mx[1], mx[2]);
        if (Math.max(Math.abs(c[0] - a[0]), Math.abs(c[1] - a[1]), Math.abs(c[2] - a[2])) < MIN_PART_M && !hubName) { skipped++; continue; }
      }
      kept++;
      const mat = prim.getMaterial();
      const col = mat ? mat.getBaseColorFactor() : [0.6, 0.6, 0.6, 1];
      const key = `${hubName ?? 'field'}|${col.map((v) => v.toFixed(3)).join(',')}`;
      if (!buckets.has(key)) buckets.set(key, { color: col, hub: hubName, pos: [], nrm: [], idx: [] });
      const b = buckets.get(key);
      const base = b.pos.length / 3;
      for (let i = 0; i < P.length; i += 3) {
        const v = tp(m, P[i], P[i + 1], P[i + 2]);
        b.pos.push(v[0], v[1], v[2]);
        if (N) {
          const nx = m[0] * N[i] + m[4] * N[i + 1] + m[8] * N[i + 2];
          const ny = m[1] * N[i] + m[5] * N[i + 1] + m[9] * N[i + 2];
          const nz = m[2] * N[i] + m[6] * N[i + 1] + m[10] * N[i + 2];
          const l = Math.hypot(nx, ny, nz) || 1;
          b.nrm.push(nx / l, ny / l, nz / l);
        } else b.nrm.push(0, 1, 0);
        if (hubName) {
          const h = hubs.get(hubName) ?? { lo: [Infinity, Infinity, Infinity], hi: [-Infinity, -Infinity, -Infinity], fLo: [Infinity, Infinity, Infinity], fHi: [-Infinity, -Infinity, -Infinity] };
          for (let k = 0; k < 3; k++) { h.lo[k] = Math.min(h.lo[k], v[k]); h.hi[k] = Math.max(h.hi[k], v[k]); }
          if (/Hub Funnel Side/i.test(joined)) for (let k = 0; k < 3; k++) { h.fLo[k] = Math.min(h.fLo[k], v[k]); h.fHi[k] = Math.max(h.fHi[k], v[k]); }
          hubs.set(hubName, h);
        }
      }
      const n = P.length / 3;
      if (I) for (let i = 0; i < I.length; i++) b.idx.push(base + I[i]);
      else for (let i = 0; i < n; i++) b.idx.push(base + i);
    }
  }
  for (const c of node.listChildren()) walk(c, here);
}
for (const n of src.getRoot().getDefaultScene().listChildren()) walk(n, []);


// ---- physics grid for the simulation: floor height + solid obstacles, from downward raycasts
// Each 5 cm cell stores the floor height in mm (0–200: carpet, bumps, ramps) or 255 = solid
// (walls, HUB, trench posts, towers). Rays start at 0.45 m, so the trench arm (0.565 m up) is
// correctly *not* an obstacle: the robot fits under it.
const CELL = 0.05, GX0 = -8.45, GZ0 = -4.2, NX = Math.round(16.9 / CELL), NZ = Math.round(8.4 / CELL);
const heights = new Uint8Array(NX * NZ);
{
  let nTri = 0;
  for (const b of buckets.values()) if (b.color[3] > 0.5) nTri += b.idx.length / 3;
  const pos = new Float32Array(nTri * 9);
  let o = 0;
  for (const b of buckets.values()) {
    if (b.color[3] <= 0.5) continue; // nets and clear panels don't stop a robot
    for (let i = 0; i < b.idx.length; i++) { const v = b.idx[i] * 3; pos[o++] = b.pos[v]; pos[o++] = b.pos[v + 1]; pos[o++] = b.pos[v + 2]; }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const bvh = new MeshBVH(geo);
  const ray = new THREE.Ray(new THREE.Vector3(), new THREE.Vector3(0, -1, 0));
  let solid = 0;
  for (let iz = 0; iz < NZ; iz++) for (let ix = 0; ix < NX; ix++) {
    ray.origin.set(GX0 + (ix + 0.5) * CELL, 0.45, GZ0 + (iz + 0.5) * CELL);
    const hit = bvh.raycastFirst(ray, THREE.DoubleSide);
    const h = hit ? hit.point.y : 0;
    const outside = Math.abs(ray.origin.x) > 8.27 || Math.abs(ray.origin.z) > 4.035;
    let v;
    if (outside || h > 0.2) { v = 255; solid++; } else v = Math.max(0, Math.min(200, Math.round(h * 1000)));
    heights[iz * NX + ix] = v;
  }
  console.log(`Physics grid ${NX}×${NZ} @ ${CELL * 100} cm: ${(solid / (NX * NZ) * 100).toFixed(1)}% solid`);
}

// ---- field document
const doc = new Document();
const buffer = doc.createBuffer();
const scene = doc.createScene('Field');
doc.getRoot().setDefaultScene(scene);
const root = doc.createNode('2026 REBUILT Field');
scene.addChild(root);
const groupNodes = new Map();
for (const [key, b] of buckets) {
  const group = b.hub ? `hub:${[...hubs.keys()].indexOf(b.hub)}` : 'field';
  if (!groupNodes.has(group)) { const g = doc.createNode(group).setExtras({ group }); groupNodes.set(group, g); root.addChild(g); }
  const mat = doc.createMaterial(key).setBaseColorFactor(b.color).setMetallicFactor(0).setRoughnessFactor(0.7);
  if (b.color[3] < 0.99) mat.setAlphaMode('BLEND').setDoubleSided(true);
  const prim = doc.createPrimitive()
    .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(new Float32Array(b.pos)).setBuffer(buffer))
    .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(new Float32Array(b.nrm)).setBuffer(buffer))
    .setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(b.idx)).setBuffer(buffer))
    .setMaterial(mat);
  groupNodes.get(group).addChild(doc.createNode(key).setMesh(doc.createMesh(key).addPrimitive(prim)));
}
await doc.transform(weld());
for (const mesh of doc.getRoot().listMeshes()) for (const prim of mesh.listPrimitives()) {
  const a = prim.getAttribute('POSITION'); const mn = a.getMin([]), mx = a.getMax([]);
  const extent = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) || 1;
  // absolute error; Prune drops small disconnected pieces; big merged meshes get a sloppy pass too
  // (attribute seams stop the regular simplifier early). The field is seen from meters away.
  const P = a.getArray();
  let idx = MeshoptSimplifier.simplify(new Uint32Array(prim.getIndices().getArray()), P, 3, 0, SIMPLIFY_M, ['ErrorAbsolute', 'Prune'])[0];
  if (idx.length > 60000) idx = MeshoptSimplifier.simplifySloppy(idx, P, 3, null, Math.floor(idx.length * 0.45 / 3) * 3, (SIMPLIFY_M * 1.5) / extent)[0];
  prim.setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(idx)).setBuffer(prim.getIndices().getBuffer()));
  compactPrimitive(prim);
}
await doc.transform(dedup(), prune());
let tris = 0;
for (const mesh of doc.getRoot().listMeshes()) for (const p of mesh.listPrimitives()) tris += p.getIndices().getCount() / 3;
await doc.transform(meshopt({ encoder: MeshoptEncoder, level: 'high', quantizePosition: 16, quantizeNormal: 8 }));
doc.createExtension(EXTMeshoptCompression).setRequired(true).setEncoderOptions({ method: EXTMeshoptCompression.EncoderMethod.QUANTIZE });
fs.mkdirSync(OUT, { recursive: true });
const fieldBytes = await io.writeBinary(doc);
fs.writeFileSync(path.join(OUT, 'field-2026.glb'), fieldBytes);

// ---- one fuel ball
const fuelDoc = await io.read(path.join(SRC, 'model_0.glb'));
let fuelRadius = 0.075;
for (const mesh of fuelDoc.getRoot().listMeshes()) for (const p of mesh.listPrimitives()) {
  const a = p.getAttribute('POSITION'); const mn = a.getMin([]), mx = a.getMax([]);
  fuelRadius = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) / 2;
}
await fuelDoc.transform(meshopt({ encoder: MeshoptEncoder, level: 'high' }));
fuelDoc.createExtension(EXTMeshoptCompression).setRequired(true).setEncoderOptions({ method: EXTMeshoptCompression.EncoderMethod.QUANTIZE });
fs.writeFileSync(path.join(OUT, 'fuel.glb'), await io.writeBinary(fuelDoc));

// ---- scoring data
const r3 = (a) => a.map((v) => Math.round(v * 1000) / 1000);
const hubData = [...hubs.values()].map((h) => ({
  min: r3(h.lo), max: r3(h.hi),
  center: r3([(h.fLo[0] + h.fHi[0]) / 2, h.fHi[1], (h.fLo[2] + h.fHi[2]) / 2]),
  opening: r3([h.fHi[0] - h.fLo[0], h.fHi[2] - h.fLo[2]]),
})).sort((a, b) => a.center[0] - b.center[0]);
const config = JSON.parse(fs.readFileSync(path.join(SRC, 'config.json'), 'utf8'));
const data = {
  grid: { x0: GX0, z0: GZ0, cell: CELL, nx: NX, nz: NZ, heights: Buffer.from(heights).toString('base64') },
  source: 'AdvantageScope Field3d_2026FRCFieldV2 (Mechanical-Advantage/AdvantageScopeAssets)',
  units: 'meters, +Y up, origin at field center',
  carpet: { length: config.widthInches * 0.0254, width: config.heightInches * 0.0254 },
  fuelRadius: Math.round(fuelRadius * 10000) / 10000,
  hubs: hubData,
  fuel,
};
fs.writeFileSync(path.join(OUT, 'field-2026.json'), JSON.stringify(data));
console.log(`Field: kept ${kept} meshes, skipped ${skipped} hardware meshes, ${Math.round(srcTris).toLocaleString()} → ${Math.round(tris).toLocaleString()} tris`);
console.log(`Wrote public/field/field-2026.glb ${(fieldBytes.byteLength / 1e6).toFixed(2)} MB, fuel.glb, field-2026.json (${fuel.length} fuel, radius ${data.fuelRadius} m)`);
for (const h of hubData) console.log(`HUB center ${h.center.join(', ')}  opening ${h.opening.join(' × ')} m`);
