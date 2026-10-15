#!/usr/bin/env node
// Builds public/models/chuck-norris.glb from the raw Onshape export in cad/.
//
//   npm run model            -> optimize (reads cad/, never modifies it)
//   npm run model -- --inspect  -> print the tree, bounds and coverage only
//
// Pipeline: read (Draco) -> tag every mesh with its subsystem -> bake world
// transforms -> normalize units / up-axis / ground -> merge per subsystem per
// finish (CAD colors kept as vertex colors) -> weld -> (light simplify only if
// over budget) -> meshopt compression.

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { NodeIO, Document } from '@gltf-transform/core';
import { ALL_EXTENSIONS, EXTMeshoptCompression } from '@gltf-transform/extensions';
import { weld, simplify, meshopt, prune, dedup } from '@gltf-transform/functions';
import draco3d from 'draco3dgltf';
import { MeshoptEncoder, MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CAD_DIR = path.join(ROOT, 'cad');
const OUT = path.join(ROOT, 'public', 'models', 'chuck-norris.glb');
const CONFIG = JSON.parse(fs.readFileSync(path.join(ROOT, 'src', 'subsystems.json'), 'utf8'));
const INSPECT = process.argv.includes('--inspect');

const TARGET_MB = 15;
const HARD_CAP_MB = 25;
// Sub-millimetre: invisible even fully zoomed in. Tried in order only while over budget.
const SIMPLIFY_STEPS_M = [0, 0.00005, 0.0001, 0.00015, 0.0002, 0.0004];
const TRI_BUDGET = 3_500_000; // keeps the iPad at 60 fps with the ghost pass

// ---------------------------------------------------------------- find input
function findModel() {
  if (!fs.existsSync(CAD_DIR)) return null;
  const files = fs.readdirSync(CAD_DIR).filter((f) => /\.(glb|gltf)$/i.test(f));
  files.sort((a, b) => fs.statSync(path.join(CAD_DIR, b)).size - fs.statSync(path.join(CAD_DIR, a)).size);
  return files.length ? path.join(CAD_DIR, files[0]) : null;
}

// ---------------------------------------------------------------- matching
const subsystems = CONFIG.subsystems;
const fallback = subsystems.find((s) => !s.match || s.match.length === 0) ?? subsystems[subsystems.length - 1];
const excludes = (CONFIG.robot.exclude ?? []).map((s) => s.toLowerCase());
const clean = (n) => (n ?? '').replace(/^occurrence of\s*/i, '').trim();

function matchName(name) {
  const n = clean(name).toLowerCase();
  for (const s of subsystems) for (const p of s.match ?? []) if (n.includes(p.toLowerCase())) return s.id;
  return null;
}
// Deepest named ancestor wins, so "Breaker Mount" inside "Drivetrain" is electrical.
function subsystemForPath(names) {
  for (let i = names.length - 1; i >= 0; i--) {
    const id = matchName(names[i]);
    if (id) return id;
  }
  return fallback.id;
}

// ---------------------------------------------------------------- finishes
const METAL = /alumin|steel|stainless|brass|tube|plate|gusset|shaft|hex|bearing|screw|bolt|nut\b|locknut|rivet|angle|standoff|axle|sprocket|gear|flywheel|bracket|mount|ring|channel|bhcs|shcs|fhcs|washer|pin\b|hinge|pan\b/i;
const RUBBER = /rubber|tread|wheel|belt|sushi|grip|mecanum|compliant|bumper|tire/i;
const MOTOR = /kraken|falcon|neo\b|vortex|motor|cim/i;
const ELECTRONIC = /limelight|radio|pdh|power distribution|breaker|mpm|rsl|signal light|battery|cancoder|rio|camera/i;

function finishFor(names, rgb) {
  const leaf = names.map(clean).filter(Boolean).slice(-2).join(' ');
  const all = names.map(clean).join(' / ');
  if (/polycarb|polycarbonate|lexan/i.test(leaf) && !/bracket|mount/i.test(leaf)) return 'polycarb';
  if (/bumper/i.test(all) && !METAL.test(leaf)) return 'fabric';
  if (RUBBER.test(leaf)) return 'rubber';
  if (MOTOR.test(leaf)) return 'motor';
  if (ELECTRONIC.test(leaf) && !/mount|bracket/i.test(leaf)) return 'plastic';
  if (METAL.test(leaf)) return 'metal';
  const [r, g, b] = rgb;
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  if (max - min < 0.06 && max > 0.45) return 'metal'; // light neutral grey parts are almost always aluminum
  return 'plastic';
}

// ---------------------------------------------------------------- math
function normalMatrix(m) {
  const a = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
  const [a00, a01, a02, a10, a11, a12, a20, a21, a22] = a;
  const b01 = a22 * a11 - a12 * a21, b11 = -a22 * a10 + a12 * a20, b21 = a21 * a10 - a11 * a20;
  const det = a00 * b01 + a01 * b11 + a02 * b21;
  const id = 1 / det;
  // inverse, then transpose
  const inv = [
    b01 * id, (-a22 * a01 + a02 * a21) * id, (a12 * a01 - a02 * a11) * id,
    b11 * id, (a22 * a00 - a02 * a20) * id, (-a12 * a00 + a02 * a10) * id,
    b21 * id, (-a21 * a00 + a01 * a20) * id, (a11 * a00 - a01 * a10) * id,
  ];
  return { n: [inv[0], inv[3], inv[6], inv[1], inv[4], inv[7], inv[2], inv[5], inv[8]], det };
}

class Bucket {
  constructor() { this.jobs = []; this.vcount = 0; this.icount = 0; }
  fill() {
    this.pos = new Float32Array(this.vcount * 3);
    this.nrm = new Float32Array(this.vcount * 3);
    this.col = new Uint8Array(this.vcount * 3);
    this.idx = new Uint32Array(this.icount);
    let v = 0, t = 0;
    for (const { prim, m, n, flip, rgb } of this.jobs) {
      const P = prim.getAttribute('POSITION').getArray();
      const N = prim.getAttribute('NORMAL')?.getArray();
      const vc = P.length / 3;
      for (let i = 0; i < vc; i++) {
        const x = P[i * 3], y = P[i * 3 + 1], z = P[i * 3 + 2], o = (v + i) * 3;
        this.pos[o] = m[0] * x + m[4] * y + m[8] * z + m[12];
        this.pos[o + 1] = m[1] * x + m[5] * y + m[9] * z + m[13];
        this.pos[o + 2] = m[2] * x + m[6] * y + m[10] * z + m[14];
        if (N) {
          const nx = N[i * 3], ny = N[i * 3 + 1], nz = N[i * 3 + 2];
          const ox = n[0] * nx + n[3] * ny + n[6] * nz, oy = n[1] * nx + n[4] * ny + n[7] * nz, oz = n[2] * nx + n[5] * ny + n[8] * nz;
          const l = Math.hypot(ox, oy, oz) || 1;
          this.nrm[o] = ox / l; this.nrm[o + 1] = oy / l; this.nrm[o + 2] = oz / l;
        } else this.nrm[o + 1] = 1;
        this.col[o] = rgb[0]; this.col[o + 1] = rgb[1]; this.col[o + 2] = rgb[2];
      }
      const I = prim.getIndices()?.getArray();
      const ic = I ? I.length : vc;
      for (let k = 0; k < ic; k += 3) {
        const i0 = I ? I[k] : k, i1 = I ? I[k + 1] : k + 1, i2 = I ? I[k + 2] : k + 2;
        this.idx[t + k] = v + i0;
        this.idx[t + k + 1] = v + (flip ? i2 : i1);
        this.idx[t + k + 2] = v + (flip ? i1 : i2);
      }
      v += vc; t += ic;
    }
    this.jobs = null;
  }
}

// ---------------------------------------------------------------- main
async function main() {
  const input = findModel();
  if (!input) {
    console.log('No .glb/.gltf found in cad/. The site will use the built-in stand-in robot.');
    console.log('Export from Onshape as GLB and save it as cad/chuck-norris.glb, then run: npm run model');
    return;
  }
  console.log(`Reading ${path.relative(ROOT, input)} (${(fs.statSync(input).size / 1e6).toFixed(1)} MB)…`);
  await MeshoptDecoder.ready; await MeshoptEncoder.ready; await MeshoptSimplifier.ready;
  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      'draco3d.decoder': await draco3d.createDecoderModule(),
      'meshopt.decoder': MeshoptDecoder,
      'meshopt.encoder': MeshoptEncoder,
    });
  const src = await io.read(input);
  const scene = src.getRoot().getDefaultScene() ?? src.getRoot().listScenes()[0];

  // ---- walk, tag, print tree
  const buckets = new Map(); // `${sub}|${finish}` -> Bucket
  const coverage = new Map(subsystems.map((s) => [s.id, { parts: 0, tris: 0, names: new Set() }]));
  let excluded = 0;
  const treeLines = [];
  const stats = new Map(); // node -> {meshes, tris}

  function count(node) {
    if (stats.has(node)) return stats.get(node);
    let meshes = 0, tris = 0;
    const mesh = node.getMesh();
    if (mesh) {
      meshes = 1;
      for (const p of mesh.listPrimitives()) tris += (p.getIndices()?.getCount() ?? p.getAttribute('POSITION').getCount()) / 3;
    }
    for (const c of node.listChildren()) { const s = count(c); meshes += s.meshes; tris += s.tris; }
    const r = { meshes, tris }; stats.set(node, r); return r;
  }

  function visit(node, names, depth) {
    const nm = node.getName();
    const here = [...names, nm];
    if (excludes.some((e) => clean(nm).toLowerCase().includes(e))) { excluded++; return; }
    const kids = node.listChildren();
    const s = count(node);
    if (kids.length && depth <= 3 && s.meshes > 1) {
      treeLines.push(`${'  '.repeat(depth)}${clean(nm) || '(unnamed)'}  [${subsystemForPath(here)}]  meshes=${s.meshes} tris=${Math.round(s.tris).toLocaleString()}`);
    }
    const mesh = node.getMesh();
    if (mesh) {
      const sub = subsystemForPath(here);
      const cov = coverage.get(sub);
      cov.parts++; cov.names.add(clean(nm));
      if (INSPECT) for (const p of mesh.listPrimitives()) cov.tris += (p.getIndices()?.getCount() ?? 0) / 3;
      if (!INSPECT) {
        const m = node.getWorldMatrix();
        const { n, det } = normalMatrix(m);
        const flip = det < 0;
        for (const prim of mesh.listPrimitives()) {
          if (prim.getMode() !== 4) continue;
          const mat = prim.getMaterial();
          const c = mat ? mat.getBaseColorFactor() : [0.7, 0.7, 0.7, 1];
          const finish = finishFor(here, c);
          const key = `${sub}|${finish}`;
          if (!buckets.has(key)) buckets.set(key, new Bucket());
          const b = buckets.get(key);
          const vc = prim.getAttribute('POSITION').getCount();
          const tc = prim.getIndices()?.getCount() ?? vc;
          const rgb = [0, 1, 2].map((k) => Math.round(Math.min(1, Math.max(0, c[k])) * 255));
          b.jobs.push({ prim, m, n, flip, rgb });
          b.icount += tc;
          b.vcount += vc; cov.tris += tc / 3;
        }
      }
    }
    for (const c of kids) visit(c, here, depth + 1);
  }
  for (const n of scene.listChildren()) visit(n, [], 0);
  if (!INSPECT) for (const b of buckets.values()) b.fill();

  console.log('\n=== Assembly tree (depth ≤ 3) [subsystem] ===');
  for (const l of treeLines) console.log(l);

  // ---- bounds (raw)
  const allPos = [];
  let min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  const centroid = {};
  for (const [key, b] of buckets) {
    const sub = key.split('|')[0];
    const c = (centroid[sub] ??= { s: [0, 0, 0], n: 0 });
    for (let i = 0; i < b.pos.length; i += 3) {
      for (let k = 0; k < 3; k++) { const v = b.pos[i + k]; if (v < min[k]) min[k] = v; if (v > max[k]) max[k] = v; c.s[k] += v; }
      c.n++;
    }
  }

  console.log('\n=== Coverage ===');
  let totalParts = 0;
  for (const s of subsystems) {
    const c = coverage.get(s.id); totalParts += c.parts;
    console.log(`${s.name.padEnd(24)} ${String(c.parts).padStart(5)} parts  ${Math.round(c.tris).toLocaleString().padStart(11)} tris${s === fallback ? '   (fallback: everything unmatched)' : ''}`);
  }
  console.log(`Total ${totalParts} mesh parts, 100% assigned (${excluded} excluded by config: ${excludes.join(', ') || 'none'})`);
  if (INSPECT) return;

  const size = max.map((v, i) => v - min[i]);
  const maxExtent = Math.max(...size);
  let unitScale = 1, unitName = 'meters';
  if (maxExtent > 10) { unitScale = 0.001; unitName = 'millimeters'; }
  else if (maxExtent > 2.5) { unitScale = 0.0254; unitName = 'inches'; }
  // up-axis: the turrets/top sit above the swerve modules
  const cen = (id) => centroid[id] ? centroid[id].s.map((v) => v / centroid[id].n) : null;
  let upAxis = 1, upSign = 1;
  const lo = cen('swerve'), hi = cen('turrets') ?? cen('electrical');
  if (lo && hi) {
    const d = hi.map((v, i) => v - lo[i]);
    upAxis = [0, 1, 2].reduce((a, i) => (Math.abs(d[i]) > Math.abs(d[a]) ? i : a), 0);
    upSign = Math.sign(d[upAxis]) || 1;
  }
  const axisName = (a, s) => `${s > 0 ? '+' : '-'}${'XYZ'[a]}`;
  console.log(`\n=== Raw bounds ===\nmin ${min.map((v) => v.toFixed(4))}  max ${max.map((v) => v.toFixed(4))}\nsize ${size.map((v) => v.toFixed(3))} → units: ${unitName}; up-axis: ${axisName(upAxis, upSign)}`);

  // Rotation that maps detected up to +Y (right-handed, det = +1)
  const R = (() => {
    if (upAxis === 1) return upSign > 0 ? [[1, 0, 0], [0, 1, 0], [0, 0, 1]] : [[1, 0, 0], [0, -1, 0], [0, 0, -1]];
    if (upAxis === 2) return upSign > 0 ? [[1, 0, 0], [0, 0, 1], [0, -1, 0]] : [[1, 0, 0], [0, 0, -1], [0, 1, 0]];
    return upSign > 0 ? [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] : [[0, -1, 0], [1, 0, 0], [0, 0, 1]];
  })(); // rows: new x,y,z as combos of old
  const rot = (v) => R.map((r) => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);

  // new bounds after rotation + scale
  const corners = [];
  for (const x of [min[0], max[0]]) for (const y of [min[1], max[1]]) for (const z of [min[2], max[2]]) corners.push(rot([x, y, z]).map((v) => v * unitScale));
  const nmin = [0, 1, 2].map((k) => Math.min(...corners.map((c) => c[k])));
  const nmax = [0, 1, 2].map((k) => Math.max(...corners.map((c) => c[k])));
  const offset = [-(nmin[0] + nmax[0]) / 2, -nmin[1], -(nmin[2] + nmax[2]) / 2];
  console.log(`Normalized: Y-up, ground at y=0, centered. Size ${[0, 1, 2].map((k) => (nmax[k] - nmin[k]).toFixed(3)).join(' × ')} m`);

  for (const b of buckets.values()) {
    for (let i = 0; i < b.vcount; i++) {
      const o = i * 3;
      const p = rot([b.pos[o], b.pos[o + 1], b.pos[o + 2]]);
      b.pos[o] = p[0] * unitScale + offset[0]; b.pos[o + 1] = p[1] * unitScale + offset[1]; b.pos[o + 2] = p[2] * unitScale + offset[2];
      const q = rot([b.nrm[o], b.nrm[o + 1], b.nrm[o + 2]]);
      b.nrm[o] = q[0]; b.nrm[o + 1] = q[1]; b.nrm[o + 2] = q[2];
    }
  }

  // ---- build output document
  async function build(simplifyError) {
    const doc = new Document();
    const buffer = doc.createBuffer();
    const outScene = doc.createScene('Chuck Norris');
    const root = doc.createNode('Chuck Norris');
    outScene.addChild(root);
    const subNodes = new Map();
    for (const s of subsystems) {
      const n = doc.createNode(`subsystem:${s.id}`).setExtras({ subsystem: s.id, name: s.name });
      subNodes.set(s.id, n);
    }
    const materials = new Map();
    for (const [key, b] of buckets) {
      const [sub, finish] = key.split('|');
      if (!materials.has(finish)) {
        materials.set(finish, doc.createMaterial(finish).setExtras({ finish }).setDoubleSided(false)
          .setMetallicFactor(finish === 'metal' ? 1 : 0).setRoughnessFactor(finish === 'metal' ? 0.38 : 0.6));
      }
      const P = b.pos.slice(), N = b.nrm.slice(), C = b.col.slice(), I = b.idx.slice();
      const prim = doc.createPrimitive()
        .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(P).setBuffer(buffer))
        .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(N).setBuffer(buffer))
        .setAttribute('COLOR_0', doc.createAccessor().setType('VEC3').setArray(C).setNormalized(true).setBuffer(buffer))
        .setIndices(doc.createAccessor().setType('SCALAR').setArray(I).setBuffer(buffer))
        .setMaterial(materials.get(finish));
      const mesh = doc.createMesh(`${sub}:${finish}`).addPrimitive(prim);
      subNodes.get(sub).addChild(doc.createNode(`${sub}:${finish}`).setMesh(mesh).setExtras({ subsystem: sub }));
    }
    for (const n of subNodes.values()) if (n.listChildren().length) root.addChild(n);
    root.setExtras({ generator: 'scripts/optimize-model.mjs', source: path.basename(input), units: 'meters', up: '+Y' });

    await doc.transform(weld());
    if (simplifyError > 0) {
      // meshopt's error is relative to each mesh's size; subsystem meshes are ~0.5 m across,
      // so this keeps the absolute error at or below the requested distance.
      await doc.transform(simplify({ simplifier: MeshoptSimplifier, ratio: 0, error: simplifyError / 0.5 }));
    }
    await doc.transform(dedup(), prune());
    let tris = 0;
    for (const mesh of doc.getRoot().listMeshes()) for (const p of mesh.listPrimitives()) tris += p.getIndices().getCount() / 3;
    await doc.transform(meshopt({ encoder: MeshoptEncoder, level: 'high', quantizePosition: 16, quantizeNormal: 10, quantizeColor: 8 }));
    doc.createExtension(EXTMeshoptCompression).setRequired(true).setEncoderOptions({ method: EXTMeshoptCompression.EncoderMethod.QUANTIZE });
    const bytes = await io.writeBinary(doc);
    return { bytes, tris, drawCalls: doc.getRoot().listMeshes().length };
  }

  let result;
  for (const err of SIMPLIFY_STEPS_M) {
    process.stdout.write(`\nBuilding (simplify error ${err ? (err * 1000).toFixed(2) + ' mm' : 'off'})… `);
    result = await build(err);
    const mb = result.bytes.byteLength / 1e6;
    console.log(`${mb.toFixed(2)} MB, ${Math.round(result.tris).toLocaleString()} tris, ${result.drawCalls} draw calls`);
    if (mb <= TARGET_MB && result.tris <= TRI_BUDGET) break;
  }
  const mb = result.bytes.byteLength / 1e6;
  if (mb > HARD_CAP_MB) throw new Error(`Model is ${mb.toFixed(1)} MB, over the ${HARD_CAP_MB} MB hard cap.`);
  fs.mkdirSync(path.dirname(OUT), { recursive: true });
  fs.writeFileSync(OUT, result.bytes);
  console.log(`\nWrote ${path.relative(ROOT, OUT)}  ${mb.toFixed(2)} MB  ·  ${Math.round(result.tris).toLocaleString()} triangles  ·  ${result.drawCalls} meshes`);
}

main().catch((e) => { console.error(e); process.exit(1); });
