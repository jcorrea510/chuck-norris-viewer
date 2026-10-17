#!/usr/bin/env node
// Builds public/models/chuck-norris.glb (+ chuck-norris.parts.json) from the Onshape export in cad/.
//
//   npm run model                 -> optimize (reads cad/, never modifies it)
//   npm run model -- --inspect    -> print the tree, bounds and coverage only
//
// Pipeline: read (Draco) -> tag every part with its subsystem, mechanism and a numeric part id
// -> bake world transforms -> normalize units / up-axis / ground -> merge per subsystem per
// mechanism per finish (CAD colors as vertex colors, part id as a vertex attribute so single
// parts stay tappable after merging) -> weld -> light simplify only if over budget -> meshopt.

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { NodeIO, Document } from '@gltf-transform/core';
import { ALL_EXTENSIONS, EXTMeshoptCompression } from '@gltf-transform/extensions';
import { weld, simplifyPrimitive, compactPrimitive, meshopt, prune, dedup } from '@gltf-transform/functions';
import draco3d from 'draco3dgltf';
import { MeshoptEncoder, MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const CAD_DIR = path.join(ROOT, 'cad');
const OUT = path.join(ROOT, 'public', 'models', 'chuck-norris.glb');
const OUT_PARTS = path.join(ROOT, 'public', 'models', 'chuck-norris.parts.json');
// Low-detail copy for the simulation (same nodes and part ids, far fewer triangles): the robot is
// small on screen there, and the field + fuel + shadows need the GPU budget on an M1 MacBook Air.
const OUT_SIM = path.join(ROOT, 'public', 'models', 'chuck-norris-sim.glb');
const SIM_ERRORS_M = [0.003, 0.005, 0.008];
const SIM_TRI_BUDGET = 200_000;
const CONFIG = JSON.parse(fs.readFileSync(path.join(ROOT, 'src', 'subsystems.json'), 'utf8'));
const INSPECT = process.argv.includes('--inspect');

const TARGET_MB = 15;
const HARD_CAP_MB = 25;
// Absolute simplification error in meters, tried in order only while over budget.
// 0.15 mm is invisible even fully zoomed in on an iPad.
const SIMPLIFY_STEPS_M = [0, 0.00005, 0.0001, 0.00015, 0.0002, 0.0003];
const TRI_BUDGET = 1_400_000;

// ---------------------------------------------------------------- find input
function findModel() {
  if (!fs.existsSync(CAD_DIR)) return null;
  const files = fs.readdirSync(CAD_DIR).filter((f) => /\.(glb|gltf)$/i.test(f));
  // Newest export wins (so dropping in a new file "just works").
  files.sort((a, b) => fs.statSync(path.join(CAD_DIR, b)).mtimeMs - fs.statSync(path.join(CAD_DIR, a)).mtimeMs);
  return files.length ? path.join(CAD_DIR, files[0]) : null;
}

// ---------------------------------------------------------------- names & matching
const clean = (n) => (n ?? '').replace(/^occurrence of\s*/i, '').trim();
const display = (n) => clean(n).replace(/\s*<\d+>$/, '').trim() || 'Part';
const subsystems = CONFIG.subsystems;
const fallback = subsystems.find((s) => !s.match || s.match.length === 0) ?? subsystems[subsystems.length - 1];
const excludes = (CONFIG.robot.exclude ?? []).map((s) => s.toLowerCase());
const mechanisms = CONFIG.mechanisms ?? [];
const mechanismsChildrenFirst = [...mechanisms.filter((m) => m.parent), ...mechanisms.filter((m) => !m.parent)];
const isExcluded = (nm) => { const c = clean(nm).toLowerCase(); return excludes.some((e) => c === e || c.startsWith(e + ' <') || c.startsWith(e + ' (')); };

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

/** The mechanism a part belongs to (before deciding which four-bar link it rides on). */
function mechanismOf(lower, cadBox) {
  // children (the shooter hood) claim their parts before the parent mechanism (the turret) does
  for (const m of mechanismsChildrenFirst) {
    const p = m.parts ?? {};
    const wi = p.within ? lower.findIndex((n) => n.includes(p.within.toLowerCase())) : 0;
    if (wi < 0) continue;
    if (p.match && !lower.slice(wi).some((n) => p.match.some((x) => n.includes(x.toLowerCase())))) continue;
    if (p.region) {
      const { min, max } = p.region;
      if (!(cadBox.lo.every((v, k) => v >= min[k] - 1e-4) && cadBox.hi.every((v, k) => v <= max[k] + 1e-4))) continue;
    }
    return { m, wi };
  }
  return null;
}

/**
 * Four-bar link from names: crank / follower / swivel names match anywhere below the mechanism's
 * assembly (so whole sub-assemblies like "First Hopper Roller" ride the crank they are bolted to),
 * then parts sitting on a ground pivot axis; everything else is on the coupler.
 */
function linkByName(m, lower, wi, cadBox) {
  const c = cadBox.lo.map((v, k) => (v + cadBox.hi[k]) / 2);
  const nearAxis = (pv, r) => Math.hypot(c[1] - pv[1], c[2] - pv[2]) < r; // axis is X
  const below = lower.slice(wi + 1);
  const has = (list) => (list ?? []).some((x) => below.some((n) => n.includes(x.toLowerCase())));
  // hinged parts riding on the coupler (the kicker ramp and the bars through it swing clear of the bumper)
  if (has(m.swivel?.parts)) return 'swivel';
  if (has(m.crank) || nearAxis(m.pivot, 0.045)) return 'crank';
  if (has(m.followerLinks) || nearAxis(m.follower, 0.03)) return 'follower';
  return 'coupler';
}

// Screws, nuts, rivets and belts go with whatever they are fastened to: the link of the structural
// part their box overlaps most (filled by a first pass over the CAD, see collectLinks).
const ATTACHED = (leaf) => HARDWARE.test(leaf) || /\bbelt\b/i.test(leaf);
const linkParts = new Map(); // mech id -> [{ lo, hi, role }]
function overlap(a, b, grow = 0.001) {
  let v = 1;
  for (let k = 0; k < 3; k++) {
    const d = Math.min(a.hi[k], b.hi[k]) - Math.max(a.lo[k], b.lo[k]) + grow * 2;
    if (d <= 0) return 0;
    v *= d;
  }
  return v;
}

/** Which mechanism (and, for a four-bar, which link) a part rides on. */
function mechanismFor(names, cadBox) {
  const lower = names.map((n) => clean(n).toLowerCase());
  const hit = mechanismOf(lower, cadBox);
  if (!hit) return null;
  const { m, wi } = hit;
  if (m.type !== 'fourbar') return { mech: m, role: 'rotate' };
  if (ATTACHED(lower.slice(-2).join(' '))) {
    let best = null, bestV = 0;
    for (const p of linkParts.get(m.id) ?? []) {
      const v = overlap(cadBox, p);
      if (v > bestV) { bestV = v; best = p.role; }
    }
    if (best) return { mech: m, role: best };
  }
  return { mech: m, role: linkByName(m, lower, wi, cadBox) };
}

/** First pass: the four-bar link of every structural (non-hardware) part, for attaching hardware. */
function collectLinks(scene) {
  const visit = (node, names) => {
    const here = [...names, node.getName()];
    if (isExcluded(node.getName())) return;
    const mesh = node.getMesh();
    if (mesh) {
      const lower = here.map((n) => clean(n).toLowerCase());
      const cadBox = cadBoxOf(mesh, node.getWorldMatrix());
      const hit = mechanismOf(lower, cadBox);
      if (hit && hit.m.type === 'fourbar' && !ATTACHED(lower.slice(-2).join(' '))) {
        if (!linkParts.has(hit.m.id)) linkParts.set(hit.m.id, []);
        linkParts.get(hit.m.id).push({ ...cadBox, role: linkByName(hit.m, lower, hit.wi, cadBox) });
      }
    }
    for (const c of node.listChildren()) visit(c, here);
  };
  for (const n of scene.listChildren()) visit(n, []);
}

// ---------------------------------------------------------------- realistic colors
/** Onshape exports most parts white or black; give them the real-world look of their material. */
function colorFor(names, rgba, finish) {
  const leaf = names.map(clean).filter(Boolean).slice(-2).join(' ').toLowerCase();
  let [r, g, b] = rgba;
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  const neutral = max - min < 0.06;
  if (/brass/.test(leaf)) return [0.78, 0.58, 0.27];
  if (/belt/.test(leaf)) return [0.07, 0.07, 0.075];
  if (/chain/.test(leaf)) return [0.5, 0.51, 0.53];
  if (/kraken|falcon/.test(leaf) && neutral) return [0.06, 0.062, 0.07];
  if (/\bhex \(|hex shaft|output shaft|axle/.test(leaf) && neutral) return [0.34, 0.35, 0.37];
  if (finish === 'steel' && neutral) return max > 0.5 ? [0.62, 0.63, 0.65] : [0.05, 0.05, 0.055];
  if (finish === 'metal' && neutral && max > 0.6) return [0.78, 0.79, 0.81]; // natural 6061 aluminum
  if (neutral && max < 0.06) return [0.03, 0.032, 0.036]; // black anodize / black polycarb
  if (!neutral) {
    // Boost saturation of genuinely colored parts so they read under studio light.
    const l = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    const k = 1.35;
    [r, g, b] = [r, g, b].map((v) => Math.min(1, Math.max(0, l + (v - l) * k)));
  }
  return [r, g, b];
}

// ---------------------------------------------------------------- finishes
const HARDWARE = /screw|bolt|\bnut\b|locknut|rivet|washer|e-clip|shcs|bhcs|fhcs|collar|tee nut|pem\b|insert/i;
const METAL = /alumin|steel|stainless|brass|tube|plate|gusset|shaft|hex|bearing|angle|standoff|axle|sprocket|gear|flywheel|bracket|mount|channel|hinge|pan\b|chain|spacer|hub\b|block/i;
const RUBBER = /rubber|tread|wheel|belt|sushi|grip|compliant|tire|flex/i;
const MOTOR = /kraken|falcon|neo\b|vortex|motor|cim|planetary/i;
const ELECTRONIC = /limelight|radio|pdh|power distribution|breaker|mpm|rsl|signal light|battery|cancoder|rio|camera|servo/i;

function finishFor(names, rgba) {
  const leaf = names.map(clean).filter(Boolean).slice(-2).join(' ');
  const all = names.map(clean).join(' / ');
  if (/polycarb|polycarbonate|lexan/i.test(leaf) && !/bracket|mount|spacer/i.test(leaf)) return 'polycarb';
  if (rgba[3] < 0.95) return 'polycarb';
  if (/bumper/i.test(all) && !METAL.test(leaf) && !HARDWARE.test(leaf)) return 'fabric';
  if (HARDWARE.test(leaf)) return 'steel';
  if (/3dp|printed|pla\b|nylon/i.test(leaf)) return 'plastic';
  if (RUBBER.test(leaf)) return 'rubber';
  if (MOTOR.test(leaf)) return 'motor';
  if (ELECTRONIC.test(leaf) && !/mount|bracket/i.test(leaf)) return 'plastic';
  if (METAL.test(leaf)) return 'metal';
  const [r, g, b] = rgba;
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  if (max - min < 0.06 && max > 0.45) return 'metal'; // light neutral parts are almost always aluminum
  return 'plastic';
}

// ---------------------------------------------------------------- math
const tp = (m, x, y, z) => [m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13], m[2] * x + m[6] * y + m[10] * z + m[14]];
function normalMatrix(m) {
  const [a00, a01, a02, a10, a11, a12, a20, a21, a22] = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
  const b01 = a22 * a11 - a12 * a21, b11 = -a22 * a10 + a12 * a20, b21 = a21 * a10 - a11 * a20;
  const det = a00 * b01 + a01 * b11 + a02 * b21;
  const id = 1 / det;
  const inv = [
    b01 * id, (-a22 * a01 + a02 * a21) * id, (a12 * a01 - a02 * a11) * id,
    b11 * id, (a22 * a00 - a02 * a20) * id, (-a12 * a00 + a02 * a10) * id,
    b21 * id, (-a21 * a00 + a01 * a20) * id, (a11 * a00 - a01 * a10) * id,
  ];
  return { n: [inv[0], inv[3], inv[6], inv[1], inv[4], inv[7], inv[2], inv[5], inv[8]], det };
}
function cadBoxOf(mesh, m) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of mesh.listPrimitives()) {
    const a = p.getAttribute('POSITION');
    const mn = a.getMin([]), mx = a.getMax([]);
    for (const x of [mn[0], mx[0]]) for (const y of [mn[1], mx[1]]) for (const z of [mn[2], mx[2]]) {
      const v = tp(m, x, y, z);
      for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], v[k]); hi[k] = Math.max(hi[k], v[k]); }
    }
  }
  return { lo, hi };
}

class Bucket {
  constructor() { this.jobs = []; this.vcount = 0; this.icount = 0; }
  fill() {
    this.pos = new Float32Array(this.vcount * 3);
    this.nrm = new Float32Array(this.vcount * 3);
    this.col = new Uint8Array(this.vcount * 3);
    this.pid = new Uint16Array(this.vcount);
    this.idx = new Uint32Array(this.icount);
    let v = 0, t = 0;
    for (const { prim, m, n, flip, rgb, pid } of this.jobs) {
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
        this.pid[v + i] = pid;
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
    console.log('Export from Onshape as GLB, save it in cad/, then run: npm run model');
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
  collectLinks(scene);

  // ---- walk: hierarchy, tagging, buckets
  const buckets = new Map(); // `${sub}|${mech}|${finish}` -> Bucket
  const coverage = new Map(subsystems.map((s) => [s.id, { parts: 0, tris: 0 }]));
  const mechCount = new Map(mechanisms.map((m) => [m.id, 0]));
  const roleNames = new Map(); // `${id}#${role}` -> names (for the report)
  const pivots = new Map(); // mech id -> CAD-space pivot
  let excluded = 0;
  const treeLines = [];
  const stats = new Map();

  // Part hierarchy for the app: tree nodes [name, parent, kind(0 asm / 1 part), subsystemIndex, mechIndex]
  const tree = [['Chuck Norris', -1, 0, -1, -1]];
  const partNode = [0]; // pid -> tree index (pid 0 unused)
  const subIndex = new Map(subsystems.map((s, i) => [s.id, i]));

  function count(node) {
    if (stats.has(node)) return stats.get(node);
    let meshes = 0, tris = 0;
    const mesh = node.getMesh();
    if (mesh) { meshes = 1; for (const p of mesh.listPrimitives()) tris += (p.getIndices()?.getCount() ?? p.getAttribute('POSITION').getCount()) / 3; }
    for (const c of node.listChildren()) { const s = count(c); meshes += s.meshes; tris += s.tris; }
    const r = { meshes, tris }; stats.set(node, r); return r;
  }

  function visit(node, names, depth, treeParent) {
    const nm = node.getName();
    const here = [...names, nm];
    if (isExcluded(nm)) { excluded++; return; }
    const kids = node.listChildren();
    const s = count(node);
    if (kids.length && depth <= 3 && s.meshes > 1) {
      treeLines.push(`${'  '.repeat(depth)}${clean(nm) || '(unnamed)'}  [${subsystemForPath(here)}]  meshes=${s.meshes} tris=${Math.round(s.tris).toLocaleString()}`);
    }
    const mesh = node.getMesh();
    // Onshape wraps each part in "occurrence of X" → "X"(mesh). Collapse that pair into one part.
    const isWrapper = !mesh && kids.length === 1 && kids[0].getMesh() && kids[0].listChildren().length === 0;
    let myTree = treeParent;
    if (!mesh && !isWrapper && depth > 0) {
      const sub = subsystemForPath(here);
      tree.push([display(nm), treeParent, 0, subIndex.get(sub), -1]);
      myTree = tree.length - 1;
    }
    if (mesh) {
      const sub0 = subsystemForPath(here);
      const m = node.getWorldMatrix();
      const cadBox = cadBoxOf(mesh, m);
      const hit = mechanismFor(here, cadBox);
      const mech = hit?.mech ?? null;
      const sub = mech && mech.ownsParts !== false ? mech.subsystem ?? sub0 : sub0;
      for (const mm of mechanisms) {
        if (mm.swivel?.hingePart && !pivots.has(mm.id + '#swivel') && here.some((h) => clean(h).toLowerCase().includes(mm.swivel.hingePart.toLowerCase()))) {
          pivots.set(mm.id + '#swivel', cadBox.lo.map((v, k) => (v + cadBox.hi[k]) / 2));
        }
        if (mm.pivotPart && !pivots.has(mm.id) && mm.pivotPart.every((p) => here.some((h) => clean(h).toLowerCase().includes(p.toLowerCase())))) {
          pivots.set(mm.id, cadBox.lo.map((v, k) => (v + cadBox.hi[k]) / 2));
        }
      }
      const partName = display(names[names.length - 1]?.toLowerCase().startsWith('occurrence of') ? names[names.length - 1] : nm);
      tree.push([partName, treeParent, 1, subIndex.get(sub), mech ? mechanisms.indexOf(mech) : -1]);
      const pid = partNode.length;
      partNode.push(tree.length - 1);
      if (pid > 65535) throw new Error('More than 65535 parts');
      const cov = coverage.get(sub);
      cov.parts++;
      if (mech) {
        mechCount.set(mech.id, mechCount.get(mech.id) + 1);
        const k = `${mech.id}#${hit.role}`;
        if (!roleNames.has(k)) roleNames.set(k, new Map());
        const nmap = roleNames.get(k); const dn = display(nm); nmap.set(dn, (nmap.get(dn) ?? 0) + 1);
      }
      const { n, det } = normalMatrix(m);
      for (const prim of mesh.listPrimitives()) {
        if (prim.getMode() !== 4) continue;
        const mat = prim.getMaterial();
        const c = mat ? mat.getBaseColorFactor() : [0.7, 0.7, 0.7, 1];
        const tc = prim.getIndices()?.getCount() ?? prim.getAttribute('POSITION').getCount();
        cov.tris += tc / 3;
        if (INSPECT) continue;
        const finish = finishFor(here, c);
        const key = `${sub}|${mech ? `${mech.id}#${hit.role}` : '-'}|${finish}`;
        if (!buckets.has(key)) buckets.set(key, new Bucket());
        const b = buckets.get(key);
        const rgb = colorFor(here, c, finish).map((v) => Math.round(Math.min(1, Math.max(0, v)) * 255));
        b.jobs.push({ prim, m, n, flip: det < 0, rgb, pid });
        b.icount += tc;
        b.vcount += prim.getAttribute('POSITION').getCount();
      }
    }
    for (const c of kids) visit(c, isWrapper ? here : here, depth + 1, myTree);
  }
  for (const n of scene.listChildren()) visit(n, [], 0, 0);

  console.log('\n=== Assembly tree (depth ≤ 3) [subsystem] ===');
  for (const l of treeLines) console.log(l);
  console.log('\n=== Coverage ===');
  let totalParts = 0;
  for (const s of subsystems) {
    const c = coverage.get(s.id); totalParts += c.parts;
    console.log(`${s.name.padEnd(24)} ${String(c.parts).padStart(5)} parts  ${Math.round(c.tris).toLocaleString().padStart(11)} tris${s === fallback ? '   (fallback: everything unmatched)' : ''}`);
  }
  console.log(`Total ${totalParts} parts, 100% assigned (${excluded} excluded by config: ${excludes.join(', ') || 'none'})`);
  console.log('\n=== Mechanisms ===');
  for (const m of mechanisms) {
    const pv = m.pivot ?? pivots.get(m.id);
    console.log(`${m.name.padEnd(18)} ${String(mechCount.get(m.id)).padStart(4)} parts  pivot ${pv ? pv.map((v) => v.toFixed(4)).join(', ') : 'NOT FOUND'}  axis ${m.axis.join(',')}`);
    if (!pv) throw new Error(`Mechanism ${m.id}: pivot not found`);
    if (!mechCount.get(m.id)) throw new Error(`Mechanism ${m.id}: no parts matched`);
    if (m.type === 'fourbar') for (const role of ['crank', 'follower', 'coupler', 'swivel']) {
      const nmap = roleNames.get(`${m.id}#${role}`) ?? new Map();
      const n = [...nmap.values()].reduce((a, b) => a + b, 0);
      console.log(`   ${role.padEnd(9)} ${String(n).padStart(4)} parts: ${[...nmap.entries()].filter(([k]) => role !== 'coupler' || !HARDWARE.test(k)).slice(0, role === 'coupler' ? 6 : 12).map(([k, v]) => v > 1 ? `${k} ×${v}` : k).join(', ')}`);
    }
  }
  if (INSPECT) return;

  for (const b of buckets.values()) b.fill();

  // ---- bounds (raw)
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
  const size = max.map((v, i) => v - min[i]);
  const maxExtent = Math.max(...size);
  let unitScale = 1, unitName = 'meters';
  if (maxExtent > 10) { unitScale = 0.001; unitName = 'millimeters'; }
  else if (maxExtent > 2.5) { unitScale = 0.0254; unitName = 'inches'; }
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

  const R = (() => {
    if (upAxis === 1) return upSign > 0 ? [[1, 0, 0], [0, 1, 0], [0, 0, 1]] : [[1, 0, 0], [0, -1, 0], [0, 0, -1]];
    if (upAxis === 2) return upSign > 0 ? [[1, 0, 0], [0, 0, 1], [0, -1, 0]] : [[1, 0, 0], [0, 0, -1], [0, 1, 0]];
    return upSign > 0 ? [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] : [[0, -1, 0], [1, 0, 0], [0, 0, 1]];
  })();
  const rot = (v) => R.map((r) => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
  const corners = [];
  for (const x of [min[0], max[0]]) for (const y of [min[1], max[1]]) for (const z of [min[2], max[2]]) corners.push(rot([x, y, z]).map((v) => v * unitScale));
  const nmin = [0, 1, 2].map((k) => Math.min(...corners.map((c) => c[k])));
  const nmax = [0, 1, 2].map((k) => Math.max(...corners.map((c) => c[k])));
  const offset = [-(nmin[0] + nmax[0]) / 2, -nmin[1], -(nmin[2] + nmax[2]) / 2];
  const toOut = (p) => rot(p).map((v, k) => v * unitScale + offset[k]);
  console.log(`Normalized: Y-up, ground at y=0, centered. Size ${[0, 1, 2].map((k) => (nmax[k] - nmin[k]).toFixed(3)).join(' × ')} m`);

  const nParts = partNode.length;
  const pbox = new Float32Array(nParts * 6);
  for (let i = 0; i < nParts; i++) pbox.set([Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity], i * 6);
  for (const b of buckets.values()) {
    for (let i = 0; i < b.vcount; i++) {
      const o = i * 3;
      const p = toOut([b.pos[o], b.pos[o + 1], b.pos[o + 2]]);
      b.pos[o] = p[0]; b.pos[o + 1] = p[1]; b.pos[o + 2] = p[2];
      const q = rot([b.nrm[o], b.nrm[o + 1], b.nrm[o + 2]]);
      b.nrm[o] = q[0]; b.nrm[o + 1] = q[1]; b.nrm[o + 2] = q[2];
      const pb = b.pid[i] * 6;
      for (let k = 0; k < 3; k++) { if (p[k] < pbox[pb + k]) pbox[pb + k] = p[k]; if (p[k] > pbox[pb + 3 + k]) pbox[pb + 3 + k] = p[k]; }
    }
  }

  const vecOut = (v) => rot(v).map((x) => x * unitScale);
  const mechOut = new Map(mechanisms.map((m) => [m.id, {
    pivot: toOut(m.pivot ?? pivots.get(m.id)),
    follower: m.follower ? toOut(m.follower) : null,
    link: m.couplerJoint ? vecOut(m.couplerJoint.map((v, k) => v - m.pivot[k])) : null,
    swivel: pivots.has(m.id + '#swivel') ? toOut(pivots.get(m.id + '#swivel')) : null,
    axis: rot(m.axis),
  }]));

  // ---- build output document
  // Hardware (screws, nuts, rivets…) is left out of the simulation copy: invisible at field scale.
  const hardwarePid = new Uint8Array(nParts);
  for (let pid = 1; pid < nParts; pid++) if (HARDWARE.test(tree[partNode[pid]][0])) hardwarePid[pid] = 1;

  async function build(simplifyError, sim = false) {
    const doc = new Document();
    const buffer = doc.createBuffer();
    const outScene = doc.createScene('Chuck Norris');
    doc.getRoot().setDefaultScene(outScene);
    const root = doc.createNode('Chuck Norris');
    outScene.addChild(root);
    const subNodes = new Map(), mechNodes = new Map(), nested = [];
    for (const s of subsystems) subNodes.set(s.id, doc.createNode(`subsystem:${s.id}`).setExtras({ subsystem: s.id, name: s.name }));
    const materials = new Map();
    for (const [key, b] of buckets) {
      const [sub, mech, finish] = key.split('|');
      if (!materials.has(finish)) {
        materials.set(finish, doc.createMaterial(finish).setExtras({ finish })
          .setMetallicFactor(finish === 'metal' || finish === 'steel' ? 1 : 0).setRoughnessFactor(0.5));
      }
      const prim = doc.createPrimitive()
        .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(b.pos.slice()).setBuffer(buffer))
        .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(b.nrm.slice()).setBuffer(buffer))
        .setAttribute('COLOR_0', doc.createAccessor().setType('VEC3').setArray(b.col.slice()).setNormalized(true).setBuffer(buffer))
        .setAttribute('_PARTID', doc.createAccessor().setType('SCALAR').setArray(b.pid.slice()).setBuffer(buffer))
        .setIndices(doc.createAccessor().setType('SCALAR').setArray(b.idx.slice()).setBuffer(buffer))
        .setMaterial(materials.get(finish));
      const name = mech === '-' ? `${sub}:${finish}` : `${sub}:${mech}:${finish}`;
      const meshNode = doc.createNode(name).setMesh(doc.createMesh(name).addPrimitive(prim)).setExtras({ subsystem: sub });
      if (mech === '-') { subNodes.get(sub).addChild(meshNode); continue; }
      if (!mechNodes.has(mech)) {
        const [id, role] = mech.split('#');
        const def = mechanisms.find((m) => m.id === id);
        const out = mechOut.get(id);
        // rotate / crank turn about the pivot, follower about its own pivot, coupler only translates
        const pivot = role === 'follower' ? out.follower : role === 'coupler' ? [0, 0, 0] : role === 'swivel' ? out.swivel : out.pivot;
        const pivotNode = doc.createNode(`mechanism:${mech}`).setTranslation(pivot).setExtras({
          mechanism: id, role, type: def.type ?? 'rotate', name: def.name, axis: out.axis, link: out.link,
          min: def.min, max: def.max, default: def.default ?? 0,
          states: def.states ?? null, liftRobot: def.liftRobot ?? 0, unit: def.unit ?? '°', hidden: !!def.hidden,
        });
        const offsetNode = doc.createNode(`mechanism:${mech}:offset`).setTranslation(pivot.map((v) => -v));
        pivotNode.addChild(offsetNode);
        // a mechanism with a parent (the shooter hood on its turret) rides inside the parent's offset node
        if (def.parent) nested.push({ pivotNode, parent: `${def.parent}#rotate` }); else subNodes.get(sub).addChild(pivotNode);
        mechNodes.set(mech, offsetNode);
      }
      mechNodes.get(mech).addChild(meshNode);
    }
    for (const { pivotNode, parent } of nested) mechNodes.get(parent).addChild(pivotNode);
    for (const n of subNodes.values()) if (n.listChildren().length) root.addChild(n);
    root.setExtras({ generator: 'scripts/optimize-model.mjs', source: path.basename(input), units: 'meters', up: '+Y', parts: nParts - 1 });

    await doc.transform(weld());
    if (sim) {
      for (const mesh of doc.getRoot().listMeshes()) for (const prim of mesh.listPrimitives()) {
        const idx = prim.getIndices().getArray(), pidA = prim.getAttribute('_PARTID').getArray();
        const pos = prim.getAttribute('POSITION').getArray();
        const keep = [];
        for (let t = 0; t < idx.length; t += 3) if (!hardwarePid[pidA[idx[t]]]) keep.push(idx[t], idx[t + 1], idx[t + 2]);
        let out = new Uint32Array(keep);
        // absolute error; Prune drops small disconnected pieces entirely
        const P = pos instanceof Float32Array ? pos : new Float32Array(pos);
        // bumpers keep their shape and their team numbers (small separate pieces Prune would delete)
        const fabric = (prim.getMaterial()?.getName() ?? '') === 'fabric';
        if (out.length) out = MeshoptSimplifier.simplify(out, P, 3, 0, fabric ? 0.0008 : simplifyError, fabric ? ['ErrorAbsolute'] : ['ErrorAbsolute', 'Prune'])[0];
        // attribute seams stop the regular simplifier early; at field scale a sloppy pass is invisible
        if (out.length > 3000 && !fabric) {
          const a = prim.getAttribute('POSITION'), mn = a.getMin([]), mx = a.getMax([]);
          const extent = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) || 1;
          out = MeshoptSimplifier.simplifySloppy(out, P, 3, null, Math.floor(out.length * 0.3 / 3) * 3, simplifyError / extent)[0];
        }
        if (!out.length) out = new Uint32Array([0, 0, 0]); // keep every mesh so names still line up
        prim.setIndices(doc.createAccessor().setType('SCALAR').setArray(new Uint32Array(out)).setBuffer(buffer));
        compactPrimitive(prim);
      }
    } else if (simplifyError > 0) {
      for (const mesh of doc.getRoot().listMeshes()) for (const prim of mesh.listPrimitives()) {
        const a = prim.getAttribute('POSITION');
        const mn = a.getMin([]), mx = a.getMax([]);
        const extent = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) || 1;
        // meshopt's error is relative to the mesh extent; convert to an absolute distance.
        simplifyPrimitive(prim, { simplifier: MeshoptSimplifier, ratio: 0, error: simplifyError / extent, lockBorder: false });
      }
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
    console.log(`${mb.toFixed(2)} MB, ${Math.round(result.tris).toLocaleString()} tris, ${result.drawCalls} meshes`);
    if (mb <= TARGET_MB && result.tris <= TRI_BUDGET) break;
  }
  const mb = result.bytes.byteLength / 1e6;
  if (mb > HARD_CAP_MB) throw new Error(`Model is ${mb.toFixed(1)} MB, over the ${HARD_CAP_MB} MB hard cap.`);
  fs.mkdirSync(path.dirname(OUT), { recursive: true });
  fs.writeFileSync(OUT, result.bytes);

  let simResult;
  for (const err of SIM_ERRORS_M) {
    process.stdout.write(`\nSimulation copy (simplify error ${(err * 1000).toFixed(1)} mm)… `);
    simResult = await build(err, true);
    console.log(`${(simResult.bytes.byteLength / 1e6).toFixed(2)} MB, ${Math.round(simResult.tris).toLocaleString()} tris`);
    if (simResult.tris <= SIM_TRI_BUDGET) break;
  }
  fs.writeFileSync(OUT_SIM, simResult.bytes);

  // ---- parts file: names are pooled to keep it small
  const names = [], nameIdx = new Map();
  const pool = (s) => { if (!nameIdx.has(s)) { nameIdx.set(s, names.length); names.push(s); } return nameIdx.get(s); };
  const parts = {
    version: 1,
    subsystems: subsystems.map((s) => s.id),
    mechanisms: mechanisms.map((m) => m.id),
    names,
    // [nameIndex, parentTreeIndex, kind (0 assembly, 1 part), subsystemIndex, mechanismIndex]
    tree: tree.map(([n, p, k, s, m]) => [pool(n), p, k, s ?? -1, m]),
    partNode,
    // per part id: [minX, minY, minZ, maxX, maxY, maxZ] in model meters (millimetre precision)
    boxes: Array.from(pbox, (v) => (Number.isFinite(v) ? Math.round(v * 1000) / 1000 : 0)),
  };
  fs.writeFileSync(OUT_PARTS, JSON.stringify(parts));
  console.log(`\nWrote ${path.relative(ROOT, OUT)}  ${mb.toFixed(2)} MB  ·  ${Math.round(result.tris).toLocaleString()} triangles  ·  ${result.drawCalls} meshes`);
  console.log(`Wrote ${path.relative(ROOT, OUT_SIM)}  ${(simResult.bytes.byteLength / 1e6).toFixed(2)} MB  ·  ${Math.round(simResult.tris).toLocaleString()} triangles`);
  console.log(`Wrote ${path.relative(ROOT, OUT_PARTS)}  ${(fs.statSync(OUT_PARTS).size / 1e3).toFixed(0)} KB  ·  ${nParts - 1} parts  ·  ${tree.length} tree nodes`);
}

main().catch((e) => { console.error(e); process.exit(1); });
