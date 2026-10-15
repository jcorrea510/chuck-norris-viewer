// The CAD part hierarchy written by scripts/optimize-model.mjs (public/models/chuck-norris.parts.json).
// Every merged mesh carries a per-vertex part id, so a single screw, a motor or a whole turret can
// be picked, highlighted, hidden or searched even though the robot is drawn in ~40 draw calls.
import * as THREE from 'three';

export interface PartsFile {
  version: number;
  subsystems: string[];
  mechanisms: string[];
  names: string[];
  tree: [number, number, number, number, number][]; // [name, parent, kind, subsystem, mechanism]
  partNode: number[];
  boxes: number[];
}

const HARDWARE = /screw|bolt|\bnut\b|locknut|rivet|washer|e-clip|shcs|bhcs|fhcs|collar|spacer|standoff|bearing|tee nut|insert|plug|zip tie|\bpin\b/i;
const GENERIC = /^(part|part \d+|\d+|)$/i;

export interface SearchResult { key: string; name: string; nodes: number[]; count: number; subsystem: string | null; kind: 'part' | 'assembly' }

export class PartIndex {
  readonly count: number; // number of part ids (pid 1..count)
  private names: string[];
  private nodeName: number[];
  readonly parent: Int32Array;
  readonly kind: Uint8Array;
  private sub: Int16Array;
  private mech: Int16Array;
  readonly partNode: Int32Array;
  readonly nodePid: Int32Array;
  private children: number[][];
  private partsUnder: Uint16Array[] = [];
  private boxes: Float32Array;
  readonly subsystems: string[];
  readonly mechanisms: string[];

  constructor(f: PartsFile) {
    this.names = f.names;
    this.subsystems = f.subsystems;
    this.mechanisms = f.mechanisms;
    const n = f.tree.length;
    this.nodeName = f.tree.map((t) => t[0]);
    this.parent = Int32Array.from(f.tree, (t) => t[1]);
    this.kind = Uint8Array.from(f.tree, (t) => t[2]);
    this.sub = Int16Array.from(f.tree, (t) => t[3]);
    this.mech = Int16Array.from(f.tree, (t) => t[4]);
    this.partNode = Int32Array.from(f.partNode);
    this.count = f.partNode.length - 1;
    this.nodePid = new Int32Array(n);
    for (let pid = 1; pid <= this.count; pid++) this.nodePid[this.partNode[pid]] = pid;
    this.boxes = Float32Array.from(f.boxes);
    this.children = Array.from({ length: n }, () => []);
    for (let i = 1; i < n; i++) if (this.parent[i] >= 0) this.children[this.parent[i]].push(i);
    // parts under each node (post-order accumulation)
    const acc: number[][] = Array.from({ length: n }, () => []);
    for (let i = n - 1; i >= 0; i--) {
      if (this.kind[i] === 1) acc[i].push(this.nodePid[i]);
      const p = this.parent[i];
      if (p >= 0) for (const x of acc[i]) acc[p].push(x);
    }
    this.partsUnder = acc.map((a) => Uint16Array.from(a));
  }

  name(node: number) { return this.names[this.nodeName[node]]; }
  isPart(node: number) { return this.kind[node] === 1; }
  partsOf(node: number): Uint16Array { return this.partsUnder[node]; }
  subsystemOf(node: number): string | null { const s = this.sub[node]; return s >= 0 ? this.subsystems[s] : null; }
  mechanismOf(node: number): string | null { const m = this.mech[node]; return m >= 0 ? this.mechanisms[m] : null; }
  nodeOfPart(pid: number) { return this.partNode[pid]; }
  childCount(node: number) { return this.children[node].length; }

  /** Mechanisms that any part under this node belongs to. */
  mechanismsUnder(node: number): string[] {
    const set = new Set<string>();
    for (const pid of this.partsUnder[node]) { const m = this.mechanismOf(this.partNode[pid]); if (m) set.add(m); }
    return [...set];
  }

  isHardware(node: number) { return HARDWARE.test(this.name(node)); }
  isGeneric(node: number) { return GENERIC.test(this.name(node).trim()); }

  /** What a tap on this part should select: the part itself, unless it is a fastener or unnamed. */
  meaningful(pid: number): number {
    let node = this.partNode[pid];
    while ((this.isHardware(node) || this.isGeneric(node)) && this.parent[node] > 0) node = this.parent[node];
    return node;
  }

  /** The next level up for "tap again to select the parent": skips wrappers with a single child. */
  up(node: number): number {
    let p = this.parent[node];
    while (p > 0 && this.children[p].length === 1 && this.parent[p] > 0) p = this.parent[p];
    // stop at the top of this subsystem: an assembly that spans several subsystems is not a useful level
    if (p <= 0 || this.sub[p] !== this.sub[node] || this.spansSubsystems(p)) return -1;
    return p;
  }

  private spanCache = new Map<number, boolean>();
  private spansSubsystems(node: number) {
    if (this.spanCache.has(node)) return this.spanCache.get(node)!;
    let first = -2, spans = false;
    for (const pid of this.partsUnder[node]) {
      const s = this.sub[this.partNode[pid]];
      if (first === -2) first = s; else if (s !== first) { spans = true; break; }
    }
    this.spanCache.set(node, spans);
    return spans;
  }

  /** Breadcrumb from the subsystem's top assembly down to this node. */
  path(node: number): number[] {
    const out: number[] = [];
    for (let n = node; n > 0; n = this.parent[n]) {
      if (this.spansSubsystems(n) || this.sub[n] !== this.sub[node]) break;
      if (n !== node && this.children[n].length === 1) continue;
      out.unshift(n);
    }
    return out;
  }

  boxOfParts(pids: ArrayLike<number>, out = new THREE.Box3()): THREE.Box3 {
    out.makeEmpty();
    const b = this.boxes, v = new THREE.Vector3();
    for (let i = 0; i < pids.length; i++) {
      const o = pids[i] * 6;
      if (b[o] === 0 && b[o + 3] === 0 && b[o + 1] === 0 && b[o + 4] === 0) continue;
      out.expandByPoint(v.set(b[o], b[o + 1], b[o + 2]));
      out.expandByPoint(v.set(b[o + 3], b[o + 4], b[o + 5]));
    }
    return out;
  }

  partsOfSubsystem(id: string): number[] {
    const s = this.subsystems.indexOf(id);
    const out: number[] = [];
    for (let pid = 1; pid <= this.count; pid++) if (this.sub[this.partNode[pid]] === s) out.push(pid);
    return out;
  }

  /** Parts whose own name or any ancestor's name contains one of the patterns. */
  partsMatching(patterns: string[]): number[] {
    const pats = patterns.map((p) => p.toLowerCase());
    const out: number[] = [];
    for (let pid = 1; pid <= this.count; pid++) {
      for (let n = this.partNode[pid]; n > 0; n = this.parent[n]) {
        const nm = this.name(n).toLowerCase();
        if (pats.some((p) => nm.includes(p))) { out.push(pid); break; }
      }
    }
    return out;
  }

  partsOfMechanism(id: string): number[] {
    const m = this.mechanisms.indexOf(id);
    const out: number[] = [];
    for (let pid = 1; pid <= this.count; pid++) if (this.mech[this.partNode[pid]] === m) out.push(pid);
    return out;
  }

  // ------------------------------------------------------------ search
  private searchIndex: SearchResult[] | null = null;
  private buildSearch() {
    const groups = new Map<string, SearchResult>();
    for (let node = 1; node < this.kind.length; node++) {
      if (this.isGeneric(node)) continue;
      const name = this.name(node);
      const key = `${this.kind[node]}|${name.toLowerCase()}`;
      let g = groups.get(key);
      if (!g) {
        g = { key, name, nodes: [], count: 0, subsystem: this.subsystemOf(node), kind: this.kind[node] === 1 ? 'part' : 'assembly' };
        groups.set(key, g);
      }
      g.nodes.push(node);
      g.count++;
      if (g.subsystem !== this.subsystemOf(node)) g.subsystem = null;
    }
    this.searchIndex = [...groups.values()];
  }

  search(query: string, limit = 30): SearchResult[] {
    if (!this.searchIndex) this.buildSearch();
    const terms = query.toLowerCase().split(/\s+/).filter(Boolean);
    if (!terms.length) return [];
    const scored: { r: SearchResult; s: number }[] = [];
    for (const r of this.searchIndex!) {
      const nm = r.name.toLowerCase();
      if (!terms.every((t) => nm.includes(t))) continue;
      let s = 0;
      if (nm.startsWith(terms[0])) s += 4;
      if (new RegExp(`\\b${terms[0].replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}`).test(nm)) s += 2;
      if (r.kind === 'assembly') s += 1.5;
      if (HARDWARE.test(nm)) s -= 3;
      s -= nm.length / 60;
      scored.push({ r, s });
    }
    scored.sort((a, b) => b.s - a.s);
    return scored.slice(0, limit).map((x) => x.r);
  }
}

/** Human label for what a node is, e.g. "Motor", "Roller", "Assembly · 12 parts". */
export function kindLabel(idx: PartIndex, node: number): string {
  const nm = idx.name(node);
  const n = idx.partsOf(node).length;
  const tests: [RegExp, string][] = [
    [/kraken|falcon|neo\b|vortex|\bmotor\b|cim\b/i, 'Motor'],
    [/planetary|gearbox/i, 'Gearbox'],
    [/roller/i, 'Roller'],
    [/flywheel/i, 'Flywheel'],
    [/wheel/i, 'Wheel'],
    [/belt/i, 'Belt'],
    [/chain/i, 'Chain'],
    [/sprocket/i, 'Sprocket'],
    [/pulley/i, 'Pulley'],
    [/gear/i, 'Gear'],
    [/limelight|camera/i, 'Vision Camera'],
    [/servo/i, 'Servo'],
    [/bearing/i, 'Bearing'],
    [/hex \(|shaft|axle/i, 'Shaft'],
    [/screw|bolt|bhcs|shcs/i, 'Fastener'],
    [/nut\b|locknut/i, 'Nut'],
    [/rivet/i, 'Rivet'],
    [/polycarb/i, 'Polycarbonate'],
    [/tube/i, 'Tube'],
    [/plate|panel/i, 'Plate'],
    [/sds mk4i|swerve/i, 'Swerve Module'],
  ];
  const base = tests.find(([r]) => r.test(nm))?.[1];
  if (!idx.isPart(node) && n > 1) return `${base && base !== 'Plate' ? base + ' ' : ''}Assembly · ${n} parts`;
  return base ?? 'Part';
}
