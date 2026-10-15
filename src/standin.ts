import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

/**
 * Procedural stand-in robot, used only when public/models/chuck-norris.glb is missing.
 * It has the exact structure the build step produces (root → "subsystem:<id>" groups →
 * one mesh per finish, CAD colors as vertex colors), so the real model drops in with
 * no code changes. Intake faces +Z.
 */
type Part = { geo: THREE.BufferGeometry; color: number; finish: string };

function colored(geo: THREE.BufferGeometry, color: number): THREE.BufferGeometry {
  const g = geo.index ? geo.toNonIndexed() : geo;
  const c = new THREE.Color(color);
  const n = g.getAttribute('position').count;
  const arr = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) arr.set([c.r, c.g, c.b], i * 3);
  g.setAttribute('color', new THREE.BufferAttribute(arr, 3));
  g.deleteAttribute('uv');
  return g;
}

const box = (w: number, h: number, d: number, x: number, y: number, z: number) =>
  new THREE.BoxGeometry(w, h, d).translate(x, y, z);
const cyl = (r: number, len: number, x: number, y: number, z: number, axis: 'x' | 'y' | 'z' = 'x', seg = 48) => {
  const g = new THREE.CylinderGeometry(r, r, len, seg);
  if (axis === 'x') g.rotateZ(Math.PI / 2);
  if (axis === 'z') g.rotateX(Math.PI / 2);
  return g.translate(x, y, z);
};

const ALU = 0xb8bcc4, DARK = 0x2a2c31, MAROON = 0x7a1b33, ORANGE = 0xd9772b, POLY = 0xcfe3ff, RUBBER = 0x1c1d20, MOTOR = 0x3a3d44;

function parts(): Record<string, Part[]> {
  const W = 0.74, L = 0.74, T = 0.05;
  const frame: Part[] = [];
  for (const s of [-1, 1]) {
    frame.push({ geo: box(T, T * 0.5, L, s * (W / 2 - T / 2), 0.09, 0), color: ALU, finish: 'metal' });
    frame.push({ geo: box(W, T * 0.5, T, 0, 0.09, s * (L / 2 - T / 2)), color: ALU, finish: 'metal' });
    frame.push({ geo: box(0.1, 0.12, L + 0.16, s * (W / 2 + 0.05), 0.1, 0), color: MAROON, finish: 'fabric' });
  }
  frame.push({ geo: box(W - 0.02, 0.006, L - 0.02, 0, 0.065, 0), color: ALU, finish: 'metal' });
  for (const s of [-1, 1]) frame.push({ geo: box(0.025, 0.28, 0.025, s * 0.2, 0.25, -0.2), color: ALU, finish: 'metal' });

  const swerve: Part[] = [];
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    const x = sx * (W / 2 - 0.09), z = sz * (L / 2 - 0.09);
    swerve.push({ geo: cyl(0.05, 0.038, x, 0.05, z, 'x'), color: RUBBER, finish: 'rubber' });
    swerve.push({ geo: cyl(0.03, 0.042, x, 0.05, z, 'x'), color: ALU, finish: 'metal' });
    swerve.push({ geo: box(0.1, 0.03, 0.12, x, 0.12, z), color: ALU, finish: 'metal' });
    swerve.push({ geo: cyl(0.03, 0.08, x - sx * 0.035, 0.17, z, 'y', 32), color: MOTOR, finish: 'motor' });
    swerve.push({ geo: cyl(0.03, 0.08, x + sx * 0.035, 0.17, z, 'y', 32), color: MOTOR, finish: 'motor' });
  }

  const intake: Part[] = [
    { geo: cyl(0.03, W + 0.04, 0, 0.09, L / 2 + 0.16), color: ORANGE, finish: 'rubber' },
    { geo: cyl(0.025, W - 0.04, 0, 0.17, L / 2 + 0.08), color: ORANGE, finish: 'rubber' },
    { geo: box(0.012, 0.16, 0.24, -W / 2 + 0.01, 0.13, L / 2 + 0.07), color: ALU, finish: 'metal' },
    { geo: box(0.012, 0.16, 0.24, W / 2 - 0.01, 0.13, L / 2 + 0.07), color: ALU, finish: 'metal' },
    { geo: cyl(0.03, 0.08, W / 2 - 0.06, 0.17, L / 2 + 0.03, 'x', 32), color: MOTOR, finish: 'motor' },
  ];

  const hopper: Part[] = [
    { geo: box(0.5, 0.22, 0.005, 0, 0.21, 0.2), color: POLY, finish: 'polycarb' },
    { geo: box(0.005, 0.22, 0.42, -0.25, 0.21, 0), color: POLY, finish: 'polycarb' },
    { geo: box(0.005, 0.22, 0.42, 0.25, 0.21, 0), color: POLY, finish: 'polycarb' },
  ];
  for (let i = 0; i < 4; i++) hopper.push({ geo: cyl(0.022, 0.48, 0, 0.11, -0.12 + i * 0.09), color: 0x4a7bd1, finish: 'rubber' });

  const turrets: Part[] = [];
  for (const s of [-1, 1]) {
    const x = s * 0.17, z = -0.16;
    turrets.push({ geo: cyl(0.1, 0.02, x, 0.36, z, 'y', 64), color: DARK, finish: 'plastic' });
    turrets.push({ geo: box(0.012, 0.16, 0.2, x - 0.07, 0.45, z), color: ALU, finish: 'metal' });
    turrets.push({ geo: box(0.012, 0.16, 0.2, x + 0.07, 0.45, z), color: ALU, finish: 'metal' });
    turrets.push({ geo: cyl(0.05, 0.12, x, 0.47, z + 0.02, 'x', 64), color: 0xc9a64b, finish: 'metal' });
    turrets.push({ geo: box(0.13, 0.008, 0.16, x, 0.53, z + 0.03).rotateX(0), color: POLY, finish: 'polycarb' });
    turrets.push({ geo: cyl(0.03, 0.08, x, 0.41, z - 0.1, 'x', 32), color: MOTOR, finish: 'motor' });
  }

  const electrical: Part[] = [
    { geo: box(0.2, 0.03, 0.12, 0.12, 0.085, -0.22), color: 0x2b2f36, finish: 'plastic' },
    { geo: box(0.09, 0.02, 0.07, -0.16, 0.08, -0.22), color: 0xe8e8e8, finish: 'plastic' },
    { geo: box(0.05, 0.03, 0.05, -0.05, 0.085, -0.28), color: 0xcc2222, finish: 'plastic' },
    { geo: box(0.07, 0.05, 0.035, 0, 0.4, -0.36), color: 0x111111, finish: 'plastic' },
    { geo: box(0.07, 0.05, 0.035, 0.3, 0.2, 0.2).rotateY(0), color: 0x111111, finish: 'plastic' },
    { geo: box(0.18, 0.17, 0.08, 0, 0.12, -0.3), color: 0x1f1f22, finish: 'plastic' },
  ];

  return { swerve, intake, hopper, turrets, electrical, frame };
}

export function buildStandIn(): THREE.Group {
  const root = new THREE.Group();
  root.name = 'Chuck Norris (stand-in)';
  for (const [id, list] of Object.entries(parts())) {
    const group = new THREE.Group();
    group.name = `subsystem:${id}`;
    group.userData.subsystem = id;
    const byFinish = new Map<string, THREE.BufferGeometry[]>();
    for (const p of list) {
      const arr = byFinish.get(p.finish) ?? [];
      arr.push(colored(p.geo, p.color));
      byFinish.set(p.finish, arr);
    }
    for (const [finish, geos] of byFinish) {
      const merged = mergeGeometries(geos)!;
      const mat = new THREE.MeshStandardMaterial();
      mat.userData.finish = finish;
      const mesh = new THREE.Mesh(merged, mat);
      mesh.name = `${id}:${finish}`;
      mesh.userData.subsystem = id;
      group.add(mesh);
    }
    root.add(group);
  }
  return root;
}
