// Drawing the soft bodies (the physics is in phys/net.ts and phys/panels.ts, in the worker):
//   net     one dynamic mesh under the robot's stage, the knots interpolated between the last two steps
//   panels  the robot's hopper and intake meshes get a vertex-shader bend: each panel's two mode shapes
//           times the amplitudes the worker sends, so nothing is uploaded per frame except a few uniforms
import * as THREE from 'three';
import type { Viewer } from '../viewer';
import type { Frame } from './backend';
import { NetMesh } from './net-mesh';
import { PANELS } from './phys/panels';
import { couplerOffset } from './phys/fourbar';

const SUBSYSTEMS = new Set(['hopper', 'intake']);
const N = PANELS.length;

interface Bent { mesh: THREE.Mesh; orig: THREE.Material | THREE.Material[]; mat: THREE.Material; rfm: { value: THREE.Matrix4 }; mfr: { value: THREE.Matrix3 } }

const BEND_GLSL = `
uniform mat4 uRfm; uniform mat3 uMfr;
uniform vec4 uBA[${N}]; uniform vec4 uBB[${N}]; uniform vec4 uBX[${N}]; uniform vec4 uBQ[${N}];
vec3 bendOffset(vec3 pm) {
  vec3 p = (uRfm * vec4(pm, 1.0)).xyz;
  vec3 d = vec3(0.0);
  for (int i = 0; i < ${N}; i++) {
    vec3 r = p - uBA[i].xyz;
    int ni = int(uBX[i].x + 0.5), ui = int(uBX[i].y + 0.5), vi = int(uBX[i].z + 0.5);
    if (abs(r[ni]) > uBA[i].w) continue;
    float u = r[ui] / (2.0 * uBB[i].x) + 0.5, v = r[vi] / (2.0 * uBB[i].y) + 0.5;
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) continue;
    float su = sin(3.14159265 * u), sv = sin(3.14159265 * v);
    float f1 = su * su * sv * sv;
    float f2 = sin(6.2831853 * u) * f1 * 1.5396007;
    vec3 n = vec3(0.0);
    n[ni] = 1.0;
    d += n * (uBQ[i].x * f1 + uBQ[i].y * f2);
  }
  return uMfr * d;
}`;

export class SoftbodyRender {
  private net = new NetMesh();
  private bent: Bent[] = [];
  private ua = Array.from({ length: N }, () => new THREE.Vector4());
  private ub = Array.from({ length: N }, () => new THREE.Vector4());
  private ux = Array.from({ length: N }, () => new THREE.Vector4());
  private uq = Array.from({ length: N }, () => new THREE.Vector4());
  private inv = new THREE.Matrix4();
  private rel = new THREE.Matrix4();
  private lin = new THREE.Matrix3();

  constructor(private viewer: Viewer) {
    viewer.stage.add(this.net.mesh);
    PANELS.forEach((p, k) => { this.ub[k].set(p.hu, p.hv, 0, 0); this.ux[k].set(p.n, p.u, p.v, 0); this.ua[k].set(p.c[0], p.c[1], p.c[2], p.tol); });
    viewer.root.traverse((o) => {
      const m = o as THREE.Mesh;
      if (!m.isMesh || m.userData.ghost || m.userData.wiring || !SUBSYSTEMS.has(m.userData.subsystem as string) || Array.isArray(m.material)) return;
      const orig = m.material as THREE.Material;
      const mat = orig.clone();
      mat.userData = { ...orig.userData };
      const base = orig.onBeforeCompile;
      const rfm = { value: new THREE.Matrix4() }, mfr = { value: new THREE.Matrix3() };
      mat.onBeforeCompile = (shader, r) => {
        base.call(mat, shader, r);
        Object.assign(shader.uniforms, { uRfm: rfm, uMfr: mfr, uBA: { value: this.ua }, uBB: { value: this.ub }, uBX: { value: this.ux }, uBQ: { value: this.uq } });
        shader.vertexShader = shader.vertexShader
          .replace('#include <common>', `#include <common>\n${BEND_GLSL}`)
          .replace('#include <begin_vertex>', '#include <begin_vertex>\ntransformed += bendOffset(transformed);');
      };
      const key = orig.customProgramCacheKey?.bind(orig);
      mat.customProgramCacheKey = () => `${key ? key() : orig.type}-bend${N}`;
      m.material = mat;
      this.bent.push({ mesh: m, orig, mat, rfm, mfr });
    });
  }

  update(fr: Frame, alpha: number) {
    this.net.update(fr.net.local, fr.net.prev, alpha);
    // panels: centers (the intake's ride the four-bar), amplitudes, and the mesh → robot-frame transforms
    const [dz, dy] = couplerOffset(fr.robot.intakeAngle);
    PANELS.forEach((p, k) => {
      this.ua[k].set(p.c[0], p.c[1] + (p.coupler ? dy : 0), p.c[2] + (p.coupler ? dz : 0), p.tol);
      this.uq[k].set(fr.panels[2 * k], fr.panels[2 * k + 1], 0, 0);
    });
    const st = this.viewer.stage;
    this.inv.copy(st.matrixWorld).invert();
    for (const b of this.bent) {
      b.mesh.updateWorldMatrix(true, false);
      this.rel.multiplyMatrices(this.inv, b.mesh.matrixWorld);
      b.rfm.value.copy(this.rel);
      // a displacement in the robot frame back into the mesh's own space: the inverse of the linear part
      this.lin.setFromMatrix4(this.rel).invert();
      b.mfr.value.copy(this.lin);
    }
  }

  dispose() {
    for (const b of this.bent) { b.mesh.material = b.orig; b.mat.dispose(); }
    this.bent.length = 0;
    this.viewer.stage.remove(this.net.mesh);
    this.net.dispose();
  }
}
