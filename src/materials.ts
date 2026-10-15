import * as THREE from 'three';

/**
 * Per-part state, one RGBA texel per part id, read in the vertex shader:
 *   R  visible (255) / hidden (0)
 *   G  touch highlight
 *   B  focus amount now   (255 = solid, 0 = ghost)
 *   A  focus amount before (crossfaded with B by uFocusMix)
 * Part 0 (meshes without part ids, e.g. the stand-in robot) is always visible and solid.
 */
export class PartState {
  readonly width = 64;
  readonly height: number;
  readonly data: Uint8Array;
  readonly texture: THREE.DataTexture;
  readonly uniforms = {
    uParts: { value: null as unknown as THREE.DataTexture },
    uPartsW: { value: 64 },
    uFocusMix: { value: 1 },
  };

  constructor(count: number) {
    this.height = Math.max(1, Math.ceil((count + 1) / this.width));
    this.data = new Uint8Array(this.width * this.height * 4);
    for (let i = 0; i < this.width * this.height; i++) this.data.set([255, 0, 255, 255], i * 4);
    this.texture = new THREE.DataTexture(this.data, this.width, this.height, THREE.RGBAFormat, THREE.UnsignedByteType);
    this.texture.magFilter = this.texture.minFilter = THREE.NearestFilter;
    this.texture.generateMipmaps = false;
    this.texture.needsUpdate = true;
    this.uniforms.uParts.value = this.texture;
  }

  get max() { return this.width * this.height - 1; }
  setVisible(pid: number, v: boolean) { this.data[pid * 4] = v ? 255 : 0; }
  isVisible(pid: number) { return this.data[pid * 4] > 127; }
  setHighlight(pid: number, v: number) { this.data[pid * 4 + 1] = Math.round(v * 255); }

  /** Starts a crossfade to a new focus set (null = everything solid). */
  setFocus(set: Set<number> | null) {
    const t = this.uniforms.uFocusMix.value;
    const d = this.data;
    for (let pid = 1, n = this.width * this.height; pid < n; pid++) {
      const o = pid * 4;
      const now = d[o + 3] + (d[o + 2] - d[o + 3]) * t; // where the previous fade had got to
      d[o + 3] = Math.round(now);
      d[o + 2] = !set || set.has(pid) ? 255 : 0;
    }
    this.uniforms.uFocusMix.value = 0;
    this.texture.needsUpdate = true;
  }
  commit() { this.texture.needsUpdate = true; }
}

/** Reveal (scan) uniforms shared by every robot material. */
export const revealFx = { uReveal: { value: 1e3 }, uBand: { value: 0 } };
/** Hologram look, shared. */
export const ghostFx = {
  uGhostColor: { value: new THREE.Color(0.45, 0.78, 1.0) },
  uGhostStrength: { value: 1 },
  uLight: { value: 0 }, // 1 in light mode
  uCut: { value: 0 },   // view depth of the selection's near side; holograms in front of it fade (cutaway)
};
export const highlightColor = { value: new THREE.Color(0.62, 0.12, 0.25) };

const STATE_VERT = /* glsl */ `
  attribute float _partid;
  uniform sampler2D uParts;
  uniform float uPartsW, uFocusMix;
  varying float vFocus;
  varying float vHi;
  vec4 partState() {
    float id = _partid;
    return texelFetch(uParts, ivec2(int(mod(id, uPartsW)), int(floor(id / uPartsW))), 0);
  }
`;

// Hidden parts (and fully ghosted parts in the solid pass) are dropped in the vertex shader:
// every vertex moves outside the clip volume, so the triangle costs nothing to rasterize.
const KILL = 'gl_Position = vec4(0.0, 0.0, 2.0, 1.0);';

export type Finish = 'metal' | 'steel' | 'plastic' | 'rubber' | 'motor' | 'polycarb' | 'fabric';

function baseMaterial(finish: string): THREE.MeshStandardMaterial | THREE.MeshPhysicalMaterial {
  switch (finish as Finish) {
    case 'metal': // 6061 aluminum: bright, satin, slightly brushed
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 1, roughness: 0.3, envMapIntensity: 1, specularIntensity: 1 });
    case 'steel': // black-oxide / zinc hardware
      return new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.9, roughness: 0.42 });
    case 'motor': // anodized motor housings
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0.65, roughness: 0.32, clearcoat: 0.3, clearcoatRoughness: 0.35 });
    case 'rubber':
      return new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0, roughness: 0.82 });
    case 'fabric': // bumper cordura
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0, roughness: 0.92, sheen: 0.8, sheenRoughness: 0.6, sheenColor: new THREE.Color(0.35, 0.35, 0.35) });
    case 'polycarb':
      return new THREE.MeshPhysicalMaterial({
        vertexColors: true, metalness: 0, roughness: 0.06, transparent: true, opacity: 0.42,
        depthWrite: false, side: THREE.DoubleSide, clearcoat: 1, clearcoatRoughness: 0.04, envMapIntensity: 1.3,
      });
    default: // powder coat, nylon, 3D prints, electronics
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0, roughness: 0.48, clearcoat: 0.15, clearcoatRoughness: 0.5 });
  }
}

/** Solid robot material: CAD color, realistic finish, per-part visibility / highlight / focus, scan reveal. */
export function makeSolidMaterial(finish: string, state: PartState): THREE.Material {
  const mat = baseMaterial(finish);
  mat.userData.finish = finish;
  mat.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, state.uniforms, revealFx, { uHiColor: highlightColor });
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `#include <common>\n${STATE_VERT}\nvarying float vFxY;`)
      .replace('#include <project_vertex>', `#include <project_vertex>
        vFxY = (modelMatrix * vec4(transformed, 1.0)).y;
        vec4 ps = partState();
        vFocus = mix(ps.a, ps.b, uFocusMix);
        vHi = ps.g;
        if (ps.r < 0.5 || vFocus < 0.004) { ${KILL} }`);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>
        varying float vFxY, vFocus, vHi;
        uniform float uReveal, uBand;
        uniform vec3 uHiColor;
        float bayer4(vec2 p) {
          vec2 q = mod(floor(p), 4.0);
          int i = int(q.x) + int(q.y) * 4;
          float m[16] = float[16](0.,8.,2.,10.,12.,4.,14.,6.,3.,11.,1.,9.,15.,7.,13.,5.);
          return (m[i] + 0.5) / 16.0;
        }`)
      .replace('void main() {', `void main() {
        if (vFxY > uReveal) discard;
        if (vFocus < 0.996 && vFocus < bayer4(gl_FragCoord.xy)) discard; // screen-door fade into the hologram`)
      .replace('#include <tonemapping_fragment>', `{
          float fres = pow(1.0 - clamp(abs(dot(normalize(vNormal), normalize(vViewPosition))), 0.0, 1.0), 2.0);
          gl_FragColor.rgb += uHiColor * vHi * (0.45 + 0.8 * fres);
          float band = smoothstep(0.05, 0.0, uReveal - vFxY) * uBand;
          gl_FragColor.rgb += vec3(0.55, 0.8, 1.0) * band * 2.2;
        }
        #include <tonemapping_fragment>`);
  };
  mat.customProgramCacheKey = () => `solid-${finish}`;
  return mat;
}

/**
 * Hologram pass for de-focused parts: a cool fresnel rim with faint fill and fine horizontal
 * scan lines. Additive in dark mode (order independent, no sorting flicker), normal blend in light mode.
 */
export function makeGhostMaterial(state: PartState): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    name: 'hologram',
    transparent: true,
    depthWrite: false,
    depthTest: true,
    side: THREE.DoubleSide,
    blending: THREE.AdditiveBlending,
    uniforms: { ...state.uniforms, ...ghostFx },
    vertexShader: /* glsl */ `
      ${STATE_VERT}
      varying vec3 vN;
      varying vec3 vV;
      varying float vY;
      varying float vDepth;
      void main() {
        vec4 mv = modelViewMatrix * vec4(position, 1.0);
        vDepth = -mv.z;
        vN = normalize(normalMatrix * normal);
        vV = -mv.xyz;
        vY = (modelMatrix * vec4(position, 1.0)).y;
        gl_Position = projectionMatrix * mv;
        vec4 ps = partState();
        vFocus = mix(ps.a, ps.b, uFocusMix);
        vHi = ps.g;
        if (ps.r < 0.5 || vFocus > 0.996) { ${KILL} }
      }`,
    fragmentShader: /* glsl */ `
      uniform vec3 uGhostColor;
      uniform float uGhostStrength, uLight, uCut;
      varying vec3 vN;
      varying vec3 vV;
      varying float vY;
      varying float vDepth;
      varying float vFocus;
      varying float vHi;
      void main() {
        float ghost = (1.0 - vFocus) * uGhostStrength;
        if (uCut > 0.0) ghost *= mix(0.12, 1.0, smoothstep(uCut - 0.06, uCut + 0.02, vDepth));
        float fres = pow(1.0 - clamp(abs(dot(normalize(vN), normalize(vV))), 0.0, 1.0), 2.4);
        float lines = 0.75 + 0.25 * smoothstep(0.35, 0.5, abs(fract(vY * 160.0) - 0.5));
        float a = (0.012 + fres * 0.27) * lines * ghost + vHi * 0.25;
        if (a < 0.002) discard;
        vec3 c = uGhostColor;
        if (uLight > 0.5) gl_FragColor = vec4(c * 0.55, min(1.0, a * 1.6));
        else gl_FragColor = vec4(c * a, 1.0);
      }`,
  });
}

/** Silhouette mask of the focused set, for the accent outline. */
export function makeMaskMaterial(state: PartState): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    side: THREE.DoubleSide,
    uniforms: { ...state.uniforms },
    vertexShader: /* glsl */ `
      ${STATE_VERT}
      void main() {
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        vec4 ps = partState();
        vFocus = ps.b;
        vHi = 0.0;
        if (ps.r < 0.5 || ps.b < 0.5) { ${KILL} }
      }`,
    fragmentShader: 'void main() { gl_FragColor = vec4(1.0); }',
  });
}

/** Patches a depth material (shadows) so hidden and ghosted parts cast no shadow. */
export function patchDepthMaterial(mat: THREE.Material, state: PartState) {
  const prev = mat.onBeforeCompile;
  mat.onBeforeCompile = (shader, r) => {
    prev?.call(mat, shader, r);
    Object.assign(shader.uniforms, state.uniforms);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `#include <common>\n${STATE_VERT}`)
      .replace('#include <project_vertex>', `#include <project_vertex>
        vec4 ps = partState();
        vFocus = mix(ps.a, ps.b, uFocusMix);
        vHi = 0.0;
        if (ps.r < 0.5 || vFocus < 0.5) { ${KILL} }`);
    shader.fragmentShader = shader.fragmentShader.replace('#include <common>', '#include <common>\nvarying float vFocus, vHi;');
  };
  mat.customProgramCacheKey = () => 'parts-depth';
  return mat;
}
