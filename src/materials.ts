import * as THREE from 'three';

/** Per-subsystem effect uniforms, shared by every material in that subsystem. */
export interface SubsystemFx {
  uGhost: { value: number };     // 0 = solid, 1 = hologram ghost
  uHighlight: { value: number }; // touch-down feedback
}

/** Scene-wide uniforms for the one-time scan reveal. */
export const revealFx = {
  uReveal: { value: 1e3 }, // world-space height below which the robot is visible
  uBand: { value: 0 },     // glow strength of the light band at that height
};

export type Finish = 'metal' | 'plastic' | 'rubber' | 'motor' | 'polycarb' | 'fabric';

/** CAD colors stay as vertex colors; each finish sets how the surface reacts to light. */
function baseMaterial(finish: string): THREE.MeshStandardMaterial | THREE.MeshPhysicalMaterial {
  switch (finish as Finish) {
    case 'metal': // brushed aluminum / steel: fully metallic with a soft, satin sheen
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 1, roughness: 0.34, envMapIntensity: 1.1, clearcoat: 0.08, clearcoatRoughness: 0.4 });
    case 'motor':
      return new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.55, roughness: 0.42 });
    case 'rubber':
      return new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0, roughness: 0.88 });
    case 'fabric':
      return new THREE.MeshPhysicalMaterial({ vertexColors: true, metalness: 0, roughness: 0.95, sheen: 0.6, sheenRoughness: 0.8, sheenColor: new THREE.Color(0.5, 0.5, 0.5) });
    case 'polycarb': // slightly translucent, glossy
      return new THREE.MeshPhysicalMaterial({
        vertexColors: true, metalness: 0, roughness: 0.08, transparent: true, opacity: 0.38,
        depthWrite: false, side: THREE.DoubleSide, clearcoat: 1, clearcoatRoughness: 0.05, envMapIntensity: 1.4,
      });
    default:
      return new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0, roughness: 0.55 });
  }
}

const GHOST_GLSL = /* glsl */ `
  float fxFresnel(vec3 n, vec3 v) { return pow(1.0 - clamp(abs(dot(normalize(n), normalize(v))), 0.0, 1.0), 2.2); }
  vec4 fxGhost(vec3 base, float fres) {
    float l = dot(base, vec3(0.2126, 0.7152, 0.0722));
    vec3 cool = vec3(0.55, 0.74, 1.0);
    vec3 c = cool * (0.10 + l * 0.22) + cool * fres * 1.1;
    return vec4(c, 0.10 + fres * 0.32);
  }
`;

/**
 * Builds a material for one subsystem + finish, patched with:
 *  - ghost: desaturated, ~12% opacity, cool fresnel rim (cheap early-out when fully ghosted)
 *  - highlight: accent tint on touch-down
 *  - scan reveal: clip above a rising height, with a soft light band
 */
export function makeMaterial(finish: string, fx: SubsystemFx): THREE.Material {
  const mat = baseMaterial(finish);
  mat.userData.finish = finish;
  mat.userData.baseTransparent = mat.transparent;
  mat.userData.baseDepthWrite = mat.depthWrite;
  mat.onBeforeCompile = (shader) => {
    shader.uniforms.uGhost = fx.uGhost;
    shader.uniforms.uHighlight = fx.uHighlight;
    shader.uniforms.uReveal = revealFx.uReveal;
    shader.uniforms.uBand = revealFx.uBand;
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying float vFxY;')
      .replace('#include <project_vertex>', '#include <project_vertex>\n  vFxY = (modelMatrix * vec4(transformed, 1.0)).y;');
    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>
        varying float vFxY;
        uniform float uGhost, uHighlight, uReveal, uBand;
        ${GHOST_GLSL}`,
      )
      .replace(
        'void main() {',
        `void main() {
        if (vFxY > uReveal) discard;
        if (uGhost > 0.995) {
          // Fully ghosted: skip all lighting. The branch is uniform, so it is cheap on the GPU.
          vec3 ghostBase = vec3(0.6);
          #ifdef USE_COLOR
            ghostBase = vColor.rgb;
          #endif
          gl_FragColor = fxGhost(ghostBase, fxFresnel(vNormal, vViewPosition));
          #include <tonemapping_fragment>
          #include <colorspace_fragment>
          return;
        }`,
      )
      .replace(
        '#include <tonemapping_fragment>',
        `{
          vec3 n = normalize(vNormal);
          float fres = fxFresnel(n, vViewPosition);
          gl_FragColor.rgb += vec3(0.62, 0.12, 0.25) * uHighlight * (0.35 + 0.65 * fres);
          float band = smoothstep(0.05, 0.0, uReveal - vFxY) * uBand;
          gl_FragColor.rgb += vec3(0.6, 0.8, 1.0) * band * 2.0;
          vec3 ghostBase = vec3(0.6);
          #ifdef USE_COLOR
            ghostBase = vColor.rgb;
          #endif
          vec4 g = fxGhost(ghostBase, fres);
          gl_FragColor = vec4(mix(gl_FragColor.rgb, g.rgb, uGhost), mix(gl_FragColor.a, g.a, uGhost));
        }
        #include <tonemapping_fragment>`,
      );
  };
  // All patched materials share one program per base type + defines.
  mat.customProgramCacheKey = () => `fx-${finish}`;
  return mat;
}

/** Switches a material between solid and ghost blending without recompiling. */
export function applyGhostState(mat: THREE.Material, ghost: number) {
  const ghosted = ghost > 0.001;
  mat.transparent = ghosted || mat.userData.baseTransparent;
  mat.depthWrite = ghosted ? false : mat.userData.baseDepthWrite;
}

export function newFx(): SubsystemFx {
  return { uGhost: { value: 0 }, uHighlight: { value: 0 } };
}
