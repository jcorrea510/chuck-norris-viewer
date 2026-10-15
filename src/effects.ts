import * as THREE from 'three';
import { HorizontalBlurShader } from 'three/examples/jsm/shaders/HorizontalBlurShader.js';
import { VerticalBlurShader } from 'three/examples/jsm/shaders/VerticalBlurShader.js';
import { patchDepthMaterial, type PartState } from './materials';

// ------------------------------------------------------------------ floor grid
/** Faint holographic floor grid that fades with distance, with a soft ring under the robot. */
export function makeGrid(): THREE.Mesh {
  const geo = new THREE.PlaneGeometry(16, 16);
  geo.rotateX(-Math.PI / 2);
  const mat = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: {
      uFade: { value: new THREE.Vector2(0.7, 4.2) },
      uColor: { value: new THREE.Color(0.62, 0.84, 1.0) },
      uStrength: { value: 1 },
    },
    vertexShader: /* glsl */ `
      varying vec3 vPos;
      void main() {
        vPos = (modelMatrix * vec4(position, 1.0)).xyz;
        gl_Position = projectionMatrix * viewMatrix * vec4(vPos, 1.0);
      }`,
    fragmentShader: /* glsl */ `
      varying vec3 vPos;
      uniform vec2 uFade;
      uniform float uStrength;
      uniform vec3 uColor;
      float line(vec2 p, float spacing, float width) {
        vec2 g = abs(fract(p / spacing - 0.5) - 0.5) / fwidth(p / spacing);
        return 1.0 - min(min(g.x, g.y) / width, 1.0);
      }
      void main() {
        float d = length(vPos.xz);
        float fade = 1.0 - smoothstep(uFade.x, uFade.y, d);
        float minor = line(vPos.xz, 0.1, 1.0) * 0.03;
        float major = line(vPos.xz, 0.5, 1.2) * 0.07;
        float ring = (1.0 - smoothstep(0.0, 0.012, abs(d - 0.95))) * 0.06 + (1.0 - smoothstep(0.0, 0.006, abs(d - 1.0))) * 0.05;
        float a = (max(minor, major) + ring) * fade * uStrength;
        if (a < 0.002) discard;
        gl_FragColor = vec4(uColor, a);
      }`,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.renderOrder = -2;
  mesh.name = 'grid';
  mesh.raycast = () => {};
  return mesh;
}

/** Hides every child of `scene` except `subject`, and all ghost meshes; returns an undo function. */
function isolate(scene: THREE.Scene, subject: THREE.Object3D) {
  const off: THREE.Object3D[] = [];
  for (const c of scene.children) if (c !== subject && c.visible && !(c as THREE.Light).isLight) { off.push(c); c.visible = false; }
  subject.traverse((o) => { if (o.userData.ghost && o.visible) { off.push(o); o.visible = false; } });
  return () => { for (const o of off) o.visible = true; };
}

// ------------------------------------------------------------------ contact shadow
/**
 * Soft contact shadow, rendered from below into a small texture and blurred. Re-rendered only
 * when the robot changes (load, explode, mechanisms, hide), never per frame.
 */
export class ContactShadow {
  readonly plane: THREE.Mesh;
  private rt: THREE.WebGLRenderTarget;
  private rtBlur: THREE.WebGLRenderTarget;
  private cam: THREE.OrthographicCamera;
  private depthMat: THREE.MeshDepthMaterial;
  private blurH: THREE.ShaderMaterial;
  private blurV: THREE.ShaderMaterial;
  private blurPlane: THREE.Mesh;

  constructor(state: PartState | null, size = 2.6, height = 1.0, res = 512) {
    this.rt = new THREE.WebGLRenderTarget(res, res);
    this.rt.texture.generateMipmaps = false;
    this.rtBlur = new THREE.WebGLRenderTarget(res, res);
    this.rtBlur.texture.generateMipmaps = false;

    const geo = new THREE.PlaneGeometry(size, size).rotateX(Math.PI / 2);
    const mat = new THREE.MeshBasicMaterial({ map: this.rt.texture, transparent: true, depthWrite: false, opacity: 0.88, color: 0x000000 });
    this.plane = new THREE.Mesh(geo, mat);
    this.plane.renderOrder = -1;
    this.plane.position.y = 0.0005;
    this.plane.scale.y = -1;
    this.plane.raycast = () => {};
    this.plane.name = 'contact-shadow';

    this.cam = new THREE.OrthographicCamera(-size / 2, size / 2, size / 2, -size / 2, 0, height);
    this.cam.rotation.x = Math.PI / 2;

    this.depthMat = new THREE.MeshDepthMaterial();
    this.depthMat.userData.darkness = { value: 1.7 };
    this.depthMat.onBeforeCompile = (shader) => {
      shader.uniforms.darkness = this.depthMat.userData.darkness;
      shader.fragmentShader = /* glsl */ `
        uniform float darkness;
        ${shader.fragmentShader.replace(
          'gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );',
          'gl_FragColor = vec4( vec3( 0.0 ), ( 1.0 - fragCoordZ ) * darkness );',
        )}`;
    };
    if (state) patchDepthMaterial(this.depthMat, state);
    this.depthMat.depthTest = false;
    this.depthMat.depthWrite = false;

    this.blurH = new THREE.ShaderMaterial(HorizontalBlurShader);
    this.blurH.depthTest = false;
    this.blurV = new THREE.ShaderMaterial(VerticalBlurShader);
    this.blurV.depthTest = false;
    this.blurPlane = new THREE.Mesh(new THREE.PlaneGeometry(size, size).rotateX(Math.PI / 2));
    this.blurPlane.visible = false;
  }

  set opacity(o: number) { (this.plane.material as THREE.MeshBasicMaterial).opacity = o; }

  private blur(renderer: THREE.WebGLRenderer, amount: number) {
    this.blurPlane.position.copy(this.cam.position);
    this.blurPlane.visible = true;
    this.blurPlane.material = this.blurH;
    this.blurH.uniforms.tDiffuse.value = this.rt.texture;
    this.blurH.uniforms.h.value = amount / 256;
    renderer.setRenderTarget(this.rtBlur);
    renderer.render(this.blurPlane, this.cam);
    this.blurPlane.material = this.blurV;
    this.blurV.uniforms.tDiffuse.value = this.rtBlur.texture;
    this.blurV.uniforms.v.value = amount / 256;
    renderer.setRenderTarget(this.rt);
    renderer.render(this.blurPlane, this.cam);
    this.blurPlane.visible = false;
  }

  update(renderer: THREE.WebGLRenderer, scene: THREE.Scene, subject: THREE.Object3D) {
    const prev = { bg: scene.background, target: renderer.getRenderTarget(), alpha: renderer.getClearAlpha(), tm: renderer.toneMapping };
    const undo = isolate(scene, subject);
    this.cam.position.set(0, 0, 0);
    this.cam.updateMatrixWorld();
    scene.background = null;
    scene.overrideMaterial = this.depthMat;
    renderer.toneMapping = THREE.NoToneMapping;
    renderer.setClearAlpha(0);
    renderer.setRenderTarget(this.rt);
    renderer.clear();
    renderer.render(scene, this.cam);
    scene.overrideMaterial = null;
    this.blur(renderer, 1.5);
    this.blur(renderer, 0.6);
    undo();
    scene.background = prev.bg;
    renderer.toneMapping = prev.tm;
    renderer.setRenderTarget(prev.target);
    renderer.setClearAlpha(prev.alpha);
  }
}

// ------------------------------------------------------------------ focus outline
/**
 * Thin, soft accent outline around the focused parts: render their silhouette into a
 * half-resolution mask, then draw the dilated edge of that mask over the frame.
 */
export class Outline {
  private rt: THREE.WebGLRenderTarget;
  private quad: THREE.Mesh;
  private quadScene = new THREE.Scene();
  private quadCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
  readonly strength = { value: 0 };
  readonly color = { value: new THREE.Color(0xf08ca3) };

  constructor(private maskMat: THREE.Material) {
    this.rt = new THREE.WebGLRenderTarget(4, 4, { samples: 0 });
    this.rt.texture.generateMipmaps = false;
    const mat = new THREE.ShaderMaterial({
      transparent: true,
      depthTest: false,
      depthWrite: false,
      uniforms: { tMask: { value: this.rt.texture }, uTexel: { value: new THREE.Vector2() }, uColor: this.color, uStrength: this.strength },
      vertexShader: 'varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }',
      fragmentShader: /* glsl */ `
        uniform sampler2D tMask; uniform vec2 uTexel; uniform vec3 uColor; uniform float uStrength;
        varying vec2 vUv;
        void main() {
          float c = texture2D(tMask, vUv).r;
          float m = 0.0;
          for (int i = 0; i < 12; i++) {
            float a = float(i) * 0.5235988;
            vec2 d = vec2(cos(a), sin(a));
            m += texture2D(tMask, vUv + d * uTexel * 1.0).r * 0.65;
            m += texture2D(tMask, vUv + d * uTexel * 2.0).r * 0.35;
          }
          m = clamp(m / 6.0, 0.0, 1.0);
          float edge = clamp(m - c, 0.0, 1.0);
          float a = smoothstep(0.0, 0.7, edge) * uStrength * 0.9;
          if (a < 0.003) discard;
          gl_FragColor = vec4(uColor, a);
        }`,
    });
    this.quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), mat);
    this.quad.frustumCulled = false;
    this.quadScene.add(this.quad);
  }

  setSize(w: number, h: number) {
    const mw = Math.max(4, Math.round(w / 2)), mh = Math.max(4, Math.round(h / 2));
    this.rt.setSize(mw, mh);
    (this.quad.material as THREE.ShaderMaterial).uniforms.uTexel.value.set(1 / mw, 1 / mh);
  }

  render(renderer: THREE.WebGLRenderer, scene: THREE.Scene, camera: THREE.Camera, subject: THREE.Object3D) {
    if (this.strength.value < 0.005) return;
    const prev = { target: renderer.getRenderTarget(), alpha: renderer.getClearAlpha(), tm: renderer.toneMapping };
    const undo = isolate(scene, subject);
    scene.overrideMaterial = this.maskMat;
    renderer.toneMapping = THREE.NoToneMapping;
    renderer.setRenderTarget(this.rt);
    renderer.setClearAlpha(0);
    renderer.clear();
    renderer.render(scene, camera);
    scene.overrideMaterial = null;
    undo();
    renderer.setRenderTarget(prev.target);
    renderer.setClearAlpha(prev.alpha);
    const autoClear = renderer.autoClear;
    renderer.autoClear = false;
    renderer.render(this.quadScene, this.quadCam);
    renderer.autoClear = autoClear;
    renderer.toneMapping = prev.tm;
  }
}
