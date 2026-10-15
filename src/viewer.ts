import * as THREE from 'three';
import CameraControls from 'camera-controls';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';
import { PartState, makeSolidMaterial, makeGhostMaterial, makeMaskMaterial, patchDepthMaterial, revealFx, ghostFx } from './materials';
import { makeGrid, ContactShadow, Outline } from './effects';
import { buildStandIn } from './standin';
import { studioScene } from './studio';
import { PartIndex, type PartsFile } from './parts';
import { subsystemById, robot as robotConfig } from './config';

CameraControls.install({
  THREE: {
    Vector2: THREE.Vector2, Vector3: THREE.Vector3, Vector4: THREE.Vector4, Quaternion: THREE.Quaternion,
    Matrix4: THREE.Matrix4, Spherical: THREE.Spherical, Box3: THREE.Box3, Sphere: THREE.Sphere,
    Raycaster: THREE.Raycaster,
  },
});
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

const DEG = Math.PI / 180;
const FOV = 32;
export const HOME_AZIMUTH = 32 * DEG;
export const HOME_POLAR = 62 * DEG;

/** Screen-space areas covered by UI, in CSS px; framing keeps the subject inside the rest. */
export interface Insets { top: number; right: number; bottom: number; left: number }
export interface FrameSpec { azimuth?: number | 'keep'; polar?: number | 'keep'; zoom?: number; transition?: boolean }
export type Quality = 'battery' | 'balanced' | 'max';

/** Critically damped spring toward a target (no overshoot), Unity-style SmoothDamp. */
export class Damped {
  value: number; target: number; vel = 0;
  constructor(v: number, public smoothTime: number) { this.value = v; this.target = v; }
  update(dt: number): boolean {
    if (this.value === this.target && this.vel === 0) return false;
    const omega = 2 / this.smoothTime, x = omega * dt;
    const exp = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);
    const change = this.value - this.target;
    const temp = (this.vel + omega * change) * dt;
    this.vel = (this.vel - omega * temp) * exp;
    let out = this.target + (change + temp) * exp;
    if ((this.target - this.value > 0) === (out > this.target)) { out = this.target; this.vel = 0; }
    this.value = out;
    if (Math.abs(this.value - this.target) < 1e-4 && Math.abs(this.vel) < 1e-3) { this.value = this.target; this.vel = 0; }
    return true;
  }
  snap(v = this.target) { this.value = this.target = v; this.vel = 0; }
}

interface Sub {
  id: string;
  group: THREE.Group;
  meshes: THREE.Mesh[];
  anchor: { obj: THREE.Object3D; local: THREE.Vector3 } | null;
  explodeDir: THREE.Vector3;
  outward: number | null;
}

export interface MechNode {
  node: THREE.Object3D;
  role: 'rotate' | 'crank' | 'follower' | 'coupler';
  axis: THREE.Vector3;
  link: THREE.Vector3 | null; // crank pivot → coupler joint (four-bar)
  restQ: THREE.Quaternion;
  restP: THREE.Vector3;
}

export interface Mechanism {
  id: string;
  name: string;
  type: 'rotate' | 'fourbar';
  nodes: MechNode[];
  /** Main pivot node (for pivot position / dragging). */
  node: THREE.Object3D;
  axis: THREE.Vector3;
  angle: Damped; // radians
  min: number; max: number; def: number; // degrees
  states: Record<string, number> | null;
  liftRobot: number;
  unit: string;
  /** Internal mechanisms (swerve steering) that don't get sliders. */
  hidden: boolean;
}

export interface PinInfo { id: string; x: number; y: number; far: boolean; visible: boolean }
export interface PickHit { pid: number; sub: string | null; point: THREE.Vector3 }

export class Viewer {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.PerspectiveCamera(FOV, 1, 0.008, 80);
  readonly controls: CameraControls;
  readonly canvas: HTMLCanvasElement;
  /** Robot root: carries the front rotation; its parent `stage` carries lift and (in simulation) pose. */
  readonly root = new THREE.Group();
  readonly stage = new THREE.Group();
  readonly keyLight = new THREE.DirectionalLight(0xfff6ec, 2.0);
  source: 'model' | 'standin' = 'model';
  parts: PartIndex | null = null;
  state: PartState = new PartState(0);
  onRender: (() => void) | null = null;
  onContext: ((lost: boolean) => void) | null = null;
  /** Extra per-frame updaters (simulation); return true while animating. */
  readonly hooks = new Set<(dt: number) => boolean>();

  private subs = new Map<string, Sub>();
  readonly mechanisms = new Map<string, Mechanism>();
  private meshes: THREE.Mesh[] = [];
  private ghosts: THREE.Mesh[] = [];
  private carrier: THREE.Object3D[] = []; // pid -> object whose matrixWorld places that part
  private pmrem: THREE.PMREMGenerator;
  private envRT: THREE.WebGLRenderTarget | null = null;
  readonly grid = makeGrid();
  private shadow!: ContactShadow;
  private outline!: Outline;
  private ghostMat!: THREE.ShaderMaterial;
  private shadowDirty = true;
  private dirty = true;
  private explode = new Damped(0, 0.22);
  private lift = new Damped(0, 0.3);
  private outlineStrength = new Damped(0, 0.1);
  private spinFactor = new Damped(0, 0.6);
  private focusMix = new Damped(1, 0.12);
  private floorAlpha = 1;
  private poseDirty = false;
  private reveal: { t: number; height: number } | null = null;
  private last = performance.now();
  private size = { w: 1, h: 1 };
  private focusPids: number[] | null = null;
  private running = true;
  private raycaster = new THREE.Raycaster();
  private robotHeight = 0.6;
  private robotRadius = 0.6;
  private light = false;
  private envLight = false;
  quality: Quality = 'balanced';
  reducedMotion = false;
  spinning = false;
  simActive = false;
  /** Pit showcase: keep turning while a subsystem is highlighted. */
  showcase = false;
  spinPeriod = robotConfig.secondsPerRevolution;
  stats = { fps: 0, calls: 0, triangles: 0, frames: 0, since: performance.now() };

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x000000, 0);
    // Khronos PBR Neutral keeps CAD colors true (AgX/ACES desaturate and shift hues).
    this.renderer.toneMapping = THREE.NeutralToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.info.autoReset = false;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.shadowMap.autoUpdate = false;
    this.pmrem = new THREE.PMREMGenerator(this.renderer);
    this.buildEnvironment();

    // Studio key light with soft shadows; the environment does the fill.
    const kl = this.keyLight;
    kl.position.set(1.4, 2.6, 1.9);
    kl.castShadow = true;
    kl.shadow.mapSize.set(2048, 2048);
    kl.shadow.camera.left = kl.shadow.camera.bottom = -0.95;
    kl.shadow.camera.right = kl.shadow.camera.top = 0.95;
    kl.shadow.camera.near = 0.5;
    kl.shadow.camera.far = 6;
    kl.shadow.bias = -0.0004;
    kl.shadow.normalBias = 0.004;
    kl.shadow.radius = 3;
    kl.target.position.set(0, 0.25, 0);
    this.scene.add(kl, kl.target);

    this.stage.add(this.root);
    this.scene.add(this.grid, this.stage);
    this.raycaster.firstHitOnly = false;

    const c = new CameraControls(this.camera, canvas);
    const A = CameraControls.ACTION;
    c.mouseButtons.left = A.ROTATE;
    c.mouseButtons.right = A.TRUCK;
    c.mouseButtons.middle = A.DOLLY;
    c.mouseButtons.wheel = A.DOLLY;
    c.touches.one = A.TOUCH_ROTATE;
    c.touches.two = A.TOUCH_DOLLY_TRUCK;
    c.touches.three = A.NONE;
    c.smoothTime = 0.26;
    c.draggingSmoothTime = 0.11;
    c.azimuthRotateSpeed = 0.85;
    c.polarRotateSpeed = 0.85;
    c.dollySpeed = 0.7;
    c.truckSpeed = 1.6;
    c.dollyToCursor = true; // zoom toward the finger / cursor
    c.minPolarAngle = 4 * DEG;
    c.maxPolarAngle = 168 * DEG; // the underside is reachable; the floor fades away below
    c.minDistance = 0.04;
    c.maxDistance = 3.6;
    c.restThreshold = 0.0025;
    this.controls = c;

    canvas.addEventListener('webglcontextlost', (e) => {
      e.preventDefault();
      this.running = false;
      this.onContext?.(true);
    });
    canvas.addEventListener('webglcontextrestored', () => {
      this.buildEnvironment();
      this.shadowDirty = true;
      this.state.commit();
      this.running = true;
      this.dirty = true;
      this.onContext?.(false);
    });

    requestAnimationFrame(this.tick);
  }

  private buildEnvironment() {
    this.envRT?.dispose();
    const studio = studioScene(this.light);
    this.envRT = this.pmrem.fromScene(studio, 0.02);
    this.scene.environment = this.envRT.texture;
    this.scene.environmentIntensity = this.light ? 0.85 : 1.0;
    studio.traverse((o) => { const m = o as THREE.Mesh; m.geometry?.dispose(); (m.material as THREE.Material | undefined)?.dispose(); });
  }

  // ---------------------------------------------------------------- loading
  async load(modelUrl: string, partsUrl: string, onProgress: (f: number) => void): Promise<void> {
    let model: THREE.Object3D;
    const partsPromise = fetch(partsUrl).then((r) => (r.ok ? (r.json() as Promise<PartsFile>) : null)).catch(() => null);
    try {
      const buf = await fetchWithProgress(modelUrl, onProgress);
      const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
      const gltf = await loader.parseAsync(buf, '');
      model = gltf.scene;
      this.source = 'model';
      const pf = await partsPromise;
      if (pf) this.parts = new PartIndex(pf);
    } catch (err) {
      console.info('[viewer] Model not available, using the stand-in robot.', err);
      model = buildStandIn();
      this.source = 'standin';
      onProgress(1);
    }
    this.state = new PartState(this.parts?.count ?? 0);
    this.setupRobot(model);
  }

  private setupRobot(model: THREE.Object3D) {
    this.ghostMat = makeGhostMaterial(this.state);
    this.outline = new Outline(makeMaskMaterial(this.state));
    this.shadow = new ContactShadow(this.state);
    this.scene.add(this.shadow.plane);
    const depthMat = patchDepthMaterial(new THREE.MeshDepthMaterial({ depthPacking: THREE.RGBADepthPacking }), this.state);
    const solid = new Map<string, THREE.Material>();

    model.updateMatrixWorld(true);
    // Subsystem groups keep their internal structure (mechanism pivots live inside them).
    const groups: THREE.Object3D[] = [];
    // GLTFLoader strips ':' from node names, so subsystem groups are found by their glTF extras
    // (the stand-in robot sets the same userData).
    model.traverse((o) => { if (!(o as THREE.Mesh).isMesh && typeof o.userData.subsystem === 'string' && !o.userData.mechanism) groups.push(o); });
    for (const g of groups) {
      const id = g.userData.subsystem as string;
      if (!subsystemById.has(id)) continue;
      const group = new THREE.Group();
      group.name = g.name;
      group.userData.subsystem = id;
      for (const child of [...g.children]) group.add(child);
      const meshes: THREE.Mesh[] = [];
      group.traverse((o) => {
        const m = o as THREE.Mesh;
        if (!m.isMesh) return;
        const src = Array.isArray(m.material) ? m.material[0] : m.material;
        const finish = (src?.userData?.finish as string) ?? src?.name ?? 'plastic';
        if (!solid.has(finish)) solid.set(finish, makeSolidMaterial(finish, this.state));
        src?.dispose();
        m.material = solid.get(finish)!;
        m.userData.subsystem = id;
        m.castShadow = finish !== 'polycarb';
        m.receiveShadow = true;
        m.customDepthMaterial = depthMat;
        if (finish === 'polycarb') m.renderOrder = 1;
        if (!m.geometry.attributes.normal) m.geometry.computeVertexNormals();
        meshes.push(m);
      });
      for (const m of meshes) {
        const ghost = new THREE.Mesh(m.geometry, this.ghostMat);
        ghost.userData.ghost = true;
        ghost.renderOrder = 5;
        ghost.visible = false;
        ghost.raycast = () => {};
        ghost.castShadow = false;
        m.add(ghost);
        this.ghosts.push(ghost);
        this.meshes.push(m);
      }
      this.root.add(group);
      this.subs.set(id, { id, group, meshes, anchor: null, explodeDir: new THREE.Vector3(), outward: null });
    }

    // Mechanisms (pivot nodes written by the build step). A four-bar has a crank, a follower and a
    // coupler node that all follow one angle; a plain mechanism has one rotating node.
    this.root.traverse((o) => {
      const u = o.userData;
      if (!u.mechanism) return;
      const axis = new THREE.Vector3().fromArray(u.axis ?? [0, 1, 0]).normalize();
      let m = this.mechanisms.get(u.mechanism);
      if (!m) {
        m = {
          id: u.mechanism, name: u.name ?? u.mechanism, type: u.type === 'fourbar' ? 'fourbar' : 'rotate', nodes: [], node: o, axis,
          angle: new Damped((u.default ?? 0) * DEG, u.type === 'fourbar' ? 0.42 : 0.32),
          min: u.min ?? -180, max: u.max ?? 180, def: u.default ?? 0,
          states: u.states ?? null, liftRobot: u.liftRobot ?? 0, unit: u.unit ?? '°', hidden: !!u.hidden,
        };
        this.mechanisms.set(u.mechanism, m);
      }
      const role = (u.role ?? 'rotate') as MechNode['role'];
      if (role === 'crank' || role === 'rotate') m.node = o;
      m.nodes.push({ node: o, role, axis, link: u.link ? new THREE.Vector3().fromArray(u.link) : null, restQ: o.quaternion.clone(), restP: o.position.clone() });
    });

    // Which object carries each part id (so moving parts report correct boxes).
    if (this.parts) {
      this.carrier = new Array(this.parts.count + 1).fill(this.root);
      for (const m of this.meshes) {
        const a = m.geometry.getAttribute('_partid') as THREE.BufferAttribute | undefined;
        if (!a) continue;
        let prev = -1;
        for (let i = 0; i < a.count; i++) { const pid = a.getX(i); if (pid !== prev) { this.carrier[pid] = m.parent!; prev = pid; } }
      }
    }

    this.root.rotation.y = this.frontRotation();
    this.root.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.root);
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    this.robotHeight = size.y;
    this.robotRadius = box.getBoundingSphere(new THREE.Sphere()).radius;

    // Pins, explode directions and outward viewing angles.
    const homeDir = new THREE.Vector3().setFromSphericalCoords(1, HOME_POLAR, HOME_AZIMUTH);
    const ordered = [...this.subs.values()].map((s) => ({ s, box: new THREE.Box3().setFromObject(s.group) }));
    ordered.sort((a, b) => a.box.getCenter(new THREE.Vector3()).y - b.box.getCenter(new THREE.Vector3()).y);
    ordered.forEach(({ s, box: b }, rank) => {
      const c = b.getCenter(new THREE.Vector3());
      const target = c.clone().addScaledVector(homeDir, this.robotRadius * 0.9);
      target.y = Math.max(target.y, b.max.y);
      s.anchor = nearestVertex(s.meshes, target);
      const dx = c.x - center.x, dz = c.z - center.z;
      const r = Math.hypot(dx, dz);
      s.outward = r > 0.08 ? Math.atan2(dx, dz) : null;
      const spread = rank === 0 ? 0 : Math.min(1, r / 0.1) * Math.max(size.x, size.z) * 0.32;
      const lift = rank === 0 ? 0 : 0.07 + rank * 0.085;
      const dirWorld = new THREE.Vector3(r > 1e-3 ? (dx / r) * spread : 0, lift, r > 1e-3 ? (dz / r) * spread : 0);
      s.explodeDir.copy(dirWorld.applyQuaternion(this.root.quaternion.clone().invert()));
    });

    this.frameOverview({ top: 0, right: 0, bottom: 0, left: 0 }, false, true);
    this.shadowDirty = true;
    this.dirty = true;
  }

  private frontRotation(): number {
    const f = (robotConfig.front || 'auto').toLowerCase().trim();
    const fixed: Record<string, number> = { '+z': 0, '-z': Math.PI, '+x': -Math.PI / 2, '-x': Math.PI / 2 };
    if (f in fixed) return fixed[f];
    const intake = this.subs.get('intake') ?? [...this.subs.values()][0];
    if (!intake) return 0;
    const all = new THREE.Box3();
    for (const s of this.subs.values()) all.union(new THREE.Box3().setFromObject(s.group));
    const c = all.getCenter(new THREE.Vector3());
    const ic = new THREE.Box3().setFromObject(intake.group).getCenter(new THREE.Vector3());
    const dx = ic.x - c.x, dz = ic.z - c.z;
    if (Math.hypot(dx, dz) < 0.02) return 0;
    const a = Math.atan2(dx, dz);
    return -Math.round(a / (Math.PI / 2)) * (Math.PI / 2);
  }

  buildPickingInBackground() {
    const queue = [...this.meshes].sort((a, b) => a.geometry.attributes.position.count - b.geometry.attributes.position.count);
    const step = () => {
      const m = queue.shift();
      if (!m) return;
      if (!m.geometry.boundsTree) m.geometry.computeBoundsTree();
      setTimeout(step, 24);
    };
    setTimeout(step, 50);
  }
  get pickingReady() { return this.meshes.length > 0 && this.meshes.every((m) => m.geometry.boundsTree); }

  // ---------------------------------------------------------------- reveal
  startReveal() {
    if (this.reducedMotion) { revealFx.uReveal.value = 1e3; revealFx.uBand.value = 0; this.dirty = true; return; }
    this.reveal = { t: 0, height: this.robotHeight + 0.4 };
    revealFx.uReveal.value = -0.02;
    revealFx.uBand.value = 1;
  }
  get revealing() { return this.reveal !== null; }

  // ---------------------------------------------------------------- appearance
  setTheme(light: boolean) {
    this.light = light;
    const g = this.grid.material as THREE.ShaderMaterial;
    g.uniforms.uColor.value.set(light ? 0x1d3550 : 0x9fd6ff);
    ghostFx.uLight.value = light ? 1 : 0;
    ghostFx.uGhostColor.value.set(light ? 0x1f6fd1 : 0x74c7ff);
    if (this.ghostMat) { this.ghostMat.blending = light ? THREE.NormalBlending : THREE.AdditiveBlending; this.ghostMat.needsUpdate = true; }
    if (this.envLight !== light) { this.envLight = light; this.buildEnvironment(); }
    this.floorAlpha = -1; // force refresh
    this.dirty = true;
  }
  setAccent(hex: string) { this.outline?.color.value.set(hex); this.dirty = true; }
  private ghostBase = 1;
  private ghostScale = 1;
  setGhostStrength(v: number) { this.ghostBase = v; ghostFx.uGhostStrength.value = v * this.ghostScale; this.dirty = true; }
  setQuality(q: Quality) {
    this.quality = q;
    const cap = q === 'battery' ? 1 : q === 'balanced' ? 1.5 : 2;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, cap));
    this.keyLight.castShadow = q !== 'battery';
    const ms = q === 'max' ? 4096 : 2048;
    if (this.keyLight.shadow.mapSize.x !== ms) {
      this.keyLight.shadow.mapSize.set(ms, ms);
      this.keyLight.shadow.map?.dispose();
      this.keyLight.shadow.map = null;
    }
    this.resize(this.size.w, this.size.h);
    this.shadowDirty = true;
  }

  // ---------------------------------------------------------------- framing
  private fitBox(box: THREE.Box3, azimuths: number[], polar: number, insets: Insets, pad = 1.06) {
    const { w, h } = this.size;
    const tanV = Math.tan((FOV * DEG) / 2);
    const wf = Math.max(80, w - insets.left - insets.right);
    const hf = Math.max(80, h - insets.top - insets.bottom);
    const tx = tanV * (wf / h), ty = tanV * (hf / h);
    const c = box.getCenter(new THREE.Vector3());
    const corners: THREE.Vector3[] = [];
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) corners.push(new THREE.Vector3(x, y, z).sub(c));
    let d = 0;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler(), v = new THREE.Vector3();
    for (const az of azimuths) {
      e.set(polar - Math.PI / 2, az, 0, 'YXZ');
      m.makeRotationFromQuaternion(q.setFromEuler(e)).invert();
      for (const p of corners) {
        v.copy(p).applyMatrix4(m);
        d = Math.max(d, v.z + Math.abs(v.x) / tx, v.z + Math.abs(v.y) / ty);
      }
    }
    return d * pad;
  }

  private applyFocalOffset(d: number, insets: Insets, t: boolean) {
    const { w, h } = this.size;
    const k = (2 * d * Math.tan((FOV * DEG) / 2)) / h;
    const cx = (insets.left + (w - insets.right)) / 2;
    const cy = (insets.top + (h - insets.bottom)) / 2;
    // camera-controls treats +y focal offset as screen-down
    this.controls.setFocalOffset((w / 2 - cx) * k, (h / 2 - cy) * k, 0, t);
  }

  private shortestAzimuth(target: number) {
    this.controls.normalizeRotations();
    const cur = this.controls.azimuthAngle;
    let d = (target - cur) % (Math.PI * 2);
    if (d > Math.PI) d -= Math.PI * 2;
    if (d < -Math.PI) d += Math.PI * 2;
    return cur + d;
  }

  /** World box of a set of parts where they will be once explode / lift / mechanisms settle. */
  boxOfParts(pids: ArrayLike<number>, atTarget = true): THREE.Box3 {
    const out = new THREE.Box3();
    this.withTargetPose(atTarget, () => {
      if (!this.parts) {
        for (const s of this.subs.values()) out.union(new THREE.Box3().setFromObject(s.group));
        return;
      }
      const b = new THREE.Box3();
      const byCarrier = new Map<THREE.Object3D, number[]>();
      for (let i = 0; i < pids.length; i++) {
        const c = this.carrier[pids[i]] ?? this.root;
        (byCarrier.get(c) ?? byCarrier.set(c, []).get(c)!).push(pids[i]);
      }
      for (const [c, list] of byCarrier) {
        this.parts!.boxOfParts(list, b);
        if (!b.isEmpty()) out.union(b.applyMatrix4(c.matrixWorld));
      }
    });
    return out;
  }

  /** Temporarily applies the target explode / lift / mechanism pose so framing doesn't chase animations. */
  private withTargetPose(enabled: boolean, fn: () => void) {
    if (!enabled) { this.stage.updateMatrixWorld(true); fn(); return; }
    const mechs = [...this.mechanisms.values()];
    const saved = { e: this.explode.value, l: this.lift.value, m: mechs.map((m) => m.angle.value) };
    this.applyPose(this.explode.target, this.lift.target, mechs.map((m) => m.angle.target));
    fn();
    this.applyPose(saved.e, saved.l, saved.m);
  }

  private applyPose(explode: number, lift: number, angles: number[]) {
    for (const s of this.subs.values()) s.group.position.copy(s.explodeDir).multiplyScalar(explode);
    if (!this.simActive) this.stage.position.y = lift;
    const q = new THREE.Quaternion(), v = new THREE.Vector3();
    [...this.mechanisms.values()].forEach((m, i) => {
      for (const n of m.nodes) {
        q.setFromAxisAngle(n.axis, angles[i]);
        if (n.role === 'coupler') {
          // parallel four-bar: the coupler translates along the arc of the crank's far joint, without rotating
          if (n.link) n.node.position.copy(n.restP).add(v.copy(n.link).applyQuaternion(q).sub(n.link));
        } else n.node.quaternion.copy(n.restQ).multiply(q);
      }
    });
    this.stage.updateMatrixWorld(true);
  }

  robotBox(): THREE.Box3 {
    if (!this.parts) return this.boxOfParts([]);
    const pids: number[] = [];
    for (let p = 1; p <= this.parts.count; p++) if (this.state.isVisible(p)) pids.push(p);
    return this.boxOfParts(pids);
  }

  private updateBoundary(box: THREE.Box3) { this.controls.setBoundary(box.clone().expandByScalar(0.05)); }

  private overviewDistance(box: THREE.Box3, insets: Insets) {
    const azs = Array.from({ length: 16 }, (_, i) => (i / 16) * Math.PI * 2);
    return this.fitBox(box, azs, HOME_POLAR, insets, 1.04);
  }

  frameOverview(insets: Insets, transition: boolean, resetAngle = false) {
    if (!this.subs.size) return;
    const box = this.robotBox();
    if (box.isEmpty()) return;
    this.updateBoundary(box);
    const center = box.getCenter(new THREE.Vector3());
    const d = this.overviewDistance(box, insets);
    this.withMotion(transition, (t) => {
      this.controls.moveTo(center.x, center.y, center.z, t);
      this.controls.dollyTo(d, t);
      this.applyFocalOffset(d, insets, t);
      if (resetAngle) this.controls.rotateTo(this.shortestAzimuth(HOME_AZIMUTH), HOME_POLAR, t);
      else if (this.lift.target === 0) this.controls.rotateTo(this.controls.azimuthAngle, THREE.MathUtils.clamp(this.controls.polarAngle, 48 * DEG, 72 * DEG), t);
    });
  }

  /** Default viewing angle for a subsystem (config override, else from outside its side). */
  subsystemAngle(id: string): FrameSpec {
    const cfg = subsystemById.get(id)?.camera ?? {};
    const s = this.subs.get(id);
    let az: number | 'keep' = 'keep';
    if (cfg.azimuth !== undefined) az = cfg.azimuth * DEG;
    else if (s?.outward != null) az = s.outward;
    return { azimuth: az, polar: (90 - (cfg.elevation ?? 32)) * DEG, zoom: cfg.zoom ?? 1 };
  }

  /** Camera flight to a set of parts. */
  frameParts(pids: ArrayLike<number> | null, insets: Insets, spec: FrameSpec = {}) {
    const box = pids ? this.boxOfParts(pids) : this.robotBox();
    if (box.isEmpty()) return;
    this.updateBoundary(this.robotBox().union(box));
    const center = box.getCenter(new THREE.Vector3());
    const az = spec.azimuth === undefined || spec.azimuth === 'keep' ? this.controls.azimuthAngle : this.shortestAzimuth(spec.azimuth);
    const polar = spec.polar === undefined || spec.polar === 'keep' ? this.controls.polarAngle : spec.polar;
    const size = box.getSize(new THREE.Vector3()).length();
    const pad = size < 0.08 ? 1.6 : size < 0.2 ? 1.35 : 1.22;
    const d = THREE.MathUtils.clamp(this.fitBox(box, [az], polar, insets, pad) / (spec.zoom ?? 1), 0.07, 3.4);
    this.withMotion(spec.transition ?? true, (t) => {
      this.controls.moveTo(center.x, center.y, center.z, t);
      this.controls.rotateTo(az, polar, t);
      this.controls.dollyTo(d, t);
      this.applyFocalOffset(d, insets, t);
    });
  }

  private withMotion(transition: boolean, fn: (t: boolean) => void) {
    if (transition && this.reducedMotion) {
      this.canvas.style.transition = 'opacity 120ms linear';
      this.canvas.style.opacity = '0';
      setTimeout(() => {
        fn(false);
        this.controls.update(0);
        this.dirty = true;
        this.canvas.style.opacity = '1';
      }, 125);
      return;
    }
    fn(transition);
    this.dirty = true;
  }

  // ---------------------------------------------------------------- focus / visibility
  /** Solid parts = `pids`; everything else becomes hologram. null = whole robot solid. */
  setFocus(pids: ArrayLike<number> | null) {
    this.focusPids = pids ? Array.from(pids) : null;
    if (this.parts) {
      this.state.setFocus(pids ? new Set(Array.from(pids)) : null);
      this.focusMix.snap(0);
      this.focusMix.target = 1;
      if (this.reducedMotion) this.focusMix.snap(1);
      this.state.uniforms.uFocusMix.value = this.focusMix.value;
    }
    for (const g of this.ghosts) g.visible = true;
    // Close-ups of small parts stack many hologram layers behind them; calm the hologram down.
    const size = pids ? this.boxOfParts(this.focusPids!, true).getSize(new THREE.Vector3()).length() : 1;
    this.ghostScale = THREE.MathUtils.clamp(size / 0.45, 0.4, 1);
    ghostFx.uGhostStrength.value = this.ghostBase * this.ghostScale;
    this.outlineStrength.target = pids ? 1 : 0;
    this.shadowDirty = true;
    this.dirty = true;
  }
  get focused() { return this.focusPids !== null; }

  setHidden(pids: ArrayLike<number>, hidden: boolean) {
    for (let i = 0; i < pids.length; i++) this.state.setVisible(pids[i], !hidden);
    this.state.commit();
    this.shadowDirty = true;
    this.dirty = true;
  }
  showAll() {
    if (!this.parts) return;
    for (let pid = 1; pid <= this.parts.count; pid++) this.state.setVisible(pid, true);
    this.state.commit();
    this.shadowDirty = true;
    this.dirty = true;
  }
  hiddenCount() {
    if (!this.parts) return 0;
    let n = 0;
    for (let pid = 1; pid <= this.parts.count; pid++) if (!this.state.isVisible(pid)) n++;
    return n;
  }

  setHighlight(pids: ArrayLike<number> | null) {
    if (!this.parts) return;
    const d = this.state.data;
    for (let i = 1; i < d.length; i += 4) d[i] = 0;
    if (pids) for (let i = 0; i < pids.length; i++) this.state.setHighlight(pids[i], 1);
    this.state.commit();
    this.dirty = true;
  }

  // ---------------------------------------------------------------- mechanisms
  setMechanism(id: string, degrees: number, animate = true) {
    const m = this.mechanisms.get(id);
    if (!m) return;
    m.angle.target = THREE.MathUtils.clamp(degrees, m.min, m.max) * DEG;
    if (!animate || this.reducedMotion) { m.angle.snap(); this.poseDirty = true; }
    let lift = 0;
    for (const mm of this.mechanisms.values()) if (mm.liftRobot && Math.abs(mm.angle.target) > 0.01) lift = Math.max(lift, mm.liftRobot);
    this.lift.target = this.simActive ? 0 : lift;
    if (this.reducedMotion) this.lift.snap();
    this.shadowDirty = true;
    this.dirty = true;
  }
  /** Direct manipulation: follow the finger closely, then settle normally on release. */
  dragMechanism(id: string, degrees: number) {
    const m = this.mechanisms.get(id);
    if (!m) return;
    m.angle.smoothTime = 0.06;
    m.angle.target = THREE.MathUtils.clamp(degrees, m.min, m.max) * DEG;
    this.shadowDirty = true;
    this.dirty = true;
  }
  endDragMechanism(id: string) {
    const m = this.mechanisms.get(id);
    if (m) m.angle.smoothTime = m.type === 'fourbar' ? 0.42 : 0.32;
  }
  /** Applies the current mechanism angles immediately (the simulation drives them itself). */
  syncPose() {
    this.applyPose(this.explode.value, this.lift.value, [...this.mechanisms.values()].map((m) => m.angle.value));
    this.shadowDirty = true;
  }
  /** Sets a mechanism's current angle directly (no smoothing), in degrees. */
  setMechanismNow(id: string, degrees: number) {
    const m = this.mechanisms.get(id);
    if (!m) return;
    m.angle.snap(THREE.MathUtils.clamp(degrees, m.min, m.max) * DEG);
  }
  mechanismAngle(id: string) { const m = this.mechanisms.get(id); return m ? m.angle.target / DEG : 0; }
  mechanismValue(id: string) { const m = this.mechanisms.get(id); return m ? m.angle.value / DEG : 0; }
  get lifted() { return this.lift.target > 0; }

  // ---------------------------------------------------------------- explode
  setExploded(on: boolean) {
    this.explode.target = on ? 1 : 0;
    if (this.reducedMotion) this.explode.snap();
    this.shadowDirty = true;
    this.dirty = true;
  }

  // ---------------------------------------------------------------- picking
  pick(clientX: number, clientY: number): PickHit | null {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const ready = this.meshes.filter((m) => m.geometry.boundsTree);
    if (!ready.length) return null;
    const hits = this.raycaster.intersectObjects(ready, false);
    for (const h of hits) {
      const m = h.object as THREE.Mesh;
      const a = m.geometry.getAttribute('_partid') as THREE.BufferAttribute | undefined;
      const pid = a && h.face ? a.getX(h.face.a) : 0;
      if (pid && !this.state.isVisible(pid)) continue;
      return { pid, sub: (m.userData.subsystem as string) ?? null, point: h.point.clone() };
    }
    return null;
  }

  // ---------------------------------------------------------------- pins & overlays
  pins(): PinInfo[] {
    const out: PinInfo[] = [];
    const camDir = this.camera.getWorldDirection(new THREE.Vector3());
    const center = this.controls.getTarget(new THREE.Vector3());
    const v = new THREE.Vector3();
    for (const s of this.subs.values()) {
      if (!s.anchor) continue;
      s.anchor.obj.localToWorld(v.copy(s.anchor.local));
      const far = v.clone().sub(center).dot(camDir) > this.robotRadius * 0.12;
      v.project(this.camera);
      const visible = v.z < 1 && Math.abs(v.x) < 1.05 && Math.abs(v.y) < 1.05;
      out.push({ id: s.id, x: (v.x * 0.5 + 0.5) * this.size.w, y: (-v.y * 0.5 + 0.5) * this.size.h, far, visible });
    }
    return out;
  }

  /** Screen rectangle (CSS px) around the focused parts, for the targeting brackets. */
  focusRect(): { x: number; y: number; w: number; h: number } | null {
    if (!this.focusPids || !this.focusPids.length) return null;
    const box = this.boxOfParts(this.focusPids, false);
    if (box.isEmpty()) return null;
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    const v = new THREE.Vector3();
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
      v.set(x, y, z).project(this.camera);
      if (v.z > 1) return null;
      const sx = (v.x * 0.5 + 0.5) * this.size.w, sy = (-v.y * 0.5 + 0.5) * this.size.h;
      x0 = Math.min(x0, sx); y0 = Math.min(y0, sy); x1 = Math.max(x1, sx); y1 = Math.max(y1, sy);
    }
    return { x: x0, y: y0, w: x1 - x0, h: y1 - y0 };
  }

  get ids() { return [...this.subs.keys()]; }
  subsystemPids(id: string) { return this.parts ? this.parts.partsOfSubsystem(id) : []; }

  // ---------------------------------------------------------------- size
  resize(w: number, h: number) {
    this.size = { w, h };
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    const pr = this.renderer.getPixelRatio();
    this.outline?.setSize(w * pr, h * pr);
    this.dirty = true;
  }
  get viewSize() { return this.size; }
  invalidate(shadows = false) { this.dirty = true; if (shadows) this.shadowDirty = true; }

  // ---------------------------------------------------------------- loop
  private tick = (now: number) => {
    requestAnimationFrame(this.tick);
    const dt = Math.min(0.05, Math.max(0.001, (now - this.last) / 1000));
    this.last = now;
    if (!this.running) return;
    if (this.advance(dt) || this.dirty) this.render();
  };

  private advance(dt: number): boolean {
    let active = false;
    for (const h of this.hooks) if (h(dt)) active = true;

    this.spinFactor.target = this.spinning && !this.reducedMotion && (!this.focusPids || this.showcase) && !this.simActive ? 1 : 0;
    if (!this.spinning) this.spinFactor.snap(0);
    active = this.spinFactor.update(dt) || active;
    if (this.spinFactor.value > 0.0005) {
      this.controls.rotate(((Math.PI * 2) / this.spinPeriod) * dt * this.spinFactor.value, 0, true);
      active = true;
    }
    active = this.controls.update(dt) || active;

    if (this.reveal) {
      this.reveal.t += dt / 1.6;
      const t = Math.min(1, this.reveal.t);
      const e = t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
      revealFx.uReveal.value = -0.02 + e * this.reveal.height;
      revealFx.uBand.value = t < 0.85 ? 1 : 1 - (t - 0.85) / 0.15;
      if (t >= 1) { this.reveal = null; revealFx.uReveal.value = 1e3; revealFx.uBand.value = 0; }
      active = true;
    }

    let pose = this.poseDirty;
    this.poseDirty = false;
    if (this.explode.update(dt)) pose = true;
    if (this.lift.update(dt)) pose = true;
    for (const m of this.mechanisms.values()) if (m.angle.update(dt)) pose = true;
    if (pose) {
      this.cutKey = '';
      this.applyPose(this.explode.value, this.lift.value, [...this.mechanisms.values()].map((m) => m.angle.value));
      this.shadowDirty = true;
      active = true;
    }
    if (this.focusMix.update(dt)) {
      this.state.uniforms.uFocusMix.value = this.focusMix.value;
      if (this.focusMix.value >= 1 && !this.focusPids) for (const g of this.ghosts) g.visible = false;
      this.shadowDirty = true;
      active = true;
    }
    if (this.outlineStrength.update(dt) && this.outline) { this.outline.strength.value = this.outlineStrength.value; active = true; }
    this.updateCutaway();

    // Floor fades away when the camera goes below it (underside view) or the robot is lifted.
    const camY = this.camera.position.y;
    const fa = this.simActive ? 0 : THREE.MathUtils.clamp(camY / 0.25, 0, 1) * (1 - THREE.MathUtils.clamp(this.lift.value / 0.25, 0, 0.6));
    if (Math.abs(fa - this.floorAlpha) > 0.002) {
      this.floorAlpha = fa;
      (this.grid.material as THREE.ShaderMaterial).uniforms.uStrength.value = fa * (this.light ? 1.6 : 1);
      if (this.shadow) this.shadow.opacity = fa * (this.light ? 0.55 : 0.88);
      active = true;
    }
    return active;
  }

  /** Holograms between the camera and the selection fade, so the selected part reads clearly. */
  private cutBox: THREE.Box3 | null = null;
  private cutKey = '';
  private updateCutaway() {
    if (!this.focusPids || this.focusPids.length > 400 || this.simActive) { ghostFx.uCut.value = 0; return; }
    const key = this.focusPids.length + ':' + this.focusPids[0];
    if (key !== this.cutKey) { this.cutKey = key; this.cutBox = this.boxOfParts(this.focusPids, false); }
    if (!this.cutBox || this.cutBox.isEmpty()) { ghostFx.uCut.value = 0; return; }
    this.camera.updateMatrixWorld();
    const v = new THREE.Vector3();
    let near = Infinity;
    const b = this.cutBox;
    for (const x of [b.min.x, b.max.x]) for (const y of [b.min.y, b.max.y]) for (const z of [b.min.z, b.max.z]) {
      v.set(x, y, z).applyMatrix4(this.camera.matrixWorldInverse);
      near = Math.min(near, -v.z);
    }
    ghostFx.uCut.value = Math.max(0.001, near);
  }

  /** Test hook: jumps every animation to its end state and renders once. */
  settle() {
    const spinning = this.spinning;
    this.spinning = false;
    if (this.reveal) this.reveal.t = 1;
    for (let i = 0; i < 8; i++) this.advance(2);
    this.spinning = spinning;
    this.render();
  }

  private render() {
    this.dirty = false;
    const r = this.renderer;
    if (this.shadowDirty && this.subs.size) {
      this.stage.updateMatrixWorld(true);
      if (!this.simActive) this.shadow.update(r, this.scene, this.stage);
      r.shadowMap.needsUpdate = true;
      this.shadowDirty = false;
    }
    if (this.simActive) r.shadowMap.needsUpdate = true;
    r.info.reset();
    r.render(this.scene, this.camera);
    this.stats.calls = r.info.render.calls;
    this.stats.triangles = r.info.render.triangles;
    if (!this.simActive && this.outline) this.outline.render(r, this.scene, this.camera, this.stage);
    this.stats.frames++;
    const now = performance.now();
    if (now - this.stats.since > 500) {
      this.stats.fps = (this.stats.frames * 1000) / (now - this.stats.since);
      this.stats.frames = 0;
      this.stats.since = now;
    }
    this.onRender?.();
  }

  modelStats() {
    let tris = 0;
    for (const m of this.meshes) tris += (m.geometry.index?.count ?? m.geometry.attributes.position.count) / 3;
    return { meshes: this.meshes.length, triangles: Math.round(tris), source: this.source, parts: this.parts?.count ?? 0 };
  }
}

async function fetchWithProgress(url: string, onProgress: (f: number) => void): Promise<ArrayBuffer> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${url}`);
  const total = Number(res.headers.get('content-length')) || 13e6;
  if (!res.body) { onProgress(1); return res.arrayBuffer(); }
  const reader = res.body.getReader();
  const chunks: Uint8Array[] = [];
  let got = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    got += value.byteLength;
    onProgress(Math.min(0.98, got / total));
  }
  const out = new Uint8Array(got);
  let o = 0;
  for (const c of chunks) { out.set(c, o); o += c.byteLength; }
  onProgress(1);
  if (out[0] !== 0x67 || out[1] !== 0x6c || out[2] !== 0x54 || out[3] !== 0x46) throw new Error('Not a GLB file');
  return out.buffer;
}

function nearestVertex(meshes: THREE.Mesh[], target: THREE.Vector3): { obj: THREE.Object3D; local: THREE.Vector3 } | null {
  let best: { obj: THREE.Object3D; local: THREE.Vector3 } | null = null, bestD = Infinity;
  const v = new THREE.Vector3();
  for (const m of meshes) {
    const pos = m.geometry.attributes.position;
    m.updateMatrixWorld(true);
    const stride = Math.max(1, Math.floor(pos.count / 40000));
    for (let i = 0; i < pos.count; i += stride) {
      v.fromBufferAttribute(pos, i).applyMatrix4(m.matrixWorld);
      const d = (v.x - target.x) ** 2 + (v.z - target.z) ** 2 + 0.35 * (v.y - target.y) ** 2;
      if (d < bestD) { bestD = d; best = { obj: m, local: new THREE.Vector3().fromBufferAttribute(pos, i) }; }
    }
  }
  return best;
}
