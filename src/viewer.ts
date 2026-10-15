import * as THREE from 'three';
import CameraControls from 'camera-controls';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';
import { makeMaterial, applyGhostState, newFx, revealFx, type SubsystemFx } from './materials';
import { makeGrid, ContactShadow, Outline, OUTLINE_LAYER } from './effects';
import { buildStandIn } from './standin';
import { subsystems, subsystemById, robot as robotConfig } from './config';

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
const HOME_AZIMUTH = 32 * DEG;
const HOME_POLAR = 62 * DEG;

/** Screen-space areas covered by UI, in CSS px; framing keeps the robot inside the rest. */
export interface Insets { top: number; right: number; bottom: number; left: number }

/** Critically damped spring toward a target (no overshoot), Unity-style SmoothDamp. */
class Damped {
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
  materials: THREE.Material[];
  fx: SubsystemFx;
  ghost: Damped;
  highlight: Damped;
  anchor: THREE.Vector3;       // pin position in group-local space
  explodeDir: THREE.Vector3;   // offset at full explode, in root-local space
  outward: number | null;      // azimuth that looks at this subsystem from outside
}

export interface PinInfo { id: string; x: number; y: number; far: boolean; visible: boolean }

export class Viewer {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.PerspectiveCamera(FOV, 1, 0.02, 50);
  readonly controls: CameraControls;
  readonly canvas: HTMLCanvasElement;
  readonly root = new THREE.Group();
  source: 'model' | 'standin' = 'model';
  onRender: (() => void) | null = null;
  onContext: ((lost: boolean) => void) | null = null;

  private subs = new Map<string, Sub>();
  private pickables: THREE.Mesh[] = [];
  private robotRoot: THREE.Object3D | null = null;
  private pmrem: THREE.PMREMGenerator;
  private envRT: THREE.WebGLRenderTarget | null = null;
  private grid = makeGrid();
  private shadow = new ContactShadow();
  private outline = new Outline();
  private shadowDirty = true;
  private dirty = true;
  private explode = new Damped(0, 0.22);
  private outlineStrength = new Damped(0, 0.1);
  private spinFactor = new Damped(0, 0.6);
  private reveal: { t: number; height: number } | null = null;
  private last = performance.now();
  private size = { w: 1, h: 1 };
  private focused: string | null = null;
  private running = true;
  private raycaster = new THREE.Raycaster();
  private robotHeight = 0.6;
  private robotRadius = 0.6;
  reducedMotion = false;
  spinning = false;
  /** Seconds per full turn of the turntable. */
  spinPeriod = robotConfig.secondsPerRevolution;
  stats = { fps: 0, calls: 0, triangles: 0, frames: 0, since: performance.now() };

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.toneMapping = THREE.AgXToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.info.autoReset = false;
    this.pmrem = new THREE.PMREMGenerator(this.renderer);
    this.buildEnvironment();

    this.scene.add(this.grid, this.shadow.plane, this.root);
    this.raycaster.firstHitOnly = true;

    const c = new CameraControls(this.camera, canvas);
    const A = CameraControls.ACTION;
    c.mouseButtons.left = A.ROTATE;
    c.mouseButtons.right = A.TRUCK;
    c.mouseButtons.middle = A.DOLLY;
    c.mouseButtons.wheel = A.DOLLY;
    c.touches.one = A.TOUCH_ROTATE;
    c.touches.two = A.TOUCH_DOLLY_TRUCK;
    c.touches.three = A.NONE;
    c.smoothTime = 0.26;          // ≈0.9 s critically damped flights, no bounce
    c.draggingSmoothTime = 0.11;  // weight + inertia under the finger
    c.azimuthRotateSpeed = 0.85;
    c.polarRotateSpeed = 0.85;
    c.dollySpeed = 0.7;
    c.truckSpeed = 1.6;
    c.dollyToCursor = false;
    c.minPolarAngle = 6 * DEG;
    c.maxPolarAngle = 86 * DEG;   // never under the floor
    c.minDistance = 0.22;
    c.maxDistance = 3.4;
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
      this.running = true;
      this.dirty = true;
      this.onContext?.(false);
    });

    requestAnimationFrame(this.tick);
  }

  private buildEnvironment() {
    this.envRT?.dispose();
    const room = new RoomEnvironment();
    this.envRT = this.pmrem.fromScene(room, 0.035);
    this.scene.environment = this.envRT.texture;
    this.scene.environmentIntensity = 0.95;
    room.traverse((o) => { const m = o as THREE.Mesh; m.geometry?.dispose(); });
  }

  // ---------------------------------------------------------------- loading
  async load(url: string, onProgress: (f: number) => void): Promise<void> {
    let model: THREE.Object3D;
    try {
      const buf = await fetchWithProgress(url, onProgress);
      const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
      const gltf = await loader.parseAsync(buf, '');
      model = gltf.scene;
      this.source = 'model';
    } catch (err) {
      console.info('[viewer] Model not available, using the stand-in robot.', err);
      model = buildStandIn();
      this.source = 'standin';
      onProgress(1);
    }
    this.setupRobot(model);
  }

  private setupRobot(model: THREE.Object3D) {
    // Collect meshes by subsystem (node extras → userData, or "subsystem:<id>" names).
    const byId = new Map<string, THREE.Mesh[]>();
    model.traverse((o) => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      let id: string | undefined;
      for (let p: THREE.Object3D | null = m; p && !id; p = p.parent) {
        id = p.userData?.subsystem ?? (p.name.startsWith('subsystem:') ? p.name.slice(10) : undefined);
      }
      if (!id || !subsystemById.has(id)) id = subsystems[subsystems.length - 1].id;
      (byId.get(id) ?? byId.set(id, []).get(id)!).push(m);
    });

    model.updateMatrixWorld(true);
    for (const s of subsystems) {
      const meshes = byId.get(s.id);
      if (!meshes?.length) continue;
      const group = new THREE.Group();
      group.name = `subsystem:${s.id}`;
      group.userData.subsystem = s.id;
      const fx = newFx();
      const materials: THREE.Material[] = [];
      const matByFinish = new Map<string, THREE.Material>();
      for (const m of meshes) {
        const src = Array.isArray(m.material) ? m.material[0] : m.material;
        const finish = (src?.userData?.finish as string) ?? src?.name ?? 'plastic';
        let mat = matByFinish.get(finish);
        if (!mat) { mat = makeMaterial(finish, fx); matByFinish.set(finish, mat); materials.push(mat); }
        src?.dispose();
        m.material = mat;
        m.userData.subsystem = s.id;
        // Bake the mesh's world transform into a flat child of the subsystem group.
        m.matrixWorld.decompose(m.position, m.quaternion, m.scale);
        m.removeFromParent();
        group.add(m);
        if (!m.geometry.attributes.normal) m.geometry.computeVertexNormals();
        if (m.material && (m.material as THREE.Material).transparent) m.renderOrder = 1;
        this.pickables.push(m);
      }
      this.subs.set(s.id, {
        id: s.id, group, meshes, materials, fx,
        ghost: new Damped(0, 0.14), highlight: new Damped(0, 0.06),
        anchor: new THREE.Vector3(), explodeDir: new THREE.Vector3(), outward: null,
      });
      this.root.add(group);
    }
    this.robotRoot = model;

    // Face the front toward the starting camera.
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
      // Pin sits on the part of the subsystem that faces the starting camera, near its middle.
      const target = c.clone().addScaledVector(homeDir, this.robotRadius * 0.9);
      target.y = Math.max(target.y, b.max.y);
      const p = nearestVertex(s.meshes, target) ?? target;
      p.y += 0.02;
      s.anchor.copy(s.group.worldToLocal(p.clone()));
      const dx = c.x - center.x, dz = c.z - center.z;
      const r = Math.hypot(dx, dz);
      s.outward = r > 0.08 ? Math.atan2(dx, dz) : null;
      // Lowest subsystem (the drivetrain) stays on the floor; the rest lift in layers and spread out.
      const spread = rank === 0 ? 0 : Math.min(1, r / 0.1) * Math.max(size.x, size.z) * 0.32;
      const lift = rank === 0 ? 0 : 0.07 + rank * 0.085;
      const dirWorld = new THREE.Vector3(r > 1e-3 ? (dx / r) * spread : 0, lift, r > 1e-3 ? (dz / r) * spread : 0);
      // store in root-local space (groups are children of the rotated root)
      s.explodeDir.copy(dirWorld.applyQuaternion(this.root.quaternion.clone().invert()));
    });

    // Initial camera: front three-quarter view.
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
    // snap to the nearest axis so the robot sits square to the stage
    const a = Math.atan2(dx, dz);
    return -Math.round(a / (Math.PI / 2)) * (Math.PI / 2);
  }

  /** Builds picking acceleration structures a mesh at a time so the UI never stalls. */
  buildPickingInBackground() {
    const queue = [...this.pickables].sort((a, b) => a.geometry.attributes.position.count - b.geometry.attributes.position.count);
    const step = () => {
      const m = queue.shift();
      if (!m) return;
      m.geometry.computeBoundsTree();
      setTimeout(step, 30);
    };
    setTimeout(step, 50);
  }

  // ---------------------------------------------------------------- reveal
  startReveal() {
    if (this.reducedMotion) { revealFx.uReveal.value = 1e3; revealFx.uBand.value = 0; this.dirty = true; return; }
    this.reveal = { t: 0, height: this.robotHeight + 0.4 };
    revealFx.uReveal.value = -0.02;
    revealFx.uBand.value = 1;
  }
  get revealing() { return this.reveal !== null; }

  // ---------------------------------------------------------------- framing
  /**
   * Exact distance at which every corner of `box` fits inside the free area, looking from the
   * given angles at the box center (the focal offset then centers it in that area).
   */
  private fitBox(box: THREE.Box3, azimuths: number[], polar: number, insets: Insets, pad = 1.06) {
    const { w, h } = this.size;
    const tanV = Math.tan((FOV * DEG) / 2);
    const wf = Math.max(80, w - insets.left - insets.right);
    const hf = Math.max(80, h - insets.top - insets.bottom);
    const tx = tanV * (wf / h), ty = tanV * (hf / h); // half-extent tangents of the free area
    const c = box.getCenter(new THREE.Vector3());
    const corners: THREE.Vector3[] = [];
    for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) corners.push(new THREE.Vector3(x, y, z).sub(c));
    let d = 0;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler(), v = new THREE.Vector3();
    for (const az of azimuths) {
      // camera orientation: yaw by azimuth, then pitch down by (90° − polar)
      e.set(polar - Math.PI / 2, az, 0, 'YXZ');
      m.makeRotationFromQuaternion(q.setFromEuler(e)).invert();
      for (const p of corners) {
        v.copy(p).applyMatrix4(m); // view space: +z toward the camera
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

  private currentBox(group: THREE.Object3D) {
    group.updateMatrixWorld(true);
    return new THREE.Box3().setFromObject(group);
  }

  /** Box of the robot at the explode amount we are heading to (so framing doesn't chase the animation). */
  private robotBoxAtTarget(): THREE.Box3 {
    const box = new THREE.Box3();
    for (const s of this.subs.values()) {
      const b = this.currentBox(s.group);
      const delta = s.explodeDir.clone().multiplyScalar(this.explode.target - this.explode.value).applyQuaternion(this.root.quaternion);
      box.union(b.translate(delta));
    }
    return box;
  }

  private updateBoundary() {
    const box = this.robotBoxAtTarget().expandByScalar(0.04);
    box.min.y = Math.max(0, box.min.y);
    this.controls.setBoundary(box);
  }

  /** Fits the whole robot at every angle of a full turn, so the turntable never crops it. */
  private overviewDistance(box: THREE.Box3, insets: Insets) {
    const azs = Array.from({ length: 16 }, (_, i) => (i / 16) * Math.PI * 2);
    return this.fitBox(box, azs, HOME_POLAR, insets, 1.04);
  }

  frameOverview(insets: Insets, transition: boolean, resetAngle = false) {
    if (!this.subs.size) return;
    this.focused = null;
    this.setFocusVisuals(null);
    this.updateBoundary();
    const box = this.robotBoxAtTarget();
    const center = box.getCenter(new THREE.Vector3());
    const d = this.overviewDistance(box, insets);
    this.controls.minDistance = 0.22;
    this.withMotion(transition, (t) => {
      this.controls.moveTo(center.x, center.y, center.z, t);
      this.controls.dollyTo(d, t);
      this.applyFocalOffset(d, insets, t);
      if (resetAngle) this.controls.rotateTo(this.shortestAzimuth(HOME_AZIMUTH), HOME_POLAR, t);
      else this.controls.rotateTo(this.controls.azimuthAngle, THREE.MathUtils.clamp(this.controls.polarAngle, 48 * DEG, 72 * DEG), t);
    });
  }

  focus(id: string, insets: Insets, transition = true) {
    const s = this.subs.get(id);
    if (!s) return;
    this.focused = id;
    this.setFocusVisuals(id);
    this.updateBoundary();
    const cfg = subsystemById.get(id)?.camera ?? {};
    // Frame the box where the subsystem will be once any explode animation settles.
    const box = this.currentBox(s.group);
    box.translate(s.explodeDir.clone().multiplyScalar(this.explode.target - this.explode.value).applyQuaternion(this.root.quaternion));
    const center = box.getCenter(new THREE.Vector3());
    let az: number;
    if (cfg.azimuth !== undefined) az = cfg.azimuth * DEG; // 0 = straight at the front
    else if (s.outward !== null) az = s.outward;
    else az = this.controls.azimuthAngle;
    const polar = (90 - (cfg.elevation ?? 32)) * DEG;
    const d = Math.max(0.2, this.fitBox(box, [az], polar, insets, 1.22) / (cfg.zoom ?? 1));
    this.controls.minDistance = 0.12;
    this.withMotion(transition, (t) => {
      this.controls.moveTo(center.x, center.y, center.z, t);
      this.controls.rotateTo(this.shortestAzimuth(az), polar, t);
      this.controls.dollyTo(d, t);
      this.applyFocalOffset(d, insets, t);
    });
  }

  /** Re-frames after a resize without jumping. */
  reframe(insets: Insets) {
    if (this.focused) this.focus(this.focused, insets, true);
    else {
      const d = this.overviewDistance(this.robotBoxAtTarget(), insets);
      this.controls.dollyTo(d, true);
      this.applyFocalOffset(d, insets, true);
    }
  }

  private withMotion(transition: boolean, fn: (t: boolean) => void) {
    if (transition && this.reducedMotion) {
      // Reduced motion: a quick crossfade instead of a camera flight.
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

  private setFocusVisuals(id: string | null) {
    for (const s of this.subs.values()) {
      s.ghost.target = id && s.id !== id ? 1 : 0;
      for (const m of s.meshes) {
        if (s.id === id) m.layers.enable(OUTLINE_LAYER);
        else m.layers.disable(OUTLINE_LAYER);
      }
    }
    this.outlineStrength.target = id ? 1 : 0;
    if (this.reducedMotion) {
      for (const s of this.subs.values()) s.ghost.snap();
    }
    this.dirty = true;
  }

  // ---------------------------------------------------------------- explode
  setExploded(on: boolean, insets: Insets) {
    this.explode.target = on ? 1 : 0;
    if (this.reducedMotion) this.explode.snap();
    this.shadowDirty = true;
    if (this.focused) this.focus(this.focused, insets, true);
    else this.frameOverview(insets, true);
    this.dirty = true;
  }

  // ---------------------------------------------------------------- picking
  pick(clientX: number, clientY: number): string | null {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const ready = this.pickables.filter((m) => m.geometry.boundsTree);
    if (!ready.length) return null;
    const hits = this.raycaster.intersectObjects(ready, false);
    if (!hits.length) return null;
    // Prefer solid parts over ghosts / polycarb when both are under the finger.
    const solid = hits.find((h) => !((h.object as THREE.Mesh).material as THREE.Material).transparent);
    return ((solid ?? hits[0]).object.userData.subsystem as string) ?? null;
  }

  setHighlight(id: string | null) {
    for (const s of this.subs.values()) s.highlight.target = s.id === id ? 1 : 0;
    this.dirty = true;
  }

  // ---------------------------------------------------------------- pins
  pins(): PinInfo[] {
    const out: PinInfo[] = [];
    const camDir = this.camera.getWorldDirection(new THREE.Vector3());
    const center = this.controls.getTarget(new THREE.Vector3());
    const v = new THREE.Vector3();
    for (const s of this.subs.values()) {
      s.group.localToWorld(v.copy(s.anchor));
      const far = v.clone().sub(center).dot(camDir) > this.robotRadius * 0.12;
      v.project(this.camera);
      const visible = v.z < 1 && Math.abs(v.x) < 1.05 && Math.abs(v.y) < 1.05;
      out.push({ id: s.id, x: (v.x * 0.5 + 0.5) * this.size.w, y: (-v.y * 0.5 + 0.5) * this.size.h, far, visible });
    }
    return out;
  }

  get ids() { return [...this.subs.keys()]; }
  get pickingReady() { return this.pickables.length > 0 && this.pickables.every((m) => m.geometry.boundsTree); }

  // ---------------------------------------------------------------- size
  resize(w: number, h: number) {
    this.size = { w, h };
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    const pr = this.renderer.getPixelRatio();
    this.outline.setSize(w * pr, h * pr);
    this.dirty = true;
  }

  invalidate() { this.dirty = true; }

  // ---------------------------------------------------------------- loop
  private tick = (now: number) => {
    requestAnimationFrame(this.tick);
    const dt = Math.min(0.05, Math.max(0.001, (now - this.last) / 1000));
    this.last = now;
    if (!this.running) return;
    if (this.advance(dt) || this.dirty) this.render();
  };

  /** Steps every animation by dt seconds; returns true if anything moved. */
  private advance(dt: number): boolean {
    let active = false;
    this.spinFactor.target = this.spinning && !this.reducedMotion && !this.focused ? 1 : 0;
    if (!this.spinning) this.spinFactor.snap(0); // a touch stops the turntable instantly
    active = this.spinFactor.update(dt) || active;
    if (this.spinFactor.value > 0.0005) {
      this.controls.rotate(((Math.PI * 2) / this.spinPeriod) * dt * this.spinFactor.value, 0, true);
      active = true;
    }
    active = this.controls.update(dt) || active;

    if (this.reveal) {
      this.reveal.t += dt / 1.5;
      const t = Math.min(1, this.reveal.t);
      const e = t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
      revealFx.uReveal.value = -0.02 + e * this.reveal.height;
      revealFx.uBand.value = t < 0.85 ? 1 : 1 - (t - 0.85) / 0.15;
      if (t >= 1) { this.reveal = null; revealFx.uReveal.value = 1e3; revealFx.uBand.value = 0; }
      active = true;
    }

    if (this.explode.update(dt)) {
      for (const s of this.subs.values()) s.group.position.copy(s.explodeDir).multiplyScalar(this.explode.value);
      this.shadowDirty = true;
      active = true;
    }
    for (const s of this.subs.values()) {
      if (s.ghost.update(dt)) {
        s.fx.uGhost.value = s.ghost.value;
        for (const m of s.materials) applyGhostState(m, s.ghost.value);
        for (const m of s.meshes) m.renderOrder = s.ghost.value > 0.001 ? 2 : (m.material as THREE.Material).userData.baseTransparent ? 1 : 0;
        active = true;
      }
      if (s.highlight.update(dt)) { s.fx.uHighlight.value = s.highlight.value; active = true; }
    }
    if (this.outlineStrength.update(dt)) { this.outline.strength.value = this.outlineStrength.value; active = true; }
    return active;
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
      this.root.updateMatrixWorld(true);
      this.shadow.update(r, this.scene, this.root);
      this.shadowDirty = false;
    }
    r.info.reset();
    r.render(this.scene, this.camera);
    this.stats.calls = r.info.render.calls;
    this.stats.triangles = r.info.render.triangles;
    this.outline.render(r, this.scene, this.camera);
    this.stats.frames++;
    const now = performance.now();
    if (now - this.stats.since > 500) {
      this.stats.fps = (this.stats.frames * 1000) / (now - this.stats.since);
      this.stats.frames = 0;
      this.stats.since = now;
    }
    this.onRender?.();
  }

  /** Model statistics for the debug view and the build report. */
  modelStats() {
    let tris = 0;
    for (const m of this.pickables) tris += (m.geometry.index?.count ?? m.geometry.attributes.position.count) / 3;
    return { meshes: this.pickables.length, triangles: Math.round(tris), source: this.source, root: this.robotRoot?.name ?? '' };
  }
}

async function fetchWithProgress(url: string, onProgress: (f: number) => void): Promise<ArrayBuffer> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${url}`);
  const total = Number(res.headers.get('content-length')) || 12e6;
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
  // A missing file on a static host can come back as an HTML page.
  if (out[0] !== 0x67 || out[1] !== 0x6c || out[2] !== 0x54 || out[3] !== 0x46) throw new Error('Not a GLB file');
  return out.buffer;
}

function nearestVertex(meshes: THREE.Mesh[], target: THREE.Vector3): THREE.Vector3 | null {
  let best: THREE.Vector3 | null = null, bestD = Infinity;
  const v = new THREE.Vector3();
  for (const m of meshes) {
    const pos = m.geometry.attributes.position;
    m.updateMatrixWorld(true);
    const stride = Math.max(1, Math.floor(pos.count / 40000));
    for (let i = 0; i < pos.count; i += stride) {
      v.fromBufferAttribute(pos, i).applyMatrix4(m.matrixWorld);
      // favor points near the top-center, but weigh height less so pins sit on real geometry
      const d = (v.x - target.x) ** 2 + (v.z - target.z) ** 2 + 0.35 * (v.y - target.y) ** 2;
      if (d < bestD) { bestD = d; best = v.clone(); }
    }
  }
  return best;
}
