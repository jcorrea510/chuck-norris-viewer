// Simulation mode: Chuck Norris on the 2026 REBUILT field. The physics and the game live in
// world.ts (Rapier, fixed 240 Hz step, no rendering); this file draws them, interpolating every
// body between the last two physics steps, points the three cameras, and turns the driver's
// controls into field-relative commands.
// Field: AdvantageScope "2026 FRC Field" (BSD-3). Internally our alliance is always at −X; playing
// blue turns the whole scene 180° and swaps the alliance colors (the field is rotationally symmetric).
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import type { Viewer } from '../viewer';
import { initRapier, type CollisionIndex } from './phys/engine';
import { BS, SQUASH_TIME } from './phys/fuel';
import { FUEL, STEP } from './phys/config';
import { HUB } from './phys/hub';
import { SimWorld, NO_INPUT, type FieldData, type SimMode, type SimStatus, type DriverInput, type HsPhase } from './world';
import { WorkerPlanner } from './planner';
import { DriverControls, type RawInput } from './input';
import { Drivepad } from './drivepad';
import { SimAudio } from './audio';
import { DebugOverlay } from './debug';

export type { SimMode, SimStatus } from './world';
export type SimView = 'driver' | 'follow' | 'overhead';
export type Alliance = 'red' | 'blue';
export const SIM_VIEWS: { id: SimView; label: string; title: string }[] = [
  { id: 'driver', label: 'Driver', title: 'From our driver station' },
  { id: 'follow', label: 'Follow', title: 'Behind the robot, facing the opponent' },
  { id: 'overhead', label: 'Overhead', title: 'The whole field from above' },
];

const DEG = Math.PI / 180;

/** Critically damped spring on a vector (smooth-damp): never overshoots, never snaps. */
class Spring3 {
  readonly v = new THREE.Vector3();
  private vel = new THREE.Vector3();
  private init = false;
  update(target: THREE.Vector3, smooth: number, dt: number) {
    if (!this.init || dt <= 0) { this.v.copy(target); this.vel.set(0, 0, 0); this.init = true; return this.v; }
    const omega = 2 / smooth, x = omega * dt;
    const exp = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);
    const ch = this.v.clone().sub(target);
    const temp = this.vel.clone().addScaledVector(ch, omega).multiplyScalar(dt);
    this.vel.addScaledVector(temp, -omega).multiplyScalar(exp);
    this.v.copy(target).add(ch.add(temp).multiplyScalar(exp));
    return this.v;
  }
  snap(t: THREE.Vector3) { this.v.copy(t); this.vel.set(0, 0, 0); this.init = true; }
}
class Spring1 {
  v = 0; private vel = 0; private init = false;
  update(target: number, smooth: number, dt: number) {
    if (!this.init || dt <= 0) { this.v = target; this.vel = 0; this.init = true; return this.v; }
    const omega = 2 / smooth, x = omega * dt;
    const exp = 1 / (1 + x + 0.48 * x * x + 0.235 * x * x * x);
    const ch = this.v - target, temp = (this.vel + omega * ch) * dt;
    this.vel = (this.vel - omega * temp) * exp;
    this.v = target + (ch + temp) * exp;
    return this.v;
  }
}

export class Simulation {
  private group = new THREE.Group();
  world!: SimWorld;
  readonly controls = new DriverControls();
  private pad: Drivepad | null = null;
  private inst!: THREE.InstancedMesh;
  private shadows!: THREE.InstancedMesh;
  private nets: { mesh: THREE.Mesh; base: Float32Array; hub: number }[] = [];
  private fieldMats: { mat: THREE.MeshStandardMaterial; color: THREE.Color }[] = [];
  private acc = 0;
  private hook = (dt: number) => this.frame(dt);
  private saved: Record<string, unknown> | null = null;
  private lod: Map<string, THREE.BufferGeometry> | null = null;
  private statusClock = 0;
  private moduleMech: { id: string; h0: number }[] = [];
  private camPos = new Spring3();
  private camTgt = new Spring3();
  private camFov = new Spring1();
  private flip = new THREE.Matrix4();
  private audio = new SimAudio();
  private debug: DebugOverlay | null = null;
  private lastRaw: RawInput | null = null;
  private frameMs = 0;
  view: SimView = 'driver';
  alliance: Alliance = 'red';
  autoOrbit = false;
  active = false;
  paused = false;
  touchControls = matchMedia('(pointer: coarse)').matches;
  onStatus: ((s: SimStatus) => void) | null = null;
  onActivity: (() => void) | null = null;
  onRespawn: (() => void) | null = null;
  onPad: ((name: string | null) => void) | null = null;
  onHsPhase: ((p: HsPhase) => void) | null = null;

  constructor(private viewer: Viewer) {}

  get mode(): SimMode { return this.world?.mode ?? 'free'; }
  get robot() { return this.world.robot; }
  get fuel() { return this.world.fuel; }
  get grid() { return this.world.field; }

  async start(mode: SimMode = 'free', alliance: Alliance = 'red', view: SimView = 'driver') {
    const v = this.viewer;
    this.alliance = alliance;
    this.view = view;
    this.emitLoading(mode);
    const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const [gltf, data, lodGltf, bin, index] = await Promise.all([
      loader.loadAsync('field/field-2026.glb'),
      fetch('field/field-2026.json').then((r) => r.json() as Promise<FieldData>),
      loader.loadAsync('models/chuck-norris-sim.glb').catch(() => null),
      fetch('field/field-collision.bin').then((r) => r.arrayBuffer()),
      fetch('field/field-collision.json').then((r) => r.json() as Promise<CollisionIndex>),
      initRapier(),
    ]);
    this.lod = new Map();
    lodGltf?.scene.traverse((o) => { const m = o as THREE.Mesh; if (m.isMesh) this.lod!.set(m.name, m.geometry); });
    this.prepareField(gltf.scene);
    this.group.add(gltf.scene);

    const debug = new URLSearchParams(location.search).get('debug') === 'physics';
    this.world = new SimWorld(data, { bin, index }, (Date.now() & 0xffff) || 1, new WorkerPlanner(data.grid, data.hubs));
    const w = this.world;
    w.bestStore = {
      get: () => { try { return Number(localStorage.getItem('chuck-norris-highscore') ?? 0) || 0; } catch { return 0; } },
      set: (n) => { try { localStorage.setItem('chuck-norris-highscore', String(n)); } catch { /* private mode */ } },
    };
    w.onRespawn = () => this.onRespawn?.();
    w.onHsPhase = (p) => this.onHsPhase?.(p);
    w.fuel.onImpact = (i, sp, x, y, z) => this.audio.impact(sp, this.panOf(x, z), this.distTo(x, y, z), y < FUEL.radius + 0.02 ? 'carpet' : 'hard');
    this.buildFuel();
    this.buildNets();
    if (debug) this.debug = new DebugOverlay(this.group, w);

    // take over the stage
    const kl = v.keyLight, sc = kl.shadow.camera;
    this.saved = {
      maxDist: v.controls.maxDistance, minDist: v.controls.minDistance, maxPolar: v.controls.maxPolarAngle,
      keyPos: kl.position.clone(), shadow: [sc.left, sc.right, sc.top, sc.bottom, sc.far], rootPos: v.root.position.clone(),
      dollyToCursor: v.controls.dollyToCursor, smoothTime: v.controls.smoothTime, enabled: v.controls.enabled,
    };
    v.setSimRendering(this.lod, 50);
    v.setFocus(null);
    v.setExploded(false);
    for (const m of v.mechanisms.values()) v.setMechanism(m.id, m.def, false);
    v.simActive = true;
    v.spinning = false;
    // the robot drives about its drivetrain center (the swerve modules)
    const swerve = v.boxOfParts(v.subsystemPids('swerve'), false).getCenter(new THREE.Vector3());
    v.root.position.set(-swerve.x, 0, -swerve.z);
    v.scene.add(this.group);
    v.grid.visible = false;
    sc.left = sc.bottom = -4.5; sc.right = sc.top = 4.5; sc.far = 16;
    sc.updateProjectionMatrix();
    // the sim's cameras are fixed rigs: no orbiting by hand (touches are for driving)
    v.controls.enabled = false;
    v.controls.smoothTime = 0;
    v.controls.maxDistance = 60;
    v.controls.minDistance = 0.3;
    v.controls.maxPolarAngle = Math.PI;
    v.controls.setBoundary(new THREE.Box3(new THREE.Vector3(-30, -1, -30), new THREE.Vector3(30, 40, 30)));
    v.controls.setFocalOffset(0, 0, 0, false);
    v.stage.position.set(0, 0, 0); v.stage.quaternion.identity();
    v.stage.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(v.stage.matrixWorld).invert();
    this.moduleMech = w.robot.modules.map((m) => {
      let best = '', bd = Infinity, h0 = 0;
      for (const [id, mech] of v.mechanisms) {
        if (!id.startsWith('module-')) continue;
        const p = mech.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(inv);
        const d = Math.hypot(p.x - m.px, p.z - m.pz);
        if (d < bd) { bd = d; best = id; const r = Math.hypot(p.x, p.z) || 1; h0 = Math.atan2(-p.z / r, p.x / r); }
      }
      return { id: best, h0 };
    });
    this.controls.attach();
    this.controls.onPad = (name) => this.onPad?.(name);
    this.pad = new Drivepad(document.getElementById('ui')!, this.controls.touch);
    v.hooks.add(this.hook);
    this.active = true;
    this.setAlliance(alliance);
    this.setMode(mode);
  }

  /** Field materials and meshes: transparency that never flickers, alliance colors we can swap. */
  private prepareField(scene: THREE.Object3D) {
    let order = 0;
    const meshes: THREE.Mesh[] = [];
    scene.traverse((o) => { if ((o as THREE.Mesh).isMesh) meshes.push(o as THREE.Mesh); });
    meshes.forEach((m) => {
      m.raycast = () => {};
      const mat = m.material as THREE.MeshStandardMaterial;
      m.receiveShadow = !mat.transparent;
      m.castShadow = !mat.transparent;
      mat.envMapIntensity = 0.7;
      const c = mat.color;
      // the HUB net is drawn by the simulation (it deforms when fuel hits it)
      if (mat.transparent && c.r < 0.02 && c.g < 0.02 && c.b < 0.02 && Math.abs(mat.opacity - 0.25) < 0.02) { m.visible = false; return; }
      if (mat.transparent) {
        // Polycarbonate: several translucent layers in the HUB and guardrails are coplanar. Sorted per
        // frame by distance, their order flipped as the camera moved (black ↔ gray flicker). Fixed
        // order, no depth writes, back faces first: stable from every angle.
        mat.depthWrite = false;
        mat.side = THREE.FrontSide;
        m.renderOrder = 10 + order++;
        const back = m.clone();
        back.material = mat.clone();
        (back.material as THREE.Material).side = THREE.BackSide;
        back.renderOrder = m.renderOrder - 5;
        m.parent?.add(back);
      }
      // alliance-colored parts (BUMPs, TOWER uprights, HUB light bars)
      const red = c.r > 0.9 && c.g < 0.1 && c.b < 0.1, blue = c.b > 0.9 && c.r < 0.1 && c.g < 0.1;
      if (red || blue) this.fieldMats.push({ mat, color: c.clone() });
    });
  }

  /** All FUEL: one instanced draw call (foam with a subtle seam so spin shows), plus blob shadows. */
  private buildFuel() {
    const n = this.world.fuel.n;
    const cv = document.createElement('canvas');
    cv.width = 256; cv.height = 128;
    const g = cv.getContext('2d')!;
    g.fillStyle = '#f2b51c'; g.fillRect(0, 0, 256, 128);
    // a little foam grain
    for (let k = 0; k < 900; k++) { g.fillStyle = `rgba(${150 + Math.random() * 60},${90 + Math.random() * 40},0,0.06)`; g.fillRect(Math.random() * 256, Math.random() * 128, 2, 2); }
    g.fillStyle = 'rgba(120,70,0,0.55)'; g.fillRect(0, 62, 256, 3);
    const tex = new THREE.CanvasTexture(cv);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 4;
    const geo = new THREE.SphereGeometry(FUEL.radius, 20, 14);
    const mat = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.78, metalness: 0 });
    this.inst = new THREE.InstancedMesh(geo, mat, n);
    this.inst.castShadow = false;
    this.inst.receiveShadow = true;
    this.inst.frustumCulled = false;
    this.inst.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.group.add(this.inst);
    // contact shadows: a soft dark disc under each ball, fainter as it rises
    const sc = document.createElement('canvas');
    sc.width = sc.height = 64;
    const sg = sc.getContext('2d')!;
    const grad = sg.createRadialGradient(32, 32, 2, 32, 32, 31);
    grad.addColorStop(0, 'rgba(0,0,0,1)'); grad.addColorStop(0.5, 'rgba(0,0,0,0.5)'); grad.addColorStop(1, 'rgba(0,0,0,0)');
    sg.fillStyle = grad; sg.fillRect(0, 0, 64, 64);
    const sh = new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(sc), transparent: true, depthWrite: false, opacity: 0.55, color: 0x000000, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
    this.shadows = new THREE.InstancedMesh(new THREE.PlaneGeometry(0.26, 0.26).rotateX(-Math.PI / 2), sh, n);
    this.shadows.frustumCulled = false;
    this.shadows.renderOrder = 1;
    this.shadows.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.group.add(this.shadows);
  }

  /** The HUB nets: a mesh of cord that bulges where fuel hits it and springs back. */
  private buildNets() {
    const cv = document.createElement('canvas');
    cv.width = cv.height = 128;
    const g = cv.getContext('2d')!;
    g.strokeStyle = 'rgba(20,20,22,0.95)'; g.lineWidth = 3;
    for (let k = 0; k <= 128; k += 16) { g.beginPath(); g.moveTo(k, 0); g.lineTo(k, 128); g.stroke(); g.beginPath(); g.moveTo(0, k); g.lineTo(128, k); g.stroke(); }
    const tex = new THREE.CanvasTexture(cv);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    const W = HUB.net.hz * 2, H = HUB.net.y1 - HUB.net.y0;
    tex.repeat.set(W / 0.1, H / 0.1);
    const mat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, side: THREE.DoubleSide, depthWrite: false, alphaTest: 0.02 });
    for (let h = 0; h < 2; h++) {
      const geo = new THREE.PlaneGeometry(W, H, 28, 26).rotateY(Math.PI / 2);
      const mesh = new THREE.Mesh(geo, mat);
      const s = h === 0 ? 1 : -1;
      mesh.position.set((h === 0 ? -HUB.cx : HUB.cx) + s * HUB.net.off, (HUB.net.y0 + HUB.net.y1) / 2, 0);
      mesh.renderOrder = 40;
      mesh.frustumCulled = false;
      this.group.add(mesh);
      this.nets.push({ mesh, base: (geo.getAttribute('position').array as Float32Array).slice(), hub: h });
    }
  }

  setMode(mode: SimMode) {
    this.world.setMode(mode);
    this.acc = 0;
    this.writeFuel(1);
    this.present(1);
    this.aimCamera(0);
    this.emit();
  }
  restart() { this.setMode(this.mode); }

  setAlliance(a: Alliance) {
    this.alliance = a;
    const blue = a === 'blue';
    this.group.rotation.y = blue ? Math.PI : 0;
    this.group.updateMatrixWorld(true);
    this.flip.makeRotationY(blue ? Math.PI : 0);
    // our end is always internal −x: playing blue, that end wears blue
    for (const f of this.fieldMats) {
      const red = f.color.r > 0.9;
      f.mat.color.copy(blue ? (red ? new THREE.Color(0, 0, 1) : new THREE.Color(1, 0, 0)) : f.color);
    }
    this.viewer.setBumperColor(a);
    if (this.world) { this.present(1); this.aimCamera(0); }
  }

  stop() {
    const v = this.viewer;
    if (!this.saved) return;
    v.hooks.delete(this.hook);
    this.controls.detach();
    this.pad?.dispose(); this.pad = null;
    this.debug?.dispose(); this.debug = null;
    this.audio.stop();
    v.setSimRendering(null, null);
    v.scene.remove(this.group);
    v.grid.visible = true;
    v.simActive = false;
    const s = this.saved as { maxDist: number; minDist: number; maxPolar: number; keyPos: THREE.Vector3; shadow: number[]; rootPos: THREE.Vector3; dollyToCursor: boolean; smoothTime: number; enabled: boolean };
    v.root.position.copy(s.rootPos);
    v.stage.position.set(0, 0, 0);
    v.stage.quaternion.identity();
    v.stage.updateMatrixWorld(true);
    v.keyLight.position.copy(s.keyPos);
    v.keyLight.target.position.set(0, 0.25, 0);
    v.keyLight.target.updateMatrixWorld();
    const sc = v.keyLight.shadow.camera;
    [sc.left, sc.right, sc.top, sc.bottom, sc.far] = s.shadow;
    sc.updateProjectionMatrix();
    v.controls.enabled = s.enabled;
    v.controls.maxDistance = s.maxDist;
    v.controls.minDistance = s.minDist;
    v.controls.maxPolarAngle = s.maxPolar;
    v.controls.dollyToCursor = s.dollyToCursor;
    v.controls.smoothTime = s.smoothTime;
    v.setBumperColor(null);
    for (const m of v.mechanisms.values()) {
      m.angle.smoothTime = m.type === 'fourbar' ? 0.42 : 0.32;
      v.setMechanism(m.id, m.def, true);
    }
    this.world.dispose();
    this.active = false;
    this.saved = null;
    v.invalidate(true);
  }

  setView(view: SimView) { this.view = view; }
  nextView() { const i = SIM_VIEWS.findIndex((x) => x.id === this.view); this.setView(SIM_VIEWS[(i + 1) % SIM_VIEWS.length].id); }
  handBack() { this.world.resumeAI(); }
  setTouchControls(on: boolean) { this.touchControls = on; }
  setSound(on: boolean, volume: number) { this.audio.setEnabled(on, volume); }

  // ------------------------------------------------------------------ driver input → field frame
  /** Field-relative, always: up on the stick drives toward the opponent's wall (+x internally),
   *  which is up on screen in all three cameras, whatever the alliance. */
  private driverInput(raw: RawInput): DriverInput {
    return {
      mx: raw.my, mz: raw.mx, turn: raw.turn,
      shoot: raw.shoot, intake: false, eject: raw.eject, lock: raw.lock, slow: raw.slow,
      toggleArm: raw.arm, toggleIntake: raw.intakeToggle, handBack: raw.handBack,
    };
  }

  // ------------------------------------------------------------------ per frame
  frame(dt: number): boolean {
    if (!this.active || this.paused) return true;
    const t0 = performance.now();
    const w = this.world, r = w.robot;
    this.pad?.setVisible(this.touchControls);
    this.pad?.setIntake(r.rollers === 'in');
    const raw = this.controls.read();
    this.lastRaw = raw;
    if (raw.active || raw.handBack || raw.camera) this.onActivity?.();
    if (raw.camera) this.nextView();
    const input = raw.active || raw.handBack || w.control === 'driver' || w.mode === 'highscore' ? this.driverInput(raw) : NO_INPUT;
    // fixed 240 Hz steps; the render interpolates between the last two
    this.acc += Math.min(dt, 0.1);
    let steps = 0, phys = 0;
    while (this.acc >= STEP && steps < 16) {
      this.acc -= STEP;
      w.step(steps === 0 ? input : { ...input, toggleArm: false, toggleIntake: false, handBack: false });
      phys += w.stepMs;
      steps++;
    }
    if (steps === 16) this.acc = 0;
    const alpha = this.acc / STEP;
    this.present(alpha);
    this.writeFuel(alpha);
    this.updateNets();
    this.aimCamera(dt);
    // key light follows the robot so its shadows stay crisp
    const kl = this.viewer.keyLight;
    const rp = new THREE.Vector3(r.x, 0, r.z).applyMatrix4(this.flip);
    kl.position.set(rp.x + 2.4, 6.5, rp.z + 3.2);
    kl.target.position.copy(rp);
    kl.target.updateMatrixWorld();
    this.audio.update(w, this.panOf(r.x, r.z), this.distTo(r.x, 0.3, r.z));
    this.statusClock += dt;
    if (this.statusClock > 0.1) { this.statusClock = 0; this.emit(); }
    this.debug?.update(this.frameMs, phys, steps, alpha);
    this.frameMs = performance.now() - t0;
    return true;
  }

  /** The robot: chassis pose (interpolated), mechanisms, swerve modules. */
  private present(alpha: number) {
    const v = this.viewer, r = this.world.robot;
    const st = v.stage;
    const p = new THREE.Vector3().fromArray(r.prevP).lerp(new THREE.Vector3().fromArray(r.p), alpha);
    const q = new THREE.Quaternion(r.prevQ.x, r.prevQ.y, r.prevQ.z, r.prevQ.w).slerp(new THREE.Quaternion(r.q.x, r.q.y, r.q.z, r.q.w), alpha);
    if (this.alliance === 'blue') { p.applyMatrix4(this.flip); q.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI)); }
    st.position.copy(p);
    st.quaternion.copy(q);
    st.updateMatrix();
    st.matrixWorld.copy(st.matrix);
    v.setMechanismNow('intake', r.intakeAngle);
    for (const t of r.turrets) v.setMechanismNow(t.id, t.a / DEG);
    this.moduleMech.forEach((m, i) => {
      let a = r.modules[i].steer - m.h0;
      a = Math.atan2(Math.sin(a), Math.cos(a));
      v.setMechanismNow(m.id, a / DEG);
    });
    v.syncPose();
  }

  private writeFuel(alpha: number) {
    const f = this.world.fuel;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), q1 = new THREE.Quaternion(), p = new THREE.Vector3(), s = new THREE.Vector3(1, 1, 1);
    const hide = new THREE.Matrix4().makeScale(0, 0, 0);
    const sq = new THREE.Matrix4(), rn = new THREE.Matrix4(), n = new THREE.Vector3(), up = new THREE.Vector3(0, 1, 0), qa = new THREE.Quaternion();
    const shm = new THREE.Matrix4();
    for (let i = 0; i < f.n; i++) {
      const st = f.st[i], o = i * 3, r4 = i * 4;
      if (st === BS.HUB) { this.inst.setMatrixAt(i, hide); this.shadows.setMatrixAt(i, hide); continue; }
      p.set(f.prev[o] + (f.pos[o] - f.prev[o]) * alpha, f.prev[o + 1] + (f.pos[o + 1] - f.prev[o + 1]) * alpha, f.prev[o + 2] + (f.pos[o + 2] - f.prev[o + 2]) * alpha);
      q.set(f.prevRot[r4], f.prevRot[r4 + 1], f.prevRot[r4 + 2], f.prevRot[r4 + 3]);
      q1.set(f.rot[r4], f.rot[r4 + 1], f.rot[r4 + 2], f.rot[r4 + 3]);
      q.slerp(q1, alpha);
      m.compose(p, q, s);
      // foam squash on hard hits: flattened along the impact for the real contact time
      const t = f.squashT[i];
      if (t > 0) {
        const k = Math.sin(Math.min(1, t / (SQUASH_TIME * 1.6)) * Math.PI) * f.squashA[i] * 0.22;
        n.set(f.squashN[o], f.squashN[o + 1], f.squashN[o + 2]);
        qa.setFromUnitVectors(up, n.lengthSq() > 0.5 ? n : up);
        rn.makeRotationFromQuaternion(qa);
        sq.makeScale(1 + k * 0.5, 1 - k, 1 + k * 0.5);
        const pre = new THREE.Matrix4().makeTranslation(p.x, p.y, p.z).multiply(rn).multiply(sq).multiply(rn.clone().transpose()).multiply(new THREE.Matrix4().makeRotationFromQuaternion(q));
        m.copy(pre);
      }
      this.inst.setMatrixAt(i, m);
      // contact shadow on the floor below (none for fuel inside the robot or off the field)
      const fh = this.world.field.floor(p.x, p.z);
      const hgt = p.y - FUEL.radius - fh;
      if (st === BS.PARKED || st === BS.INDEXER || hgt > 1.5 || Math.abs(p.x) > 8.3 || Math.abs(p.z) > 4.05 || this.onRobot(p)) { this.shadows.setMatrixAt(i, hide); continue; }
      const sc = Math.max(0.35, 1 - hgt * 0.5);
      shm.makeScale(sc, 1, sc).setPosition(p.x, fh + 0.003, p.z);
      this.shadows.setMatrixAt(i, shm);
    }
    this.inst.instanceMatrix.needsUpdate = true;
    this.shadows.instanceMatrix.needsUpdate = true;
  }

  private onRobot(p: THREE.Vector3) {
    const r = this.world.robot;
    const dx = p.x - r.x, dz = p.z - r.z;
    if (dx * dx + dz * dz > 0.6) return false;
    const c = Math.cos(r.th), s = Math.sin(r.th);
    const lx = dx * c - dz * s, lz = dx * s + dz * c;
    return Math.abs(lx) < 0.45 && lz > -0.44 && lz < 0.7 && p.y > 0.17;
  }

  private updateNets() {
    const hits = this.world.hubs.netHits;
    for (const n of this.nets) {
      const pos = n.mesh.geometry.getAttribute('position') as THREE.BufferAttribute;
      const a = pos.array as Float32Array;
      let any = false;
      for (let k = 0; k < a.length; k += 3) {
        const y = a[k + 1] + (HUB.net.y0 + HUB.net.y1) / 2, z = n.base[k + 2];
        let d = 0;
        for (const h of hits) {
          if (h.hub !== n.hub) continue;
          const r2 = (y - h.y) ** 2 + (z - h.z) ** 2;
          const env = Math.exp(-h.t * 7) * Math.cos(Math.min(h.t * 18, Math.PI / 2) - 0.0);
          d += h.dir * h.depth * Math.exp(-r2 / 0.03) * env;
        }
        const nx = n.base[k] + Math.max(-0.3, Math.min(0.3, d));
        if (a[k] !== nx) { a[k] = nx; any = true; }
      }
      if (any) pos.needsUpdate = true;
    }
  }

  // ------------------------------------------------------------------ cameras
  /** Height a camera must keep above the field around (x, z) so it never goes through an element. */
  private clearHeight(x: number, z: number) {
    let h = 0.6;
    for (const cx of [-HUB.cx, HUB.cx]) if (Math.abs(x - cx) < 1.6 && Math.abs(z) < 1.2) h = Math.max(h, 3.5);
    if (Math.abs(Math.abs(x) - 3.62) < 0.9 && Math.abs(z) > 2.4) h = Math.max(h, 1.4);   // TRENCHes
    return h;
  }

  private aimCamera(dt: number) {
    const v = this.viewer, r = this.world.robot, cam = v.camera;
    const aspect = cam.aspect || 1.4;
    const pos = new THREE.Vector3(), tgt = new THREE.Vector3();
    let fov = 50;
    const rx = r.x, rz = r.z;
    if (this.view === 'driver') {
      // our drive team, standing behind the alliance wall: fixed yaw down the field; pans and zooms
      // gently to keep the robot in frame
      const far = THREE.MathUtils.clamp((rx + 8.3) / 16.5, 0, 1);
      // driver station 2 (manual: the TOWER sits between stations 2 and 3, so 2 sees past it)
      pos.set(-8.25, 1.8, 1.25 + THREE.MathUtils.clamp((rz - 1.25) * 0.15, -0.35, 0.35));
      tgt.set(pos.x + 8, THREE.MathUtils.lerp(-0.6, 0.35, far), THREE.MathUtils.clamp(rz * 0.55, -2.6, 2.6));
      // aim between the robot and the middle of the field
      fov = THREE.MathUtils.lerp(58, 36, far) * (aspect < 1 ? 1.35 : 1);
    } else if (this.view === 'follow') {
      // behind and above, yaw locked to the field (looking at the opponent's wall)
      const back = aspect < 1 ? 4.2 : 3.4, up = aspect < 1 ? 3.2 : 2.5;
      // never behind our alliance wall: near it, the camera climbs instead of backing through it
      const cx = Math.max(rx - back, -8.15), cz = rz;
      const extra = (cx - (rx - back)) * 0.8;
      pos.set(cx, Math.max(up + extra, this.clearHeight(cx, cz)), cz);
      tgt.set(rx + 1.6, 0.15, rz);
      fov = aspect < 1 ? 62 : 50;
    } else {
      // the whole field from above, our alliance at the bottom
      const halfLen = 8.6, halfW = 4.4;
      fov = 40;
      const vt = Math.tan((fov * DEG) / 2);
      const hNeed = Math.max(halfLen / vt, halfW / (vt * aspect)) * 1.06;
      pos.set(-hNeed * 0.12, hNeed, 0);
      tgt.set(0.35, 0, 0);
    }
    const smooth = 0.35;
    if (dt <= 0) { this.camPos.snap(pos); this.camTgt.snap(tgt); }
    const p = this.camPos.update(pos, smooth, dt).clone().applyMatrix4(this.flip);
    const t = this.camTgt.update(tgt, smooth * 0.8, dt).clone().applyMatrix4(this.flip);
    const f = this.camFov.update(fov, 0.4, dt);
    if (Math.abs(cam.fov - f) > 0.01) v.setFov(f);
    v.controls.setLookAt(p.x, p.y, p.z, t.x, t.y, t.z, false);
  }

  // ------------------------------------------------------------------ sound helpers
  private panOf(x: number, z: number) {
    const p = new THREE.Vector3(x, 0.3, z).applyMatrix4(this.flip).project(this.viewer.camera);
    return THREE.MathUtils.clamp(p.x, -1, 1) * 0.7;
  }
  private distTo(x: number, y: number, z: number) {
    return this.viewer.camera.position.distanceTo(new THREE.Vector3(x, y, z).applyMatrix4(this.flip));
  }

  private emitLoading(mode: SimMode) {
    this.onStatus?.({ mode, scored: 0, rate: 0, shooting: false, ferry: false, outOfRange: false, control: 'ai', resumeIn: null, intakeDown: true, rollers: false, stored: 0, capacity: 0, hs: null, loading: true });
  }
  private emit() { this.onStatus?.(this.status()); }
  status(): SimStatus { return this.world.status(); }
  get inputSource() { return this.lastRaw?.source ?? null; }
}
