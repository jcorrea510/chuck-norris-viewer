// Simulation mode: Chuck Norris on the 2026 REBUILT field. The physics and the robot's brains live
// in world.ts (and run without any of this); this file draws them, points the cameras, reads the
// driver's controls and plays the robot's sounds.
// Field: AdvantageScope "2026 FRC Field" (scripts/optimize-field.mjs). Red alliance wall at −X.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import type { Viewer } from '../viewer';
import { sound } from '../sound';
import { S, R } from './balls';
import { SimWorld, STEP, NO_INPUT, type FieldData, type SimMode, type SimStatus, type DriverInput } from './world';
import { EndlessBrain } from './brain';
import { DriverControls, type RawInput } from './input';
import { Drivepad } from './drivepad';
import { DEG } from './geometry';

export type { SimMode, SimStatus } from './world';
/** tv: broadcast camera that keeps the robot and where it is going in frame; follow: chase camera
 *  behind the robot; field: the whole field from above; driver: the red driver station. */
export type SimView = 'tv' | 'follow' | 'field' | 'driver';
export const SIM_VIEWS: { id: SimView; label: string; title: string }[] = [
  { id: 'tv', label: 'TV', title: 'Broadcast camera: the robot and where it is going' },
  { id: 'follow', label: 'Chase', title: 'Behind the robot' },
  { id: 'field', label: 'Field', title: 'The whole field' },
  { id: 'driver', label: 'Driver', title: 'From the driver station' },
];

const FOV_SIM = 46;
const USER_HOLD = 4;               // seconds the camera stays where the user put it

export class Simulation {
  private group = new THREE.Group();
  world!: SimWorld;
  readonly controls = new DriverControls();
  private pad: Drivepad | null = null;
  private inst!: THREE.InstancedMesh;
  private trails!: THREE.LineSegments;
  private trailPos = new Map<number, number[]>();
  private pulse!: THREE.Mesh;
  private locator!: THREE.Mesh;
  private userT = 99;
  private onUserStart = () => { this.userT = 0; this.userActive = true; };
  private onUserEnd = () => { this.userActive = false; };
  private userActive = false;
  private camHeading = 0;
  private tvSide = 1;
  private camFocus = new THREE.Vector3();
  // the robot's "brain", visualized: planned path, the fuel it is hunting, turret aim lines
  private pathLine!: THREE.Line;
  private pathRef: unknown = null;
  private targetRing!: THREE.Mesh;
  private aimLines!: THREE.LineSegments;
  private pulseT = 1;
  private acc = 0;
  private hook = (dt: number) => this.frame(dt);
  private saved: Record<string, unknown> | null = null;
  private lod: Map<string, THREE.BufferGeometry> | null = null;
  private statusClock = 0;
  /** module mechanisms in MODULES order, with the wheel's rolling heading in the CAD pose */
  private moduleMech: { id: string; h0: number }[] = [];
  /** the field direction "forward" on the stick means, latched while the stick is held */
  private driveYaw: number | null = null;
  private frustum = new THREE.Frustum();
  private projScreen = new THREE.Matrix4();
  private euler = new THREE.Euler();
  private lastRaw: RawInput | null = null;
  view: SimView = 'tv';
  autoOrbit = false;
  active = false;
  paused = false;
  /** show the on-screen driving controls (touch devices) */
  touchControls = matchMedia('(pointer: coarse)').matches;
  onStatus: ((s: SimStatus) => void) | null = null;
  /** someone used the driving controls (wakes the pit display) */
  onActivity: (() => void) | null = null;
  onRespawn: (() => void) | null = null;
  onPad: ((name: string | null) => void) | null = null;

  constructor(private viewer: Viewer) {}

  get mode(): SimMode { return this.world?.mode ?? 'auto'; }
  get robot() { return this.world.robot; }
  get balls() { return this.world.balls; }
  get grid() { return this.world.field; }
  get brain() { return this.world.brain; }

  async start(mode: SimMode = 'auto') {
    const v = this.viewer;
    this.emitLoading(mode);
    const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const [gltf, data, lodGltf] = await Promise.all([
      loader.loadAsync('field/field-2026.glb'),
      fetch('field/field-2026.json').then((r) => r.json() as Promise<FieldData>),
      loader.loadAsync('models/chuck-norris-sim.glb').catch(() => null),
    ]);
    this.lod = new Map();
    lodGltf?.scene.traverse((o) => { const m = o as THREE.Mesh; if (m.isMesh) this.lod!.set(m.name, m.geometry); });
    gltf.scene.traverse((o) => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      m.receiveShadow = true;
      const mat = m.material as THREE.MeshStandardMaterial;
      m.castShadow = !mat.transparent;
      if (mat.transparent) mat.depthWrite = false;
      mat.envMapIntensity = 0.7;
      m.raycast = () => {};
    });
    this.group.add(gltf.scene);

    this.world = new SimWorld(data);
    const w = this.world;
    w.onScore = () => { this.pulseT = 0; sound.play('score'); };
    w.onRespawn = () => this.onRespawn?.();
    w.onControl = (c) => sound.play(c === 'driver' ? 'takeover' : 'handback');
    w.isVisible = (x, y, z) => this.frustum.containsPoint(new THREE.Vector3(x, y, z));
    w.robot.onShot = (_i, side) => sound.play('shot', { pan: this.panOf(w.robot.x, w.robot.z) + side * 0.1 });
    w.robot.onIntake = () => sound.play('intake', { pan: this.panOf(w.robot.x, w.robot.z) });
    w.balls.onBounce = (i, sp) => {
      const p = w.balls.pos;
      const d = this.viewer.camera.position.distanceTo(new THREE.Vector3(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]));
      if (d < 7) sound.play('bounce', { gain: Math.min(1, sp / 7) * (1 - d / 7), pan: this.panOf(p[i * 3], p[i * 3 + 2]) });
    };
    const red = [...data.hubs].sort((a, b) => a.center[0] - b.center[0])[0];

    const geo = new THREE.SphereGeometry(R, 12, 9);
    const mat = new THREE.MeshPhysicalMaterial({ color: new THREE.Color(0.98, 0.66, 0.05), roughness: 0.62, sheen: 0.4, sheenColor: new THREE.Color(1, 0.85, 0.4) });
    this.inst = new THREE.InstancedMesh(geo, mat, w.balls.n);
    this.inst.castShadow = true;
    this.inst.receiveShadow = true;
    this.inst.frustumCulled = false;
    this.group.add(this.inst);

    // glowing shot trails
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(64 * 16 * 6), 3));
    tg.setAttribute('color', new THREE.BufferAttribute(new Float32Array(64 * 16 * 6), 3));
    this.trails = new THREE.LineSegments(tg, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false }));
    this.trails.frustumCulled = false;
    this.group.add(this.trails);

    this.pulse = new THREE.Mesh(
      new THREE.TorusGeometry(0.58, 0.018, 12, 72).rotateX(Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0xffb54a, transparent: true, opacity: 0, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.pulse.position.set(red.center[0], red.center[1] + 0.02, red.center[2]);
    this.group.add(this.pulse);

    // a soft alliance-red halo on the carpet so the robot is easy to find from far views
    this.locator = new THREE.Mesh(
      new THREE.RingGeometry(0.66, 0.8, 64).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0xff3b4e, transparent: true, opacity: 0.55, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.locator.renderOrder = 2;
    this.group.add(this.locator);

    const holo = new THREE.Color(0x6fd3ff);
    this.pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: holo, dashSize: 0.18, gapSize: 0.1, transparent: true, opacity: 0.85, depthWrite: false }));
    this.pathLine.frustumCulled = false;
    this.pathLine.renderOrder = 3;
    this.group.add(this.pathLine);
    this.targetRing = new THREE.Mesh(
      new THREE.RingGeometry(0.2, 0.235, 48).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: holo, transparent: true, opacity: 0.9, depthWrite: false, side: THREE.DoubleSide }),
    );
    this.targetRing.visible = false;
    this.group.add(this.targetRing);
    const ag = new THREE.BufferGeometry();
    ag.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12), 3));
    this.aimLines = new THREE.LineSegments(ag, new THREE.LineBasicMaterial({ color: 0xff8fa8, transparent: true, opacity: 0.35, depthWrite: false }));
    this.aimLines.frustumCulled = false;
    this.group.add(this.aimLines);

    // Take over the stage
    const kl = v.keyLight, sc = kl.shadow.camera;
    this.saved = {
      maxDist: v.controls.maxDistance, minDist: v.controls.minDistance, maxPolar: v.controls.maxPolarAngle,
      keyPos: kl.position.clone(), shadow: [sc.left, sc.right, sc.top, sc.bottom, sc.far], rootPos: v.root.position.clone(),
      dollyToCursor: v.controls.dollyToCursor, smoothTime: v.controls.smoothTime,
    };
    v.setSimRendering(this.lod, FOV_SIM);
    v.controls.addEventListener('controlstart', this.onUserStart);
    v.controls.addEventListener('controlend', this.onUserEnd);
    v.setFocus(null);
    v.setExploded(false);
    for (const m of v.mechanisms.values()) v.setMechanism(m.id, m.def, false);
    v.simActive = true;
    v.spinning = false;
    // drive about the drivetrain center (the swerve modules), not the model's bounding-box center
    const swerve = v.boxOfParts(v.subsystemPids('swerve'), false).getCenter(new THREE.Vector3());
    v.root.position.set(-swerve.x, 0, -swerve.z);
    v.scene.add(this.group);
    v.grid.visible = false;
    sc.left = sc.bottom = -4.5; sc.right = sc.top = 4.5; sc.far = 16;
    sc.updateProjectionMatrix();
    v.controls.setBoundary(new THREE.Box3(new THREE.Vector3(-12, 0, -7), new THREE.Vector3(12, 8, 7)));
    v.controls.maxDistance = 24;
    v.controls.minDistance = 0.8;
    v.controls.maxPolarAngle = 86 * DEG;
    v.controls.dollyToCursor = false;
    v.controls.smoothTime = 0.45;
    v.controls.setFocalOffset(0, 0, 0, false);
    v.stage.position.set(0, 0, 0); v.stage.quaternion.identity();
    v.stage.updateMatrixWorld(true);
    // swerve modules: which mechanism is which corner, and the wheel's rolling direction in the CAD
    // (the axle points away from the robot's center, so the wheel rolls tangentially)
    const inv = new THREE.Matrix4().copy(v.stage.matrixWorld).invert();
    const corners: [number, number][] = [[0.289, 0.276], [-0.289, 0.276], [0.289, -0.276], [-0.289, -0.276]];
    this.moduleMech = corners.map(([cx, cz]) => {
      let best = '', bd = Infinity, h0 = 0;
      for (const [id, mech] of v.mechanisms) {
        if (!id.startsWith('module-')) continue;
        const p = mech.node.getWorldPosition(new THREE.Vector3()).applyMatrix4(inv);
        const d = Math.hypot(p.x - cx, p.z - cz);
        if (d < bd) { bd = d; best = id; const r = Math.hypot(p.x, p.z) || 1; h0 = Math.atan2(-p.z / r, p.x / r); }
      }
      return { id: best, h0 };
    });

    this.controls.attach();
    this.controls.onPad = (name) => this.onPad?.(name);
    this.pad = new Drivepad(document.getElementById('ui')!, this.controls.touch);
    v.hooks.add(this.hook);
    this.active = true;
    this.setMode(mode);
    sound.simStart();
    sound.play('boot');
  }

  setMode(mode: SimMode) {
    this.world.setMode(mode);
    this.trailPos.clear();
    this.acc = 0;
    this.driveYaw = null;
    this.present();
    this.writeInstances();
    this.userT = 99;
    this.setView(this.view, false);
    this.emit();
  }

  restart() { this.setMode(this.mode); sound.play('select'); }

  stop() {
    const v = this.viewer;
    if (!this.saved) return;
    v.hooks.delete(this.hook);
    this.controls.detach();
    this.pad?.dispose(); this.pad = null;
    sound.simStop();
    v.controls.removeEventListener('controlstart', this.onUserStart);
    v.controls.removeEventListener('controlend', this.onUserEnd);
    v.setSimRendering(null, null);
    v.scene.remove(this.group);
    v.grid.visible = true;
    v.simActive = false;
    const s = this.saved as { maxDist: number; minDist: number; maxPolar: number; keyPos: THREE.Vector3; shadow: number[]; rootPos: THREE.Vector3; dollyToCursor: boolean; smoothTime: number };
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
    v.controls.maxDistance = s.maxDist;
    v.controls.minDistance = s.minDist;
    v.controls.maxPolarAngle = s.maxPolar;
    v.controls.dollyToCursor = s.dollyToCursor;
    v.controls.smoothTime = s.smoothTime;
    for (const m of v.mechanisms.values()) {
      m.angle.smoothTime = m.type === 'fourbar' ? 0.42 : 0.32;
      v.setMechanism(m.id, m.def, true);
    }
    this.active = false;
    this.saved = null;
    v.invalidate(true);
  }

  setView(view: SimView, animate = true) {
    this.view = view;
    this.userT = 99;
    const r = this.world.robot;
    this.camHeading = r.th;
    this.camFocus.set(r.x, 0, r.z);
    this.tvSide = r.z > 0 ? 1 : -1;
    this.viewer.setFov(view === 'driver' ? 50 : view === 'field' ? 40 : FOV_SIM);
    this.driveYaw = null;
    this.aimCamera(0, animate);
  }

  nextView() {
    const i = SIM_VIEWS.findIndex((x) => x.id === this.view);
    this.setView(SIM_VIEWS[(i + 1) % SIM_VIEWS.length].id);
    sound.play('tap');
  }

  /** Hands the robot back to its autonomous brain right away. */
  handBack() { this.world.resumeAI(); }
  setTouchControls(on: boolean) { this.touchControls = on; }

  // ------------------------------------------------------------------ driver input → field frame
  /**
   * Turns the stick into a field-frame velocity. "camera": up on the stick drives away from the
   * camera (the direction is held while the stick is, so a moving camera never changes what up
   * means mid-move); "field": away from the red driver station, like a real FRC driver;
   * "robot": the robot's own front.
   */
  private driverInput(raw: RawInput): DriverInput {
    const r = this.world.robot;
    const frame = this.controls.bindings.frame;
    let fwd: number;
    if (frame === 'field') fwd = Math.PI / 2; // +x, away from the red alliance wall
    else if (frame === 'robot') fwd = r.th;
    else {
      const held = Math.hypot(raw.mx, raw.my) > 0.05;
      if (!held || this.driveYaw === null) {
        // where the camera is headed (not where the last rendered frame put it)
        const c = this.viewer.controls, from = c.getPosition(new THREE.Vector3()), to = c.getTarget(new THREE.Vector3());
        const dx = to.x - from.x, dz = to.z - from.z;
        if (Math.hypot(dx, dz) > 0.05) this.driveYaw = Math.atan2(dx, dz);
      }
      fwd = this.driveYaw ?? Math.PI / 2;
    }
    const fx = Math.sin(fwd), fz = Math.cos(fwd);
    const rx = -fz, rz = fx;
    return {
      mx: fx * raw.my + rx * raw.mx, mz: fz * raw.my + rz * raw.mx, turn: raw.turn,
      shoot: raw.shoot, intake: raw.intake, eject: raw.eject, lock: raw.lock, slow: raw.slow,
      toggleArm: raw.arm, handBack: raw.handBack,
    };
  }

  // ------------------------------------------------------------------ per frame
  frame(dt: number): boolean {
    if (!this.active || this.paused) return true;
    const w = this.world, r = w.robot;
    this.pad?.setVisible(this.touchControls && !this.autoOrbit);
    this.pad?.setArm(r.intakeDown);
    const raw = this.controls.read();
    this.lastRaw = raw;
    if (raw.active || raw.handBack || raw.camera) this.onActivity?.();
    if (raw.camera) this.nextView();
    const input = raw.active || raw.handBack ? this.driverInput(raw) : w.control === 'driver' ? this.driverInput(raw) : NO_INPUT;
    // the camera's view of where things are (fuel is only tidied away off screen)
    const cam = this.viewer.camera;
    this.projScreen.multiplyMatrices(cam.projectionMatrix, cam.matrixWorldInverse);
    this.frustum.setFromProjectionMatrix(this.projScreen);

    this.acc += Math.min(dt, 0.1);
    let steps = 0, impact = 0;
    while (this.acc >= STEP && steps < 8) {
      this.acc -= STEP;
      w.step(STEP, steps === 0 ? input : { ...input, toggleArm: false, handBack: false });
      impact = Math.max(impact, r.drive.impact);
      steps++;
    }
    if (impact > 25) sound.play('impact', { gain: Math.min(1, impact / 160), pan: this.panOf(r.x, r.z) });
    this.present();
    this.locator.position.set(r.x, w.field.floor(r.x, r.z) + 0.012, r.z);
    const lm = this.locator.material as THREE.MeshBasicMaterial;
    lm.color.setHex(w.control === 'driver' ? 0xffd34a : 0xff3b4e);
    lm.opacity = 0.38 + 0.14 * Math.sin(w.clock * 3);
    this.writeInstances();
    this.updateTrails();
    this.updateBrainViz();
    this.updateCamera(dt);
    this.updateSound();
    if (this.pulseT < 1) {
      this.pulseT = Math.min(1, this.pulseT + dt * 1.8);
      (this.pulse.material as THREE.MeshBasicMaterial).opacity = (1 - this.pulseT) * 0.9;
      this.pulse.scale.setScalar(1 + this.pulseT * 0.35);
    }
    // key light follows the robot so its shadows stay crisp
    const kl = this.viewer.keyLight;
    kl.position.set(r.x + 2.4, 6.5, r.z + 3.2);
    kl.target.position.set(r.x, 0, r.z);
    kl.target.updateMatrixWorld();
    this.statusClock += dt;
    if (this.statusClock > 0.12) { this.statusClock = 0; this.emit(); }
    return true;
  }

  /** The robot model: chassis pose (with the weight-transfer rock), mechanisms, and the fuel inside it. */
  private present() {
    const v = this.viewer, r = this.world.robot, d = r.drive;
    const st = v.stage;
    st.position.set(r.x, r.py, r.z);
    st.quaternion.setFromEuler(this.euler.set(r.pitch + d.tiltP, r.th, r.roll + d.tiltR, 'YXZ'));
    st.updateMatrix();
    st.matrixWorld.copy(st.matrix);
    v.setMechanismNow('intake', r.intakeAngle);
    for (const t of r.turrets) v.setMechanismNow(t.id, t.a / DEG);
    this.moduleMech.forEach((m, i) => {
      let a = d.steer[i] - m.h0;
      a = Math.atan2(Math.sin(a), Math.cos(a));
      v.setMechanismNow(m.id, a / DEG);
    });
    v.syncPose();
    // fuel the robot is holding, where it actually is inside the robot
    const pos = this.world.balls.pos, m = st.matrixWorld, p = new THREE.Vector3();
    r.forEachHeld((i, x, y, z) => {
      p.set(x, y, z).applyMatrix4(m);
      pos[i * 3] = p.x; pos[i * 3 + 1] = p.y; pos[i * 3 + 2] = p.z;
    });
  }

  /** Where the robot is headed: a point ~2.5 m along its path, the fuel it is hunting, or the HUB. */
  private interest(out: THREE.Vector3) {
    const r = this.world.robot;
    out.set(r.x, 0, r.z);
    if (this.world.control === 'driver') {
      // lead the camera in the direction the driver is going
      out.set(r.x + r.vx * 0.6, 0, r.z + r.vz * 0.6);
    } else if (r.order && !r.done) {
      let left = 2.5, px = r.x, pz = r.z;
      for (const [x, z] of r.order.path) {
        const d = Math.hypot(x - px, z - pz);
        if (d >= left) { px += ((x - px) * left) / d; pz += ((z - pz) * left) / d; left = 0; break; }
        left -= d; px = x; pz = z;
      }
      out.set(px, 0, pz);
    } else if (r.shooting) out.set(this.world.hub.cx, 0.6, this.world.hub.cz);
    return out;
  }

  /** Visible area (fraction of the screen) not covered by the HUD at the top and the controls at the bottom. */
  private screenSafe() {
    const h = this.viewer.viewSize.h || 1;
    return { top: Math.min(0.2, 70 / h), bottom: Math.min(0.24, 96 / h) };
  }

  private aimCamera(dt: number, animate: boolean) {
    const c = this.viewer.controls, r = this.world.robot, cam = this.viewer.camera;
    const driving = this.world.control === 'driver';
    const vfov = cam.fov * DEG, aspect = cam.aspect || 1.4;
    const hfov = 2 * Math.atan(Math.tan(vfov / 2) * aspect);
    const safe = this.screenSafe();
    const lift = (safe.bottom - safe.top) / 2;
    const k = dt > 0 ? 1 - Math.exp(-dt * 2.2) : 1;
    if (this.view === 'tv') {
      const want = this.interest(new THREE.Vector3());
      this.camFocus.lerp(want, k);
      const fx = r.x * 0.62 + this.camFocus.x * 0.38, fz = r.z * 0.62 + this.camFocus.z * 0.38;
      const spread = Math.hypot(this.camFocus.x - r.x, this.camFocus.z - r.z);
      const need = Math.max(2.6, spread * 1.25 + 1.6);
      const dist = THREE.MathUtils.clamp(need / 2 / Math.tan(Math.min(hfov, vfov * 1.5) / 2), 6.6, 12);
      // shoot steeply from the robot's own side of the field; while someone drives it never swaps
      // sides (that would flip what "up" on the stick means)
      if (!driving) { if (r.z < -1.4) this.tvSide = -1; else if (r.z > 1.4) this.tvSide = 1; }
      const el = (55 + 15 * THREE.MathUtils.clamp((Math.abs(r.z) - 2.2) / 1.1, 0, 1)) * DEG, side = this.tvSide;
      const tx = fx, tz = fz - side * lift * dist * Math.tan(vfov / 2) * 0.9;
      c.setLookAt(tx, Math.sin(el) * dist, tz + side * Math.cos(el) * dist, tx, 0.25, tz, animate);
    } else if (this.view === 'follow') {
      // chase camera: behind the direction of travel, or behind the robot itself while someone drives
      const sp = Math.hypot(r.vx, r.vz);
      const want = driving ? r.th : sp > 0.6 ? Math.atan2(r.vx, r.vz) : r.th;
      let d = want - this.camHeading;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      this.camHeading += d * (dt > 0 ? 1 - Math.exp(-dt * (driving ? 2.4 : 1.6)) : 1);
      const fx = Math.sin(this.camHeading), fz = Math.cos(this.camHeading);
      const back = 3.3, up = 2.0;
      c.setLookAt(r.x - fx * back, up, r.z - fz * back, r.x + fx * 1.6, 0.25, r.z + fz * 1.6, animate);
    } else if (this.view === 'field') {
      const el = 62 * DEG;
      const usable = 1 - safe.top - safe.bottom;
      const halfW = 8.45, halfD = 4.25;
      const dW = halfW / Math.tan(hfov / 2);
      const dD = (halfD * Math.sin(el) + 0.6) / (Math.tan(vfov / 2) * usable);
      const dist = Math.max(dW, dD) * 1.04;
      const tz = lift * dist * Math.tan(vfov / 2) * 1.6;
      c.setLookAt(0, Math.sin(el) * dist, tz - Math.cos(el) * dist, 0, 0, tz, animate);
    } else {
      const want = new THREE.Vector3(r.x, 0.3, r.z);
      this.camFocus.lerp(want, k);
      const ez = THREE.MathUtils.clamp(this.camFocus.z * 0.35, -2.4, 2.4);
      c.setLookAt(-8.45, 2.25, ez, this.camFocus.x * 0.85 - 0.8, 0.2, this.camFocus.z * 0.8, animate);
    }
  }

  private updateBrainViz() {
    const w = this.world, r = w.robot;
    const order = r.done || w.control === 'driver' ? null : r.order;
    if (order !== this.pathRef) {
      this.pathRef = order;
      if (order) {
        const pts = [new THREE.Vector3(r.x, 0.02, r.z), ...order.path.map(([x, z]) => new THREE.Vector3(x, w.field.floor(x, z) + 0.02, z))];
        this.pathLine.geometry.dispose();
        this.pathLine.geometry = new THREE.BufferGeometry().setFromPoints(pts);
        this.pathLine.computeLineDistances();
      }
      this.pathLine.visible = !!order;
    }
    const t = w.brain instanceof EndlessBrain && w.control === 'ai' ? w.brain.targetBall : -1;
    this.targetRing.visible = t >= 0 && w.balls.available(t);
    if (this.targetRing.visible) {
      const p = w.balls.pos;
      this.targetRing.position.set(p[t * 3], w.field.floor(p[t * 3], p[t * 3 + 2]) + 0.012, p[t * 3 + 2]);
      this.targetRing.scale.setScalar(1 + 0.12 * Math.sin(w.clock * 6));
    }
    const a = this.aimLines.geometry.getAttribute('position') as THREE.BufferAttribute;
    const ends = r.turretExits();
    ends.forEach((e, k) => { a.setXYZ(k * 2, e[0], e[1], e[2]); a.setXYZ(k * 2 + 1, w.hubTarget[0], w.hubTarget[1], w.hubTarget[2]); });
    a.needsUpdate = true;
    this.aimLines.visible = r.stored > 0 && (w.control === 'ai' || r.shoot);
  }

  private updateCamera(dt: number) {
    if (this.userActive) { this.userT = 0; return; }
    this.userT += dt;
    if (this.autoOrbit && this.world.control === 'ai') { this.direct(dt); return; }
    if (this.directing) { this.directing = false; this.setView(this.view); return; }
    // while driving, the camera comes back sooner after a look around
    if (this.userT < (this.world.control === 'driver' ? 1.5 : USER_HOLD)) return;
    this.aimCamera(dt, true);
  }

  // ------------------------------------------------------------------ pit display director
  // Left alone, the simulation plays like a broadcast: it cycles shots every few seconds (a slow orbit
  // around the robot, the TV camera, a low chase and a high crane shot) and keeps the robot playing.
  private directing = false;
  private shotT = 0;
  private shot = 0;
  private orbitAz = 0;
  private direct(dt: number) {
    const c = this.viewer.controls, w = this.world, r = w.robot;
    if (!this.directing) { this.directing = true; this.shotT = 0; this.shot = 0; this.orbitAz = r.th + Math.PI * 0.75; this.viewer.setFov(FOV_SIM); }
    this.shotT += dt;
    const LEN = [11, 9, 8, 10];
    if (this.shotT > LEN[this.shot]) { this.shotT = 0; this.shot = (this.shot + 1) % LEN.length; }
    const y = w.field.floor(r.x, r.z);
    if (this.shot === 0) {
      this.orbitAz += dt * 0.22;
      const d = 3.6, el = 22 * DEG;
      c.setLookAt(r.x + Math.sin(this.orbitAz) * Math.cos(el) * d, y + 0.35 + Math.sin(el) * d, r.z + Math.cos(this.orbitAz) * Math.cos(el) * d, r.x, y + 0.35, r.z, true);
    } else if (this.shot === 1) {
      const v = this.view; this.view = 'tv'; this.aimCamera(dt, true); this.view = v;
    } else if (this.shot === 2) {
      const v = this.view; this.view = 'follow'; this.aimCamera(dt, true); this.view = v;
    } else {
      this.orbitAz += dt * 0.06;
      const d = 9, el = 48 * DEG;
      const tx = r.x * 0.7 + w.hub.cx * 0.3, tz = r.z * 0.7;
      c.setLookAt(tx + Math.sin(this.orbitAz) * Math.cos(el) * d, Math.sin(el) * d, tz + Math.cos(this.orbitAz) * Math.cos(el) * d, tx, 0.3, tz, true);
    }
    // Auto ends after 20 s; the pit display keeps playing Endless
    const b = w.brain as { ended?: boolean; t?: number };
    if (w.mode === 'auto' && b.ended && (b.t ?? 0) > 24) this.setMode('endless');
  }

  // ------------------------------------------------------------------ sound
  private panOf(x: number, z: number) {
    const p = new THREE.Vector3(x, 0.3, z).project(this.viewer.camera);
    return THREE.MathUtils.clamp(p.x, -1, 1) * 0.7;
  }
  private updateSound() {
    const r = this.world.robot, d = r.drive;
    const dist = this.viewer.camera.position.distanceTo(new THREE.Vector3(r.x, 0.3, r.z));
    sound.simUpdate({
      speed: d.speed, turn: Math.abs(d.w), effort: d.effort, slipping: d.slipping,
      flywheels: r.turrets.map((t) => t.fly),
      rollers: r.rollers === 'off' || r.intakeAngle < -40 ? 0 : 1,
      arm: Math.abs(r.intakeAngle - (r.intakeDown ? 0 : -73)) > 0.5 ? 1 : 0,
      feeding: r.turrets.some((t) => t.lane.some((b) => b.firing)) ? 1 : 0,
      near: THREE.MathUtils.clamp(1.6 / Math.max(1.6, dist * 0.55), 0.25, 1),
      pan: this.panOf(r.x, r.z),
    });
  }

  private writeInstances() {
    const m = new THREE.Matrix4();
    const hide = new THREE.Matrix4().makeScale(0, 0, 0);
    const { pos, st, n } = this.world.balls;
    for (let i = 0; i < n; i++) {
      if (st[i] === S.IN_HUB) this.inst.setMatrixAt(i, hide);
      else this.inst.setMatrixAt(i, m.makeTranslation(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]));
    }
    this.inst.instanceMatrix.needsUpdate = true;
  }

  private updateTrails() {
    const { pos, st } = this.world.balls;
    for (let i = 0; i < this.world.balls.n; i++) {
      const tr = this.trailPos.get(i);
      if (st[i] === S.FLYING && pos[i * 3 + 1] > 0.3) {
        const arr = tr ?? [];
        arr.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
        if (arr.length > 17 * 3) arr.splice(0, 3);
        this.trailPos.set(i, arr);
      } else if (tr) {
        tr.splice(0, 6);
        if (tr.length < 6) this.trailPos.delete(i);
      }
    }
    const P = this.trails.geometry.getAttribute('position') as THREE.BufferAttribute;
    const C = this.trails.geometry.getAttribute('color') as THREE.BufferAttribute;
    let o = 0;
    const max = P.count;
    for (const arr of this.trailPos.values()) {
      const n = arr.length / 3;
      for (let k = 0; k < n - 1 && o < max - 2; k++) {
        const a = (k + 1) / n;
        P.setXYZ(o, arr[k * 3], arr[k * 3 + 1], arr[k * 3 + 2]);
        C.setXYZ(o++, 1 * a * 0.6, 0.72 * a * 0.6, 0.25 * a * 0.6);
        P.setXYZ(o, arr[k * 3 + 3], arr[k * 3 + 4], arr[k * 3 + 5]);
        C.setXYZ(o++, 1 * a * 0.7, 0.72 * a * 0.7, 0.25 * a * 0.7);
      }
    }
    this.trails.geometry.setDrawRange(0, o);
    P.needsUpdate = true;
    C.needsUpdate = true;
  }

  private emitLoading(mode: SimMode) {
    this.onStatus?.({ mode, phase: 'Loading field…', scored: 0, stored: 0, capacity: 0, time: null, bps: 0, ended: false, loading: true, control: 'ai', resumeIn: null, outOfRange: false, intakeDown: true });
  }

  private emit() { this.onStatus?.(this.status()); }

  status(): SimStatus { return this.world.status(); }
  /** The source of the last input (for the HUD's hints). */
  get inputSource() { return this.lastRaw?.source ?? null; }
}
