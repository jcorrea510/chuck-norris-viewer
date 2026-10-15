// Simulation mode: Chuck Norris on the 2026 REBUILT field.
//   Auto    — the team's autonomous: start in the left TRENCH lane, intake up, facing the middle;
//             under the trench, drop the intake, sweep the neutral zone, back under the trench and
//             shoot, then again. It never touches a BUMP.
//   Endless — keeps finding fuel wherever it is (scored fuel pours out of the HUB into the
//             neutral zone), collects it and scores it, forever; each crossing picks the trench or
//             the bump at random.
// Field: AdvantageScope "2026 FRC Field" (scripts/optimize-field.mjs). Red alliance wall at −X.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import type { Viewer } from '../viewer';
import { sound } from '../sound';
import { FieldGrid, type GridData, type HubData } from './field';
import { Balls, S, R, hubShape, type HubShape } from './balls';
import { RobotSim, VMAX } from './robot';

interface FieldData { fuelRadius: number; hubs: HubData[]; fuel: number[][]; grid: GridData }

export type SimMode = 'auto' | 'endless';
/** tv: broadcast camera that keeps the robot and where it is going in frame; follow: chase camera
 *  behind the robot; field: the whole field from above; driver: the red driver station. */
export type SimView = 'tv' | 'follow' | 'field' | 'driver';
export const SIM_VIEWS: { id: SimView; label: string; title: string }[] = [
  { id: 'tv', label: 'TV', title: 'Broadcast camera: the robot and where it is going' },
  { id: 'follow', label: 'Chase', title: 'Behind the robot' },
  { id: 'field', label: 'Field', title: 'The whole field' },
  { id: 'driver', label: 'Driver', title: 'From the driver station' },
];
export interface SimStatus {
  mode: SimMode; phase: string; scored: number; stored: number; capacity: number;
  time: number | null; bps: number; ended: boolean; loading: boolean;
}

const DEG = Math.PI / 180;
const STEP = 1 / 120;
const AUTO_SECONDS = 20;
const ALLIANCE_LINE = -4.24;       // red alliance zone is x < this
const SHOOT_X = -4.45;             // robot center must be this deep to score
const CLEAR = 0.5;                 // planning clearance (robot half-diagonal with margin)
// Each alliance's BUMPs and TRENCHes sit on one line across the field (x = ±CROSS_X). A route
// across it enters at CROSS_X ∓ CROSS_OUT and leaves on the other side.
const CROSS_X = 3.62;
const CROSS_OUT = 1.2;
const BUMP_Z = 1.5;
const FOV_SIM = 46;
const USER_HOLD = 4;               // seconds the camera stays where the user put it

export class Simulation {
  private group = new THREE.Group();
  private data!: FieldData;
  private field!: FieldGrid;
  private hub!: HubShape;
  private hubTarget!: number[];
  balls!: Balls;
  robot!: RobotSim;
  private inst!: THREE.InstancedMesh;
  private trails!: THREE.LineSegments;
  private trailPos = new Map<number, number[]>();
  private pulse!: THREE.Mesh;
  private locator!: THREE.Mesh;
  /** Trench lane center (|z|), measured from the field grid. */
  trenchZ = 3.38;
  private crossMemo = new Map<number, { kind: 'trench' | 'bump'; z: number; t: number; from: number }>();
  private userT = 99;
  private onUserStart = () => { this.userT = 0; this.userActive = true; };
  private onUserEnd = () => { this.userActive = false; };
  private userActive = false;
  private camHeading = 0;
  private camFocus = new THREE.Vector3();
  // the robot's "brain", visualized: planned path, the fuel it is hunting, turret aim lines
  private pathLine!: THREE.Line;
  private pathRef: unknown = null;
  private targetRing!: THREE.Mesh;
  private aimLines!: THREE.LineSegments;
  private pulseT = 1;
  private acc = 0;
  private hook = (dt: number) => this.frame(dt);
  brain: AutoBrain | EndlessBrain | null = null;
  private saved: Record<string, unknown> | null = null;
  private lod: Map<string, THREE.BufferGeometry> | null = null;
  private scoreTimes: number[] = [];
  private clock = 0;
  private statusClock = 0;
  mode: SimMode = 'auto';
  view: SimView = 'tv';
  autoOrbit = false;
  active = false;
  paused = false;
  onStatus: ((s: SimStatus) => void) | null = null;

  constructor(private viewer: Viewer) {}

  async start(mode: SimMode = 'auto') {
    const v = this.viewer;
    this.emitLoading();
    const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const [gltf, data, lodGltf] = await Promise.all([
      loader.loadAsync('field/field-2026.glb'),
      fetch('field/field-2026.json').then((r) => r.json() as Promise<FieldData>),
      loader.loadAsync('models/chuck-norris-sim.glb').catch(() => null),
    ]);
    this.data = data;
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

    // Red alliance HUB
    const red = [...data.hubs].sort((a, b) => a.center[0] - b.center[0])[0];
    this.field = new FieldGrid(data.grid, data.hubs);
    this.hub = hubShape(red);
    // aim a little past the center of the funnel so near-misses still roll in
    this.hubTarget = [red.center[0] + 0.05, red.center[1] + 0.02, red.center[2]];

    this.balls = new Balls(this.field, this.hub, data.fuel);
    this.balls.onScore = () => {
      this.pulseT = 0;
      this.scoreTimes.push(this.clock);
      sound.play('score');
    };
    const geo = new THREE.SphereGeometry(R, 14, 10);
    const mat = new THREE.MeshPhysicalMaterial({ color: new THREE.Color(0.98, 0.66, 0.05), roughness: 0.62, sheen: 0.4, sheenColor: new THREE.Color(1, 0.85, 0.4) });
    this.inst = new THREE.InstancedMesh(geo, mat, this.balls.n);
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

    // trench lane: the widest free gap between the side wall and the trench supports
    let best = 0;
    for (let z = 2.4; z < 4.05; z += 0.025) {
      const d = Math.min(this.field.distAt(-CROSS_X, -z), this.field.distAt(-CROSS_X - 0.4, -z), this.field.distAt(-CROSS_X + 0.4, -z));
      if (d > best) { best = d; this.trenchZ = z; }
    }

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
    v.stage.updateMatrixWorld(true);

    this.robot = new RobotSim(v, this.field, this.balls, this.hubTarget);
    this.robot.onShot = () => sound.play('shot');
    this.robot.onIntake = (i) => { if (i % 3 === 0) sound.play('intake'); };
    v.hooks.add(this.hook);
    this.active = true;
    this.setMode(mode);
    sound.play('boot');
  }

  setMode(mode: SimMode) {
    this.mode = mode;
    this.balls.reset(this.data.fuel);
    this.trailPos.clear();
    this.scoreTimes = [];
    this.clock = 0;
    this.acc = 0;
    // preload: 8 fuel from the red outpost (staged outside the field)
    const pre: number[] = [];
    for (let i = 0; i < this.balls.n && pre.length < 8; i++) if (this.balls.st[i] === S.PARKED && this.balls.pos[i * 3] < -8) pre.push(i);
    this.crossMemo.clear();
    if (mode === 'auto') {
      this.robot.setIntakeNow(false);
      this.robot.place(ALLIANCE_LINE - 0.445 - 0.03, -this.trenchZ, Math.PI / 2);
      this.brain = new AutoBrain(this);
    } else {
      this.robot.setIntakeNow(true);
      this.robot.place(-5.6, 1.0, Math.PI / 2);
      this.brain = new EndlessBrain(this);
    }
    this.robot.preload(pre);
    this.robot.present();
    this.writeInstances();
    this.userT = 99;
    this.setView(this.view, false);
    this.emit(true);
  }

  restart() { this.setMode(this.mode); sound.play('select'); }

  stop() {
    const v = this.viewer;
    if (!this.saved) return;
    v.hooks.delete(this.hook);
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
    this.camHeading = this.robot.th;
    this.camFocus.set(this.robot.x, 0, this.robot.z);
    this.viewer.setFov(view === 'driver' ? 50 : view === 'field' ? 40 : FOV_SIM);
    this.aimCamera(0, animate);
  }

  /** Where the robot is headed: a point ~2.5 m along its path, the fuel it is hunting, or the HUB. */
  private interest(out: THREE.Vector3) {
    const r = this.robot;
    out.set(r.x, 0, r.z);
    if (r.order && !r.done) {
      let left = 2.5, px = r.x, pz = r.z;
      for (const [x, z] of r.order.path) {
        const d = Math.hypot(x - px, z - pz);
        if (d >= left) { px += ((x - px) * left) / d; pz += ((z - pz) * left) / d; left = 0; break; }
        left -= d; px = x; pz = z;
      }
      out.set(px, 0, pz);
    } else if (r.shooting) out.set(this.hub.cx, 0.6, this.hub.cz);
    return out;
  }

  /** Visible area (fraction of the screen) not covered by the HUD at the top and the controls at the bottom. */
  private screenSafe() {
    const h = this.viewer.viewSize.h || 1;
    return { top: Math.min(0.2, 70 / h), bottom: Math.min(0.24, 96 / h) };
  }

  private aimCamera(dt: number, animate: boolean) {
    const c = this.viewer.controls, r = this.robot, cam = this.viewer.camera;
    const vfov = cam.fov * DEG, aspect = cam.aspect || 1.4;
    const hfov = 2 * Math.atan(Math.tan(vfov / 2) * aspect);
    const safe = this.screenSafe();
    // shift the aim so the subject sits in the middle of the uncovered part of the screen
    const lift = (safe.bottom - safe.top) / 2;
    const k = dt > 0 ? 1 - Math.exp(-dt * 2.2) : 1;
    if (this.view === 'tv') {
      // broadcast camera on the scoring-table side: keeps the robot and where it is going in frame
      const want = this.interest(new THREE.Vector3());
      this.camFocus.lerp(want, k);
      const fx = r.x * 0.62 + this.camFocus.x * 0.38, fz = r.z * 0.62 + this.camFocus.z * 0.38;
      const spread = Math.hypot(this.camFocus.x - r.x, this.camFocus.z - r.z);
      const need = Math.max(2.6, spread * 1.25 + 1.6); // meters of field that must fit across
      const dist = THREE.MathUtils.clamp(need / 2 / Math.tan(Math.min(hfov, vfov * 1.5) / 2), 5.5, 12);
      const el = 38 * DEG;
      const tx = fx, tz = fz + lift * dist * Math.tan(vfov / 2) * 0.9;
      c.setLookAt(tx, Math.sin(el) * dist, tz - Math.cos(el) * dist, tx, 0.25, tz, animate);
    } else if (this.view === 'follow') {
      // chase camera behind the robot's direction of travel (or its heading when it is still)
      const sp = Math.hypot(r.vx, r.vz);
      const want = sp > 0.6 ? Math.atan2(r.vx, r.vz) : r.th;
      let d = want - this.camHeading;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      this.camHeading += d * (dt > 0 ? 1 - Math.exp(-dt * 1.6) : 1);
      const fx = Math.sin(this.camHeading), fz = Math.cos(this.camHeading);
      const back = 3.3, up = 2.0;
      c.setLookAt(r.x - fx * back, up, r.z - fz * back, r.x + fx * 1.6, 0.25, r.z + fz * 1.6, animate);
    } else if (this.view === 'field') {
      // the whole field, long side across the screen, sized to fit between the HUD bars
      const el = 62 * DEG;
      const usable = 1 - safe.top - safe.bottom;
      const halfW = 8.45, halfD = 4.25;
      const dW = halfW / Math.tan(hfov / 2);
      const dD = (halfD * Math.sin(el) + 0.6) / (Math.tan(vfov / 2) * usable);
      const dist = Math.max(dW, dD) * 1.04;
      const tz = lift * dist * Math.tan(vfov / 2) * 1.6;
      c.setLookAt(0, Math.sin(el) * dist, tz - Math.cos(el) * dist, 0, 0, tz, animate);
    } else {
      // the red driver station, looking at the robot
      const want = new THREE.Vector3(r.x, 0.3, r.z);
      this.camFocus.lerp(want, k);
      const ez = THREE.MathUtils.clamp(this.camFocus.z * 0.35, -2.4, 2.4);
      c.setLookAt(-9.7, 2.05, ez, this.camFocus.x * 0.85 - 1.0, 0.2, this.camFocus.z * 0.8, animate);
    }
  }

  // ------------------------------------------------------------------ routing
  /**
   * Plans to (gx, gz). When the trip crosses an alliance's BUMP/TRENCH line it goes through a trench
   * lane or over a bump: `prefer` forces one, otherwise each crossing is chosen at random (and kept
   * until the robot is across, so re-plans don't flip-flop).
   */
  route(gx: number, gz: number, prefer?: 'trench' | 'bump'): { path: [number, number][]; via: string; segHeading?: (number | null)[] } | null {
    const r = this.robot, f = this.field;
    const waypoints: [number, number][] = [];
    let via = '';
    for (const L of [-CROSS_X, CROSS_X]) {
      if ((r.x - L) * (gx - L) >= 0) continue;
      const dir = Math.sign(gx - L);
      let memo = this.crossMemo.get(L);
      if (!memo || prefer && memo.kind !== prefer) {
        const kind = prefer ?? (Math.random() < 0.5 ? 'trench' : 'bump');
        const lane = kind === 'trench' ? this.trenchZ : BUMP_Z;
        // the lane on the side that makes the shorter trip
        const cost = (z: number) => Math.abs(r.z - z) + Math.abs(gz - z);
        const z = cost(lane) <= cost(-lane) ? lane : -lane;
        memo = { kind, z, t: this.clock, from: Math.sign(r.x - L) };
        this.crossMemo.set(L, memo);
      }
      // line up in the open first, so the deployed intake never swings into the trench supports
      waypoints.push([L - dir * (CROSS_OUT + 0.55), memo.z], [L - dir * CROSS_OUT, memo.z], [L + dir * CROSS_OUT, memo.z]);
      via = memo.kind === 'trench' ? 'under the trench' : 'over the bump';
    }
    if (!waypoints.length) {
      const path = f.plan(r.x, r.z, gx, gz, CLEAR);
      return path ? { path, via } : null;
    }
    const out: [number, number][] = [];
    const segHeading: (number | null)[] = [];
    let sx = r.x, sz = r.z;
    for (let k = 0; k < waypoints.length; k += 3) {
      const [pre, entry, exit] = [waypoints[k], waypoints[k + 1], waypoints[k + 2]];
      const dir = Math.sign(exit[0] - pre[0]);
      const lane = dir > 0 ? Math.PI / 2 : -Math.PI / 2;
      // already lined up in (or inside) the lane: carry on from here instead of going back
      const inLane = Math.abs(sz - pre[1]) < 0.4 && (sx - pre[0]) * dir > -0.1;
      if (inLane) {
        if (!out.length) out.push([sx, sz]);
        if ((sx - entry[0]) * dir < -0.05) { out.push(entry); segHeading.push(lane); }
        out.push(exit); segHeading.push(lane);
        [sx, sz] = exit;
        continue;
      }
      const leg = f.plan(sx, sz, pre[0], pre[1], CLEAR, 50);
      if (!leg) return null;
      out.push(...(out.length ? leg.slice(1) : leg));
      while (segHeading.length < out.length - 1) segHeading.push(null);
      // line up with the lane on the approach (intake leading), then drive straight through
      if (out.length >= 2) segHeading[out.length - 2] = lane;
      out.push(entry, exit);
      segHeading.push(lane, lane);
      [sx, sz] = exit;
    }
    const last = f.plan(sx, sz, gx, gz, CLEAR, 50);
    if (!last) return null;
    out.push(...last.slice(1));
    while (segHeading.length < out.length - 1) segHeading.push(null);
    return { path: out, via, segHeading };
  }

  // ------------------------------------------------------------------ per frame
  frame(dt: number): boolean {
    if (!this.active || this.paused) return true;
    this.acc += Math.min(dt, 0.1);
    let steps = 0;
    while (this.acc >= STEP && steps < 8) {
      this.acc -= STEP;
      steps++;
      this.clock += STEP;
      this.brain?.update(STEP);
      this.robot.update(STEP);
      this.balls.step(STEP, this.robot);
    }
    this.robot.present();
    this.locator.position.set(this.robot.x, this.field.floor(this.robot.x, this.robot.z) + 0.012, this.robot.z);
    (this.locator.material as THREE.MeshBasicMaterial).opacity = 0.38 + 0.14 * Math.sin(this.clock * 3);
    // a crossing choice lasts until the robot is through it
    for (const [L, m] of this.crossMemo) {
      const across = Math.sign(this.robot.x - L) !== m.from && Math.abs(this.robot.x - L) > CROSS_OUT + 0.3;
      if (across || this.clock - m.t > 20) this.crossMemo.delete(L);
    }
    this.writeInstances();
    this.updateTrails();
    this.updateBrainViz();
    this.updateCamera(dt);
    if (this.pulseT < 1) {
      this.pulseT = Math.min(1, this.pulseT + dt * 1.8);
      (this.pulse.material as THREE.MeshBasicMaterial).opacity = (1 - this.pulseT) * 0.9;
      this.pulse.scale.setScalar(1 + this.pulseT * 0.35);
    }
    // key light follows the robot so its shadows stay crisp
    const kl = this.viewer.keyLight;
    kl.position.set(this.robot.x + 2.4, 6.5, this.robot.z + 3.2);
    kl.target.position.set(this.robot.x, 0, this.robot.z);
    kl.target.updateMatrixWorld();
    this.statusClock += dt;
    if (this.statusClock > 0.12) { this.statusClock = 0; this.emit(false); }
    return true;
  }

  private updateBrainViz() {
    const r = this.robot;
    // planned path (rebuilt when the plan changes), drawn just above the carpet
    const order = r.done ? null : r.order;
    if (order !== this.pathRef) {
      this.pathRef = order;
      if (order) {
        const pts = [new THREE.Vector3(r.x, 0.02, r.z), ...order.path.map(([x, z]) => new THREE.Vector3(x, this.field.floor(x, z) + 0.02, z))];
        this.pathLine.geometry.dispose();
        this.pathLine.geometry = new THREE.BufferGeometry().setFromPoints(pts);
        this.pathLine.computeLineDistances();
      }
      this.pathLine.visible = !!order;
    }
    // the fuel Endless is going after
    const t = this.brain instanceof EndlessBrain ? this.brain.targetBall : -1;
    this.targetRing.visible = t >= 0 && this.balls.available(t);
    if (this.targetRing.visible) {
      const p = this.balls.pos;
      this.targetRing.position.set(p[t * 3], this.field.floor(p[t * 3], p[t * 3 + 2]) + 0.012, p[t * 3 + 2]);
      this.targetRing.scale.setScalar(1 + 0.12 * Math.sin(this.clock * 6));
    }
    // aim lines from each turret to the HUB while it has fuel to shoot
    const a = this.aimLines.geometry.getAttribute('position') as THREE.BufferAttribute;
    const ends = r.turretExits();
    ends.forEach((e, k) => { a.setXYZ(k * 2, e.x, e.y, e.z); a.setXYZ(k * 2 + 1, this.hubTarget[0], this.hubTarget[1], this.hubTarget[2]); });
    a.needsUpdate = true;
    this.aimLines.visible = r.stored.length > 0;
  }

  private updateCamera(dt: number) {
    // the user can orbit and zoom any time; the camera takes over again a few seconds later
    if (this.userActive) { this.userT = 0; return; }
    this.userT += dt;
    if (this.userT < USER_HOLD) return;
    if (this.autoOrbit && this.view === 'field') { this.viewer.controls.rotate(dt * 0.05, 0, false); return; }
    this.aimCamera(dt, true);
  }

  private writeInstances() {
    const m = new THREE.Matrix4();
    const hide = new THREE.Matrix4().makeScale(0, 0, 0);
    const { pos, st, n } = this.balls;
    for (let i = 0; i < n; i++) {
      if (st[i] === S.IN_HUB) this.inst.setMatrixAt(i, hide);
      else this.inst.setMatrixAt(i, m.makeTranslation(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]));
    }
    this.inst.instanceMatrix.needsUpdate = true;
  }

  private updateTrails() {
    const { pos, st } = this.balls;
    for (let i = 0; i < this.balls.n; i++) {
      const tr = this.trailPos.get(i);
      if (st[i] === S.FLYING) {
        const arr = tr ?? [];
        arr.push(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
        if (arr.length > 17 * 3) arr.splice(0, 3);
        this.trailPos.set(i, arr);
      } else if (tr) {
        tr.splice(0, 6); // fade out by shortening
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

  private emitLoading() {
    this.onStatus?.({ mode: this.mode, phase: 'Loading field…', scored: 0, stored: 0, capacity: 0, time: null, bps: 0, ended: false, loading: true });
  }

  private emit(force: boolean) {
    void force;
    this.onStatus?.(this.status());
  }

  status(): SimStatus {
    const now = this.clock;
    this.scoreTimes = this.scoreTimes.filter((t) => now - t < 3);
    const brain = this.brain;
    return {
      mode: this.mode,
      phase: brain?.phase ?? '',
      scored: this.balls.scored,
      stored: this.robot.load,
      capacity: this.robot.capacity,
      time: this.mode === 'auto' ? Math.max(0, AUTO_SECONDS - (brain as AutoBrain).t) : null,
      bps: this.scoreTimes.length / 3,
      ended: brain instanceof AutoBrain && brain.ended,
      loading: false,
    };
  }

  // ------------------------------------------------------------------ helpers for the brains
  get grid() { return this.field; }
  get hubCenter() { return this.hub; }
  plan(x: number, z: number, clear = CLEAR) { return this.field.plan(this.robot.x, this.robot.z, x, z, clear); }
  setIntake(down: boolean) { this.robot.intakeDown = down; }
  canShoot() { return this.robot.x < SHOOT_X; }
}

// ====================================================================== Auto
type Step =
  | { kind: 'drive'; path: [number, number][]; speed: number; heading?: number; stop?: boolean; label: string; shoot?: boolean }
  | { kind: 'shoot'; max: number; label: string }
  | { kind: 'intake'; down: boolean };

class AutoBrain {
  t = 0;
  ended = false;
  phase = 'Autonomous';
  private steps: Step[];
  private i = 0;
  private stepT = 0;

  constructor(private sim: Simulation) {
    const start = sim.robot;
    const sx = start.x, sz = start.z;
    // Scripted from the team's auto: start in the left TRENCH lane, intake up, facing the middle.
    // Every crossing goes under the trench; it never touches a bump.
    const tz = -sim.trenchZ;
    const shootAt: [number, number] = [-5.75, tz + 0.85];
    this.steps = [
      { kind: 'drive', label: 'Under the trench', path: [[sx, sz], [-2.45, tz]], speed: 3.4 },
      { kind: 'intake', down: true },
      { kind: 'drive', label: 'Into the fuel', path: [[-2.45, tz], [-1.4, -2.35], [-1.0, -1.88]], speed: 2.6 },
      { kind: 'drive', label: 'Sweeping the neutral zone', path: [[-1.0, -1.88], [1.0, -1.88]], speed: 1.25 },
      { kind: 'drive', label: 'Back under the trench', path: [[1.0, -1.88], [1.3, -2.6], [0.6, tz + 0.1], [-2.45, tz], [-4.9, tz], shootAt], speed: 3.6, stop: true, shoot: true },
      { kind: 'shoot', max: 3.5, label: 'Scoring' },
      { kind: 'drive', label: 'Under the trench', path: [shootAt, [-4.9, tz], [-2.45, tz]], speed: 3.6, shoot: true, heading: Math.PI / 2 },
      { kind: 'drive', label: 'Into the fuel', path: [[-2.45, tz], [-1.35, -2.35], [-1.0, -1.27]], speed: 2.6 },
      { kind: 'drive', label: 'Second sweep', path: [[-1.0, -1.27], [1.0, -1.27]], speed: 1.25 },
      { kind: 'drive', label: 'Back under the trench', path: [[1.0, -1.27], [1.35, -2.3], [0.6, tz + 0.1], [-2.45, tz], [-4.9, tz], shootAt], speed: 3.6, stop: true, shoot: true },
      { kind: 'shoot', max: 6, label: 'Scoring' },
    ];
    this.begin();
  }

  private begin() {
    const s = this.steps[this.i];
    this.stepT = 0;
    if (!s) return;
    if (s.kind === 'intake') { this.sim.setIntake(s.down); this.i++; this.begin(); return; }
    if (s.kind === 'drive') {
      this.phase = s.label;
      this.sim.robot.drive({ path: s.path, speed: s.speed, heading: s.heading ?? 'travel', stopAtEnd: !!s.stop });
    } else this.phase = s.label;
  }

  update(dt: number) {
    if (this.ended) { this.sim.robot.aimAndFire(dt, false); return; }
    this.t += dt;
    this.stepT += dt;
    const r = this.sim.robot;
    const s = this.steps[this.i];
    r.aimAndFire(dt, this.sim.canShoot() && (!s || s.kind === 'shoot' || (s.kind === 'drive' && !!s.shoot)));
    if (this.t >= AUTO_SECONDS) {
      this.ended = true;
      r.drive(null);
      this.phase = 'Auto complete';
      sound.play('score');
      return;
    }
    if (!s) return;
    if (s.kind === 'drive' && r.done) { this.i++; this.begin(); }
    else if (s.kind === 'shoot' && (r.stored.length === 0 || this.stepT > s.max)) { this.i++; this.begin(); }
  }
}

// ====================================================================== Endless
class EndlessBrain {
  phase = 'Endless';
  private mode: 'collect' | 'score' = 'score';
  private target = -1;
  get targetBall() { return this.mode === 'collect' ? this.target : -1; }
  private replan = 0;
  private progress = { x: 0, z: 0, t: 0, strikes: 0 };
  private blacklist = new Map<number, number>();
  private clock = 0;
  private sweeping = false;
  private idle = 0;

  constructor(private sim: Simulation) {
    this.progress.x = sim.robot.x; this.progress.z = sim.robot.z;
    this.toScore();
  }

  update(dt: number) {
    const sim = this.sim, r = sim.robot;
    this.clock += dt;
    this.replan -= dt;
    r.aimAndFire(dt, sim.canShoot());

    if (this.mode === 'score') {
      if (r.load === 0 && r.stored.length === 0) { this.toCollect(); return; }
      if (r.done) {
        // parked in the alliance zone: keep firing until empty, then go
        this.phase = 'Scoring';
        if (!sim.canShoot()) this.toScore();
      }
    } else {
      const full = r.load >= r.capacity - 1;
      if (full) { this.toScore(); return; }
      const t = this.target;
      const gone = t < 0 || !sim.balls.available(t);
      if (gone && r.load >= 10) { this.toScore(); return; }
      if (gone || this.replan <= 0 || (r.done && !this.sweeping)) this.pickTarget();
      else if (r.done && this.sweeping) this.pickTarget();
    }
    this.checkStuck(dt);
  }

  private toScore() {
    const sim = this.sim, r = sim.robot;
    this.mode = 'score';
    this.sweeping = false;
    this.phase = sim.canShoot() ? 'Scoring' : 'Heading to the HUB';
    // nearest open shooting spot in the alliance zone, 2–4.2 m from the HUB
    const h = sim.hubCenter;
    let best: [number, number] | null = null, bestD = Infinity;
    for (let x = -7.0; x <= -4.7; x += 0.35) for (let z = -3.3; z <= 3.3; z += 0.35) {
      const dh = Math.hypot(x - h.cx, z - h.cz);
      // room to turn with the intake out (half-diagonal 0.83 m)
      if (dh < 2.0 || dh > 4.0 || sim.grid.distAt(x, z) < 0.86) continue;
      const d = Math.hypot(x - r.x, z - r.z) + Math.abs(z) * 0.15;
      if (d < bestD) { bestD = d; best = [x, z]; }
    }
    const goal = best ?? [-5.6, 0.9];
    const route = sim.route(goal[0], goal[1]);
    if (route) {
      r.drive({ path: route.path, speed: VMAX * 0.82, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading });
      if (route.via && !sim.canShoot()) this.phase = `To the HUB, ${route.via}`;
    }
    this.replan = 6;
  }

  private toCollect() {
    this.mode = 'collect';
    this.target = -1;
    this.pickTarget();
  }

  /** Best fuel to go get: lots of fuel nearby, not too far, reachable with the intake leading. */
  private pickTarget() {
    const sim = this.sim, r = sim.robot, b = sim.balls, g = sim.grid;
    this.replan = 2.2;
    const now = this.clock;
    const cand: { i: number; d: number }[] = [];
    for (let i = 0; i < b.n; i++) {
      if (!b.available(i)) continue;
      const x = b.pos[i * 3], z = b.pos[i * 3 + 2];
      // anywhere on the field (fuel drifts toward the far side over time); dense, nearby fuel wins
      if (x > 7.7 || g.distAt(x, z) < 0.07) continue;
      if ((this.blacklist.get(i) ?? 0) > now) continue;
      cand.push({ i, d: Math.hypot(x - r.x, z - r.z) });
    }
    if (!cand.length) {
      this.phase = 'Waiting for fuel';
      if (r.load > 0) this.toScore();
      else if (r.done) { this.idle += 1; r.drive({ path: [[r.x, r.z], [Math.max(-6, Math.min(-1, r.x + 0.01)), r.z]], speed: 1, heading: 'travel', stopAtEnd: true }); }
      return;
    }
    cand.sort((p, q) => p.d - q.d);
    const top = cand.slice(0, 90);
    let best = top[0], bestScore = -Infinity;
    for (const c of top) {
      const x = b.pos[c.i * 3], z = b.pos[c.i * 3 + 2];
      let dens = 0;
      for (const o of top) { const dx = b.pos[o.i * 3] - x, dz = b.pos[o.i * 3 + 2] - z; if (dx * dx + dz * dz < 0.36) dens++; }
      const score = Math.pow(dens, 0.8) / (1 + c.d / 2.0);
      if (score > bestScore) { bestScore = score; best = c; }
    }
    this.target = best.i;
    this.phase = 'Collecting fuel';
    const bx = b.pos[best.i * 3], bz = b.pos[best.i * 3 + 2];
    // approach with the intake leading; drive straight at walls when fuel is against one
    let dx = bx - r.x, dz = bz - r.z;
    const dl = Math.hypot(dx, dz) || 1;
    dx /= dl; dz /= dl;
    if (g.distAt(bx, bz) < 0.6) {
      const [ax, az] = g.awayFromWalls(bx, bz);
      if (ax || az) { dx = -ax; dz = -az; }
    }
    // try the direct approach, then rotated ones
    for (const rot of [0, 0.5, -0.5, 1, -1, 1.6, -1.6]) {
      const c = Math.cos(rot), s = Math.sin(rot);
      const ux = dx * c - dz * s, uz = dx * s + dz * c;
      const ax = bx - ux * 0.95, az = bz - uz * 0.95;
      if (g.distAt(ax, az) < CLEAR) continue;
      const route = sim.route(ax, az);
      if (!route) continue;
      const path = route.path;
      this.phase = route.via ? `Collecting fuel, ${route.via}` : 'Collecting fuel';
      // sweep through the fuel; stop short of walls
      let run = 0.25;
      while (run < 1.0 && g.lineFree(ax, az, bx + ux * (run + 0.1), bz + uz * (run + 0.1), 0.43)) run += 0.1;
      path.push([bx + ux * run, bz + uz * run]);
      r.drive({ path, speed: 3.6, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading });
      this.sweeping = true;
      return;
    }
    // unreachable for now
    this.blacklist.set(best.i, now + 12);
    this.target = -1;
  }

  /** If the robot stops making progress, try something else (and back off if wedged). */
  private checkStuck(dt: number) {
    const r = this.sim.robot, p = this.progress;
    p.t += dt;
    if (p.t < 1.2) return;
    const moved = Math.hypot(r.x - p.x, r.z - p.z);
    p.t = 0; p.x = r.x; p.z = r.z;
    const holding = r.done && (this.mode === 'score' ? true : false);
    if (moved > 0.15 || holding || r.shooting) { p.strikes = 0; return; }
    p.strikes++;
    if (this.target >= 0) this.blacklist.set(this.target, this.clock + 10);
    if (p.strikes >= 2) {
      // wedged: back out in whichever direction opens up the most room, keeping the heading
      let best: [number, number] = [-Math.sin(r.th), -Math.cos(r.th)], bestC = -Infinity;
      for (let k = 0; k < 12; k++) {
        const a = (k / 12) * Math.PI * 2, dx = Math.sin(a), dz = Math.cos(a);
        const c = Math.min(r.clearance(r.x + dx * 0.25, r.z + dz * 0.25, r.th), r.clearance(r.x + dx * 0.6, r.z + dz * 0.6, r.th));
        if (c > bestC) { bestC = c; best = [dx, dz]; }
      }
      const bx = r.x + best[0] * 0.6, bz = r.z + best[1] * 0.6;
      r.drive({ path: [[r.x, r.z], [bx, bz]], speed: 1.5, heading: r.th, stopAtEnd: true });
      p.strikes = 0;
      return;
    }
    if (this.mode === 'collect') this.pickTarget(); else this.toScore();
  }
}
