// Simulation mode: Chuck Norris on the 2026 REBUILT field.
//   Auto    — the team's autonomous: start against the left BUMP, intake up, facing the middle;
//             drop the intake, sweep the neutral zone, come back and shoot, sweep again, shoot.
//   Endless — keeps finding fuel wherever it is (scored fuel pours out of the HUB into the
//             neutral zone), collects it and scores it, forever.
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
export type SimView = 'follow' | 'field' | 'driver';
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
  private scoreTimes: number[] = [];
  private clock = 0;
  private statusClock = 0;
  mode: SimMode = 'auto';
  view: SimView = 'follow';
  autoOrbit = false;
  active = false;
  paused = false;
  onStatus: ((s: SimStatus) => void) | null = null;

  constructor(private viewer: Viewer) {}

  async start(mode: SimMode = 'auto') {
    const v = this.viewer;
    this.emitLoading();
    const [gltf, data] = await Promise.all([
      new GLTFLoader().setMeshoptDecoder(MeshoptDecoder).loadAsync('field/field-2026.glb'),
      fetch('field/field-2026.json').then((r) => r.json() as Promise<FieldData>),
    ]);
    this.data = data;
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
    const geo = new THREE.SphereGeometry(R, 18, 14);
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
      dollyToCursor: v.controls.dollyToCursor,
    };
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
    if (mode === 'auto') {
      this.robot.setIntakeNow(false);
      this.robot.place(ALLIANCE_LINE - 0.445 - 0.03, -1.52, Math.PI / 2);
      this.brain = new AutoBrain(this);
    } else {
      this.robot.setIntakeNow(true);
      this.robot.place(-5.6, 1.0, Math.PI / 2);
      this.brain = new EndlessBrain(this);
    }
    this.robot.preload(pre);
    this.writeInstances();
    this.setView(this.view, false);
    this.emit(true);
  }

  restart() { this.setMode(this.mode); sound.play('select'); }

  stop() {
    const v = this.viewer;
    if (!this.saved) return;
    v.hooks.delete(this.hook);
    v.scene.remove(this.group);
    v.grid.visible = true;
    v.simActive = false;
    const s = this.saved as { maxDist: number; minDist: number; maxPolar: number; keyPos: THREE.Vector3; shadow: number[]; rootPos: THREE.Vector3; dollyToCursor: boolean };
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
    const c = this.viewer.controls, r = this.robot;
    if (view === 'follow') {
      c.moveTo(r.x, 0.35, r.z, animate);
      c.dollyTo(5.6, animate);
      c.rotateTo(r.th + Math.PI + 0.5, 46 * DEG, animate);
    } else if (view === 'field') {
      c.moveTo(-1.4, 0, 0, animate);
      c.dollyTo(15.5, animate);
      c.rotateTo(-Math.PI / 2 + 0.0001, 44 * DEG, animate);
    } else {
      // the red driver station: what the drive team actually sees
      c.setLookAt(-9.4, 2.6, -2.9, -1.6, 0.2, 0.4, animate);
    }
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

  private lastRobot = new THREE.Vector3();
  private updateCamera(dt: number) {
    const c = this.viewer.controls, r = this.robot;
    if (this.view === 'follow') {
      const t = new THREE.Vector3();
      c.getTarget(t);
      // glide with the robot; the user can still orbit and zoom
      const k = 1 - Math.exp(-dt * 6);
      c.moveTo(t.x + (r.x - t.x) * k, 0.35, t.z + (r.z - t.z) * k, false);
      if (this.autoOrbit) c.rotate(dt * 0.18, 0, false);
    } else if (this.view === 'field' && this.autoOrbit) c.rotate(dt * 0.05, 0, false);
    this.lastRobot.set(r.x, 0, r.z);
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
    const now = this.clock;
    this.scoreTimes = this.scoreTimes.filter((t) => now - t < 3);
    const brain = this.brain;
    this.onStatus?.({
      mode: this.mode,
      phase: brain?.phase ?? '',
      scored: this.balls.scored,
      stored: this.robot.load,
      capacity: this.robot.capacity,
      time: this.mode === 'auto' ? Math.max(0, AUTO_SECONDS - (brain as AutoBrain).t) : null,
      bps: this.scoreTimes.length / 3,
      ended: brain instanceof AutoBrain && brain.ended,
      loading: false,
    });
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
    // Scripted from the team's auto: left BUMP start, intake up, facing the middle.
    this.steps = [
      { kind: 'intake', down: true },
      { kind: 'drive', label: 'Over the bump', path: [[sx, sz], [-3.65, -1.52], [-2.45, -1.55], [-1.5, -1.72]], speed: 3.2 },
      { kind: 'drive', label: 'Sweeping the neutral zone', path: [[-1.5, -1.72], [1.0, -1.72], [1.45, -1.35], [1.0, -0.98], [-1.35, -0.98]], speed: 2.3 },
      { kind: 'drive', label: 'Back to the HUB', path: [[-1.35, -0.98], [-2.5, -1.45], [-3.65, -1.52], [-4.9, -1.45], [-5.45, -1.05]], speed: 3.4, stop: true, shoot: true },
      { kind: 'shoot', max: 3.2, label: 'Scoring' },
      { kind: 'drive', label: 'Over the bump', path: [[-5.45, -1.05], [-4.8, -1.48], [-3.65, -1.52], [-2.45, -1.5], [-1.5, -0.35]], speed: 3.4, shoot: true },
      { kind: 'drive', label: 'Second sweep', path: [[-1.5, -0.35], [1.0, -0.35], [1.42, 0.02], [1.0, 0.4], [-1.3, 0.4]], speed: 2.3 },
      { kind: 'drive', label: 'Back to the HUB', path: [[-1.3, 0.4], [-2.4, -1.2], [-3.65, -1.52], [-4.9, -1.45], [-5.45, -1.05]], speed: 3.4, stop: true, shoot: true },
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
      if (dh < 2.0 || dh > 4.0 || sim.grid.distAt(x, z) < 0.62) continue;
      const d = Math.hypot(x - r.x, z - r.z) + Math.abs(z) * 0.15;
      if (d < bestD) { bestD = d; best = [x, z]; }
    }
    const goal = best ?? [-5.6, 0.9];
    const path = sim.plan(goal[0], goal[1]);
    if (path) r.drive({ path, speed: VMAX * 0.82, heading: 'travel', stopAtEnd: true });
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
      const path = sim.plan(ax, az);
      if (!path) continue;
      // sweep through the fuel; stop short of walls
      let run = 0.25;
      while (run < 1.0 && g.lineFree(ax, az, bx + ux * (run + 0.1), bz + uz * (run + 0.1), 0.43)) run += 0.1;
      path.push([bx + ux * run, bz + uz * run]);
      r.drive({ path, speed: 3.6, heading: 'travel', stopAtEnd: true });
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
    if (p.strikes >= 3) {
      // back away from whatever we're wedged against
      const [ax, az] = this.sim.grid.awayFromWalls(r.x, r.z);
      const bx = r.x + (ax || -Math.sin(r.th)) * 0.7, bz = r.z + (az || -Math.cos(r.th)) * 0.7;
      r.drive({ path: [[r.x, r.z], [bx, bz]], speed: 1.5, heading: r.th, stopAtEnd: true });
      p.strikes = 0;
      return;
    }
    if (this.mode === 'collect') this.pickTarget(); else this.toScore();
  }
}
