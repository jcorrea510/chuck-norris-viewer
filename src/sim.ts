// Simulation mode: Chuck Norris on the 2026 REBUILT field, collecting FUEL from the neutral zone
// and scoring it in the HUB with both turrets tracking on their real turntable pivots.
//
// Field: AdvantageScope "2026 FRC Field" model (built by scripts/optimize-field.mjs).
// Coordinates: meters, +Y up, origin at field center. The red alliance wall is at −X.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import type { Viewer } from './viewer';
import { sound } from './sound';

interface FieldData {
  fuelRadius: number;
  hubs: { min: number[]; max: number[]; center: number[]; opening: number[] }[];
  fuel: number[][];
}

const G = 9.81;
const DEG = Math.PI / 180;
const FIELD_X = 8.27, FIELD_Z = 4.03;
const CAPACITY = 48;
const FIRE_INTERVAL = 0.2; // seconds between shots per turret

// Ball states
const GROUND = 0, ROLLING = 1, STORED = 2, FLYING = 3, IN_HUB = 4;

export interface SimStatus { scored: number; stored: number; phase: string; loading: boolean }

export class Simulation {
  private group = new THREE.Group();
  private data!: FieldData;
  private balls!: THREE.InstancedMesh;
  private n = 0;
  private pos!: Float32Array;
  private vel!: Float32Array;
  private st!: Uint8Array;
  private timer!: Float32Array;
  private legs: { curve: THREE.Curve<THREE.Vector3>; speed: number; phase: string; shoot: boolean }[] = [];
  private leg = 0;
  private legT = 0;
  private robot = new THREE.Vector3();
  private heading = 0;
  private stored = 8;
  private scored = 0;
  private fireClock = [0, 0.1];
  private hub!: { center: THREE.Vector3; exitX: number; min: THREE.Vector3; max: THREE.Vector3 };
  private pulse: THREE.Mesh | null = null;
  private pulseT = 1;
  private follow = true;
  private saved: { boundary: THREE.Box3 | null; maxDist: number; minDist: number; maxPolar: number; keyPos: THREE.Vector3; shadow: { l: number; r: number; t: number; b: number; far: number } } | null = null;
  private hook = (dt: number) => this.step(dt);
  private turrets: { id: string; pivot: THREE.Vector3; base: number }[] = [];
  onStatus: ((s: SimStatus) => void) | null = null;
  active = false;

  constructor(private viewer: Viewer) {}

  async start() {
    const v = this.viewer;
    this.onStatus?.({ scored: 0, stored: 0, phase: 'Loading field…', loading: true });
    const [gltf, data] = await Promise.all([
      new GLTFLoader().setMeshoptDecoder(MeshoptDecoder).loadAsync('field/field-2026.glb'),
      fetch('field/field-2026.json').then((r) => r.json() as Promise<FieldData>),
    ]);
    this.data = data;
    const field = gltf.scene;
    field.traverse((o) => {
      const m = o as THREE.Mesh;
      if (!m.isMesh) return;
      m.receiveShadow = true;
      m.castShadow = true;
      const mat = m.material as THREE.MeshStandardMaterial;
      if (mat.transparent) { m.castShadow = false; mat.depthWrite = false; }
      mat.envMapIntensity = 0.8;
    });
    this.group.add(field);

    // Red alliance HUB (−X): FUEL leaves it through the low exit facing the neutral zone (+X).
    const h = [...data.hubs].sort((a, b) => a.center[0] - b.center[0])[0];
    this.hub = {
      center: new THREE.Vector3().fromArray(h.center),
      exitX: h.max[0] + 0.12,
      min: new THREE.Vector3().fromArray(h.min),
      max: new THREE.Vector3().fromArray(h.max),
    };
    this.pulse = new THREE.Mesh(
      new THREE.TorusGeometry(0.55, 0.02, 12, 64).rotateX(Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0xffb54a, transparent: true, opacity: 0, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.pulse.position.copy(this.hub.center).add(new THREE.Vector3(0, 0.02, 0));
    this.group.add(this.pulse);

    // FUEL: one instanced sphere per piece (a 5.91" foam ball is a sphere; 336 tris each).
    const r = data.fuelRadius;
    const geo = new THREE.SphereGeometry(r, 16, 12);
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(0.96, 0.6, 0.06), roughness: 0.75, metalness: 0 });
    this.n = data.fuel.length;
    this.balls = new THREE.InstancedMesh(geo, mat, this.n);
    this.balls.castShadow = true;
    this.balls.receiveShadow = true;
    this.balls.frustumCulled = false;
    this.pos = new Float32Array(this.n * 3);
    this.vel = new Float32Array(this.n * 3);
    this.st = new Uint8Array(this.n);
    this.timer = new Float32Array(this.n);
    this.group.add(this.balls);

    // Turrets: pivot (robot frame) and the shooting heading at 0° (from the CAD flywheel shaft).
    this.turrets = [];
    for (const id of ['turret-left', 'turret-right']) {
      const m = v.mechanisms.get(id);
      if (!m) continue;
      const p = new THREE.Vector3();
      m.node.getWorldPosition(p);
      v.stage.worldToLocal(p);
      this.turrets.push({ id, pivot: p, base: Math.atan2(Math.sign(p.x) * 0.8, -0.6) });
    }

    this.buildRoute();
    this.reset();

    // Take over the stage
    this.saved = {
      boundary: null,
      maxDist: v.controls.maxDistance, minDist: v.controls.minDistance, maxPolar: v.controls.maxPolarAngle,
      keyPos: v.keyLight.position.clone(),
      shadow: { l: v.keyLight.shadow.camera.left, r: v.keyLight.shadow.camera.right, t: v.keyLight.shadow.camera.top, b: v.keyLight.shadow.camera.bottom, far: v.keyLight.shadow.camera.far },
    };
    v.simActive = true;
    v.spinning = false;
    v.setExploded(false);
    for (const m of v.mechanisms.values()) v.setMechanism(m.id, m.def, false);
    v.scene.add(this.group);
    v.grid.visible = false;
    const sc = v.keyLight.shadow.camera;
    sc.left = sc.bottom = -4; sc.right = sc.top = 4; sc.far = 14;
    sc.updateProjectionMatrix();
    v.controls.setBoundary(new THREE.Box3(new THREE.Vector3(-12, 0, -7), new THREE.Vector3(12, 6, 7)));
    v.controls.maxDistance = 22;
    v.controls.minDistance = 0.6;
    v.controls.maxPolarAngle = 86 * DEG;
    v.controls.setFocalOffset(0, 0, 0, false);
    this.placeRobot();
    this.setView('follow', false);
    v.hooks.add(this.hook);
    this.active = true;
    sound.play('boot');
    this.emit();
  }

  stop() {
    const v = this.viewer;
    if (!this.saved) return;
    v.hooks.delete(this.hook);
    v.scene.remove(this.group);
    v.grid.visible = true;
    v.simActive = false;
    v.stage.position.set(0, 0, 0);
    v.stage.rotation.set(0, 0, 0);
    v.stage.updateMatrixWorld(true);
    v.keyLight.position.copy(this.saved.keyPos);
    v.keyLight.target.position.set(0, 0.25, 0);
    const sc = v.keyLight.shadow.camera;
    sc.left = this.saved.shadow.l; sc.right = this.saved.shadow.r; sc.top = this.saved.shadow.t; sc.bottom = this.saved.shadow.b; sc.far = this.saved.shadow.far;
    sc.updateProjectionMatrix();
    v.controls.maxDistance = this.saved.maxDist;
    v.controls.minDistance = this.saved.minDist;
    v.controls.maxPolarAngle = this.saved.maxPolar;
    for (const m of v.mechanisms.values()) v.setMechanism(m.id, m.def, true);
    this.active = false;
    this.saved = null;
    v.invalidate(true);
  }

  restart() { this.reset(); sound.play('select'); }

  setView(mode: 'follow' | 'field', animate = true) {
    const c = this.viewer.controls;
    this.follow = mode === 'follow';
    if (this.follow) {
      c.moveTo(this.robot.x, 0.35, this.robot.z, animate);
      c.dollyTo(4.2, animate);
      c.rotateTo(this.heading + Math.PI + 0.5, 58 * DEG, animate);
    } else {
      c.moveTo(0, 0, 0, animate);
      c.dollyTo(17, animate);
      c.rotateTo(-Math.PI / 2 + 0.0001, 40 * DEG, animate);
    }
  }
  get view() { return this.follow ? 'follow' : 'field'; }

  // ------------------------------------------------------------------ route
  private buildRoute() {
    const V = (x: number, z: number) => new THREE.Vector3(x, 0, z);
    const curve = (pts: THREE.Vector3[]) => new THREE.CatmullRomCurve3(pts, false, 'centripetal', 0.5);
    // Red alliance zone is x < −4.24; trenches run along the walls at |z| ≈ 3.3 (robot fits under the arm).
    this.legs = [
      { curve: curve([V(-6.2, 1.2), V(-5.6, 2.3), V(-4.9, 3.25), V(-3.6, 3.3), V(-2.3, 3.15), V(-1.5, 2.25)]), speed: 3.2, phase: 'Under the trench', shoot: true },
      { curve: curve([V(-1.5, 2.25), V(-0.4, 1.95), V(0.6, 1.95), V(1.5, 1.7), V(1.8, 1.1), V(1.2, 0.55), V(0.2, 0.5), V(-0.8, 0.5), V(-1.6, 0.25)]), speed: 1.8, phase: 'Collecting fuel', shoot: false },
      { curve: curve([V(-1.6, 0.25), V(-2.2, 1.6), V(-2.6, 3.2), V(-3.6, 3.32), V(-4.8, 3.2), V(-5.5, 2.2), V(-5.6, 1.0), V(-5.2, -0.6)]), speed: 3.0, phase: 'Scoring on the move', shoot: true },
      { curve: curve([V(-5.2, -0.6), V(-5.0, -2.2), V(-4.6, -3.25), V(-3.6, -3.3), V(-2.4, -3.1), V(-1.5, -2.2)]), speed: 3.2, phase: 'Under the trench', shoot: true },
      { curve: curve([V(-1.5, -2.2), V(-0.4, -1.95), V(0.6, -1.95), V(1.5, -1.7), V(1.8, -1.1), V(1.2, -0.6), V(0.2, -0.5), V(-0.8, -0.55), V(-1.6, -0.3)]), speed: 1.8, phase: 'Collecting fuel', shoot: false },
      { curve: curve([V(-1.6, -0.3), V(-2.2, -1.6), V(-2.6, -3.2), V(-3.6, -3.32), V(-4.8, -3.2), V(-5.6, -2.0), V(-6.0, -0.2), V(-6.2, 1.2)]), speed: 3.0, phase: 'Scoring on the move', shoot: true },
    ];
  }

  private reset() {
    const d = this.data;
    for (let i = 0; i < this.n; i++) {
      this.pos.set(d.fuel[i], i * 3);
      this.vel.fill(0, i * 3, i * 3 + 3);
      this.st[i] = GROUND;
    }
    // Preload: 8 pieces from the alliance depot start in the robot.
    let pre = 0;
    for (let i = 0; i < this.n && pre < 8; i++) if (this.pos[i * 3] < -7 && Math.abs(this.pos[i * 3 + 2]) < 2.6) { this.st[i] = STORED; pre++; }
    this.stored = pre;
    this.scored = 0;
    this.leg = 0;
    this.legT = 0;
    const p0 = this.legs[0].curve.getPointAt(0);
    this.robot.copy(p0);
    const t0 = this.legs[0].curve.getTangentAt(0);
    this.heading = Math.atan2(t0.x, t0.z);
    this.writeInstances();
    this.placeRobot();
    this.emit();
  }

  // ------------------------------------------------------------------ step
  private step(dt: number): boolean {
    if (!this.active) return false;
    dt = Math.min(dt, 1 / 30);
    this.drive(dt);
    this.aimAndShoot(dt);
    this.physics(dt);
    this.writeInstances();
    if (this.pulse && this.pulseT < 1) {
      this.pulseT = Math.min(1, this.pulseT + dt * 1.8);
      const m = this.pulse.material as THREE.MeshBasicMaterial;
      m.opacity = (1 - this.pulseT) * 0.9;
      this.pulse.scale.setScalar(1 + this.pulseT * 0.35);
    }
    return true;
  }

  private drive(dt: number) {
    const leg = this.legs[this.leg];
    const len = leg.curve.getLength();
    // ease in/out at the ends of each leg
    const u = this.legT;
    const ramp = Math.min(1, u * len / 0.8 + 0.15, (1 - u) * len / 0.8 + 0.15);
    this.legT += (leg.speed * ramp * dt) / len;
    if (this.legT >= 1) {
      this.legT = 0;
      this.leg = (this.leg + 1) % this.legs.length;
      this.emit();
    }
    const cur = this.legs[this.leg];
    const p = cur.curve.getPointAt(Math.min(1, this.legT));
    const tan = cur.curve.getTangentAt(Math.min(1, this.legT));
    const prev = this.robot.clone();
    this.robot.copy(p);
    // Swerve: the intake leads while collecting; elsewhere the chassis turns smoothly toward travel.
    const want = Math.atan2(tan.x, tan.z);
    let dh = want - this.heading;
    dh = Math.atan2(Math.sin(dh), Math.cos(dh));
    this.heading += dh * Math.min(1, dt * (cur.shoot ? 3 : 6));
    this.placeRobot();
    if (this.follow) {
      const c = this.viewer.controls;
      const t = new THREE.Vector3();
      c.getTarget(t);
      c.moveTo(t.x + (this.robot.x - prev.x), 0.35, t.z + (this.robot.z - prev.z), false);
    }
  }

  private placeRobot() {
    const v = this.viewer;
    v.stage.position.set(this.robot.x, 0, this.robot.z);
    v.stage.rotation.set(0, this.heading, 0);
    v.stage.updateMatrixWorld(true);
    v.keyLight.position.set(this.robot.x + 2.5, 6, this.robot.z + 3);
    v.keyLight.target.position.set(this.robot.x, 0, this.robot.z);
    v.keyLight.target.updateMatrixWorld();
  }

  private toRobotLocal(x: number, z: number) {
    const dx = x - this.robot.x, dz = z - this.robot.z;
    const yaw = this.heading + this.viewer.root.rotation.y;
    const c = Math.cos(-yaw), s = Math.sin(-yaw);
    return { x: dx * c + dz * s, z: -dx * s + dz * c };
  }

  private aimAndShoot(dt: number) {
    const v = this.viewer;
    const yaw = this.heading + v.root.rotation.y;
    const inZone = this.robot.x < -4.3;
    const hub = this.hub.center;
    this.turrets.forEach((t, i) => {
      const pw = t.pivot.clone().applyMatrix4(v.stage.matrixWorld);
      const want = Math.atan2(hub.x - pw.x, hub.z - pw.z) - yaw;
      let a = want - t.base;
      a = Math.atan2(Math.sin(a), Math.cos(a));
      v.setMechanism(t.id, a / DEG, true);
      const err = Math.abs(Math.atan2(Math.sin(a - v.mechanismValue(t.id) * DEG), Math.cos(a - v.mechanismValue(t.id) * DEG)));
      this.fireClock[i] -= dt;
      if (inZone && this.stored > 0 && err < 6 * DEG && this.fireClock[i] <= 0 && this.legs[this.leg].shoot) {
        this.fireClock[i] = FIRE_INTERVAL;
        this.fire(pw, yaw + t.base + v.mechanismValue(t.id) * DEG);
      }
    });
  }

  private fire(pivotWorld: THREE.Vector3, dir: number) {
    let i = -1;
    for (let k = 0; k < this.n; k++) if (this.st[k] === STORED) { i = k; break; }
    if (i < 0) return;
    const p0 = pivotWorld.clone().add(new THREE.Vector3(Math.sin(dir) * 0.12, 0.2, Math.cos(dir) * 0.12));
    const target = this.hub.center.clone().add(new THREE.Vector3((Math.random() - 0.5) * 0.45, 0.05, (Math.random() - 0.5) * 0.45));
    const T = THREE.MathUtils.clamp(p0.distanceTo(target) / 7.5, 0.55, 1.25);
    // v = (target − p0 − ½·g·T²) / T
    const v0 = target.clone().sub(p0).add(new THREE.Vector3(0, 0.5 * G * T * T, 0)).divideScalar(T);
    // launching from a moving robot: the robot's motion carries the ball, so compensate (shoot on the move)
    this.pos.set([p0.x, p0.y, p0.z], i * 3);
    this.vel.set([v0.x, v0.y, v0.z], i * 3);
    this.st[i] = FLYING;
    this.stored--;
    sound.play('shot');
    this.emit();
  }

  private physics(dt: number) {
    const r = this.data.fuelRadius;
    const hub = this.hub;
    const ox = hub.center.x, oz = hub.center.z;
    const halfX = this.data.hubs[0].opening[0] / 2 - 0.08, halfZ = this.data.hubs[0].opening[1] / 2 - 0.08;
    for (let i = 0; i < this.n; i++) {
      const s = this.st[i];
      if (s === STORED || s === GROUND && !this.nearRobot(i)) continue;
      const o = i * 3;
      if (s === IN_HUB) {
        this.timer[i] -= dt;
        if (this.timer[i] <= 0) {
          // out through the low exit, rolling into the neutral zone
          this.pos.set([hub.exitX, 0.55, oz + (Math.random() - 0.5) * 0.9], o);
          this.vel.set([1.6 + Math.random() * 1.4, 0.4, (Math.random() - 0.5) * 1.2], o);
          this.st[i] = ROLLING;
        }
        continue;
      }
      if (s === GROUND) {
        // robot pushes or collects resting fuel
        this.contactRobot(i);
        continue;
      }
      // integrate
      this.vel[o + 1] -= G * dt;
      this.pos[o] += this.vel[o] * dt;
      this.pos[o + 1] += this.vel[o + 1] * dt;
      this.pos[o + 2] += this.vel[o + 2] * dt;
      if (s === FLYING) {
        const y = this.pos[o + 1];
        if (this.vel[o + 1] < 0 && y < hub.center.y - 0.05 && y > hub.center.y - 0.6 &&
          Math.abs(this.pos[o] - ox) < halfX && Math.abs(this.pos[o + 2] - oz) < halfZ) {
          this.st[i] = IN_HUB;
          this.timer[i] = 0.7 + Math.random() * 0.8;
          this.scored++;
          this.pulseT = 0;
          sound.play('score');
          this.emit();
          continue;
        }
      }
      // floor
      if (this.pos[o + 1] < r) {
        this.pos[o + 1] = r;
        if (this.vel[o + 1] < -0.6) { this.vel[o + 1] *= -0.35; this.st[i] = ROLLING; }
        else this.vel[o + 1] = 0;
        const f = Math.exp(-1.6 * dt);
        this.vel[o] *= f; this.vel[o + 2] *= f;
        if (Math.hypot(this.vel[o], this.vel[o + 1], this.vel[o + 2]) < 0.05) { this.vel.fill(0, o, o + 3); this.st[i] = GROUND; }
      }
      // walls
      if (Math.abs(this.pos[o]) > FIELD_X - r) { this.pos[o] = Math.sign(this.pos[o]) * (FIELD_X - r); this.vel[o] *= -0.4; }
      if (Math.abs(this.pos[o + 2]) > FIELD_Z - r) { this.pos[o + 2] = Math.sign(this.pos[o + 2]) * (FIELD_Z - r); this.vel[o + 2] *= -0.4; }
      if (this.st[i] !== FLYING) this.contactRobot(i);
    }
  }

  private nearRobot(i: number) {
    const dx = this.pos[i * 3] - this.robot.x, dz = this.pos[i * 3 + 2] - this.robot.z;
    return dx * dx + dz * dz < 1.2;
  }

  /** Intake collects fuel touching the front rollers; the bumpers push everything else aside. */
  private contactRobot(i: number) {
    const o = i * 3;
    if (this.pos[o + 1] > 0.35) return;
    const l = this.toRobotLocal(this.pos[o], this.pos[o + 2]);
    if (l.z > 0.38 && l.z < 0.72 && Math.abs(l.x) < 0.32 && this.legs[this.leg].phase === 'Collecting fuel' && this.stored < CAPACITY) {
      this.st[i] = STORED;
      this.stored++;
      if (this.stored % 3 === 0) sound.play('intake');
      this.emit();
      return;
    }
    const hx = 0.47, hz = l.z > 0 ? 0.74 : 0.47;
    if (Math.abs(l.x) < hx && l.z > -0.47 && l.z < hz) {
      // push out along the shallowest side
      const yaw = this.heading + this.viewer.root.rotation.y;
      const pushX = Math.abs(l.x) - hx, pushZ = l.z > 0 ? l.z - hz : -0.47 - l.z;
      let nx = 0, nz = 0;
      if (pushX > pushZ) nx = Math.sign(l.x) || 1; else nz = l.z > 0 ? 1 : -1;
      const wx = nx * Math.cos(yaw) + nz * Math.sin(yaw), wz = -nx * Math.sin(yaw) + nz * Math.cos(yaw);
      this.vel[o] = wx * 1.2; this.vel[o + 2] = wz * 1.2;
      this.pos[o] += wx * 0.03; this.pos[o + 2] += wz * 0.03;
      this.st[i] = ROLLING;
    }
  }

  private writeInstances() {
    const m = new THREE.Matrix4();
    const hide = new THREE.Matrix4().makeScale(0, 0, 0);
    for (let i = 0; i < this.n; i++) {
      const s = this.st[i];
      if (s === STORED || s === IN_HUB) this.balls.setMatrixAt(i, hide);
      else this.balls.setMatrixAt(i, m.makeTranslation(this.pos[i * 3], this.pos[i * 3 + 1], this.pos[i * 3 + 2]));
    }
    this.balls.instanceMatrix.needsUpdate = true;
  }

  private emit() {
    this.onStatus?.({ scored: this.scored, stored: this.stored, phase: this.legs[this.leg]?.phase ?? '', loading: false });
  }
}
