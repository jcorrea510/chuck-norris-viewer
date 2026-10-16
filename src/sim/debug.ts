// ?debug=physics: frame and physics times, awake bodies, every collider as a wireframe, the robot's
// contact points (yellow), and the velocity (white), drag (red) and Magnus (blue) vectors of each
// ball in the air.
import * as THREE from 'three';
import type { Frame } from './backend';
import { BS } from './phys/fuel';

export class DebugOverlay {
  private el: HTMLElement;
  private lines: THREE.LineSegments;
  private vec: THREE.LineSegments;
  private contacts: THREE.Points;
  private worstFrame = 0;
  private t = 0;
  private hist: number[] = [];

  constructor(private parent: THREE.Group) {
    this.el = document.createElement('div');
    this.el.className = 'sim-debug';
    this.el.style.cssText = 'position:absolute;left:12px;bottom:12px;z-index:50;font:12px/1.4 ui-monospace,monospace;color:#cfe;background:rgba(0,0,0,.6);padding:8px 10px;border-radius:8px;pointer-events:none;white-space:pre';
    document.getElementById('ui')?.append(this.el);
    this.lines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.55, depthTest: false }));
    this.lines.frustumCulled = false;
    this.lines.renderOrder = 100;
    parent.add(this.lines);
    this.vec = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false }));
    this.vec.frustumCulled = false;
    this.vec.renderOrder = 101;
    parent.add(this.vec);
    this.contacts = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xffe14a, size: 7, sizeAttenuation: false, depthTest: false }));
    this.contacts.frustumCulled = false;
    this.contacts.renderOrder = 102;
    parent.add(this.contacts);
  }

  update(fr: Frame, frameMs: number) {
    const { physMs, steps, alpha } = fr;
    this.hist.push(frameMs);
    if (this.hist.length > 120) this.hist.shift();
    this.worstFrame = Math.max(...this.hist);
    this.t++;
    // colliders (Rapier's own debug renderer), refreshed a few times a second
    if (fr.debug) {
      const g = this.lines.geometry;
      g.setAttribute('position', new THREE.BufferAttribute(fr.debug.vertices, 3));
      g.setAttribute('color', new THREE.BufferAttribute(fr.debug.colors, 4));
    }
    if (fr.debug) this.contacts.geometry.setAttribute('position', new THREE.BufferAttribute(fr.debug.contacts, 3));
    const f = fr.fuel;
    const pos: number[] = [], col: number[] = [];
    const seg = (o: number, v: Float32Array, k: number, c: number[]) => {
      pos.push(f.pos[o], f.pos[o + 1], f.pos[o + 2], f.pos[o] + v[o] * k, f.pos[o + 1] + v[o + 1] * k, f.pos[o + 2] + v[o + 2] * k);
      col.push(...c, ...c);
    };
    for (let i = 0; i < f.n; i++) {
      if (f.st[i] !== BS.FLYING && !(f.st[i] === BS.FREE && f.pos[i * 3 + 1] > 0.3 && Math.hypot(f.vel[i * 3], f.vel[i * 3 + 1], f.vel[i * 3 + 2]) > 0.5)) continue;
      const o = i * 3;
      seg(o, f.vel, 0.06, [1, 1, 1]);
      seg(o, f.dragF, 0.25, [1, 0.25, 0.2]);
      seg(o, f.liftF, 0.25, [0.3, 0.6, 1]);
    }
    const g = this.vec.geometry;
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    const r = fr.robot;
    if (this.t % 6 === 0) {
      this.el.textContent = [
        `frame ${frameMs.toFixed(1)} ms (worst ${this.worstFrame.toFixed(1)} of last 120)`,
        `physics ${physMs.toFixed(2)} ms / ${steps} steps @120 Hz (worker) · interp ${alpha.toFixed(2)}`,
        `awake fuel ${f.awake} / ${f.n} · planner worst ${fr.plannerWorst.toFixed(1)} ms`,
        `robot ${r.speed.toFixed(2)} m/s · battery ${r.battery.toFixed(2)} V · ${r.totalCurrent.toFixed(0)} A${r.brownout ? ' BROWNOUT' : ''}`,
        `wheels N ${r.modules.map((m) => m.N.toFixed(0)).join('/')}${r.slipping ? ' slip' : ''} · pitch ${(r.pitch * 57.3).toFixed(1)}° roll ${(r.roll * 57.3).toFixed(1)}°`,
        `flywheels ${r.turrets.map((t) => `${(t.fly * 9.549).toFixed(0)}/${(t.flyGoal * 9.549).toFixed(0)} rpm`).join(' · ')}${r.ferry ? ' · FERRY' : ''}`,
      ].join('\n');
    }
  }

  dispose() { this.el.remove(); this.parent.remove(this.lines); this.parent.remove(this.vec); this.parent.remove(this.contacts); }
}
