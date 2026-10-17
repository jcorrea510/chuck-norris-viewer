// Robot-mounted and broadcast camera POVs (internal field frame, before the alliance flip).
// Every input is the interpolated pose of the frame being drawn, so a rigid mount never jitters;
// smoothing is frame-rate independent exponential damping (never overshoots).
import * as THREE from 'three';
import type { FieldGrid } from './field';
import { TURRET } from './phys/config';

const DEG = Math.PI / 180;
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
const damp = (k: number, dt: number) => (dt <= 0 ? 1 : 1 - Math.exp(-dt / k));

export interface PovIn {
  /** interpolated robot pose: position (m) and heading (rad, atan2(front.x, front.z)) */
  x: number; z: number; yaw: number;
  /** left turret azimuth (deg, WPILib robot-relative, unwrapped) */
  turretAz: number;
  aspect: number;
  field: FieldGrid;
  /** the minimum height a camera must keep at (x, z) to stay above the field elements */
  clear: (x: number, z: number) => number;
}
export interface PovOut { pos: THREE.Vector3; tgt: THREE.Vector3; fov: number; near: number }
export type PovId = 'robot' | 'turret' | 'broadcast' | 'chase';

// the left turret's mount in the robot frame (+z front, +x left): WPILib (x forward, y left)
const [TURRET_FWD, TURRET_LEFT] = TURRET.mountsWpi[0];
/** the robot's field limits for a camera at bumper height (field half-length/width minus the wall) */
const WALL_X = 8.1, WALL_Z = 3.95;

export class PovRig {
  private yaw = 0; private aim = 0; private bx = 0; private bz = 0; private tx = 0; private dist = 2.2; private y = 1;
  private init = false;
  readonly out: PovOut = { pos: new THREE.Vector3(), tgt: new THREE.Vector3(), fov: 55, near: 0.05 };

  /** an angle chasing its target: exponential damping, capped at `rate` rad/s so a quick slew sweeps the camera instead of whipping it */
  private step(a: number, target: number, k: number, rate: number, dt: number) {
    const d = wrap(target - a) * damp(k, dt);
    const lim = dt <= 0 ? Infinity : rate * dt;
    return a + Math.max(-lim, Math.min(lim, d));
  }

  /** forget smoothed state (a new run, or a different alliance) */
  reset() { this.init = false; }

  solve(id: PovId, i: PovIn, dt: number): PovOut {
    const o = this.out;
    if (!this.init) { this.yaw = i.yaw; this.aim = i.yaw + i.turretAz * DEG; this.bx = i.x; this.bz = i.z; this.tx = i.x; this.dist = 2.2; this.y = 0; this.init = true; dt = 0; }
    const sy = Math.sin(i.yaw), cy = Math.cos(i.yaw);
    // robot frame → field: +z front (sy, cy), +x left (cy, −sy)
    const at = (lx: number, lz: number, y: number, out: THREE.Vector3) => out.set(i.x + lx * cy + lz * sy, y, i.z - lx * sy + lz * cy);
    o.near = 0.05;
    if (id === 'robot') {
      // a driver cam just above the intake roller, outside the cage, looking out
      at(0, 0.78, 0.55, o.pos);
      at(0, 4, 0.55 - 0.45, o.tgt);
      o.fov = i.aspect < 1 ? 78 : 68; o.near = 0.03;
    } else if (id === 'turret') {
      // over the left turret and behind it, along its aim (smoothed: the turret slews quickly)
      const tgtAim = i.yaw + i.turretAz * DEG;
      this.aim = this.step(this.aim, tgtAim, 0.3, 3, dt);
      const mx = i.x + TURRET_LEFT * cy + TURRET_FWD * sy, mz = i.z - TURRET_LEFT * sy + TURRET_FWD * cy;
      const dx = Math.sin(this.aim), dz = Math.cos(this.aim);
      const px = mx - dx * 0.55, pz = mz - dz * 0.55;
      this.y += (Math.max(1.3, i.clear(px, pz)) - this.y) * (this.y === 0 ? 1 : damp(0.25, dt));
      o.pos.set(px, this.y, pz);
      o.tgt.set(mx + dx * 5, 1.0, mz + dz * 5);
      o.fov = i.aspect < 1 ? 80 : 62;
    } else if (id === 'broadcast') {
      // a TV rail camera on the near sideline: slides along the field behind the robot, a beat late
      this.bx += (i.x - this.bx) * damp(0.5, dt);
      this.bz += (i.z - this.bz) * damp(0.35, dt);
      this.tx += (i.x - this.tx) * damp(0.25, dt);
      const cx = THREE.MathUtils.clamp(this.bx * 0.85, -6.5, 6.5);
      o.pos.set(cx, 4.8, -7);
      o.tgt.set(THREE.MathUtils.lerp(this.tx, cx, 0.25), 0.35, this.bz * 0.7);
      o.fov = i.aspect < 1 ? 56 : 36;
    } else {
      // low behind the robot at bumper height: the yaw lags a little, the distance shrinks at walls
      this.yaw = this.step(this.yaw, i.yaw, 0.25, 3.5, dt);
      const dx = Math.sin(this.yaw), dz = Math.cos(this.yaw), h = 0.38;
      let free = 2.3;
      for (let d = 0.9; d <= 2.3; d += 0.1) {
        const x = i.x - dx * d, z = i.z - dz * d;
        if (Math.abs(x) > WALL_X || Math.abs(z) > WALL_Z || i.field.floor(x, z) > h - 0.1) { free = Math.max(0.9, d - 0.15); break; }
      }
      this.dist += (free - this.dist) * damp(free < this.dist ? 0.08 : 0.4, dt);
      o.pos.set(i.x - dx * this.dist, h, i.z - dz * this.dist);
      o.tgt.set(i.x + dx * 1.4, 0.3, i.z + dz * 1.4);
      o.fov = i.aspect < 1 ? 92 : 78;
    }
    if (id === 'robot' || id === 'turret') {
      // pressed against a wall the mount would poke through it: slide the camera (and its target) back inside
      const dx = THREE.MathUtils.clamp(o.pos.x, -WALL_X - 0.05, WALL_X + 0.05) - o.pos.x, dz = THREE.MathUtils.clamp(o.pos.z, -WALL_Z, WALL_Z) - o.pos.z;
      o.pos.x += dx; o.pos.z += dz; o.tgt.x += dx; o.tgt.z += dz;
    }
    return o;
  }
}
