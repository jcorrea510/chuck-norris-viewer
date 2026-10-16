// FUEL physics: flight with air drag and backspin lift, bouncing and rolling on the real floor
// (bumps included), walls and field elements, ball-to-ball collisions, and the HUB (funnel entry,
// net, internal ramps and the diffuser exit facing the neutral zone).
import type { FieldGrid, HubData } from './field';

export const R = 0.075;               // 5.91" FUEL
export const MASS = 0.215;            // kg
const G = 9.81;
const AREA = Math.PI * R * R;
export const K_DRAG = (0.5 * 1.2 * 0.5 * AREA) / MASS;   // ½ρC_dA/m
export const K_LIFT = (0.5 * 1.2 * 0.16 * AREA) / MASS;  // backspin (Magnus) lift
const BOUNCE = 0.38;
const ROLL_DECAY = 0.85;              // 1/s, foam on carpet
const ROLL_DRAG = 0.1;                // m/s² constant rolling resistance
const SLEEP_SPEED = 0.045;

export const enum S { REST = 0, MOVING = 1, STORED = 2, FLYING = 3, IN_HUB = 4, INTAKING = 5, PARKED = 6 }

export interface RobotContact {
  /** Called for every simulated ball near the robot; may push it or take it into the intake. */
  contact(i: number, balls: Balls): void;
  readonly x: number;
  readonly z: number;
}

export interface HubShape {
  cx: number; cz: number; top: number;
  minX: number; maxX: number; minZ: number; maxZ: number;
  openX: number; openZ: number;      // half-sizes of the funnel opening
  exitX: number; exitDir: number;    // diffuser face and the direction fuel leaves (+1 → +x)
}

export function hubShape(h: HubData): HubShape {
  const toCenter = h.center[0] < 0 ? 1 : -1;
  return {
    cx: h.center[0], cz: h.center[2], top: h.center[1],
    minX: h.min[0], maxX: h.max[0], minZ: -0.6, maxZ: 0.6,
    openX: h.opening[0] / 2 - 0.05, openZ: h.opening[1] / 2 - 0.05,
    exitX: toCenter > 0 ? h.max[0] : h.min[0], exitDir: toCenter,
  };
}

export class Balls {
  readonly n: number;
  readonly pos: Float32Array;
  readonly vel: Float32Array;
  readonly st: Uint8Array;
  readonly timer: Float32Array;
  private slow: Float32Array;
  /** seconds a just-launched ball ignores the robot that launched it */
  private grace: Float32Array;
  // spatial hash
  private readonly hx0 = -8.5; private readonly hz0 = -4.3; private readonly hc = 0.16;
  private readonly hnx: number; private readonly hnz: number;
  private head: Int32Array; private next: Int32Array;
  onScore: ((i: number) => void) | null = null;
  onHubExit: ((i: number) => void) | null = null;
  /** a ball left the field (or its numbers broke): the world puts it back in play */
  onLost: ((i: number) => void) | null = null;
  /** hard landings (speed into the floor, m/s), for sounds */
  onBounce: ((i: number, speed: number) => void) | null = null;
  scored = 0;

  constructor(private field: FieldGrid, private hub: HubShape, start: number[][]) {
    this.n = start.length;
    this.pos = new Float32Array(this.n * 3);
    this.vel = new Float32Array(this.n * 3);
    this.st = new Uint8Array(this.n);
    this.timer = new Float32Array(this.n);
    this.slow = new Float32Array(this.n);
    this.grace = new Float32Array(this.n);
    this.hnx = Math.ceil(17 / this.hc); this.hnz = Math.ceil(8.6 / this.hc);
    this.head = new Int32Array(this.hnx * this.hnz);
    this.next = new Int32Array(this.n);
    this.reset(start);
  }

  reset(start: number[][]) {
    for (let i = 0; i < this.n; i++) {
      this.pos.set(start[i], i * 3);
      this.vel.fill(0, i * 3, i * 3 + 3);
      this.timer[i] = 0;
      this.slow[i] = 0;
      this.grace[i] = 0;
      // fuel staged outside the field (behind the alliance walls, in the outposts) stays put
      const [x, , z] = start[i];
      this.st[i] = Math.abs(x) > 8.2 || Math.abs(z) > 4.0 ? S.PARKED : S.REST;
    }
    this.scored = 0;
  }

  wake(i: number) { if (this.st[i] === S.REST) { this.st[i] = S.MOVING; this.slow[i] = 0; } }
  free(i: number) { const s = this.st[i]; return s === S.REST || s === S.MOVING || s === S.FLYING; }

  launch(i: number, p: number[], v: number[]) {
    this.pos.set(p, i * 3);
    this.vel.set(v, i * 3);
    this.st[i] = S.FLYING;
    this.slow[i] = 0;
    this.grace[i] = 0.3;
  }

  step(dt: number, robot: RobotContact | null) {
    const { pos, vel, st, field, hub } = this;
    for (let i = 0; i < this.n; i++) {
      const s = st[i];
      if (s === S.IN_HUB) {
        this.timer[i] -= dt;
        if (this.timer[i] <= 0) this.exitHub(i);
        continue;
      }
      if (s !== S.MOVING && s !== S.FLYING) continue;
      const o = i * 3;
      if (this.grace[i] > 0) this.grace[i] -= dt;
      let x = pos[o], y = pos[o + 1], z = pos[o + 2];
      let vx = vel[o], vy = vel[o + 1], vz = vel[o + 2];
      const fh = field.floor(x, z);
      const grounded = y <= fh + R + 0.004 && Math.abs(vy) < 0.25;

      if (grounded) {
        // rolling: slope pulls it downhill (5/7 g sinθ for a rolling sphere), carpet slows it
        const [sx, sz] = field.slope(x, z);
        vx += -(5 / 7) * G * sx * dt;
        vz += -(5 / 7) * G * sz * dt;
        const sp = Math.hypot(vx, vz);
        if (sp > 0) {
          const k = Math.max(0, sp * Math.exp(-ROLL_DECAY * dt) - ROLL_DRAG * dt) / sp;
          vx *= k; vz *= k;
        }
        vy = 0;
        y = fh + R;
        if (sp < SLEEP_SPEED) {
          // settle quickly on flat carpet; on a slope only if it is wedged (not rolling for a full second)
          this.slow[i] += dt;
          if (this.slow[i] > (Math.abs(sx) + Math.abs(sz) < 0.02 ? 0.25 : 1.0)) { st[i] = S.REST; vx = vz = 0; }
        } else this.slow[i] = 0;
      } else {
        // flight: gravity + quadratic drag + backspin lift (perpendicular to velocity, upward)
        const sp = Math.hypot(vx, vy, vz);
        if (sp > 0.01) {
          const ux = vx / sp, uy = vy / sp, uz = vz / sp;
          let lx = -uy * ux, ly = 1 - uy * uy, lz = -uy * uz;
          const ll = Math.hypot(lx, ly, lz) || 1;
          lx /= ll; ly /= ll; lz /= ll;
          const kd = K_DRAG * sp, kl = st[i] === S.FLYING ? K_LIFT * sp * sp : 0;
          vx += (-kd * vx + kl * lx) * dt;
          vy += (-G - kd * vy + kl * ly) * dt;
          vz += (-kd * vz + kl * lz) * dt;
        } else vy -= G * dt;
      }

      // integrate per axis against solid cells (only low enough to hit field elements)
      let nx = x + vx * dt;
      if (y < 0.5 && field.distAt(nx, z) < R) { vx = -vx * 0.35; nx = x; }
      let nz = z + vz * dt;
      if (y < 0.5 && field.distAt(nx, nz) < R) { vz = -vz * 0.35; nz = z; }
      let ny = y + vy * dt;
      // still overlapping (e.g. pushed in by the robot): slide out along the distance gradient
      if (ny < 0.5) {
        const d = field.distAt(nx, nz);
        if (d < R * 0.7) {
          const [ax, az] = field.awayFromWalls(nx, nz);
          nx += ax * (R - d); nz += az * (R - d);
        }
      }
      // field perimeter
      if (Math.abs(nx) > 8.27 - R) { nx = Math.sign(nx) * (8.27 - R); vx = -vx * 0.4; }
      if (Math.abs(nz) > 4.03 - R) { nz = Math.sign(nz) * (4.03 - R); vz = -vz * 0.4; }

      // HUB: funnel opening, net behind it, and the solid body
      const inX = nx > hub.minX - R && nx < hub.maxX + R, inZ = nz > hub.minZ - R && nz < hub.maxZ + R;
      if (inX && inZ && ny < hub.top + R) {
        const ox = Math.abs(nx - hub.cx), oz = Math.abs(nz - hub.cz);
        if (vy < 0 && y >= hub.top - 0.02 && ox < hub.openX && oz < hub.openZ) {
          this.enterHub(i);
          continue;
        }
        if (y >= hub.top - 0.05 && ox < hub.openX + 0.12 && oz < hub.openZ + 0.12) {
          // landed on the funnel flare: it slides in
          this.enterHub(i);
          continue;
        }
        // side of the HUB: bounce off the nearest face
        const px = Math.min(Math.abs(nx - (hub.minX - R)), Math.abs(nx - (hub.maxX + R)));
        const pz = Math.min(Math.abs(nz - (hub.minZ - R)), Math.abs(nz - (hub.maxZ + R)));
        if (px < pz) { nx = nx - hub.cx < 0 ? hub.minX - R : hub.maxX + R; vx = -vx * 0.35; }
        else { nz = nz - hub.cz < 0 ? hub.minZ - R : hub.maxZ + R; vz = -vz * 0.35; }
      }
      // net behind the HUB catches overshots
      const netX = hub.cx + hub.exitDir * 0.75;
      if (Math.abs(nz - hub.cz) < 0.72 && ny > 1.55 && ny < 3.05 && (nx - netX) * hub.exitDir > 0 && (x - netX) * hub.exitDir <= 0) {
        nx = netX - hub.exitDir * 0.01; vx = -vx * 0.12; vz *= 0.3; vy = Math.min(vy, 0);
      }

      // floor
      const fh2 = field.floor(nx, nz);
      if (ny < fh2 + R) {
        ny = fh2 + R;
        if (vy < -0.7) {
          if (vy < -2.5) this.onBounce?.(i, -vy);
          vy = -vy * BOUNCE;
          vx *= 0.82; vz *= 0.82;
        } else vy = 0;
        if (st[i] === S.FLYING) st[i] = S.MOVING;
      }
      pos[o] = nx; pos[o + 1] = ny; pos[o + 2] = nz;
      vel[o] = vx; vel[o + 1] = vy; vel[o + 2] = vz;
      if (!Number.isFinite(nx + ny + nz + vx + vy + vz) || Math.abs(nx) > 8.6 || Math.abs(nz) > 4.4 || ny < -0.2 || ny > 12) this.lose(i);
    }
    this.collide();
    if (robot) {
      for (let i = 0; i < this.n; i++) {
        const s = st[i];
        if (s !== S.REST && s !== S.MOVING && s !== S.FLYING || this.grace[i] > 0) continue;
        const dx = pos[i * 3] - robot.x, dz = pos[i * 3 + 2] - robot.z;
        if (dx * dx + dz * dz < 1.3) robot.contact(i, this);
      }
    }
  }

  private lose(i: number) {
    this.st[i] = S.PARKED;
    this.vel.fill(0, i * 3, i * 3 + 3);
    if (this.onLost) this.onLost(i);
    else { this.pos[i * 3] = -8.6; this.pos[i * 3 + 1] = 0.9; this.pos[i * 3 + 2] = 3.4; }
  }

  private enterHub(i: number) {
    const o = i * 3;
    this.st[i] = S.IN_HUB;
    this.timer[i] = 0.55 + Math.random() * 0.65; // down the internal ramps
    this.pos[o] = this.hub.cx; this.pos[o + 1] = this.hub.top - 0.4; this.pos[o + 2] = this.hub.cz;
    this.vel.fill(0, o, o + 3);
    this.scored++;
    this.onScore?.(i);
  }

  private exitHub(i: number) {
    // out through the diffuser on the neutral-zone side, fanned by the deflectors
    const o = i * 3, h = this.hub;
    const spread = (Math.random() - 0.5) * 1.1;
    const speed = 1.4 + Math.random() * 1.6;
    this.pos[o] = h.exitX + h.exitDir * (R + 0.02);
    this.pos[o + 1] = 0.78;
    this.pos[o + 2] = h.cz + (Math.random() - 0.5) * 0.9;
    this.vel[o] = h.exitDir * speed * Math.cos(spread);
    this.vel[o + 1] = -0.3;
    this.vel[o + 2] = speed * Math.sin(spread);
    this.st[i] = S.MOVING;
    this.slow[i] = 0;
    this.onHubExit?.(i);
  }

  /** Ball-to-ball contacts with a spatial hash; resting balls only wake when something hits them. */
  private collide() {
    const { pos, vel, st, head, next, hc, hx0, hz0, hnx, hnz } = this;
    head.fill(-1);
    for (let i = 0; i < this.n; i++) {
      const s = st[i];
      if (s !== S.REST && s !== S.MOVING && s !== S.FLYING) continue;
      const cx = Math.floor((pos[i * 3] - hx0) / hc), cz = Math.floor((pos[i * 3 + 2] - hz0) / hc);
      if (cx < 0 || cz < 0 || cx >= hnx || cz >= hnz) continue;
      const c = cz * hnx + cx;
      next[i] = head[c];
      head[c] = i;
    }
    const D = 2 * R, D2 = D * D;
    for (let i = 0; i < this.n; i++) {
      const si = st[i];
      if (si !== S.MOVING && si !== S.FLYING) continue;
      const oi = i * 3;
      const cx = Math.floor((pos[oi] - hx0) / hc), cz = Math.floor((pos[oi + 2] - hz0) / hc);
      for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
        const x = cx + dx, z = cz + dz;
        if (x < 0 || z < 0 || x >= hnx || z >= hnz) continue;
        for (let j = head[z * hnx + x]; j !== -1; j = next[j]) {
          if (j === i) continue;
          const sj = st[j];
          if ((sj === S.MOVING || sj === S.FLYING) && j < i) continue; // pair handled once
          const oj = j * 3;
          const ex = pos[oj] - pos[oi], ey = pos[oj + 1] - pos[oi + 1], ez = pos[oj + 2] - pos[oi + 2];
          const d2 = ex * ex + ey * ey + ez * ez;
          if (d2 >= D2 || d2 < 1e-10) continue;
          const d = Math.sqrt(d2);
          const nx = ex / d, ny = ey / d, nz = ez / d;
          const pen = (D - d) / 2;
          const jRest = sj === S.REST;
          const rv0 = (vel[oj] - vel[oi]) * nx + (vel[oj + 1] - vel[oi + 1]) * ny + (vel[oj + 2] - vel[oi + 2]) * nz;
          // staged fuel touches its neighbours; a gentle graze must not wake a whole row
          if (jRest && pen < 0.004 && rv0 > -0.12) continue;
          // separate (a resting ball carries half unless it is wedged on the floor)
          pos[oi] -= nx * pen; pos[oi + 1] -= ny * pen; pos[oi + 2] -= nz * pen;
          pos[oj] += nx * pen; pos[oj + 1] += ny * pen; pos[oj + 2] += nz * pen;
          const rv = (vel[oj] - vel[oi]) * nx + (vel[oj + 1] - vel[oi + 1]) * ny + (vel[oj + 2] - vel[oi + 2]) * nz;
          if (rv < 0) {
            const jimp = -(1 + 0.3) * rv / 2;
            vel[oi] -= jimp * nx; vel[oi + 1] -= jimp * ny; vel[oi + 2] -= jimp * nz;
            vel[oj] += jimp * nx; vel[oj + 1] += jimp * ny; vel[oj + 2] += jimp * nz;
          }
          if (jRest) this.wake(j);
        }
      }
    }
  }

  /** Ground fuel a robot could reasonably go and get (in the field, clear of walls). */
  available(i: number) {
    const s = this.st[i];
    return (s === S.REST || s === S.MOVING) && this.pos[i * 3 + 1] < 0.3;
  }
}
