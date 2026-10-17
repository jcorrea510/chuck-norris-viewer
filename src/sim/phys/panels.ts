// Polycarbonate that bends: the hopper walls, the intake's front wall and its side plates as thin
// plates with two bending modes each (modal model).
//   plate     1/8 in (3.175 mm) polycarbonate, E = 2.3 GPa, ρ = 1200 kg/m³, ν = 0.37, with the hex
//             pattern cut out: 40 % open, so mass × 0.6 and bending stiffness × 0.45 (ligament
//             efficiency of a hex perforation), i.e. ω × 0.87. The frame holds each panel at about
//             20 cm pitch (bolts and the diagonal braces behind it, see the photos), so its modes are
//             those of a 20 cm clamped bay: ω₁ = 35.99/L² · √(D / ρh) (Leissa), second mode (odd in u)
//             at 2.04 ω₁ (clamped square). Mode shapes, normalised to 1 at the antinode:
//             φ₁ = sin²πu sin²πv (all in phase), φ₂ = sin 2πu sin²πu sin²πv (odd along u).
//   modal     q̈ + 2ζω q̇ + ω² q = F φ(u₀,v₀) / M − Γ a_n    with generalized mass M = ρ_eff h A ⟨φ²⟩,
//             Γ = ⟨φ⟩/⟨φ²⟩, ζ = 0.05; solved exactly per step (ω dt ≈ 9, an explicit scheme would blow up).
//   FUEL      a ball striking a panel is simulated through the contact (foam spring 3.5 kN/m with the
//             damping that gives the surface's restitution when the panel is rigid) coupled to the
//             modes: the force history gives the panel its amplitude, and the ball leaves slower by the
//             energy the panel took (a compliant wall). Rapier's rigid bounce is only ever reduced.
//             The contact lasts ~25 ms (3 steps): the panel's deflection during it is played back step by
//             step, then the free ring carries on (the panel is stiff next to the foam, so the ring after a
//             slow contact is small: the flex is the deflection under the ball).
//   robot     the panels ride the frame: the frame's velocity change normal to a panel (a wall hit,
//             being hit) is a half-sine acceleration pulse (bumper foam and frame: 4 ms).
import { FUEL, STEP } from './config';
import { couplerOffset } from './fourbar';
import type { Robot } from './robot';
import { BS, speedFactor, type Fuel } from './fuel';

const R = FUEL.radius;
export const PLATE = {
  E: 2.3e9, rho: 1200, nu: 0.37, h: 3.175e-3,
  /** perforation (hex pattern): open fraction and the bending stiffness left, EDIT: weigh and bend a real panel */
  open: 0.4, stiff: 0.45,
  /** distance between the frame's supports behind a panel (m): EDIT: measure on the robot */
  bay: 0.2,
  zeta: 0.05,
  /** frame pulse length when it is hit (s): EDIT */
  pulse: 0.004,
};

export interface PanelDef {
  id: string;
  /** rides the intake coupler (moves with the four-bar) */
  coupler: boolean;
  /** center at the deployed pose, robot frame; normal axis n and in-plane axes u, v (0 = x, 1 = y, 2 = z); half sizes and half thickness */
  c: [number, number, number]; n: 0 | 1 | 2; u: 0 | 1 | 2; v: 0 | 1 | 2; hu: number; hv: number; half: number;
  /** how far from the mid plane the CAD mesh's vertices still belong to the panel (renderer) */
  tol: number;
}
const HX = 0.33;
export const PANELS: PanelDef[] = [
  { id: 'hopper-left', coupler: false, c: [HX + 0.012, 0.36, 0.20], n: 0, u: 2, v: 1, hu: 0.24, hv: 0.22, half: 0.012, tol: 0.016 },
  { id: 'hopper-right', coupler: false, c: [-HX - 0.012, 0.36, 0.20], n: 0, u: 2, v: 1, hu: 0.24, hv: 0.22, half: 0.012, tol: 0.016 },
  { id: 'hopper-rear', coupler: false, c: [0, 0.40, -0.042], n: 2, u: 0, v: 1, hu: 0.35, hv: 0.18, half: 0.012, tol: 0.03 },
  { id: 'intake-front', coupler: true, c: [0, 0.396, 0.654], n: 2, u: 0, v: 1, hu: 0.33, hv: 0.155, half: 0.012, tol: 0.016 },
  { id: 'intake-left', coupler: true, c: [HX + 0.012, 0.34, 0.54], n: 0, u: 2, v: 1, hu: 0.12, hv: 0.20, half: 0.012, tol: 0.016 },
  { id: 'intake-right', coupler: true, c: [-HX - 0.012, 0.34, 0.54], n: 0, u: 2, v: 1, hu: 0.12, hv: 0.20, half: 0.012, tol: 0.016 },
];

const S = (x: number) => Math.sin(Math.PI * x);
/** max of sin 2πu sin²πu (at u = 1/3) */
const NORM2 = 1 / 0.6495190528383289;
/** mode shapes on the unit square, 1 at the antinode */
export const phi1 = (u: number, v: number) => S(u) ** 2 * S(v) ** 2;
export const phi2 = (u: number, v: number) => Math.sin(2 * Math.PI * u) * S(u) ** 2 * S(v) ** 2 * NORM2;
const MEAN = { m1: 0, m2: 0, s1: 0, s2: 0 };
{
  const n = 200;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const u = (i + 0.5) / n, v = (j + 0.5) / n, a = phi1(u, v), b = phi2(u, v);
    MEAN.m1 += a / (n * n); MEAN.m2 += b / (n * n); MEAN.s1 += a * a / (n * n); MEAN.s2 += b * b / (n * n);
  }
}

export interface Modes { omega: [number, number]; mass: [number, number]; gamma: [number, number] }
export function modesOf(def: PanelDef): Modes {
  const D = (PLATE.E * PLATE.stiff * PLATE.h ** 3) / (12 * (1 - PLATE.nu ** 2));
  const rh = PLATE.rho * (1 - PLATE.open) * PLATE.h;
  const w1 = (35.99 / PLATE.bay ** 2) * Math.sqrt(D / rh);
  const area = 4 * def.hu * def.hv;
  return {
    omega: [w1, 2.04 * w1],
    mass: [rh * area * MEAN.s1, rh * area * MEAN.s2],
    gamma: [MEAN.m1 / MEAN.s1, MEAN.m2 / MEAN.s2],
  };
}

/** Free (and constant-force) response of q̈ + 2ζω q̇ + ω² q = f over dt, exact. */
function advance(st: Float64Array, k: number, omega: number, zeta: number, f: number, dt: number) {
  const wd = omega * Math.sqrt(1 - zeta * zeta), eq = f / (omega * omega);
  const x0 = st[2 * k] - eq, v0 = st[2 * k + 1];
  const e = Math.exp(-zeta * omega * dt), c = Math.cos(wd * dt), s = Math.sin(wd * dt);
  const B = (v0 + zeta * omega * x0) / wd;
  st[2 * k] = eq + e * (x0 * c + B * s);
  st[2 * k + 1] = e * (v0 * c - ((omega * omega * x0 + zeta * omega * v0) / wd) * s);
}

interface Cand { i: number; v: Float64Array }
/** a strike in progress: the modes' deflection per step while the ball is on the panel, then the state it leaves them in */
interface Pending { trace: number[]; steps: number; tEnd: number; end: number[] }

export class Panels {
  readonly defs = PANELS;
  readonly modes: Modes[] = PANELS.map(modesOf);
  /** per panel, per mode: (q, q̇) in m and m/s along the panel's +axis */
  readonly state = new Float64Array(PANELS.length * 4);
  /** the renderer's amplitudes: q₁, q₂ per panel (m) */
  readonly q = new Float32Array(PANELS.length * 2);
  private cands: Cand[] = [];
  private pending: (Pending | null)[] = PANELS.map(() => null);
  /** largest deflection under the ball in the last strike (m) */
  lastPeak = 0;
  private vBefore = [0, 0, 0];
  /** ball strikes handled last step (tests) */
  strikes = 0;
  constructor(private robot: Robot) {}

  clear() { this.state.fill(0); this.q.fill(0); this.pending.fill(null); }

  /** a strike's end state becomes the panel's state (a new strike or the end of the contact) */
  private settle(k: number, p: Pending, extra: number) {
    for (let j = 0; j < 4; j++) this.state[4 * k + j] = p.end[j];
    advance(this.state, 2 * k, this.modes[k].omega[0], PLATE.zeta, 0, extra);
    advance(this.state, 2 * k + 1, this.modes[k].omega[1], PLATE.zeta, 0, extra);
    this.pending[k] = null;
  }

  /** Before the engine step: remember the velocities of balls near the robot and of the frame. */
  preStep(fuel: Fuel) {
    const r = this.robot;
    this.cands.length = 0;
    for (let i = 0; i < fuel.n; i++) {
      const s = fuel.st[i];
      if (s !== BS.FREE && s !== BS.FLYING) continue;
      const o = 3 * i;
      const dx = fuel.pos[o] - r.p[0], dz = fuel.pos[o + 2] - r.p[2];
      if (dx * dx + dz * dz > 0.64) continue;
      const b = fuel.bodies[i];
      if (b.isSleeping()) continue;
      const v = b.linvel();
      this.cands.push({ i, v: Float64Array.of(v.x, v.y, v.z) });
    }
    const v = r.body.linvel();
    this.vBefore[0] = v.x; this.vBefore[1] = v.y; this.vBefore[2] = v.z;
  }

  /** After the engine step: ball strikes, the frame's own acceleration, then the modes advance. */
  postStep(fuel: Fuel) {
    const r = this.robot;
    this.strikes = 0;
    const [dzc, dyc] = couplerOffset(r.intakeAngle);
    const centers = PANELS.map((p) => (p.coupler ? [p.c[0], p.c[1] + dyc, p.c[2] + dzc] : p.c));
    // frame acceleration (robot frame) over the step
    const vw = r.body.linvel();
    const dvw = [vw.x - this.vBefore[0], vw.y - this.vBefore[1], vw.z - this.vBefore[2]];
    const dv = r.toLocal(dvw[0] + r.p[0], dvw[1] + r.p[1], dvw[2] + r.p[2]);
    // base excitation, then the free ring of the rest of the step
    PANELS.forEach((p, k) => {
      const m = this.modes[k];
      const pd = this.pending[k];
      if (pd) {
        // ball on the panel: play back the contact's deflection
        pd.steps++;
        if (pd.steps * STEP >= pd.tEnd) this.settle(k, pd, pd.steps * STEP - pd.tEnd);
        return;
      }
      const dn = dv[p.n];
      for (let j = 0; j < 2; j++) {
        const w = m.omega[j];
        if (Math.abs(dn) > 0.15) {
          // a hit: half-sine pulse of area Δv over PLATE.pulse, in 16 slices
          const N = 16, dt = PLATE.pulse / N, a0 = (dn * Math.PI) / (2 * PLATE.pulse);
          for (let s = 0; s < N; s++) advance(this.state, 2 * k + j, w, PLATE.zeta, -m.gamma[j] * a0 * Math.sin((Math.PI * (s + 0.5)) / N), dt);
          advance(this.state, 2 * k + j, w, PLATE.zeta, 0, STEP - PLATE.pulse);
        } else advance(this.state, 2 * k + j, w, PLATE.zeta, (-m.gamma[j] * dn) / STEP, STEP);
      }
    });
    // balls that struck a panel this step
    const l = [0, 0, 0];
    for (const cd of this.cands) {
      const o = 3 * cd.i;
      r.toLocal(fuel.pos[o], fuel.pos[o + 1], fuel.pos[o + 2], l);
      for (let k = 0; k < PANELS.length; k++) {
        const p = PANELS[k], c = centers[k];
        if (Math.abs(l[p.u] - c[p.u]) > p.hu + 0.03 || Math.abs(l[p.v] - c[p.v]) > p.hv + 0.03) continue;
        const dn = l[p.n] - c[p.n];
        const gap = Math.abs(dn) - p.half - R;
        if (gap < -0.09 || gap > 0.02) continue;
        const side = dn >= 0 ? 1 : -1;
        // outward normal (toward the ball) in the world, and the frame's velocity at the ball
        const e = [0, 0, 0]; e[p.n] = side;
        const nw = r.toWorld(e[0], e[1], e[2], [0, 0, 0]);
        const nx = nw[0] - r.p[0], ny = nw[1] - r.p[1], nz = nw[2] - r.p[2];
        const pv = r.pointVel(l[0], l[1], l[2]);
        const b = fuel.bodies[cd.i];
        const v1 = b.linvel();
        const vnIn = -((cd.v[0] - pv[0]) * nx + (cd.v[1] - pv[1]) * ny + (cd.v[2] - pv[2]) * nz);
        const vnOut = (v1.x - pv[0]) * nx + (v1.y - pv[1]) * ny + (v1.z - pv[2]) * nz;
        if (vnIn < 1.2 || vnOut < -0.2) continue;
        const u0 = (l[p.u] - c[p.u]) / (2 * p.hu) + 0.5, v0 = (l[p.v] - c[p.v]) / (2 * p.hv) + 0.5;
        const vOut = this.strike(k, side, Math.min(1, Math.max(0, u0)), Math.min(1, Math.max(0, v0)), vnIn, fuel.mass[cd.i]);
        const d = Math.min(0, vOut - vnOut);
        if (d < -1e-4) { const m = fuel.mass[cd.i]; b.applyImpulse({ x: m * d * nx, y: m * d * ny, z: m * d * nz }, true); }
        this.strikes++;
        break;
      }
    }
    for (let k = 0; k < PANELS.length; k++) {
      const pd = this.pending[k];
      for (let j = 0; j < 2; j++) this.q[2 * k + j] = Math.max(-0.03, Math.min(0.03, pd ? pd.trace[2 * (pd.steps - 1) + j] ?? 0 : this.state[4 * k + 2 * j]));
    }
  }

  /**
   * One ball striking panel k at (u0, v0) with approach speed vn (relative to the frame), from side
   * `side` of the panel (+1: the ball is on the panel's +axis side): the contact integrated with the modes
   * coupled in. Updates the modes and returns the ball's exit speed.
   */
  strike(k: number, side: number, u0: number, v0: number, vn: number, m: number) {
    const mo = this.modes[k], st = this.state;
    const old = this.pending[k];
    if (old) this.settle(k, old, 0);
    const ph = [phi1(u0, v0), phi2(u0, v0)];
    // damping giving the surface's restitution at this speed when the panel is rigid
    const e0 = Math.min(0.95, FUEL.restitution.robot * speedFactor(vn));
    const le = Math.log(e0), zb = -le / Math.sqrt(Math.PI * Math.PI + le * le);
    const kb = FUEL.springRate, cb = 2 * zb * Math.sqrt(kb * m);
    // the ball pushes the panel along −side: use coordinates along the push (x = −side · q)
    const q = [-side * st[4 * k], -side * st[4 * k + 2]], qd = [-side * st[4 * k + 1], -side * st[4 * k + 3]];
    const dt = 1e-4;
    let p = 0, vb = vn, t = 0, peak = 0;
    const trace: number[] = [], every = Math.round(STEP / dt);
    let iter = 0;
    for (; t < 0.12; t += dt) {
      const up = ph[0] * q[0] + ph[1] * q[1], upd = ph[0] * qd[0] + ph[1] * qd[1];
      const delta = p - up;
      let F = 0;
      if (delta > 0) F = Math.max(0, kb * delta + cb * (vb - upd));
      else if (t > 0) break;
      vb -= (F / m) * dt;
      p += vb * dt;
      for (let j = 0; j < 2; j++) {
        const w = mo.omega[j];
        qd[j] += ((F * ph[j]) / mo.mass[j] - 2 * PLATE.zeta * w * qd[j] - w * w * q[j]) * dt;
        q[j] += qd[j] * dt;
      }
      peak = Math.max(peak, Math.abs(ph[0] * q[0] + ph[1] * q[1]));
      if (++iter % every === 0) trace.push(-side * q[0], -side * q[1]);
    }
    this.lastPeak = peak;
    // the contact is over at t; the deflection so far plays back, one sample per step
    this.pending[k] = { trace, steps: 1, tEnd: t, end: [-side * q[0], -side * qd[0], -side * q[1], -side * qd[1]] };
    void st;
    return Math.max(0, -vb);
  }

  /** Envelope of a panel's ringing: √(q² + (q̇/ω)²) for its first mode (m). */
  amplitude(k: number, j = 0) {
    const pd = this.pending[k];
    if (pd) return Math.abs(pd.trace[2 * Math.max(0, pd.steps - 1) + j] ?? 0);
    const w = this.modes[k].omega[j];
    return Math.hypot(this.state[4 * k + 2 * j], this.state[4 * k + 2 * j + 1] / w);
  }
}
