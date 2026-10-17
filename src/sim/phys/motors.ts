// DC motors on mechanisms, the way a Talon FX drives them: a commanded voltage (feedforward +
// PID, computed by the caller from the Talon's gains), limited by the battery, the stator and
// supply current limits, acting through a motor curve (torque constant, back-EMF, winding
// resistance) on a rotating inertia. The back-EMF, the controller's derivative terms and the
// voltage ramp are solved implicitly, so a stiff loop (a 1 kHz Talon loop on a light turret) stays
// stable at any step size.
import type { MotorSpec } from './config';

export interface Motor { kt: number; ke: number; r: number; spec: MotorSpec }
/** Torque constant (N·m/A), back-EMF constant (V·s/rad) and winding resistance (Ω) from the datasheet's stall and free points at 12 V. */
export function motorOf(spec: MotorSpec): Motor {
  const r = 12 / spec.stallCurrent;
  return { kt: spec.stallTorque / spec.stallCurrent, ke: (12 - spec.freeCurrent * r) / spec.free, r, spec };
}

export interface MotorStep {
  motor: Motor;
  /** motors driving the mechanism (a leader and its follower: 2) */
  n: number;
  /** motor revolutions per mechanism revolution, and gear efficiency */
  gear: number; eta: number;
  /** mechanism inertia (kg·m², without the rotors) and the speed (rad/s) it starts the step at */
  inertia: number; w: number;
  /** the controller's output voltage before limits: feedforward + P (and the D terms below are added on top) */
  v0: number;
  /** D term on the mechanism's velocity (V per rad/s: position loops) and on its acceleration (V per rad/s²: the derivative of a velocity loop's error) */
  kDamp?: number; kAcc?: number;
  /** outputs the controller may produce this step: the battery, and the ramp window */
  vLo: number; vHi: number;
  /** stator and supply current limits per motor (A), and the bus voltage the supply current is drawn at */
  stator: number; supply: number; vBus: number;
  /** external torque on the mechanism (N·m), friction torque magnitude (N·m, opposes motion) */
  tau: number; friction: number;
  /** neutral-coast: the motor leads are open */
  coast?: boolean;
  dt: number;
}
export interface MotorOut { w: number; alpha: number; i: number; v: number; supplyA: number }

/** Advances the mechanism's speed by one step. */
export function stepMotor(s: MotorStep): MotorOut {
  const m = s.motor;
  const keG = m.ke * s.gear;                                  // back-EMF per mechanism rad/s
  const ktG = s.n * m.kt * s.gear * s.eta;                    // mechanism torque per amp
  const J = s.inertia + s.n * m.spec.rotor * s.gear * s.gear;
  // the mechanism's friction (Coulomb, opposing the speed) and the motor's own losses: its free
  // current flows at free speed, so the loss torque grows with speed
  const fr = s.friction * Math.max(-1, Math.min(1, s.w / 0.02)) + ((s.n * m.kt * m.spec.freeCurrent * s.gear * s.gear) / m.spec.free) * s.w;
  const tauNet = s.tau - fr;
  if (s.coast) {
    const alpha = tauNet / J;
    return { w: s.w + alpha * s.dt, alpha, i: 0, v: 0, supplyA: 0 };
  }
  const c = ktG / (J * m.r);
  const kd = s.kDamp ?? 0, ka = s.kAcc ?? 0;
  // implicit solve of  J α = ktG I + tauNet,  V = v0 − kd w' − ka α,  V = I R + keG w',  w' = w + α dt
  let alpha = (c * (s.v0 - (kd + keG) * s.w) + tauNet / J) / (1 + c * ka + c * (kd + keG) * s.dt);
  let v = s.v0 - kd * (s.w + alpha * s.dt) - ka * alpha;
  if (v > s.vHi || v < s.vLo) {
    v = Math.max(s.vLo, Math.min(s.vHi, v));
    alpha = (c * (v - keG * s.w) + tauNet / J) / (1 + c * keG * s.dt);
  }
  let i = (J * alpha - tauNet) / ktG;
  // current limits: the stator limit directly; the supply limit as the stator current at which
  // the battery's share (I·V/Vbus) reaches the limit
  const emf = keG * (s.w + alpha * s.dt);
  const sup = (-emf + Math.sqrt(emf * emf + 4 * m.r * s.supply * s.vBus)) / (2 * m.r);
  const iMax = Math.min(s.stator, sup), iMin = -Math.min(s.stator, (emf + Math.sqrt(emf * emf + 4 * m.r * s.supply * s.vBus)) / (2 * m.r));
  if (i > iMax || i < iMin) {
    i = Math.max(iMin, Math.min(iMax, i));
    alpha = (ktG * i + tauNet) / J;
    v = i * m.r + keG * (s.w + alpha * s.dt);
  }
  const wNew = s.w + alpha * s.dt;
  // supply current drawn by all n motors (zero when regenerating)
  const supplyA = Math.max(0, s.n * i * v / Math.max(1, s.vBus));
  return { w: wNew, alpha, i, v, supplyA };
}

/**
 * Stator and supply current of a Talon driving a motor at shaft speed `w` (mechanism rad/s) with
 * commanded voltage `vCmd`, for loops where the mechanism is integrated elsewhere (the drive wheels).
 */
export function motorCurrent(m: Motor, gear: number, vCmd: number, w: number, vBus: number, stator: number, supply: number) {
  const keG = m.ke * gear;
  const v = Math.max(-vBus, Math.min(vBus, vCmd));
  const cap = (sign: number) => Math.min(stator, (-sign * keG * w + Math.sqrt(keG * keG * w * w + 4 * m.r * supply * vBus)) / (2 * m.r));
  const i = Math.max(-cap(-1), Math.min(cap(1), (v - keG * w) / m.r));
  const vApplied = i * m.r + keG * w;
  return { i, supplyA: Math.max(0, (i * vApplied) / Math.max(1, vBus)) };
}

/** WPILib TrapezoidProfile state. */
export interface ProfileState { pos: number; vel: number }

/**
 * WPILib TrapezoidProfile.calculate(t, current, goal) with a goal at rest: where the profile is
 * `t` seconds after `current`, under the velocity and acceleration limits.
 */
export function trapezoid(t: number, cur: ProfileState, goalPos: number, vmax: number, amax: number): ProfileState {
  const dir = cur.pos > goalPos ? -1 : 1;
  let pos = cur.pos * dir, vel = cur.vel * dir;
  const goal = goalPos * dir;
  if (vel > vmax) vel = vmax;
  // a truncated profile (nonzero initial velocity) is treated as a full trapezoid that began at rest
  const cutoffBegin = vel / amax;
  const cutoffDistBegin = (cutoffBegin * cutoffBegin * amax) / 2;
  const fullTrapezoidDist = cutoffDistBegin + (goal - pos);
  let accelTime = vmax / amax;
  let fullSpeedDist = fullTrapezoidDist - accelTime * accelTime * amax;
  if (fullSpeedDist < 0) { accelTime = Math.sqrt(fullTrapezoidDist / amax); fullSpeedDist = 0; }
  const endAccel = accelTime - cutoffBegin;
  const endFullSpeed = endAccel + fullSpeedDist / vmax;
  const endDecel = endFullSpeed + accelTime;
  const out = { pos, vel };
  if (t < endAccel) {
    out.vel += t * amax;
    out.pos += (vel + (t * amax) / 2) * t;
  } else if (t < endFullSpeed) {
    out.vel = vmax;
    out.pos += (vel + (endAccel * amax) / 2) * endAccel + vmax * (t - endAccel);
  } else if (t <= endDecel) {
    const left = endDecel - t;
    out.vel = left * amax;
    out.pos = goal - (left * amax / 2) * left;
  } else { out.pos = goal; out.vel = 0; }
  return { pos: out.pos * dir, vel: out.vel * dir };
}
