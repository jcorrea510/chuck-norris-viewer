// Robot sound, driven only by the physics state (Web Audio, synthesized, nothing canned):
//   drive    gear whine of the four Krakens: pitch = pinion mesh frequency at the motor's actual
//            speed, level = speed and stator current (load)
//   shooter  each flywheel's hum at its actual speed plus air rush; the RPM dip of every shot is
//            audible as a pitch drop and recovery
//   impacts  low, short foam thuds scaled by impact speed and surface, rate-limited so a field of
//            500 balls never turns into noise
// Off by default (see overnight/MORNING.md for the listening verdict).
import type { Frame } from './backend';
import { DRIVE, ROBOT } from './phys/config';

const PINION = 14;          // teeth on the drive pinion (gear mesh = 14 × motor rev/s)

export class SimAudio {
  private ctx: AudioContext | null = null;
  private out: GainNode | null = null;
  private pan: StereoPannerNode | null = null;
  private drive: { osc: OscillatorNode; osc2: OscillatorNode; gain: GainNode; bp: BiquadFilterNode } | null = null;
  private fly: { osc: OscillatorNode; gain: GainNode; noise: AudioBufferSourceNode; ng: GainNode; nf: BiquadFilterNode }[] = [];
  private noise: AudioBuffer | null = null;
  private thuds: number[] = [];
  enabled = false;
  volume = 0.6;

  setEnabled(on: boolean, volume = this.volume) {
    this.volume = volume;
    this.enabled = on;
    if (on && !this.ctx) this.build();
    if (this.ctx && this.out) {
      if (on) void this.ctx.resume();
      this.out.gain.setTargetAtTime(on ? volume : 0, this.ctx.currentTime, 0.05);
    }
  }

  private build() {
    const AC = window.AudioContext ?? (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
    if (!AC) return;
    const c = this.ctx = new AC();
    this.out = c.createGain();
    this.out.gain.value = 0;
    const lim = c.createDynamicsCompressor();
    lim.threshold.value = -6; lim.ratio.value = 12; lim.attack.value = 0.002; lim.release.value = 0.1;
    this.pan = c.createStereoPanner();
    this.pan.connect(this.out).connect(lim).connect(c.destination);
    const n = c.sampleRate;
    this.noise = c.createBuffer(1, n, n);
    const d = this.noise.getChannelData(0);
    let b = 0;
    for (let i = 0; i < n; i++) { b = 0.97 * b + 0.03 * (Math.random() * 2 - 1); d[i] = b * 4; }
    // drive gear whine
    const bp = c.createBiquadFilter(); bp.type = 'bandpass'; bp.Q.value = 6;
    const osc = c.createOscillator(); osc.type = 'sawtooth';
    const osc2 = c.createOscillator(); osc2.type = 'triangle';
    const g = c.createGain(); g.gain.value = 0;
    osc.connect(bp); osc2.connect(bp); bp.connect(g).connect(this.pan);
    osc.start(); osc2.start();
    this.drive = { osc, osc2, gain: g, bp };
    // flywheels
    for (let k = 0; k < 2; k++) {
      const o = c.createOscillator(); o.type = 'triangle';
      const og = c.createGain(); og.gain.value = 0;
      o.connect(og).connect(this.pan); o.start();
      const ns = c.createBufferSource(); ns.buffer = this.noise; ns.loop = true;
      const nf = c.createBiquadFilter(); nf.type = 'bandpass'; nf.Q.value = 1.2;
      const ng = c.createGain(); ng.gain.value = 0;
      ns.connect(nf).connect(ng).connect(this.pan); ns.start();
      this.fly.push({ osc: o, gain: og, noise: ns, ng, nf });
    }
  }

  update(w: Frame, pan: number, dist: number) {
    const c = this.ctx;
    if (!c || !this.enabled || !this.drive || !this.pan) return;
    const t = c.currentTime;
    const near = Math.min(1, 3 / Math.max(3, dist));
    this.pan.pan.setTargetAtTime(pan, t, 0.05);
    const r = w.robot;
    // motor speed from the wheels' surface speed
    let ws = 0, load = 0;
    for (const m of r.modules) { ws += Math.abs(m.wheel) / 4; load += Math.abs(m.current) / DRIVE.statorLimit / 4; }
    const motorRevS = (ws / ROBOT.wheelRadius) * DRIVE.gear / (2 * Math.PI);
    const mesh = Math.max(20, motorRevS * PINION);
    this.drive.osc.frequency.setTargetAtTime(mesh, t, 0.03);
    this.drive.osc2.frequency.setTargetAtTime(mesh * 2.01, t, 0.03);
    this.drive.bp.frequency.setTargetAtTime(Math.max(200, mesh * 1.3), t, 0.03);
    const lvl = Math.min(1, ws / 4) * 0.05 + load * 0.08;
    this.drive.gain.gain.setTargetAtTime(lvl * near * (ws > 0.05 || load > 0.05 ? 1 : 0), t, 0.04);
    r.turrets.forEach((tu, k) => {
      const f = this.fly[k];
      const revS = tu.fly / (2 * Math.PI);
      f.osc.frequency.setTargetAtTime(Math.max(10, revS * 6), t, 0.015);
      const a = Math.min(1, tu.fly / 450);
      f.gain.gain.setTargetAtTime(a * a * 0.05 * near, t, 0.03);
      f.nf.frequency.setTargetAtTime(400 + revS * 30, t, 0.03);
      f.ng.gain.setTargetAtTime(a * a * a * 0.04 * near, t, 0.03);
    });
  }

  /** A foam ball hitting something: a soft, low, short thud. */
  impact(speed: number, pan: number, dist: number, surface: 'carpet' | 'hard') {
    const c = this.ctx;
    if (!c || !this.enabled || !this.out || !this.noise || speed < 2) return;
    const now = c.currentTime;
    this.thuds = this.thuds.filter((x) => now - x < 0.1);
    if (this.thuds.length >= 5) return;
    const gain = Math.min(1, (speed / 12) ** 1.5) * Math.min(1, 4 / Math.max(4, dist)) * (surface === 'carpet' ? 0.5 : 0.9);
    if (gain < 0.02) return;
    this.thuds.push(now);
    const s = c.createBufferSource(); s.buffer = this.noise;
    const lp = c.createBiquadFilter(); lp.type = 'lowpass'; lp.frequency.value = surface === 'carpet' ? 260 : 520; lp.Q.value = 0.9;
    const g = c.createGain();
    const p = c.createStereoPanner(); p.pan.value = pan;
    g.gain.setValueAtTime(0, now);
    g.gain.linearRampToValueAtTime(gain * 0.5, now + 0.004);
    g.gain.exponentialRampToValueAtTime(0.0005, now + (surface === 'carpet' ? 0.06 : 0.09));
    s.connect(lp).connect(g).connect(p).connect(this.out);
    s.start(now, Math.random() * 0.5, 0.12);
  }

  stop() {
    void this.ctx?.close();
    this.ctx = null; this.drive = null; this.fly = []; this.out = null; this.pan = null;
  }
}
