// Futuristic interface sounds, synthesized live with Web Audio (no files, works offline).
// Everything is short, soft and filtered, closer to a workshop hologram than a video game.

type Kind = 'tap' | 'select' | 'focus' | 'back' | 'hide' | 'show' | 'toggle' | 'boot' | 'shot' | 'score' | 'intake' | 'error' | 'open';

export class Sound {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private servo: { osc: OscillatorNode; osc2: OscillatorNode; gain: GainNode; filter: BiquadFilterNode } | null = null;
  enabled = true;
  volume = 0.6;

  /** Must be called from a user gesture (iOS unlocks audio only then). */
  unlock() {
    if (!this.ctx) {
      const AC = window.AudioContext ?? (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
      if (!AC) return;
      this.ctx = new AC();
      this.master = this.ctx.createGain();
      this.master.gain.value = this.volume * 0.5;
      const comp = this.ctx.createDynamicsCompressor();
      comp.threshold.value = -18;
      this.master.connect(comp).connect(this.ctx.destination);
    }
    if (this.ctx.state === 'suspended') void this.ctx.resume();
  }

  setVolume(v: number) {
    this.volume = v;
    if (this.master) this.master.gain.value = v * 0.5;
  }

  private ok() { return this.enabled && this.ctx && this.master && this.ctx.state === 'running'; }

  private tone(freq: number, to: number, dur: number, type: OscillatorType, gain: number, delay = 0, q = 0) {
    const c = this.ctx!, t = c.currentTime + delay;
    const o = c.createOscillator(), g = c.createGain();
    o.type = type;
    o.frequency.setValueAtTime(freq, t);
    o.frequency.exponentialRampToValueAtTime(Math.max(20, to), t + dur);
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(gain, t + Math.min(0.012, dur / 4));
    g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    let node: AudioNode = o;
    if (q) {
      const f = c.createBiquadFilter();
      f.type = 'bandpass'; f.frequency.value = freq; f.Q.value = q;
      o.connect(f); node = f;
    }
    node.connect(g).connect(this.master!);
    o.start(t);
    o.stop(t + dur + 0.02);
  }

  private noise(dur: number, from: number, to: number, gain: number, delay = 0, type: BiquadFilterType = 'bandpass', q = 1.2) {
    const c = this.ctx!, t = c.currentTime + delay;
    const len = Math.ceil(c.sampleRate * dur);
    const buf = c.createBuffer(1, len, c.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < len; i++) d[i] = Math.random() * 2 - 1;
    const s = c.createBufferSource();
    s.buffer = buf;
    const f = c.createBiquadFilter();
    f.type = type; f.Q.value = q;
    f.frequency.setValueAtTime(from, t);
    f.frequency.exponentialRampToValueAtTime(to, t + dur);
    const g = c.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(gain, t + dur * 0.3);
    g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    s.connect(f).connect(g).connect(this.master!);
    s.start(t);
  }

  play(kind: Kind) {
    if (!this.ok()) return;
    switch (kind) {
      case 'tap': this.tone(2400, 2900, 0.05, 'sine', 0.08); break;
      case 'select':
        this.tone(1320, 1760, 0.07, 'sine', 0.1);
        this.tone(2640, 2640, 0.09, 'sine', 0.04, 0.045);
        break;
      case 'focus':
        this.noise(0.42, 500, 2600, 0.12);
        this.tone(660, 990, 0.28, 'triangle', 0.05, 0.05);
        this.tone(1980, 1980, 0.18, 'sine', 0.035, 0.2);
        break;
      case 'back':
        this.noise(0.32, 2200, 450, 0.09);
        this.tone(990, 600, 0.22, 'triangle', 0.045);
        break;
      case 'hide':
        this.tone(880, 330, 0.16, 'sine', 0.09);
        this.noise(0.14, 3000, 800, 0.04);
        break;
      case 'show':
        this.tone(330, 990, 0.18, 'sine', 0.08);
        this.tone(1320, 1320, 0.1, 'sine', 0.03, 0.12);
        break;
      case 'toggle': this.tone(1600, 1200, 0.06, 'square', 0.025, 0, 6); break;
      case 'open':
        this.noise(0.5, 300, 1400, 0.1, 0, 'lowpass', 0.8);
        this.tone(220, 440, 0.45, 'sawtooth', 0.02, 0, 3);
        break;
      case 'boot':
        [523, 659, 784, 1047].forEach((f, i) => this.tone(f, f * 1.003, 0.9 - i * 0.12, 'sine', 0.05, i * 0.11));
        this.noise(1.2, 200, 5000, 0.05, 0, 'bandpass', 0.7);
        this.tone(2093, 2093, 0.6, 'sine', 0.025, 0.5);
        break;
      case 'shot':
        this.tone(140, 60, 0.12, 'sine', 0.16);
        this.noise(0.09, 1800, 600, 0.07);
        break;
      case 'score':
        this.tone(1568, 1568, 0.16, 'sine', 0.07);
        this.tone(2093, 2093, 0.22, 'sine', 0.06, 0.08);
        break;
      case 'intake': this.noise(0.12, 900, 300, 0.05, 0, 'lowpass', 1); break;
      case 'error': this.tone(300, 200, 0.18, 'square', 0.03, 0, 4); break;
    }
  }

  /** Continuous servo whine while a mechanism moves; speed 0..1 sets pitch and level. */
  servoSpeed(speed: number) {
    if (!this.ok()) { this.stopServo(); return; }
    const c = this.ctx!;
    if (speed < 0.02) { this.stopServo(); return; }
    if (!this.servo) {
      const osc = c.createOscillator(), osc2 = c.createOscillator(), gain = c.createGain(), filter = c.createBiquadFilter();
      osc.type = 'sawtooth'; osc2.type = 'square';
      filter.type = 'bandpass'; filter.Q.value = 4;
      gain.gain.value = 0.0001;
      osc.connect(filter); osc2.connect(filter);
      filter.connect(gain).connect(this.master!);
      osc.start(); osc2.start();
      this.servo = { osc, osc2, gain, filter };
    }
    const t = c.currentTime;
    const s = Math.min(1, speed);
    this.servo.osc.frequency.setTargetAtTime(90 + s * 260, t, 0.05);
    this.servo.osc2.frequency.setTargetAtTime(181 + s * 523, t, 0.05);
    this.servo.filter.frequency.setTargetAtTime(500 + s * 1600, t, 0.05);
    this.servo.gain.gain.setTargetAtTime(0.012 + s * 0.03, t, 0.04);
  }

  stopServo() {
    if (!this.servo || !this.ctx) return;
    const s = this.servo;
    this.servo = null;
    s.gain.gain.setTargetAtTime(0.0001, this.ctx.currentTime, 0.05);
    setTimeout(() => { s.osc.stop(); s.osc2.stop(); }, 300);
  }
}

export const sound = new Sound();
