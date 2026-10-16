// Sounds, synthesized live with Web Audio (no files, works offline).
//   Interface — soft glassy FM chimes, airy whooshes and crisp mechanical ticks, through a small
//               stereo room so nothing sounds dry or "beepy".
//   Robot     — layered one-shots (the thump and slap of a shot, a bumper hitting a wall, fuel
//               being grabbed and dropping into the HUB) plus continuous machinery that follows the
//               simulation: Kraken drive whine and gear noise tracking speed and current, both
//               flywheels singing at their actual speed (slightly apart, so they beat), roller
//               rumble, indexer belts, tread chirp when the wheels break loose. Everything is panned
//               to where the robot is on screen and fades with camera distance.

type Kind = 'tap' | 'select' | 'focus' | 'back' | 'hide' | 'show' | 'toggle' | 'boot' | 'shot' | 'score' | 'intake' | 'error' | 'open'
  | 'bounce' | 'impact' | 'takeover' | 'handback' | 'respawn' | 'connect';
interface PlayOpts { pan?: number; gain?: number }

export interface SimSoundState {
  speed: number; turn: number; effort: number; slipping: boolean;
  flywheels: number[];   // rad/s
  rollers: number; arm: number; feeding: number;
  near: number;          // 0..1 distance attenuation
  pan: number;           // −1..1
}

/**
 * Level of each sound in dB, measured with scripts/soundcheck.mjs so the mix is balanced: soft
 * interface ticks near −24 dBFS, chimes near −18, a shot −12, a wall hit −9 (at 60 % volume).
 */
const TRIM: Partial<Record<Kind, number>> = {
  tap: 26.8, select: 13.7, focus: 13.8, back: 15.2, hide: 27.2, show: 16.5, toggle: 36, open: 19.9, boot: 11.1,
  error: 13.5, shot: 23.4, score: 17.1, intake: 26.4, bounce: 29, impact: 21.3, takeover: 13.8, handback: 14.8,
  respawn: 16.5, connect: 13.3,
};

interface Layer { src: AudioScheduledSourceNode[]; gain: GainNode; set: (s: SimSoundState, t: number) => void }

export class Sound {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private dry: GainNode | null = null;
  private wet: GainNode | null = null;
  private noiseBuf: AudioBuffer | null = null;
  private pinkBuf: AudioBuffer | null = null;
  private servo: { nodes: AudioScheduledSourceNode[]; gain: GainNode; f1: OscillatorNode; f2: OscillatorNode; filter: BiquadFilterNode } | null = null;
  private sim: { bus: GainNode; pan: StereoPannerNode; layers: Layer[] } | null = null;
  private last = new Map<Kind, number>();
  enabled = true;
  volume = 0.6;

  /** Must be called from a user gesture (iOS unlocks audio only then). */
  unlock() {
    if (!this.ctx) {
      const AC = window.AudioContext ?? (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
      if (!AC) return;
      const c = this.ctx = new AC();
      this.master = c.createGain();
      this.master.gain.value = this.volume * 0.9;
      // glue: a gentle compressor, then a limiter so stacked shots never clip
      const comp = c.createDynamicsCompressor();
      comp.threshold.value = -12; comp.knee.value = 10; comp.ratio.value = 3; comp.attack.value = 0.004; comp.release.value = 0.2;
      const lim = c.createDynamicsCompressor();
      lim.threshold.value = -1.5; lim.knee.value = 0; lim.ratio.value = 20; lim.attack.value = 0.001; lim.release.value = 0.08;
      this.master.connect(comp).connect(lim).connect(c.destination);
      this.dry = c.createGain();
      this.dry.connect(this.master);
      // a small, bright room
      const verb = c.createConvolver();
      verb.buffer = this.impulse(1.5, 2.6);
      const pre = c.createBiquadFilter(); pre.type = 'highpass'; pre.frequency.value = 220;
      const post = c.createBiquadFilter(); post.type = 'lowpass'; post.frequency.value = 7000;
      this.wet = c.createGain();
      this.wet.gain.value = 0.28;
      this.wet.connect(pre).connect(verb).connect(post).connect(this.master);
      this.noiseBuf = this.makeNoise(false);
      this.pinkBuf = this.makeNoise(true);
    }
    if (this.ctx.state === 'suspended') void this.ctx.resume();
  }

  setVolume(v: number) {
    this.volume = v;
    if (this.master && this.ctx) this.master.gain.setTargetAtTime(v * 0.9, this.ctx.currentTime, 0.05);
  }

  private ok() { return this.enabled && this.ctx && this.master && this.ctx.state === 'running'; }

  // ------------------------------------------------------------------ building blocks
  private makeNoise(pink: boolean) {
    const c = this.ctx!, len = c.sampleRate * 2;
    const buf = c.createBuffer(1, len, c.sampleRate), d = buf.getChannelData(0);
    let b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
    for (let i = 0; i < len; i++) {
      const w = Math.random() * 2 - 1;
      if (!pink) { d[i] = w; continue; }
      // Paul Kellet's pink filter
      b0 = 0.99886 * b0 + w * 0.0555179; b1 = 0.99332 * b1 + w * 0.0750759; b2 = 0.969 * b2 + w * 0.153852;
      b3 = 0.8665 * b3 + w * 0.3104856; b4 = 0.55 * b4 + w * 0.5329522; b5 = -0.7616 * b5 - w * 0.016898;
      d[i] = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362) * 0.11;
      b6 = w * 0.115926;
    }
    return buf;
  }

  /** Stereo room: a few early reflections, then a dense decaying tail. */
  private impulse(seconds: number, decay: number) {
    const c = this.ctx!, len = Math.floor(c.sampleRate * seconds);
    const buf = c.createBuffer(2, len, c.sampleRate);
    for (let ch = 0; ch < 2; ch++) {
      const d = buf.getChannelData(ch);
      for (let i = 0; i < len; i++) d[i] = (Math.random() * 2 - 1) * Math.pow(1 - i / len, decay) * 0.5;
      for (const [ms, g] of [[11, 0.5], [17, 0.35], [29, 0.3], [41, 0.22], [53, 0.16]]) {
        const k = Math.floor((ms + ch * 3) * c.sampleRate / 1000);
        if (k < len) d[k] += g * (ch ? -1 : 1);
      }
    }
    return buf;
  }

  /** Output: a stereo position, then dry plus a send to the room. */
  private out(pan = 0, send = 0.25): AudioNode {
    const c = this.ctx!;
    const p = c.createStereoPanner();
    p.pan.value = Math.max(-1, Math.min(1, pan));
    p.connect(this.dry!);
    if (send > 0) { const s = c.createGain(); s.gain.value = send; p.connect(s).connect(this.wet!); }
    const trim = c.createGain();
    trim.gain.value = this.trim;
    trim.connect(p);
    return trim;
  }
  /** level of the sound being played (see TRIM) */
  private trim = 1;

  /** Attack / exponential decay envelope on a fresh gain node. */
  private env(t: number, peak: number, attack: number, decay: number): GainNode {
    const g = this.ctx!.createGain();
    g.gain.setValueAtTime(0.00001, t);
    g.gain.exponentialRampToValueAtTime(Math.max(0.00002, peak), t + attack);
    g.gain.exponentialRampToValueAtTime(0.00001, t + attack + decay);
    return g;
  }

  /** Glassy FM bell: a sine carrier, a sine modulator at `ratio`, its index decaying faster than the note. */
  private bell(f: number, t: number, peak: number, decay: number, dest: AudioNode, ratio = 3.5, index = 2.2) {
    const c = this.ctx!;
    const car = c.createOscillator(), mod = c.createOscillator(), mg = c.createGain();
    car.frequency.value = f; mod.frequency.value = f * ratio;
    mg.gain.setValueAtTime(f * index, t);
    mg.gain.exponentialRampToValueAtTime(f * 0.05, t + decay * 0.45);
    mod.connect(mg).connect(car.frequency);
    const g = this.env(t, peak, 0.004, decay);
    car.connect(g).connect(dest);
    car.start(t); mod.start(t); car.stop(t + decay + 0.05); mod.stop(t + decay + 0.05);
  }

  private tone(type: OscillatorType, f0: number, f1: number, t: number, peak: number, attack: number, decay: number, dest: AudioNode, filter?: { type: BiquadFilterType; f: number; q?: number }) {
    const c = this.ctx!, o = c.createOscillator();
    o.type = type;
    o.frequency.setValueAtTime(f0, t);
    o.frequency.exponentialRampToValueAtTime(Math.max(20, f1), t + attack + decay);
    let node: AudioNode = o;
    if (filter) { const fl = c.createBiquadFilter(); fl.type = filter.type; fl.frequency.value = filter.f; fl.Q.value = filter.q ?? 0.7; o.connect(fl); node = fl; }
    node.connect(this.env(t, peak, attack, decay)).connect(dest);
    o.start(t); o.stop(t + attack + decay + 0.05);
  }

  private noise(t: number, peak: number, attack: number, decay: number, dest: AudioNode, type: BiquadFilterType, f0: number, f1: number, q = 1, pink = false) {
    const c = this.ctx!, s = c.createBufferSource();
    s.buffer = pink ? this.pinkBuf : this.noiseBuf;
    s.playbackRate.value = 0.9 + Math.random() * 0.2;
    const fl = c.createBiquadFilter();
    fl.type = type; fl.Q.value = q;
    fl.frequency.setValueAtTime(f0, t);
    fl.frequency.exponentialRampToValueAtTime(Math.max(30, f1), t + attack + decay);
    s.connect(fl).connect(this.env(t, peak, attack, decay)).connect(dest);
    s.start(t, Math.random() * 1.5); s.stop(t + attack + decay + 0.05);
  }

  // ------------------------------------------------------------------ one-shots
  play(kind: Kind, o: PlayOpts = {}) {
    if (!this.ok()) return;
    const c = this.ctx!, t = c.currentTime + 0.005;
    // rate limit per kind (fast fire still reads as individual shots, not a buzz)
    const minGap: Partial<Record<Kind, number>> = { shot: 0.035, intake: 0.05, bounce: 0.06, impact: 0.12, score: 0.05, tap: 0.03 };
    const prev = this.last.get(kind) ?? -1;
    if (t - prev < (minGap[kind] ?? 0)) return;
    this.last.set(kind, t);
    const g = o.gain ?? 1, pan = o.pan ?? 0;
    this.trim = Math.pow(10, (TRIM[kind] ?? 0) / 20);
    const r = (a: number) => 1 + (Math.random() - 0.5) * a; // humanize
    switch (kind) {
      case 'tap': {
        const d = this.out(0, 0.12);
        this.bell(2350 * r(0.02), t, 0.05, 0.09, d, 3.01, 1.2);
        this.noise(t, 0.02, 0.001, 0.012, d, 'highpass', 6000, 6000, 0.7);
        break;
      }
      case 'select': {
        const d = this.out(0, 0.3);
        this.bell(1318.5, t, 0.06, 0.32, d, 2.01, 1.6);
        this.bell(1975.5, t + 0.07, 0.05, 0.45, d, 2.01, 1.4);
        break;
      }
      case 'focus': {
        const d = this.out(0, 0.45);
        this.noise(t, 0.07, 0.16, 0.34, d, 'bandpass', 420, 3200, 1.1, true);
        for (const [f, k] of [[523.25, 0], [659.25, 0.03], [987.77, 0.06]] as const) this.tone('sine', f, f * 1.002, t + 0.06 + k, 0.025, 0.06, 0.5, d);
        this.bell(2637, t + 0.2, 0.03, 0.7, d, 3.5, 1.2);
        break;
      }
      case 'back': {
        const d = this.out(0, 0.4);
        this.noise(t, 0.055, 0.04, 0.3, d, 'bandpass', 2600, 380, 1.1, true);
        this.bell(987.77, t + 0.02, 0.045, 0.3, d, 2.01, 1.2);
        this.bell(659.25, t + 0.09, 0.04, 0.38, d, 2.01, 1.0);
        break;
      }
      case 'hide': {
        const d = this.out(0, 0.25);
        this.tone('sine', 880, 280, t, 0.06, 0.005, 0.16, d);
        this.noise(t, 0.03, 0.005, 0.12, d, 'bandpass', 3200, 700, 1.4, true);
        break;
      }
      case 'show': {
        const d = this.out(0, 0.3);
        this.tone('sine', 300, 900, t, 0.055, 0.02, 0.16, d);
        this.bell(1760, t + 0.1, 0.035, 0.3, d, 2.01, 1.2);
        break;
      }
      case 'toggle': {
        // a switch: two tiny clicks a few milliseconds apart
        const d = this.out(0, 0.1);
        this.noise(t, 0.06, 0.001, 0.018, d, 'bandpass', 3400, 2800, 4);
        this.noise(t + 0.028, 0.035, 0.001, 0.022, d, 'bandpass', 1700, 1400, 4);
        this.tone('sine', 1800, 1500, t, 0.02, 0.001, 0.03, d);
        break;
      }
      case 'open': {
        const d = this.out(0, 0.3);
        this.noise(t, 0.08, 0.12, 0.45, d, 'lowpass', 300, 1800, 0.9, true);
        this.tone('sawtooth', 110, 220, t, 0.018, 0.08, 0.45, d, { type: 'lowpass', f: 900, q: 3 });
        this.noise(t + 0.5, 0.05, 0.002, 0.05, d, 'bandpass', 1400, 900, 3);
        break;
      }
      case 'boot': {
        const d = this.out(0, 0.55);
        this.tone('sine', 42, 58, t, 0.12, 0.35, 0.9, d);
        this.noise(t, 0.05, 0.7, 0.35, d, 'bandpass', 200, 5200, 0.9, true);
        [523.25, 659.25, 783.99, 987.77, 1174.66].forEach((f, i) => this.bell(f, t + 0.62 + i * 0.07, 0.035, 1.3 - i * 0.12, d, 2.01, 1.1));
        this.bell(2093, t + 1.0, 0.02, 1.2, d, 3.5, 1.4);
        break;
      }
      case 'error': {
        const d = this.out(0, 0.15);
        this.tone('triangle', 330, 300, t, 0.04, 0.005, 0.12, d, { type: 'lowpass', f: 1200 });
        this.tone('triangle', 262, 240, t + 0.13, 0.04, 0.005, 0.16, d, { type: 'lowpass', f: 1200 });
        break;
      }
      case 'shot': {
        // flywheel and hood throw a foam ball: a deep thump, the compression "whomp", a rubber slap
        const d = this.out(pan, 0.3), k = g * r(0.15);
        this.tone('sine', 165 * r(0.08), 42, t, 0.32 * k, 0.003, 0.14, d);
        this.noise(t, 0.16 * k, 0.004, 0.11, d, 'bandpass', 1300 * r(0.2), 320, 1.3, true);
        this.noise(t, 0.05 * k, 0.001, 0.03, d, 'bandpass', 2600 * r(0.15), 2000, 6);
        this.tone('sawtooth', 210 * r(0.1), 90, t + 0.004, 0.035 * k, 0.002, 0.05, d, { type: 'lowpass', f: 900 });
        break;
      }
      case 'score': {
        // fuel clunks through the HUB, and a bright two-tone ding
        const d = this.out(-0.2, 0.55);
        this.noise(t, 0.07, 0.002, 0.07, d, 'lowpass', 900, 300, 0.8, true);
        this.tone('sine', 130, 85, t, 0.08, 0.002, 0.09, d);
        this.bell(1567.98, t + 0.05, 0.04, 0.6, d, 3.5, 1.6);
        this.bell(2349.32, t + 0.12, 0.03, 0.8, d, 3.5, 1.4);
        break;
      }
      case 'intake': {
        // the roller grabs a ball: rubber thwap plus a soft body thump
        const d = this.out(pan, 0.15), k = g * r(0.2);
        this.noise(t, 0.09 * k, 0.003, 0.06, d, 'bandpass', 520 * r(0.2), 260, 1.6, true);
        this.tone('sine', 120 * r(0.1), 65, t, 0.09 * k, 0.003, 0.07, d);
        break;
      }
      case 'bounce': {
        const d = this.out(pan, 0.2), k = g * r(0.25);
        this.tone('sine', 240 * r(0.2), 120, t, 0.06 * k, 0.002, 0.05, d);
        this.noise(t, 0.03 * k, 0.001, 0.04, d, 'lowpass', 1400, 500, 0.8, true);
        break;
      }
      case 'impact': {
        // bumper into a wall: a heavy thud, the frame's rattle
        const d = this.out(pan, 0.35), k = Math.max(0.2, g);
        this.tone('sine', 82, 38, t, 0.4 * k, 0.003, 0.22, d);
        this.noise(t, 0.22 * k, 0.002, 0.18, d, 'lowpass', 600, 180, 0.9, true);
        this.noise(t + 0.01, 0.06 * k, 0.001, 0.09, d, 'bandpass', 2400, 1600, 3);
        break;
      }
      case 'takeover': {
        const d = this.out(0, 0.3);
        this.bell(880, t, 0.05, 0.25, d, 2.01, 1.4);
        this.bell(1318.5, t + 0.08, 0.05, 0.4, d, 2.01, 1.4);
        this.noise(t, 0.03, 0.002, 0.03, d, 'bandpass', 3000, 2500, 3);
        break;
      }
      case 'handback': {
        const d = this.out(0, 0.3);
        this.bell(1318.5, t, 0.045, 0.25, d, 2.01, 1.4);
        this.bell(880, t + 0.08, 0.045, 0.45, d, 2.01, 1.4);
        break;
      }
      case 'respawn': {
        const d = this.out(0, 0.5);
        this.noise(t, 0.08, 0.25, 0.4, d, 'bandpass', 300, 3000, 1, true);
        this.bell(1046.5, t + 0.35, 0.04, 0.8, d, 2.01, 1.3);
        break;
      }
      case 'connect': {
        const d = this.out(0, 0.3);
        [659.25, 987.77, 1318.5].forEach((f, i) => this.bell(f, t + i * 0.06, 0.04, 0.35, d, 2.01, 1.2));
        break;
      }
    }
  }

  // ------------------------------------------------------------------ the robot's machinery
  simStart() {
    if (!this.ctx || this.sim) return;
    const c = this.ctx;
    const bus = c.createGain();
    bus.gain.value = 0;
    const pan = c.createStereoPanner();
    bus.connect(pan);
    pan.connect(this.dry!);
    const send = c.createGain(); send.gain.value = 0.12; pan.connect(send).connect(this.wet!);
    const layers: Layer[] = [];
    const loop = (pink: boolean) => { const s = c.createBufferSource(); s.buffer = pink ? this.pinkBuf : this.noiseBuf; s.loop = true; s.start(0, Math.random()); return s; };
    const osc = (type: OscillatorType, f: number) => { const o = c.createOscillator(); o.type = type; o.frequency.value = f; o.start(); return o; };
    const filt = (type: BiquadFilterType, f: number, q = 1) => { const fl = c.createBiquadFilter(); fl.type = type; fl.frequency.value = f; fl.Q.value = q; return fl; };
    const gain = (v = 0) => { const g = c.createGain(); g.gain.value = v; return g; };
    const set = (p: AudioParam, v: number, t: number, tc = 0.06) => p.setTargetAtTime(v, t, tc);

    // drivetrain: four Krakens whining with speed, gear mesh noise, a body that hums under load
    {
      const g = gain(); g.connect(bus);
      const whine = osc('sawtooth', 120), wf = filt('bandpass', 900, 5), wg = gain();
      whine.connect(wf).connect(wg).connect(g);
      const hi = osc('sine', 1800), hg = gain(); hi.connect(hg).connect(g);
      const gear = loop(false), gf = filt('bandpass', 700, 1.6), gg = gain(); gear.connect(gf).connect(gg).connect(g);
      const body = osc('triangle', 48), bg = gain(); body.connect(bg).connect(g);
      const chirp = loop(false), cf = filt('bandpass', 2600, 5), cg = gain(); chirp.connect(cf).connect(cg).connect(g);
      layers.push({ src: [whine, hi, gear, body, chirp], gain: g, set: (s, t) => {
        const v = Math.min(1, s.speed / 4.7), w = Math.min(1, s.turn / 8), m = Math.max(v, w * 0.8);
        set(g.gain, 1, t);
        set(whine.frequency, 70 + m * 300, t, 0.05);
        set(wf.frequency, 400 + m * 1500, t, 0.05);
        set(wg.gain, 0.006 + 0.03 * s.effort + 0.02 * m, t, 0.05);
        set(hi.frequency, 1300 + m * 1500, t, 0.05);
        set(hg.gain, 0.002 + 0.008 * m * (0.4 + s.effort), t, 0.05);
        set(gf.frequency, 400 + m * 1600, t, 0.08);
        set(gg.gain, 0.012 * m + 0.02 * s.effort, t, 0.06);
        set(body.frequency, 44 + m * 30, t, 0.1);
        set(bg.gain, 0.03 * s.effort, t, 0.05);
        set(cg.gain, s.slipping ? 0.035 * Math.min(1, s.speed + 0.3) : 0, t, s.slipping ? 0.01 : 0.05);
      } });
    }
    // flywheels: each sings at its own speed (a 4" brass wheel: its hum, a hub whine, air rush)
    const fly = [0, 1].map((k) => {
      const g = gain(); g.connect(bus);
      const a = osc('triangle', 200), ag = gain(); a.connect(ag).connect(g);
      const b = osc('sine', 700), bgn = gain(); b.connect(bgn).connect(g);
      const air = loop(true), af = filt('bandpass', 1200, 2), airg = gain(); air.connect(af).connect(airg).connect(g);
      return { src: [a, b, air], gain: g, set: (s: SimSoundState, t: number) => {
        const w = s.flywheels[k] ?? 0, f0 = w / (Math.PI * 2), level = Math.min(1, w / 450);
        set(g.gain, 1, t);
        set(a.frequency, Math.max(20, f0 * 4 * (1 + k * 0.012)), t, 0.04);
        set(ag.gain, 0.022 * level * level, t, 0.05);
        set(b.frequency, Math.max(40, f0 * 13 * (1 + k * 0.012)), t, 0.04);
        set(bgn.gain, 0.006 * level * level, t, 0.05);
        set(af.frequency, 400 + f0 * 22, t, 0.05);
        set(airg.gain, 0.018 * level, t, 0.05);
      } } as Layer;
    });
    layers.push(...fly);
    // intake rollers: a low rumble with the flutter of the rollers' surface
    {
      const g = gain(); g.connect(bus);
      const rum = loop(true), rf = filt('lowpass', 220, 0.8), rg = gain(); rum.connect(rf).connect(rg).connect(g);
      const hum = osc('sawtooth', 58), hf = filt('lowpass', 300, 2), hg = gain(); hum.connect(hf).connect(hg).connect(g);
      const lfo = osc('sine', 23), lg = gain(0.012); lfo.connect(lg).connect(hg.gain);
      // indexer belts and the four-bar's motor
      const belt = loop(false), bf = filt('bandpass', 1300, 2.5), bg = gain(); belt.connect(bf).connect(bg).connect(g);
      const arm = osc('sawtooth', 160), af = filt('bandpass', 640, 4), ag = gain(); arm.connect(af).connect(ag).connect(g);
      layers.push({ src: [rum, hum, lfo, belt, arm], gain: g, set: (s, t) => {
        set(g.gain, 1, t);
        set(rg.gain, 0.05 * s.rollers, t, 0.12);
        set(hg.gain, 0.012 * s.rollers, t, 0.12);
        set(bg.gain, 0.03 * s.feeding, t, 0.04);
        set(ag.gain, 0.02 * s.arm, t, 0.03);
      } });
    }
    this.sim = { bus, pan, layers };
  }

  simUpdate(s: SimSoundState) {
    if (!this.sim || !this.ctx) { if (this.ctx && this.enabled) this.simStart(); return; }
    const t = this.ctx.currentTime;
    const on = this.ok() ? 1 : 0;
    this.sim.bus.gain.setTargetAtTime(on * 4 * (0.35 + 0.65 * s.near), t, 0.15);
    this.sim.pan.pan.setTargetAtTime(Math.max(-0.8, Math.min(0.8, s.pan)), t, 0.1);
    for (const l of this.sim.layers) l.set(s, t);
  }

  simStop() {
    if (!this.sim || !this.ctx) return;
    const s = this.sim;
    this.sim = null;
    s.bus.gain.setTargetAtTime(0, this.ctx.currentTime, 0.08);
    setTimeout(() => { for (const l of s.layers) for (const n of l.src) { try { n.stop(); } catch { /* already stopped */ } } s.bus.disconnect(); }, 600);
  }

  /** Mechanism motor while something moves on the robot (dragging the intake, the turrets…); speed 0..1. */
  servoSpeed(speed: number) {
    if (!this.ok()) { this.stopServo(); return; }
    const c = this.ctx!;
    if (speed < 0.02) { this.stopServo(); return; }
    if (!this.servo) {
      const f1 = c.createOscillator(), f2 = c.createOscillator(), gain = c.createGain(), filter = c.createBiquadFilter();
      f1.type = 'sawtooth'; f2.type = 'triangle';
      filter.type = 'bandpass'; filter.Q.value = 3;
      gain.gain.value = 0.00001;
      f1.connect(filter); f2.connect(filter);
      const gear = c.createBufferSource(); gear.buffer = this.noiseBuf; gear.loop = true;
      const gf = c.createBiquadFilter(); gf.type = 'bandpass'; gf.frequency.value = 1800; gf.Q.value = 2;
      const gg = c.createGain(); gg.gain.value = 0.25;
      gear.connect(gf).connect(gg).connect(filter);
      filter.connect(gain).connect(this.out(0, 0.15));
      f1.start(); f2.start(); gear.start();
      this.servo = { nodes: [f1, f2, gear], gain, f1, f2, filter };
    }
    const t = c.currentTime, s = Math.min(1, speed);
    this.servo.f1.frequency.setTargetAtTime(110 + s * 240, t, 0.05);
    this.servo.f2.frequency.setTargetAtTime(330 + s * 700, t, 0.05);
    this.servo.filter.frequency.setTargetAtTime(600 + s * 1400, t, 0.05);
    this.servo.gain.gain.setTargetAtTime(0.01 + s * 0.028, t, 0.04);
  }

  stopServo() {
    if (!this.servo || !this.ctx) return;
    const s = this.servo;
    this.servo = null;
    s.gain.gain.setTargetAtTime(0.00001, this.ctx.currentTime, 0.05);
    setTimeout(() => { for (const n of s.nodes) n.stop(); }, 300);
  }
}

export const sound = new Sound();
