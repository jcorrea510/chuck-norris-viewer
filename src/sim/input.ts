// Driving Chuck Norris yourself: keyboard, any standard game controller (Xbox, PlayStation,
// Switch Pro… through the browser's Gamepad API), and the on-screen touch controls. Every binding
// can be changed on the Controls page and is remembered on this device.

export type Action = 'forward' | 'back' | 'left' | 'right' | 'turnLeft' | 'turnRight' | 'shoot' | 'intake' | 'eject' | 'arm' | 'lock' | 'slow' | 'camera' | 'handBack';

export const ACTIONS: { id: Action; label: string; hint: string; pad: boolean }[] = [
  { id: 'forward', label: 'Drive forward', hint: 'Toward the opponent’s wall (up on screen)', pad: false },
  { id: 'back', label: 'Drive back', hint: 'Toward our wall', pad: false },
  { id: 'left', label: 'Strafe left', hint: '', pad: false },
  { id: 'right', label: 'Strafe right', hint: '', pad: false },
  { id: 'turnLeft', label: 'Turn left', hint: 'Counter-clockwise', pad: false },
  { id: 'turnRight', label: 'Turn right', hint: 'Clockwise', pad: false },
  { id: 'shoot', label: 'Shoot', hint: 'Hold: turrets aim and fire', pad: true },
  { id: 'intake', label: 'Intake on / off', hint: 'On: drops the intake and runs the rollers. Off: stops and stows it', pad: true },
  { id: 'eject', label: 'Eject', hint: 'Hold: rollers spit fuel out the front', pad: true },
  { id: 'arm', label: 'Intake up / down', hint: 'Raise or lower the four-bar', pad: true },
  { id: 'lock', label: 'X-lock', hint: 'Hold: wheels lock in an X', pad: true },
  { id: 'slow', label: 'Precision', hint: 'Hold: slow and fine', pad: true },
  { id: 'camera', label: 'Next camera', hint: 'Driver · Follow · Overhead', pad: true },
  { id: 'handBack', label: 'Give control back', hint: 'The robot drives itself again', pad: true },
];

export interface Bindings {
  keys: Record<Action, string[]>;
  pad: Partial<Record<Action, number>>;
}

export const DEFAULT_BINDINGS: Bindings = {
  keys: {
    forward: ['KeyW', 'ArrowUp'], back: ['KeyS', 'ArrowDown'], left: ['KeyA', 'ArrowLeft'], right: ['KeyD', 'ArrowRight'],
    turnLeft: ['KeyQ'], turnRight: ['KeyE'],
    shoot: ['Space'], intake: ['KeyF'], eject: ['KeyX'], arm: ['KeyR'], lock: ['KeyC'], slow: ['ShiftLeft'],
    camera: ['KeyV'], handBack: ['KeyH'],
  },
  // standard gamepad layout: 0 A/✕, 1 B/○, 2 X/□, 3 Y/△, 4 LB, 5 RB, 6 LT, 7 RT, 8 View, 9 Menu, 12-15 d-pad
  pad: { shoot: 7, intake: 6, arm: 0, eject: 1, lock: 2, camera: 3, slow: 4, handBack: 9 },
};

const STORE = 'chuck-norris-controls-v2';
export function loadBindings(): Bindings {
  try {
    const raw = localStorage.getItem(STORE);
    if (raw) {
      const b = JSON.parse(raw) as Partial<Bindings>;
      return { keys: { ...DEFAULT_BINDINGS.keys, ...b.keys }, pad: { ...DEFAULT_BINDINGS.pad, ...b.pad } };
    }
  } catch { /* storage unavailable */ }
  return structuredClone(DEFAULT_BINDINGS);
}
export function saveBindings(b: Bindings) { try { localStorage.setItem(STORE, JSON.stringify(b)); } catch { /* ignore */ } }

const PAD_NAMES = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'LT', 'RT', 'View', 'Menu', 'L-stick', 'R-stick', 'D-pad ↑', 'D-pad ↓', 'D-pad ←', 'D-pad →', 'Home'];
export const padName = (i: number | undefined) => i === undefined ? '—' : PAD_NAMES[i] ?? `Button ${i}`;
export function keyName(code: string) {
  if (!code) return '—';
  const m: Record<string, string> = { Space: 'Space', ShiftLeft: 'Left Shift', ShiftRight: 'Right Shift', ControlLeft: 'Left Ctrl', ControlRight: 'Right Ctrl', AltLeft: 'Left Alt', AltRight: 'Right Alt', ArrowUp: '↑', ArrowDown: '↓', ArrowLeft: '←', ArrowRight: '→', Enter: 'Return', Tab: 'Tab', Backspace: 'Delete', CapsLock: 'Caps Lock', Escape: 'Esc', Slash: '/', Period: '.', Comma: ',', Semicolon: ';', Quote: "'", BracketLeft: '[', BracketRight: ']', Backslash: '\\', Minus: '-', Equal: '=', Backquote: '`' };
  if (m[code]) return m[code];
  if (code.startsWith('Key')) return code.slice(3);
  if (code.startsWith('Digit')) return code.slice(5);
  if (code.startsWith('Numpad')) return 'Num ' + code.slice(6);
  return code;
}

/** What the person is asking for this frame, before it is turned into the field frame. */
export interface RawInput {
  /** stick, screen-relative: x right, y away from you; length ≤ 1 */
  mx: number; my: number;
  turn: number;
  shoot: boolean; eject: boolean; lock: boolean; slow: boolean;
  intakeToggle: boolean; arm: boolean; camera: boolean; handBack: boolean;   // pressed this frame
  /** any control touched this frame */
  active: boolean;
  source: 'keyboard' | 'gamepad' | 'touch' | null;
}

/** On-screen controls report into this (see drivepad.ts). */
export interface TouchState { mx: number; my: number; turn: number; shoot: boolean; intakeTap: boolean }

const DEAD = 0.12;
/** Deadband, rescaled, with a gentle curve so small stick moves are fine and full stick is full speed. */
function shape(x: number, y: number): [number, number] {
  const m = Math.hypot(x, y);
  if (m < DEAD) return [0, 0];
  const k = Math.min(1, (m - DEAD) / (1 - DEAD));
  const curved = k * (0.35 + 0.65 * k);
  return [(x / m) * curved, (y / m) * curved];
}

export class DriverControls {
  bindings = loadBindings();
  private down = new Set<string>();
  private pressed = new Set<string>();
  private padPrev: boolean[] = [];
  private attached = false;
  touch: TouchState = { mx: 0, my: 0, turn: 0, shoot: false, intakeTap: false };
  /** a rebinding in progress takes the next key or button */
  private capture: { kind: 'key' | 'pad'; fn: (code: string | number) => void } | null = null;
  padConnected: string | null = null;
  onPad: ((name: string | null) => void) | null = null;

  private onKeyDown = (e: KeyboardEvent) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    if ((e.target as HTMLElement | null)?.closest?.('input, textarea')) return;
    if (this.capture?.kind === 'key') {
      e.preventDefault(); e.stopPropagation();
      const c = this.capture; this.capture = null;
      if (e.code !== 'Escape') c.fn(e.code); else c.fn('');
      return;
    }
    if (this.bound(e.code)) {
      e.preventDefault();
      // a focused HUD button would otherwise also "click" on Space
      const f = document.activeElement as HTMLElement | null;
      if (f && f !== document.body && f.tagName === 'BUTTON') f.blur();
      if (!e.repeat) this.pressed.add(e.code);
      this.down.add(e.code);
    }
  };
  private onKeyUp = (e: KeyboardEvent) => { this.down.delete(e.code); };
  private onBlur = () => { this.down.clear(); };
  private onPadConnect = () => this.pollPad();

  private bound(code: string) {
    for (const list of Object.values(this.bindings.keys)) if (list.includes(code)) return true;
    return false;
  }

  attach() {
    if (this.attached) return;
    this.attached = true;
    window.addEventListener('keydown', this.onKeyDown, true);
    window.addEventListener('keyup', this.onKeyUp, true);
    window.addEventListener('blur', this.onBlur);
    window.addEventListener('gamepadconnected', this.onPadConnect);
    window.addEventListener('gamepaddisconnected', this.onPadConnect);
  }
  detach() {
    if (!this.attached) return;
    this.attached = false;
    window.removeEventListener('keydown', this.onKeyDown, true);
    window.removeEventListener('keyup', this.onKeyUp, true);
    window.removeEventListener('blur', this.onBlur);
    window.removeEventListener('gamepadconnected', this.onPadConnect);
    window.removeEventListener('gamepaddisconnected', this.onPadConnect);
    this.down.clear(); this.pressed.clear();
  }

  /** Waits for the next key (code) or controller button (index) for a rebinding. */
  captureNext(kind: 'key' | 'pad', fn: (code: string | number) => void) { this.capture = { kind, fn }; this.padPrev = []; }
  cancelCapture() { this.capture = null; }
  save() { saveBindings(this.bindings); }

  private pad(): Gamepad | null {
    const list = navigator.getGamepads?.() ?? [];
    for (const g of list) if (g && g.connected && (g.mapping === 'standard' || g.buttons.length >= 12)) return g;
    for (const g of list) if (g && g.connected) return g;
    return null;
  }
  private pollPad() {
    const g = this.pad();
    const name = g ? g.id.replace(/\s*\(.*?\)\s*/g, ' ').replace(/\s+/g, ' ').trim() || 'Controller' : null;
    if (name !== this.padConnected) { this.padConnected = name; this.onPad?.(name); }
    return g;
  }

  read(): RawInput {
    const keyHeld = (a: Action) => this.bindings.keys[a].some((c) => this.down.has(c));
    const keyHit = (a: Action) => this.bindings.keys[a].some((c) => this.pressed.has(c));
    let source: RawInput['source'] = null;
    // keyboard
    let kx = (keyHeld('right') ? 1 : 0) - (keyHeld('left') ? 1 : 0);
    let ky = (keyHeld('forward') ? 1 : 0) - (keyHeld('back') ? 1 : 0);
    const km = Math.hypot(kx, ky); if (km > 1) { kx /= km; ky /= km; }
    const kt = (keyHeld('turnRight') ? 1 : 0) - (keyHeld('turnLeft') ? 1 : 0);
    const out: RawInput = {
      mx: kx, my: ky, turn: kt,
      shoot: keyHeld('shoot'), eject: keyHeld('eject'), lock: keyHeld('lock'), slow: keyHeld('slow'),
      intakeToggle: keyHit('intake'), arm: keyHit('arm'), camera: keyHit('camera'), handBack: keyHit('handBack'),
      active: false, source: null,
    };
    if (this.down.size) source = 'keyboard';
    this.pressed.clear();
    // controller
    const g = this.pollPad();
    if (g) {
      const btn = (i: number | undefined) => i !== undefined && !!g.buttons[i] && (g.buttons[i].pressed || g.buttons[i].value > 0.35);
      if (this.capture?.kind === 'pad') {
        for (let i = 0; i < g.buttons.length; i++) if (btn(i) && !this.padPrev[i]) { const c = this.capture; this.capture = null; c.fn(i); break; }
      }
      const edge = (i: number | undefined) => i !== undefined && btn(i) && !this.padPrev[i];
      const [sx, sy] = shape(g.axes[0] ?? 0, -(g.axes[1] ?? 0));
      const [tx] = shape(g.axes[2] ?? 0, 0);
      const p = this.bindings.pad;
      if (Math.hypot(sx, sy) > Math.hypot(out.mx, out.my)) { out.mx = sx; out.my = sy; }
      if (Math.abs(tx) > Math.abs(out.turn)) out.turn = tx;
      out.shoot ||= btn(p.shoot); out.eject ||= btn(p.eject); out.lock ||= btn(p.lock); out.slow ||= btn(p.slow);
      out.intakeToggle ||= edge(p.intake); out.arm ||= edge(p.arm); out.camera ||= edge(p.camera); out.handBack ||= edge(p.handBack);
      if (sx || sy || tx || g.buttons.some((b, i) => i < 16 && b.pressed)) source = 'gamepad';
      this.padPrev = g.buttons.map((_, i) => btn(i));
    }
    // touch
    const t = this.touch;
    if (t.mx || t.my || t.turn || t.shoot || t.intakeTap) source = 'touch';
    if (Math.hypot(t.mx, t.my) > Math.hypot(out.mx, out.my)) { out.mx = t.mx; out.my = t.my; }
    if (Math.abs(t.turn) > Math.abs(out.turn)) out.turn = t.turn;
    out.shoot ||= t.shoot;
    if (t.intakeTap) { out.intakeToggle = true; t.intakeTap = false; }
    out.source = source;
    out.active = Math.hypot(out.mx, out.my) > 0.05 || Math.abs(out.turn) > 0.05 || out.shoot || out.intakeToggle || out.eject || out.lock || out.arm;
    return out;
  }
}

/** No driver input (also exported by world.ts; here so the renderer needn't load the physics). */
export const NO_INPUT = { mx: 0, mz: 0, turn: 0, shoot: false, intake: false, eject: false, lock: false, slow: false, toggleArm: false, toggleIntake: false, handBack: false };
