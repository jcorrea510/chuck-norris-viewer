// Driving Chuck Norris yourself: keyboard, any standard game controller (Xbox, PlayStation,
// Switch Pro… through the browser's Gamepad API), and the on-screen touch controls. The defaults
// are the team's own driver-controller map (RobotContainer.java). Every binding can be changed on
// the Controls page and is remembered on this device.

export type Action = 'forward' | 'back' | 'left' | 'right' | 'turnLeft' | 'turnRight' | 'intake' | 'retract' | 'shoot' | 'faceHub' | 'unjam' | 'reverseIntake' | 'resetHeading' | 'turretLock' | 'camera' | 'handBack';

/** `pad: false` = not a single rebindable button (a stick, or the Menu + B chord). */
export const ACTIONS: { id: Action; label: string; hint: string; pad: boolean }[] = [
  { id: 'forward', label: 'Drive forward', hint: 'Toward the opponent’s wall (up on screen)', pad: false },
  { id: 'back', label: 'Drive back', hint: 'Toward our wall', pad: false },
  { id: 'left', label: 'Strafe left', hint: '', pad: false },
  { id: 'right', label: 'Strafe right', hint: '', pad: false },
  { id: 'turnLeft', label: 'Rotate left', hint: 'Counter-clockwise', pad: false },
  { id: 'turnRight', label: 'Rotate right', hint: 'Clockwise', pad: false },
  { id: 'intake', label: 'Intake', hint: 'Hold: deploys the intake and runs the two big rollers. Let go: the rollers stop, the intake stays down', pad: true },
  { id: 'retract', label: 'Intake up', hint: 'Stows the intake', pad: true },
  { id: 'shoot', label: 'Shoot', hint: 'Hold: the five hopper rollers and the indexer feed the spinning turrets', pad: true },
  { id: 'faceHub', label: 'Face HUB', hint: 'Hold: the robot turns to point at the HUB while you drive', pad: true },
  { id: 'unjam', label: 'Unjam', hint: 'Hold: runs the rollers backwards to clear a jam', pad: true },
  { id: 'reverseIntake', label: 'Reverse intake', hint: 'Hold: spits fuel out through the intake', pad: false },
  { id: 'resetHeading', label: 'Reset heading', hint: 'Makes the way the robot faces now “forward”', pad: true },
  { id: 'turretLock', label: 'Turret lock', hint: 'Toggles both turrets locked pointing backward (over the turret end) instead of tracking', pad: true },
  { id: 'camera', label: 'Next camera', hint: 'Driver · Follow · Overhead', pad: true },
  { id: 'handBack', label: 'Give control back', hint: 'The robot drives itself again', pad: true },
];

export interface Bindings {
  keys: Record<Action, string[]>;
  pad: Partial<Record<Action, number>>;
}

export const DEFAULT_BINDINGS: Bindings = {
  keys: {
    forward: ['KeyW'], back: ['KeyS'], left: ['KeyA'], right: ['KeyD'],
    turnLeft: ['ArrowLeft'], turnRight: ['ArrowRight'],
    intake: ['ShiftLeft', 'ShiftRight'], retract: ['KeyR'], shoot: ['KeyK', 'Space'], faceHub: ['KeyF'], unjam: ['KeyB'], reverseIntake: ['KeyX'],
    resetHeading: ['KeyY'], turretLock: ['KeyL'], camera: ['Tab'], handBack: ['KeyH'],
  },
  // standard gamepad layout: 0 A/✕, 1 B/○, 2 X/□, 3 Y/△, 4 LB, 5 RB, 6 LT, 7 RT, 8 View, 9 Menu, 10 L-stick click, 12-15 d-pad
  pad: { intake: 6, retract: 5, shoot: 7, faceHub: 0, unjam: 1, resetHeading: 3, turretLock: 13, camera: 8, handBack: 10 },
};

/** Menu (Start) is the modifier of the Menu + B chord: B alone unjams, Menu + B reverses the intake. */
export const PAD_MENU = 9;

const STORE = 'chuck-norris-controls-v3';
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

/** Names for a binding's keys; the two Shift keys (or Ctrl, Alt) read as one. */
export function keyLabels(codes: string[]): string[] {
  const out: string[] = [];
  for (const c of codes) {
    const m = /^(Shift|Control|Alt)(Left|Right)$/.exec(c);
    const name = m && codes.includes(m[1] + (m[2] === 'Left' ? 'Right' : 'Left')) ? (m[1] === 'Control' ? 'Ctrl' : m[1]) : keyName(c);
    if (!out.includes(name)) out.push(name);
  }
  return out;
}

/** What the person is asking for this frame, before it is turned into the field frame. */
export interface RawInput {
  /** stick, screen-relative: x right, y away from you; raw (the robot code applies the deadband and squaring), length ≤ 1 */
  mx: number; my: number;
  /** raw right-stick X, + = clockwise */
  turn: number;
  /** held */
  intake: boolean; shoot: boolean; unjam: boolean; reverseIntake: boolean; faceHub: boolean;
  /** pressed this frame */
  retract: boolean; resetHeading: boolean; turretLock: boolean; camera: boolean; handBack: boolean;
  /** any control touched this frame */
  active: boolean;
  source: 'keyboard' | 'gamepad' | 'touch' | null;
}

/** On-screen controls report into this (see drivepad.ts): `intake` is the latched Intake button, `retractTap` the Up button. */
export interface TouchState { mx: number; my: number; turn: number; shoot: boolean; intake: boolean; retractTap: boolean }

/** Below this a stick counts as untouched when deciding whether someone is driving (drift must not take the robot over). */
const DRIFT = 0.12;
/** The team code's `leftTrigger(0.5)`: a trigger is held past half pull. */
const TRIGGER = 0.5;

/** Clamps a stick vector to the unit disc; the values stay raw. */
function clampStick(x: number, y: number): [number, number] {
  const m = Math.hypot(x, y);
  return m > 1 ? [x / m, y / m] : [x, y];
}

export class DriverControls {
  bindings = loadBindings();
  private down = new Set<string>();
  private pressed = new Set<string>();
  private padPrev: boolean[] = [];
  private attached = false;
  touch: TouchState = { mx: 0, my: 0, turn: 0, shoot: false, intake: false, retractTap: false };
  /** a rebinding in progress takes the next key or button */
  private capture: { kind: 'key' | 'pad'; fn: (code: string | number) => void } | null = null;
  padConnected: string | null = null;
  onPad: ((name: string | null) => void) | null = null;

  private onKeyDown = (e: KeyboardEvent) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const target = e.target as HTMLElement | null;
    if (target?.closest?.('input, textarea, select')) return;
    if (this.capture?.kind === 'key') {
      e.preventDefault(); e.stopPropagation();
      const c = this.capture; this.capture = null;
      if (e.code !== 'Escape') c.fn(e.code); else c.fn('');
      return;
    }
    // Tab moves focus through the settings sheet and the Controls page: only the field itself uses it
    if (e.code === 'Tab' && target?.closest?.('.sheet-backdrop')) return;
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
    const kx = (keyHeld('right') ? 1 : 0) - (keyHeld('left') ? 1 : 0);
    const ky = (keyHeld('forward') ? 1 : 0) - (keyHeld('back') ? 1 : 0);
    const [kmx, kmy] = clampStick(kx, ky);
    const out: RawInput = {
      mx: kmx, my: kmy, turn: (keyHeld('turnRight') ? 1 : 0) - (keyHeld('turnLeft') ? 1 : 0),
      intake: keyHeld('intake'), shoot: keyHeld('shoot'), unjam: keyHeld('unjam'), reverseIntake: keyHeld('reverseIntake'), faceHub: keyHeld('faceHub'),
      retract: keyHit('retract'), resetHeading: keyHit('resetHeading'), turretLock: keyHit('turretLock'), camera: keyHit('camera'), handBack: keyHit('handBack'),
      active: false, source: null,
    };
    if (this.down.size) source = 'keyboard';
    this.pressed.clear();
    // controller
    const g = this.pollPad();
    let stickActive = false;
    if (g) {
      const btn = (i: number | undefined) => { const b = i === undefined ? undefined : g.buttons[i]; return !!b && (i === 6 || i === 7 ? b.value > TRIGGER : b.pressed || b.value > TRIGGER); };
      if (this.capture?.kind === 'pad') {
        for (let i = 0; i < g.buttons.length; i++) if (btn(i) && !this.padPrev[i]) { const c = this.capture; this.capture = null; c.fn(i); break; }
      }
      const edge = (i: number | undefined) => i !== undefined && btn(i) && !this.padPrev[i];
      const [sx, sy] = clampStick(g.axes[0] ?? 0, -(g.axes[1] ?? 0));
      const tx = Math.max(-1, Math.min(1, g.axes[2] ?? 0));
      const p = this.bindings.pad;
      stickActive = Math.hypot(sx, sy) > DRIFT || Math.abs(tx) > DRIFT;
      if (Math.hypot(sx, sy) > Math.hypot(out.mx, out.my)) { out.mx = sx; out.my = sy; }
      if (Math.abs(tx) > Math.abs(out.turn)) out.turn = tx;
      // B alone unjams, Menu + B reverses the intake (driver.b().and(start().negate()) / start().and(b()))
      const menu = btn(PAD_MENU);
      out.intake ||= btn(p.intake); out.shoot ||= btn(p.shoot); out.faceHub ||= btn(p.faceHub);
      out.unjam ||= btn(p.unjam) && !menu; out.reverseIntake ||= btn(p.unjam) && menu;
      out.retract ||= edge(p.retract); out.resetHeading ||= edge(p.resetHeading); out.turretLock ||= edge(p.turretLock);
      out.camera ||= edge(p.camera); out.handBack ||= edge(p.handBack);
      if (stickActive || g.buttons.some((b, i) => i < 16 && b.pressed)) source = 'gamepad';
      this.padPrev = g.buttons.map((_, i) => btn(i));
    }
    // touch
    const t = this.touch;
    if (t.mx || t.my || t.turn || t.shoot || t.intake || t.retractTap) source = 'touch';
    const [tmx, tmy] = clampStick(t.mx, t.my);
    if (Math.hypot(tmx, tmy) > Math.hypot(out.mx, out.my)) { out.mx = tmx; out.my = tmy; }
    if (Math.abs(t.turn) > Math.abs(out.turn)) out.turn = Math.max(-1, Math.min(1, t.turn));
    out.shoot ||= t.shoot;
    if (t.retractTap) { out.retract = true; t.retractTap = false; }
    // stowing the intake also ends a latched Intake
    if (out.retract) t.intake = false;
    out.intake ||= t.intake;
    out.source = source;
    const drive = Math.hypot(out.mx, out.my) > DRIFT || Math.abs(out.turn) > DRIFT;
    out.active = drive || out.intake || out.shoot || out.unjam || out.reverseIntake || out.faceHub || out.retract || out.resetHeading || out.turretLock;
    return out;
  }
}

/** No driver input (also exported by world.ts; here so the renderer needn't load the physics). */
export const NO_INPUT = { mx: 0, mz: 0, turn: 0, intake: false, shoot: false, unjam: false, reverseIntake: false, faceHub: false, retract: false, resetHeading: false, turretLock: false, handBack: false };
