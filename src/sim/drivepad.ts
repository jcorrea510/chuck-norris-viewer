// On-screen driving controls for iPad and phones: a floating stick under the left thumb drives,
// a stick under the right thumb turns, and big buttons beside it shoot, intake, eject and raise or
// lower the intake. Each stick appears wherever the thumb lands; anywhere else still moves the
// camera.
import { icon } from '../icons';
import type { TouchState } from './input';

const RADIUS = 56; // px the knob can travel

export class Drivepad {
  readonly el: HTMLElement;
  private armBtn: HTMLButtonElement;

  constructor(parent: HTMLElement, private state: TouchState) {
    this.el = document.createElement('div');
    this.el.className = 'drivepad';
    this.el.innerHTML = `
      <div class="dp-zone dp-move" aria-label="Drive: drag to move"><div class="dp-base"><div class="dp-knob"></div></div><span class="dp-tag">Drive</span></div>
      <div class="dp-zone dp-turn" aria-label="Turn: drag left or right"><div class="dp-base dp-wide"><div class="dp-knob"></div></div><span class="dp-tag">Turn</span></div>
      <div class="dp-buttons">
        <button class="dp-btn dp-shoot" data-k="shoot" aria-label="Shoot (hold)">${icon('crosshair')}<span>Shoot</span></button>
        <button class="dp-btn dp-intake" data-k="intake" aria-label="Intake (hold)">${icon('arrow-big-down')}<span>Intake</span></button>
        <button class="dp-btn dp-eject" data-k="eject" aria-label="Eject (hold)">${icon('undo-2')}<span>Eject</span></button>
        <button class="dp-btn dp-arm" data-k="arm" aria-label="Raise or lower the intake">${icon('arrow-big-up')}<span>Raise</span></button>
      </div>`;
    parent.append(this.el);
    this.armBtn = this.el.querySelector('.dp-arm')!;
    this.stick(this.el.querySelector('.dp-move')!, (x, y) => { this.state.mx = x; this.state.my = y; }, false);
    this.stick(this.el.querySelector('.dp-turn')!, (x) => { this.state.turn = x; }, true);
    this.el.querySelectorAll<HTMLButtonElement>('.dp-btn').forEach((b) => {
      const k = b.dataset.k as 'shoot' | 'intake' | 'eject' | 'arm';
      const set = (on: boolean) => { this.state[k] = on; b.classList.toggle('on', on); };
      b.addEventListener('pointerdown', (e) => { e.preventDefault(); e.stopPropagation(); b.setPointerCapture(e.pointerId); set(true); if (navigator.vibrate) navigator.vibrate(6); });
      for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(t, () => set(false));
      b.addEventListener('contextmenu', (e) => e.preventDefault());
    });
  }

  /** A floating thumbstick: it centers where the thumb lands and follows it within its radius. */
  private stick(zone: HTMLElement, fn: (x: number, y: number) => void, horizontal: boolean) {
    const base = zone.querySelector<HTMLElement>('.dp-base')!, knob = zone.querySelector<HTMLElement>('.dp-knob')!;
    let id = -1, cx = 0, cy = 0;
    const move = (e: PointerEvent) => {
      let dx = e.clientX - cx, dy = horizontal ? 0 : e.clientY - cy;
      const m = Math.hypot(dx, dy);
      if (m > RADIUS) { dx *= RADIUS / m; dy *= RADIUS / m; }
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
      // a small dead zone, then a gentle curve (fine control near the middle)
      const k = (v: number) => { const a = Math.max(0, Math.abs(v) / RADIUS - 0.08) / 0.92; return Math.sign(v) * a * (0.35 + 0.65 * a); };
      fn(k(dx), -k(dy));
    };
    zone.addEventListener('pointerdown', (e) => {
      if (id !== -1) return;
      e.preventDefault(); e.stopPropagation();
      id = e.pointerId;
      zone.setPointerCapture(id);
      const r = zone.getBoundingClientRect();
      cx = e.clientX; cy = e.clientY;
      base.style.left = `${cx - r.left}px`; base.style.top = `${cy - r.top}px`;
      zone.classList.add('active');
      move(e);
    });
    zone.addEventListener('pointermove', (e) => { if (e.pointerId === id) move(e); });
    const end = (e: PointerEvent) => {
      if (e.pointerId !== id) return;
      id = -1;
      zone.classList.remove('active');
      knob.style.transform = '';
      base.style.left = ''; base.style.top = '';
      fn(0, 0);
    };
    zone.addEventListener('pointerup', end);
    zone.addEventListener('pointercancel', end);
    zone.addEventListener('lostpointercapture', end);
  }

  setVisible(on: boolean) { this.el.classList.toggle('on', on); if (!on) Object.assign(this.state, { mx: 0, my: 0, turn: 0, shoot: false, intake: false, eject: false, arm: false }); }
  /** The arm button says what it will do next. */
  setArm(down: boolean) {
    const label = down ? 'Raise' : 'Lower';
    if (this.armBtn.dataset.label === label) return;
    this.armBtn.dataset.label = label;
    this.armBtn.innerHTML = `${icon(down ? 'arrow-big-up' : 'arrow-big-down')}<span>${label}</span>`;
  }
  dispose() { this.el.remove(); }
}
