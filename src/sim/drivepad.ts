// On-screen driving controls for iPad and phones: a floating stick appears under the left thumb
// and drives (field-relative), a drag area under the right thumb turns the robot, and two large
// buttons sit beside it: Intake (tap: on / off) and Shoot (hold). Every touch is captured, so
// driving and shooting work together and nothing ever scrolls or zooms the page.
import { icon } from '../icons';
import type { TouchState } from './input';

const RADIUS = 60; // px the knob can travel

export class Drivepad {
  readonly el: HTMLElement;
  private intakeBtn: HTMLButtonElement;

  constructor(parent: HTMLElement, private state: TouchState) {
    this.el = document.createElement('div');
    this.el.className = 'drivepad';
    this.el.innerHTML = `
      <div class="dp-zone dp-move" aria-label="Drive: touch and drag"><div class="dp-base"><div class="dp-knob"></div></div><span class="dp-tag">Drive</span></div>
      <div class="dp-zone dp-turn" aria-label="Turn: drag left or right"><div class="dp-base dp-wide"><div class="dp-knob"></div></div><span class="dp-tag">Turn</span></div>
      <div class="dp-buttons">
        <button class="dp-btn dp-intake" aria-pressed="false" aria-label="Intake on or off">${icon('arrow-big-down')}<span>Intake</span></button>
        <button class="dp-btn dp-shoot" aria-label="Shoot (hold)">${icon('crosshair')}<span>Shoot</span></button>
      </div>`;
    parent.append(this.el);
    this.intakeBtn = this.el.querySelector('.dp-intake')!;
    this.stick(this.el.querySelector('.dp-move')!, (x, y) => { this.state.mx = x; this.state.my = y; }, false);
    this.stick(this.el.querySelector('.dp-turn')!, (x) => { this.state.turn = x; }, true);
    const shoot = this.el.querySelector<HTMLButtonElement>('.dp-shoot')!;
    const set = (on: boolean) => { this.state.shoot = on; shoot.classList.toggle('on', on); };
    shoot.addEventListener('pointerdown', (e) => { e.preventDefault(); e.stopPropagation(); try { shoot.setPointerCapture(e.pointerId); } catch { /* synthetic pointer */ } set(true); if (navigator.vibrate) navigator.vibrate(6); });
    for (const t of ['pointerup', 'pointercancel', 'lostpointercapture']) shoot.addEventListener(t, () => set(false));
    this.intakeBtn.addEventListener('pointerdown', (e) => { e.preventDefault(); e.stopPropagation(); this.state.intakeTap = true; if (navigator.vibrate) navigator.vibrate(6); });
    for (const b of [shoot, this.intakeBtn]) b.addEventListener('contextmenu', (e) => e.preventDefault());
    // no page gestures from anywhere on the pad
    this.el.addEventListener('touchmove', (e) => e.preventDefault(), { passive: false });
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
      // the robot code applies the deadband; here a gentle curve for fine control near the middle
      const k = (v: number) => { const a = Math.min(1, Math.abs(v) / RADIUS); return Math.sign(v) * a * (0.4 + 0.6 * a); };
      fn(k(dx), -k(dy));
    };
    zone.addEventListener('pointerdown', (e) => {
      if (id !== -1) return;
      e.preventDefault(); e.stopPropagation();
      id = e.pointerId;
      try { zone.setPointerCapture(id); } catch { /* synthetic pointer */ }
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

  setVisible(on: boolean) { this.el.classList.toggle('on', on); if (!on) Object.assign(this.state, { mx: 0, my: 0, turn: 0, shoot: false }); }
  /** The Intake button shows whether the rollers are running. */
  setIntake(on: boolean) {
    if (this.intakeBtn.getAttribute('aria-pressed') === String(on)) return;
    this.intakeBtn.setAttribute('aria-pressed', String(on));
    this.intakeBtn.classList.toggle('on', on);
  }
  dispose() { this.el.remove(); }
}
