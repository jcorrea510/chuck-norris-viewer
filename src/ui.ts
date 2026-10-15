import { icon } from './icons';
import { robot, subsystems, subsystemById, shortName, type Subsystem } from './config';
import type { PinInfo } from './viewer';

const esc = (s: string) => s.replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);
const h = <K extends keyof HTMLElementTagNameMap>(tag: K, cls?: string, html?: string) => {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (html !== undefined) el.innerHTML = html;
  return el;
};

export interface UiHandlers {
  onChip(id: string): void;
  onPin(id: string): void;
  onHome(): void;
  onTour(): void;
  onExplode(): void;
  onBack(): void;
  onPrev(): void;
  onNext(): void;
  onFullscreen(): void;
}

export class UI {
  readonly app: HTMLElement;
  readonly layer: HTMLElement;
  readonly pinLayer: HTMLElement;
  readonly dock: HTMLElement;
  readonly panel: HTMLElement;
  private chips = new Map<string, HTMLButtonElement>();
  private pins = new Map<string, HTMLButtonElement>();
  private tourBtn: HTMLButtonElement;
  private explodeBtn: HTMLButtonElement;
  private fsBtn: HTMLButtonElement;
  private hint: HTMLElement;
  private loader: HTMLElement;
  private ring: SVGCircleElement;
  private toastEl: HTMLElement;
  private debugEl: HTMLElement | null = null;
  private live: HTMLElement;
  private chipRow: HTMLElement;

  constructor(private handlers: UiHandlers) {
    this.app = document.getElementById('app')!;
    this.app.classList.add('app');
    this.layer = document.getElementById('ui')!;
    this.pinLayer = document.getElementById('pins')!;

    // Title (plain text on the stage — not glass, it's part of the content)
    const title = h('div', 'title', `<h1>${esc(robot.name)}</h1><p>${esc(robot.team)}</p>`);
    this.layer.append(title);

    // Top-right: fullscreen (Safari only; hidden when launched from the Home Screen)
    const corner = h('div', 'corner');
    this.fsBtn = h('button', 'icon-btn glass-clear', icon('maximize')) as HTMLButtonElement;
    this.fsBtn.setAttribute('aria-label', 'Enter full screen');
    this.fsBtn.addEventListener('click', () => handlers.onFullscreen());
    corner.append(this.fsBtn);
    this.layer.append(corner);

    // Dock: [Overview] | subsystem chips | [Tour, Explode]
    this.dock = h('nav', 'dock glass');
    this.dock.setAttribute('aria-label', 'Robot controls');
    const homeGroup = h('div', 'dock-group');
    const home = this.tool('house', 'Overview', 'Overview: reset the view', () => handlers.onHome());
    homeGroup.append(home);

    this.chipRow = h('div', 'dock-group chips');
    this.chipRow.setAttribute('role', 'group');
    this.chipRow.setAttribute('aria-label', 'Subsystems');
    for (const s of subsystems) {
      const b = h('button', 'chip', `<span class="dot" aria-hidden="true"></span><span>${esc(shortName(s))}</span>`) as HTMLButtonElement;
      b.setAttribute('aria-pressed', 'false');
      b.setAttribute('aria-label', s.name);
      b.addEventListener('click', () => handlers.onChip(s.id));
      this.chips.set(s.id, b);
      this.chipRow.append(b);
    }
    const toolGroup = h('div', 'dock-group');
    this.tourBtn = this.tool('square-play', 'Tour', 'Start guided tour', () => handlers.onTour());
    this.explodeBtn = this.tool('boxes', 'Explode', 'Explode view', () => handlers.onExplode());
    this.tourBtn.setAttribute('aria-pressed', 'false');
    this.explodeBtn.setAttribute('aria-pressed', 'false');
    toolGroup.append(this.tourBtn, this.explodeBtn);
    this.dock.append(homeGroup, h('div', 'dock-sep'), this.chipRow, h('div', 'dock-sep'), toolGroup);
    this.layer.append(this.dock);

    // First-use hint
    this.hint = h('div', 'hint', `${icon('pointer')}<span>Drag to turn · Tap any part to explore</span>`);
    this.layer.append(this.hint);

    // Inspector panel
    this.panel = h('section', 'panel glass');
    this.panel.hidden = true;
    this.panel.setAttribute('aria-live', 'polite');
    this.layer.append(this.panel);

    // Loader
    this.loader = h('div', 'loader', `
      <div class="loader-inner">
        <svg class="ring" viewBox="0 0 56 56" aria-hidden="true">
          <circle class="track" cx="28" cy="28" r="25"></circle>
          <circle class="bar" cx="28" cy="28" r="25" stroke-dasharray="157.08" stroke-dashoffset="157.08"></circle>
        </svg>
        <div><h1>${esc(robot.name)}</h1><p>${esc(robot.team)}</p></div>
      </div>`);
    this.loader.setAttribute('role', 'progressbar');
    this.loader.setAttribute('aria-label', 'Loading robot');
    this.ring = this.loader.querySelector('.bar')!;
    this.layer.append(this.loader);

    this.toastEl = h('div', 'toast hidden', `${icon('circle-check')}<span>Ready offline</span>`);
    this.layer.append(this.toastEl);

    this.live = h('div', 'sr-only');
    this.live.setAttribute('aria-live', 'polite');
    this.layer.append(this.live);

    // Pins
    for (const s of subsystems) {
      const p = h('button', 'pin hidden', `<span class="dot" aria-hidden="true"></span><span>${esc(shortName(s))}</span>`) as HTMLButtonElement;
      p.setAttribute('aria-label', `Show ${s.name}`);
      p.addEventListener('click', () => handlers.onPin(s.id));
      this.pins.set(s.id, p);
      this.pinLayer.append(p);
    }

    this.app.classList.add('loading');
  }

  private tool(ic: string, label: string, aria: string, fn: () => void) {
    const b = h('button', 'tool', `${icon(ic)}<span class="label">${esc(label)}</span>`) as HTMLButtonElement;
    b.setAttribute('aria-label', aria);
    b.addEventListener('click', fn);
    return b;
  }

  // ------------------------------------------------------------ layout
  /** Applies landscape/portrait and compact classes; returns true if the layout class changed. */
  layout(w: number, hgt: number): boolean {
    const landscape = w >= hgt && w >= 700;
    const before = this.app.className;
    this.app.classList.toggle('landscape', landscape);
    this.app.classList.toggle('portrait', !landscape);
    this.app.classList.remove('compact');
    this.chipRow.classList.add('fits');
    // Collapse tool labels to icons only if the dock would overflow.
    if (this.dock.scrollWidth > this.dock.clientWidth + 1 || this.chipRow.scrollWidth > this.chipRow.clientWidth + 1) {
      this.app.classList.add('compact');
    }
    this.chipRow.classList.toggle('fits', this.chipRow.scrollWidth <= this.chipRow.clientWidth + 1);
    return before !== this.app.className;
  }

  get isLandscape() { return this.app.classList.contains('landscape'); }

  // ------------------------------------------------------------ state
  setFocused(id: string | null) {
    this.app.classList.toggle('focused', !!id);
    for (const [cid, b] of this.chips) b.setAttribute('aria-pressed', String(cid === id));
    if (id) this.chips.get(id)?.scrollIntoView({ inline: 'nearest', block: 'nearest', behavior: 'smooth' });
  }

  setTour(on: boolean) {
    this.tourBtn.setAttribute('aria-pressed', String(on));
    this.tourBtn.querySelector('.label')!.textContent = on ? 'End Tour' : 'Tour';
    this.tourBtn.setAttribute('aria-label', on ? 'End guided tour' : 'Start guided tour');
  }

  setExploded(on: boolean) {
    this.explodeBtn.setAttribute('aria-pressed', String(on));
    this.explodeBtn.querySelector('.label')!.textContent = on ? 'Collapse' : 'Explode';
    this.explodeBtn.setAttribute('aria-label', on ? 'Collapse exploded view' : 'Explode view');
  }

  showHint(on: boolean) { this.hint.classList.toggle('hidden', !on); }

  setFullscreenAvailable(available: boolean, active: boolean) {
    this.fsBtn.hidden = !available;
    this.fsBtn.innerHTML = icon(active ? 'minimize' : 'maximize');
    this.fsBtn.setAttribute('aria-label', active ? 'Exit full screen' : 'Enter full screen');
  }

  // ------------------------------------------------------------ panel
  showPanel(s: Subsystem, index: number, total: number, tour: boolean) {
    const prev = subsystems[(index - 1 + total) % total];
    const next = subsystems[(index + 1) % total];
    const first = index === 0, last = index === total - 1;
    const specs = s.specs ?? [];
    const prevLabel = tour && first ? 'Previous' : shortName(prev);
    const nextLabel = tour && last ? 'Finish Tour' : tour ? `Next: ${shortName(next)}` : shortName(next);
    this.panel.innerHTML = `
      <div class="panel-head">
        <button class="back" data-act="back" aria-label="${tour ? 'End tour and return to overview' : 'Back to overview'}">
          ${icon('chevron-left')}<span>${tour ? 'End Tour' : 'Overview'}</span>
        </button>
        <span class="step">${tour ? 'Tour · ' : ''}${index + 1} of ${total}</span>
      </div>
      <div class="panel-body">
        <div class="panel-title"><span class="badge">${icon(s.icon)}</span><h2>${esc(s.name)}</h2></div>
        <p class="summary">${esc(s.summary)}</p>
        ${specs.length ? `<dl class="specs${specs.length % 2 ? ' odd' : ''}">${specs.map((p) => `<div class="spec"><dt>${esc(p.label)}</dt><dd>${esc(p.value)}</dd></div>`).join('')}</dl>` : ''}
        ${s.talkingPoints?.length ? `<h3>Talking Points</h3><ul class="points">${s.talkingPoints.map((t) => `<li>${esc(t)}</li>`).join('')}</ul>` : ''}
      </div>
      <div class="panel-foot">
        <button class="nav-btn" data-act="prev" ${tour && first ? 'disabled style="opacity:.4"' : ''} aria-label="Previous: ${esc(prev.name)}">
          ${icon('chevron-left')}<span class="label">${esc(prevLabel)}</span>
        </button>
        <button class="nav-btn${tour ? ' primary' : ''}" data-act="next" aria-label="${tour && last ? 'Finish tour' : `Next: ${esc(next.name)}`}">
          <span class="label">${esc(nextLabel)}</span>${icon(tour && last ? 'circle-check' : 'chevron-right')}
        </button>
      </div>`;
    this.panel.querySelector('[data-act=back]')!.addEventListener('click', () => this.handlers.onBack());
    this.panel.querySelector('[data-act=prev]')!.addEventListener('click', () => this.handlers.onPrev());
    this.panel.querySelector('[data-act=next]')!.addEventListener('click', () => this.handlers.onNext());
    this.panel.hidden = false;
    this.panel.querySelector('.panel-body')!.scrollTop = 0;
    this.live.textContent = `${s.name}. ${s.summary}`;
  }

  hidePanel() { this.panel.hidden = true; }

  /** Panel box in layout coordinates (ignores the slide-in transform). */
  panelBox() {
    const p = this.panel;
    return { left: p.offsetLeft, top: p.offsetTop, width: p.offsetWidth, height: p.offsetHeight };
  }

  dockTop() { return this.dock.offsetTop; }
  titleBottom() { const t = this.layer.querySelector('.title') as HTMLElement; return t.offsetTop + t.offsetHeight; }

  // ------------------------------------------------------------ pins
  updatePins(pins: PinInfo[], show: boolean) {
    // Near pins claim their spot first; a pin that would overlap another moves up a row.
    const placed: { x: number; y: number; w: number }[] = [];
    const order = [...pins].sort((a, b) => Number(a.far) - Number(b.far) || b.y - a.y);
    for (const p of order) {
      const el = this.pins.get(p.id);
      if (!el) continue;
      const vis = show && p.visible;
      el.classList.toggle('hidden', !vis);
      el.classList.toggle('far', p.far);
      el.tabIndex = vis ? 0 : -1;
      if (!vis) continue;
      const w = (this.pinWidths.get(p.id) ?? this.pinWidths.set(p.id, el.offsetWidth || 110).get(p.id))!;
      let y = p.y;
      for (let tries = 0; tries < 4; tries++) {
        const clash = placed.some((q) => Math.abs(q.y - y) < 48 && p.x - 22 < q.x - 22 + q.w + 6 && q.x - 22 < p.x - 22 + w + 6);
        if (!clash) break;
        y -= 50;
      }
      placed.push({ x: p.x, y, w });
      el.style.transform = `translate3d(${p.x.toFixed(1)}px, ${y.toFixed(1)}px, 0)`;
    }
  }
  private pinWidths = new Map<string, number>();

  // ------------------------------------------------------------ loader / toast / debug
  progress(f: number) { this.ring.style.strokeDashoffset = String(157.08 * (1 - f)); }

  loaded() {
    this.loader.classList.add('done');
    this.app.classList.remove('loading');
    setTimeout(() => this.loader.remove(), 700);
  }

  loadError(msg: string) {
    this.loader.querySelector('.loader-inner')!.insertAdjacentHTML('beforeend', `<p class="err">${esc(msg)}</p>`);
  }

  toast(text: string, ms = 3200) {
    this.toastEl.querySelector('span')!.textContent = text;
    this.toastEl.classList.remove('hidden');
    setTimeout(() => this.toastEl.classList.add('hidden'), ms);
  }

  restoring(on: boolean) {
    let el = this.layer.querySelector('.restoring');
    if (on && !el) { el = h('div', 'restoring', 'Restoring…'); this.layer.append(el); }
    if (!on) el?.remove();
  }

  debug(text: string) {
    if (!this.debugEl) { this.debugEl = h('div', 'debug'); this.layer.append(this.debugEl); }
    this.debugEl.textContent = text;
  }

  subsystemIndex(id: string) { return subsystems.findIndex((s) => s.id === id); }
  sub(id: string) { return subsystemById.get(id)!; }
}
