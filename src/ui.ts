import { icon } from './icons';
import qrcode from 'qrcode-generator';
import { robot, subsystems, shortName, type Subsystem, type Spec } from './config';
import type { PinInfo, Mechanism } from './viewer';
import type { SearchResult } from './parts';
import { ACCENTS, type Settings } from './settings';
import type { SimStatus, SimMode, SimView } from './sim/index';

// (kept in sync with SIM_VIEWS in sim/index.ts, which loads lazily)
const SIM_VIEWS: { id: SimView; label: string; title: string }[] = [
  { id: 'tv', label: 'TV', title: 'Broadcast camera: the robot and where it is going' },
  { id: 'follow', label: 'Chase', title: 'Behind the robot' },
  { id: 'field', label: 'Field', title: 'The whole field' },
  { id: 'driver', label: 'Driver', title: 'From the driver station' },
];

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
  onMove(): void;
  onExplode(): void;
  onSimulate(): void;
  onBack(): void;
  onPrev(): void;
  onNext(): void;
  onCrumb(node: number): void;
  onHide(): void;
  onIsolate(): void;
  onUp(): void;
  onShowAll(): void;
  onUndoHide(): void;
  onMechanism(id: string, deg: number, commit: boolean): void;
  onSearchOpen(open: boolean): void;
  onSearch(q: string): void;
  onResult(r: SearchResult): void;
  onFullscreen(): void;
  onSettings(s: Partial<Settings>): void;
  onSettingsOpen(open: boolean): void;
  onSettingsRequest(): void;
  onResetSettings(): void;
  onSimView(view: SimView): void;
  onSimMode(mode: SimMode): void;
  onSimRestart(): void;
  onSimExit(): void;
}

export interface ItemView {
  name: string;
  kind: string;
  crumbs: { node: number; name: string }[];
  subsystemName: string;
  summary?: string;
  specs: Spec[];
  canUp: boolean;
  count: number;
}

export interface MechView { mech: Mechanism; value: number }

export class UI {
  readonly app: HTMLElement;
  readonly layer: HTMLElement;
  readonly pinLayer: HTMLElement;
  readonly dock: HTMLElement;
  readonly panel: HTMLElement;
  private chips = new Map<string, HTMLButtonElement>();
  private lens: HTMLElement;
  private pins = new Map<string, HTMLButtonElement>();
  private moveBtn: HTMLButtonElement;
  private explodeBtn: HTMLButtonElement;
  private fsBtn: HTMLButtonElement;
  private searchBtn: HTMLButtonElement;
  private hint: HTMLElement;
  private loader: HTMLElement;
  private ring: SVGCircleElement;
  private toastEl: HTMLElement;
  private toastTimer = 0;
  private debugEl: HTMLElement | null = null;
  private live: HTMLElement;
  private chipRow: HTMLElement;
  private tray: HTMLElement;
  private pill: HTMLElement;
  private searchEl: HTMLElement;
  private searchInput: HTMLInputElement;
  private results: HTMLElement;
  private settingsEl: HTMLElement;
  private brackets: SVGSVGElement;
  private bracketLabel: HTMLElement;
  private simHud: HTMLElement;
  private sheetEl: HTMLElement;
  private title: HTMLElement;
  private pinWidths = new Map<string, number>();
  private sheetOpenedAt = 0;
  private tip: HTMLElement;
  private pitHint: HTMLElement;
  private pitCaption: HTMLElement;
  private pitQr: HTMLElement;

  constructor(private handlers: UiHandlers) {
    this.app = document.getElementById('app')!;
    this.app.classList.add('app');
    this.layer = document.getElementById('ui')!;
    this.pinLayer = document.getElementById('pins')!;

    // Targeting brackets (JARVIS) around the focused part
    this.brackets = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    this.brackets.classList.add('brackets');
    this.brackets.setAttribute('aria-hidden', 'true');
    this.pinLayer.before(this.brackets);
    this.bracketLabel = h('div', 'bracket-label');
    this.pinLayer.before(this.bracketLabel);

    // Title (content layer, not glass). Press and hold opens the hidden settings.
    this.title = h('div', 'title', `<h1>${esc(robot.name)}</h1><p>${esc(robot.team)}</p>`);
    this.title.setAttribute('role', 'button');
    this.title.setAttribute('aria-label', `${robot.name}. Press and hold for settings.`); // (also the gear, top right)
    this.layer.append(this.title);
    this.longPress(this.title, () => handlers.onSettingsRequest());

    // Top-right: search + full screen, one icon group on a shared glass capsule
    const corner = h('div', 'corner glass');
    this.searchBtn = this.iconButton('search', 'Search parts', () => this.openSearch(true));
    this.fsBtn = this.iconButton('maximize', 'Enter full screen', () => handlers.onFullscreen());
    const settingsBtn = this.iconButton('settings', 'Settings', () => handlers.onSettingsRequest());
    settingsBtn.classList.add('settings-btn');
    corner.append(this.searchBtn, settingsBtn, this.fsBtn);
    this.layer.append(corner);

    // Search
    this.searchEl = h('div', 'search glass');
    this.searchEl.hidden = true;
    this.searchEl.innerHTML = `
      <div class="search-field">${icon('search')}<input type="search" enterkeyhint="search" autocomplete="off" autocorrect="off" spellcheck="false" placeholder="Search motors, rollers, assemblies…" aria-label="Search parts"><button class="search-close" aria-label="Close search">${icon('x')}</button></div>
      <div class="results" role="listbox"></div>`;
    this.searchInput = this.searchEl.querySelector('input')!;
    this.results = this.searchEl.querySelector('.results')!;
    this.searchInput.addEventListener('input', () => handlers.onSearch(this.searchInput.value));
    this.searchInput.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') this.openSearch(false);
      if (e.key === 'Enter') (this.results.querySelector('button') as HTMLButtonElement | null)?.click();
    });
    this.searchEl.querySelector('.search-close')!.addEventListener('click', () => this.openSearch(false));
    this.layer.append(this.searchEl);

    // Dock: [Overview] | subsystem chips | [Move, Explode, Simulate]
    this.dock = h('nav', 'dock glass');
    this.dock.setAttribute('aria-label', 'Robot controls');
    const homeGroup = h('div', 'dock-group');
    homeGroup.append(this.tool('house', 'Overview', 'Overview: reset the view', () => handlers.onHome()));
    this.chipRow = h('div', 'dock-group chips');
    this.chipRow.setAttribute('role', 'group');
    this.chipRow.setAttribute('aria-label', 'Subsystems');
    this.lens = h('div', 'lens');
    this.chipRow.append(this.lens);
    for (const s of subsystems) {
      const b = h('button', 'chip', `<span>${esc(shortName(s))}</span>`) as HTMLButtonElement;
      b.setAttribute('aria-pressed', 'false');
      b.setAttribute('aria-label', s.name);
      b.addEventListener('click', () => handlers.onChip(s.id));
      this.chips.set(s.id, b);
      this.chipRow.append(b);
    }
    const toolGroup = h('div', 'dock-group');
    this.moveBtn = this.tool('rotate-3d', 'Move', 'Move mechanisms', () => handlers.onMove());
    this.explodeBtn = this.tool('boxes', 'Explode', 'Explode view', () => handlers.onExplode());
    const simBtn = this.tool('play', 'Simulate', 'Simulation: drive, intake and score on the field', () => handlers.onSimulate());
    this.moveBtn.setAttribute('aria-pressed', 'false');
    this.explodeBtn.setAttribute('aria-pressed', 'false');
    toolGroup.append(this.moveBtn, this.explodeBtn, simBtn);
    this.dock.append(homeGroup, h('div', 'dock-sep'), this.chipRow, h('div', 'dock-sep'), toolGroup);
    this.layer.append(this.dock);

    // Mechanism tray
    this.tray = h('section', 'tray glass');
    this.tray.hidden = true;
    this.tray.setAttribute('aria-label', 'Mechanisms');
    this.layer.append(this.tray);

    // Hidden-parts pill
    this.pill = h('div', 'pill glass-clear');
    this.pill.hidden = true;
    this.layer.append(this.pill);

    this.hint = h('div', 'hint', `${icon('pointer')}<span>Drag to turn · Tap any part · Double-tap to hide</span>`);
    this.layer.append(this.hint);

    this.panel = h('section', 'panel glass');
    this.panel.hidden = true;
    this.panel.setAttribute('aria-live', 'polite');
    this.layer.append(this.panel);

    // Simulation HUD
    this.simHud = h('div', 'sim-hud');
    this.simHud.hidden = true;
    this.simHud.innerHTML = `
      <div class="sim-status glass">
        <span class="sim-dot"></span><span class="sim-phase">Simulation</span>
        <span class="sim-timer" hidden></span>
        <span class="sim-sep"></span>
        <span class="sim-stat"><b class="sim-scored">0</b> scored</span>
        <span class="sim-stat"><b class="sim-stored">0</b><i class="sim-cap"></i> in robot</span>
        <span class="sim-stat sim-bps-wrap"><b class="sim-bps">0.0</b> fuel/s</span>
      </div>
      <div class="sim-controls glass">
        <div class="seg sim-mode" role="group" aria-label="Simulation mode">
          <button data-mode="auto" aria-pressed="true">Auto</button><button data-mode="endless" aria-pressed="false">Endless</button>
        </div>
        <div class="dock-sep"></div>
        <div class="seg sim-view" role="group" aria-label="Camera">
          ${SIM_VIEWS.map((v) => `<button data-view="${v.id}" title="${v.title}" aria-pressed="${v.id === 'tv'}">${v.label}</button>`).join('')}
        </div>
        <div class="dock-sep"></div>
        <button class="tool" data-act="restart">${icon('rotate-ccw')}<span class="label">Restart</span></button>
        <button class="tool icon-only" data-act="settings" aria-label="Settings" title="Settings">${icon('settings')}</button>
        <button class="tool" data-act="exit">${icon('x')}<span class="label">Exit</span></button>
      </div>
      <div class="sim-result glass" hidden>
        <p class="sim-result-kicker">Autonomous complete</p>
        <p class="sim-result-big"><b class="sim-result-n">0</b> fuel scored</p>
        <div class="sim-result-actions">
          <button class="act" data-act="replay">${icon('rotate-ccw')}<span>Replay Auto</span></button>
          <button class="act primary" data-act="endless">${icon('play')}<span>Endless Mode</span></button>
        </div>
      </div>`;
    this.simHud.querySelectorAll<HTMLButtonElement>('.sim-view button').forEach((b) => b.addEventListener('click', () => handlers.onSimView(b.dataset.view as SimView)));
    this.simHud.querySelector('[data-act=restart]')!.addEventListener('click', () => handlers.onSimRestart());
    this.simHud.querySelector('[data-act=exit]')!.addEventListener('click', () => handlers.onSimExit());
    this.simHud.querySelector('[data-act=settings]')!.addEventListener('click', () => handlers.onSettingsRequest());
    this.simHud.querySelector('[data-act=replay]')!.addEventListener('click', () => handlers.onSimMode('auto'));
    this.simHud.querySelector('[data-act=endless]')!.addEventListener('click', () => handlers.onSimMode('endless'));
    this.simHud.querySelectorAll<HTMLButtonElement>('.sim-mode button').forEach((b) => b.addEventListener('click', () => handlers.onSimMode(b.dataset.mode as SimMode)));
    this.layer.append(this.simHud);

    // Loader
    this.loader = h('div', 'loader', `
      <div class="loader-inner">
        <svg class="ring" viewBox="0 0 64 64" aria-hidden="true">
          <circle class="track" cx="32" cy="32" r="28"></circle>
          <circle class="bar" cx="32" cy="32" r="28" stroke-dasharray="175.93" stroke-dashoffset="175.93"></circle>
        </svg>
        <div><h1>${esc(robot.name)}</h1><p>${esc(robot.team)}</p></div>
      </div>`);
    this.loader.setAttribute('role', 'progressbar');
    this.loader.setAttribute('aria-label', 'Loading robot');
    this.ring = this.loader.querySelector('.bar')!;
    this.layer.append(this.loader);

    this.toastEl = h('div', 'toast glass-clear hidden');
    this.layer.append(this.toastEl);

    // Settings (hidden: press and hold the title, or the , key)
    this.settingsEl = h('div', 'sheet-backdrop');
    this.settingsEl.hidden = true;
    // the finger that held the title lifts right after the sheet appears: don't treat that as "tap outside"
    this.settingsEl.addEventListener('click', (e) => { if (e.target === this.settingsEl && performance.now() - this.sheetOpenedAt > 700) this.openSettings(false); });
    this.sheetEl = h('section', 'sheet glass');
    this.sheetEl.setAttribute('role', 'dialog');
    this.sheetEl.setAttribute('aria-label', 'Settings');
    this.settingsEl.append(this.sheetEl);
    this.layer.append(this.settingsEl);

    this.pitHint = h('div', 'pit-hint', `<span>Touch to explore</span>`);
    this.layer.append(this.pitHint);
    this.pitCaption = h('div', 'pit-caption', '<p class="pc-kicker"></p><h2></h2><p class="pc-line"></p>');
    this.layer.append(this.pitCaption);
    this.pitQr = h('div', 'pit-qr');
    this.layer.append(this.pitQr);
    this.tip = h('div', 'hover-tip');
    this.layer.append(this.tip);

    this.live = h('div', 'sr-only');
    this.live.setAttribute('aria-live', 'polite');
    this.layer.append(this.live);

    for (const s of subsystems) {
      const p = h('button', 'pin hidden', `<span class="dot" aria-hidden="true"></span><span class="pin-label">${esc(shortName(s))}</span>`) as HTMLButtonElement;
      p.setAttribute('aria-label', `Show ${s.name}`);
      p.addEventListener('click', () => handlers.onPin(s.id));
      this.pins.set(s.id, p);
      this.pinLayer.append(p);
    }
    this.app.classList.add('loading');
  }

  private iconButton(ic: string, aria: string, fn: () => void) {
    const b = h('button', 'icon-btn', icon(ic)) as HTMLButtonElement;
    b.setAttribute('aria-label', aria);
    b.addEventListener('click', fn);
    return b;
  }

  private tool(ic: string, label: string, aria: string, fn: () => void) {
    const b = h('button', 'tool', `${icon(ic)}<span class="label">${esc(label)}</span>`) as HTMLButtonElement;
    b.setAttribute('aria-label', aria);
    b.addEventListener('click', fn);
    return b;
  }

  private longPress(el: HTMLElement, fn: () => void, ms = 700) {
    let t = 0, x = 0, y = 0;
    el.addEventListener('pointerdown', (e) => {
      x = e.clientX; y = e.clientY;
      el.classList.add('pressing');
      t = window.setTimeout(() => {
        el.classList.remove('pressing');
        fn();
        // swallow the click that ends this same press (it would land on whatever just opened)
        const block = (ev: Event) => { ev.stopPropagation(); ev.preventDefault(); };
        window.addEventListener('click', block, true);
        window.addEventListener('pointerup', () => setTimeout(() => window.removeEventListener('click', block, true), 350), { capture: true, once: true });
      }, ms);
    });
    const cancel = () => { clearTimeout(t); el.classList.remove('pressing'); };
    el.addEventListener('pointerup', cancel);
    el.addEventListener('pointercancel', cancel);
    el.addEventListener('pointermove', (e) => { if (Math.hypot(e.clientX - x, e.clientY - y) > 10) cancel(); });
  }

  // ------------------------------------------------------------ layout
  layout(w: number, hgt: number): boolean {
    const landscape = w >= hgt && w >= 700;
    const before = this.app.className;
    this.app.classList.toggle('landscape', landscape);
    this.app.classList.toggle('portrait', !landscape);
    this.app.classList.remove('compact');
    this.chipRow.classList.add('fits');
    if (this.dock.scrollWidth > this.dock.clientWidth + 1 || this.chipRow.scrollWidth > this.chipRow.clientWidth + 1) this.app.classList.add('compact');
    this.chipRow.classList.toggle('fits', this.chipRow.scrollWidth <= this.chipRow.clientWidth + 1);
    this.moveLens();
    return before !== this.app.className;
  }
  get isLandscape() { return this.app.classList.contains('landscape'); }

  // ------------------------------------------------------------ state
  setSelectedChip(id: string | null) {
    for (const [cid, b] of this.chips) b.setAttribute('aria-pressed', String(cid === id));
    this.moveLens();
    if (id) this.chips.get(id)?.scrollIntoView({ inline: 'nearest', block: 'nearest', behavior: 'smooth' });
  }

  /** The liquid-glass lens slides between chips (shape + weight show selection, not just color). */
  private moveLens() {
    const sel = [...this.chips.values()].find((b) => b.getAttribute('aria-pressed') === 'true');
    if (!sel) { this.lens.classList.remove('on'); return; }
    this.lens.classList.add('on');
    this.lens.style.width = `${sel.offsetWidth}px`;
    this.lens.style.transform = `translateX(${sel.offsetLeft}px)`;
  }

  setPit(on: boolean) {
    this.app.classList.toggle('pit', on);
    if (on && !this.pitQr.innerHTML) {
      // "Explore on your phone": the site's own address, generated locally (works offline)
      const qr = qrcode(0, 'M');
      qr.addData(location.origin + location.pathname);
      qr.make();
      this.pitQr.innerHTML = `${qr.createSvgTag({ cellSize: 3, margin: 0, scalable: true })}<span>Explore on your phone</span>`;
    }
    if (!on) this.pitShowcase(null);
  }
  /** Caption for the pit showcase (null hides it). */
  pitShowcase(c: { kicker: string; title: string; line: string } | null) {
    if (!c) { this.pitCaption.classList.remove('on'); return; }
    this.pitCaption.querySelector('.pc-kicker')!.textContent = c.kicker;
    this.pitCaption.querySelector('h2')!.textContent = c.title;
    this.pitCaption.querySelector('.pc-line')!.textContent = c.line;
    this.pitCaption.classList.add('on');
  }
  setFocusedClass(on: boolean) { this.app.classList.toggle('focused', on); }
  setExploded(on: boolean) {
    this.explodeBtn.setAttribute('aria-pressed', String(on));
    this.explodeBtn.querySelector('.label')!.textContent = on ? 'Collapse' : 'Explode';
    this.explodeBtn.setAttribute('aria-label', on ? 'Collapse exploded view' : 'Explode view');
  }
  showHint(on: boolean) { this.hint.classList.toggle('hidden', !on); }
  setFullscreenState(available: boolean, active: boolean) {
    this.fsBtn.hidden = !available;
    this.fsBtn.innerHTML = icon(active ? 'minimize' : 'maximize');
    this.fsBtn.setAttribute('aria-label', active ? 'Exit full screen' : 'Enter full screen');
  }

  setSimMode(on: boolean) {
    this.app.classList.toggle('sim', on);
    this.simHud.hidden = !on;
  }
  simStatus(s: SimStatus, view: SimView) {
    const q = <T extends Element = HTMLElement>(sel: string) => this.simHud.querySelector(sel) as T;
    q('.sim-phase').textContent = s.loading ? 'Loading field…' : s.phase;
    q('.sim-scored').textContent = String(s.scored);
    q('.sim-stored').textContent = String(s.stored);
    q('.sim-cap').textContent = s.capacity ? `/${s.capacity}` : '';
    q('.sim-bps').textContent = s.bps.toFixed(1);
    const timer = q<HTMLElement>('.sim-timer');
    timer.hidden = s.time === null;
    if (s.time !== null) timer.textContent = `0:${String(Math.ceil(s.time)).padStart(2, '0')}`;
    this.simHud.querySelectorAll<HTMLButtonElement>('.sim-mode button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === s.mode)));
    this.simHud.querySelectorAll<HTMLButtonElement>('.sim-view button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === view)));
    const res = q<HTMLElement>('.sim-result');
    res.hidden = !s.ended;
    if (s.ended) q('.sim-result-n').textContent = String(s.scored);
  }

  // ------------------------------------------------------------ panel
  private mechHtml(list: MechView[]) {
    if (!list.length) return '';
    return `<div class="mechs">${list.map(({ mech: m, value }) => {
      const states = m.states ? Object.entries(m.states) : null;
      return `<div class="mech" data-mech="${esc(m.id)}">
        <div class="mech-row"><span class="mech-name">${esc(m.name)}</span><span class="mech-val">${Math.round(value)}°</span></div>
        ${states ? `<div class="seg" role="group">${states.map(([label, deg]) => `<button data-deg="${deg}" aria-pressed="${Math.abs(value - deg) < 1}">${esc(label)}</button>`).join('')}</div>` : ''}
        <input type="range" min="${m.min}" max="${m.max}" step="1" value="${Math.round(value)}" aria-label="${esc(m.name)} angle">
        ${!states ? `<div class="mech-presets"><button data-deg="${m.def}">Center</button></div>` : ''}
      </div>`;
    }).join('')}</div>`;
  }

  private bindMechs(root: HTMLElement) {
    root.querySelectorAll<HTMLElement>('.mech').forEach((el) => {
      const id = el.dataset.mech!;
      const range = el.querySelector('input')!;
      const val = el.querySelector('.mech-val')!;
      range.addEventListener('input', () => { val.textContent = `${range.value}°`; this.handlers.onMechanism(id, Number(range.value), false); });
      range.addEventListener('change', () => this.handlers.onMechanism(id, Number(range.value), true));
      el.querySelectorAll<HTMLButtonElement>('button[data-deg]').forEach((b) => b.addEventListener('click', () => {
        const d = Number(b.dataset.deg);
        range.value = String(d);
        val.textContent = `${d}°`;
        el.querySelectorAll('.seg button').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
        this.handlers.onMechanism(id, d, true);
      }));
    });
  }

  /** Keeps open sliders in sync while mechanisms move (e.g. from the simulation or presets). */
  syncMechs(values: Map<string, number>) {
    for (const root of [this.panel, this.tray]) root.querySelectorAll<HTMLElement>('.mech').forEach((el) => {
      const v = values.get(el.dataset.mech!);
      if (v === undefined) return;
      const range = el.querySelector('input')!;
      if (document.activeElement === range) return;
      range.value = String(Math.round(v));
      el.querySelector('.mech-val')!.textContent = `${Math.round(v)}°`;
      el.querySelectorAll<HTMLButtonElement>('.seg button').forEach((b) => b.setAttribute('aria-pressed', String(Math.abs(Number(b.dataset.deg) - v) < 1)));
    });
  }

  showSubsystem(s: Subsystem, index: number, total: number, mechs: MechView[]) {
    const prev = subsystems[(index - 1 + total) % total];
    const next = subsystems[(index + 1) % total];
    const specs = s.specs ?? [];
    this.panel.innerHTML = `
      <div class="panel-head">
        <button class="back" data-act="back" aria-label="Back to overview">${icon('chevron-left')}<span>Overview</span></button>
        <span class="step">${index + 1} of ${total}</span>
      </div>
      <div class="panel-body">
        <div class="panel-title"><span class="badge">${icon(s.icon)}</span><div><h2>${esc(s.name)}</h2></div></div>
        <p class="summary">${esc(s.summary)}</p>
        ${specs.length ? `<dl class="specs${specs.length % 2 ? ' odd' : ''}">${specs.map((p) => `<div class="spec"><dt>${esc(p.label)}</dt><dd>${esc(p.value)}</dd></div>`).join('')}</dl>` : ''}
        ${s.highlights?.length ? `<ul class="highlights">${s.highlights.map((t) => `<li>${esc(t)}</li>`).join('')}</ul>` : ''}
        ${this.mechHtml(mechs)}
        <div class="actions">
          <button class="act" data-act="isolate">${icon('focus')}<span>Isolate</span></button>
          <button class="act" data-act="hide">${icon('eye-off')}<span>Hide</span></button>
        </div>
      </div>
      <div class="panel-foot">
        <button class="nav-btn" data-act="prev" aria-label="Previous: ${esc(prev.name)}">${icon('chevron-left')}<span class="label">${esc(shortName(prev))}</span></button>
        <button class="nav-btn" data-act="next" aria-label="Next: ${esc(next.name)}"><span class="label">${esc(shortName(next))}</span>${icon('chevron-right')}</button>
      </div>`;
    this.wirePanel();
    this.live.textContent = `${s.name}. ${s.summary}`;
  }

  showItem(v: ItemView, mechs: MechView[], backLabel: string) {
    this.panel.innerHTML = `
      <div class="panel-head">
        <button class="back" data-act="back" aria-label="Back to ${esc(backLabel)}">${icon('chevron-left')}<span>${esc(backLabel)}</span></button>
        <span class="step">${esc(v.subsystemName)}</span>
      </div>
      <div class="panel-body">
        ${v.crumbs.length > 1 ? `<nav class="crumbs" aria-label="Assembly path">${v.crumbs.map((c, i) => i < v.crumbs.length - 1
          ? `<button data-node="${c.node}">${esc(c.name)}</button>${icon('chevron-right')}`
          : `<span aria-current="true">${esc(c.name)}</span>`).join('')}</nav>` : ''}
        <div class="panel-title"><div><h2 class="item-name">${esc(v.name)}</h2><p class="kind">${esc(v.kind)}</p></div></div>
        ${v.summary ? `<p class="summary">${esc(v.summary)}</p>` : ''}
        ${v.specs.length ? `<dl class="specs${v.specs.length % 2 ? ' odd' : ''}">${v.specs.map((p) => `<div class="spec"><dt>${esc(p.label)}</dt><dd>${esc(p.value)}</dd></div>`).join('')}</dl>` : ''}
        ${this.mechHtml(mechs)}
        <div class="actions">
          <button class="act" data-act="hide">${icon('eye-off')}<span>Hide</span></button>
          <button class="act" data-act="isolate">${icon('focus')}<span>Isolate</span></button>
          ${v.canUp ? `<button class="act" data-act="up">${icon('arrow-up-from-line')}<span>Select Parent</span></button>` : ''}
        </div>
        <p class="tip">Tap the same part again to select its assembly. Double-tap any part to hide it.</p>
      </div>`;
    this.wirePanel();
    this.live.textContent = `${v.name}. ${v.kind}.`;
  }

  private wirePanel() {
    const q = (s: string) => this.panel.querySelector(s);
    q('[data-act=back]')?.addEventListener('click', () => this.handlers.onBack());
    q('[data-act=prev]')?.addEventListener('click', () => this.handlers.onPrev());
    q('[data-act=next]')?.addEventListener('click', () => this.handlers.onNext());
    q('[data-act=hide]')?.addEventListener('click', () => this.handlers.onHide());
    q('[data-act=isolate]')?.addEventListener('click', () => this.handlers.onIsolate());
    q('[data-act=up]')?.addEventListener('click', () => this.handlers.onUp());
    this.panel.querySelectorAll<HTMLButtonElement>('.crumbs button').forEach((b) => b.addEventListener('click', () => this.handlers.onCrumb(Number(b.dataset.node))));
    this.bindMechs(this.panel);
    this.panel.hidden = false;
    this.panel.querySelector('.panel-body')!.scrollTop = 0;
  }

  hidePanel() { this.panel.hidden = true; }
  panelBox() { const p = this.panel; return { left: p.offsetLeft, top: p.offsetTop, width: p.offsetWidth, height: p.offsetHeight }; }
  dockTop() { return this.dock.offsetTop; }
  trayBox() { const t = this.tray; return t.hidden ? null : { left: t.offsetLeft, top: t.offsetTop, width: t.offsetWidth, height: t.offsetHeight }; }
  titleBottom() { return this.title.offsetTop + this.title.offsetHeight; }

  // ------------------------------------------------------------ tray
  showTray(list: MechView[] | null) {
    if (!list) {
      this.tray.hidden = true;
      this.moveBtn.setAttribute('aria-pressed', 'false');
      return;
    }
    this.tray.innerHTML = `<div class="tray-head"><h3>Mechanisms</h3><button class="icon-btn small" aria-label="Close mechanisms">${icon('x')}</button></div>${this.mechHtml(list)}`;
    this.tray.querySelector('.tray-head button')!.addEventListener('click', () => this.handlers.onMove());
    this.bindMechs(this.tray);
    this.tray.hidden = false;
    this.moveBtn.setAttribute('aria-pressed', 'true');
  }
  get trayOpen() { return !this.tray.hidden; }

  // ------------------------------------------------------------ hidden pill
  setHidden(count: number, lastLabel: string | null) {
    if (!count) { this.pill.hidden = true; return; }
    this.pill.innerHTML = `${icon('eye-off')}<span>${count} hidden</span>${lastLabel ? `<button data-act="undo">Undo</button>` : ''}<button data-act="all">Show All</button>`;
    this.pill.querySelector('[data-act=all]')!.addEventListener('click', () => this.handlers.onShowAll());
    this.pill.querySelector('[data-act=undo]')?.addEventListener('click', () => this.handlers.onUndoHide());
    this.pill.hidden = false;
  }

  // ------------------------------------------------------------ search
  openSearch(open: boolean) {
    this.searchEl.hidden = !open;
    this.app.classList.toggle('searching', open);
    if (open) { this.searchInput.value = ''; this.results.innerHTML = this.searchHint(); setTimeout(() => this.searchInput.focus(), 50); }
    else this.searchInput.blur();
    this.handlers.onSearchOpen(open);
  }
  get searchOpen() { return !this.searchEl.hidden; }
  private searchHint() {
    const ex = ['Kraken', 'Limelight', 'Roller', 'Turret', 'Flywheel', 'PDH'];
    return `<div class="search-empty">Try ${ex.map((e) => `<button class="suggest">${e}</button>`).join('')}</div>`;
  }
  showResults(list: SearchResult[], q: string) {
    if (!q.trim()) { this.results.innerHTML = this.searchHint(); this.bindSuggest(); return; }
    if (!list.length) { this.results.innerHTML = `<div class="search-empty">No parts named “${esc(q)}”.</div>`; return; }
    this.results.innerHTML = list.map((r, i) => {
      const sub = r.subsystem ? subsystems.find((s) => s.id === r.subsystem)?.name ?? '' : 'Several subsystems';
      return `<button class="result" role="option" data-i="${i}">
        <span class="r-icon">${icon(r.kind === 'assembly' ? 'boxes' : 'box')}</span>
        <span class="r-text"><span class="r-name">${esc(r.name)}</span><span class="r-sub">${esc(sub)}${r.kind === 'assembly' ? ' · Assembly' : ''}</span></span>
        ${r.count > 1 ? `<span class="r-count">×${r.count}</span>` : ''}
      </button>`;
    }).join('');
    this.results.querySelectorAll<HTMLButtonElement>('.result').forEach((b) => b.addEventListener('click', () => this.handlers.onResult(list[Number(b.dataset.i)])));
  }
  private bindSuggest() {
    this.results.querySelectorAll<HTMLButtonElement>('.suggest').forEach((b) => b.addEventListener('click', () => {
      this.searchInput.value = b.textContent ?? '';
      this.handlers.onSearch(this.searchInput.value);
    }));
  }

  // ------------------------------------------------------------ settings
  openSettings(open: boolean, s?: Settings) {
    if (open && s) this.renderSettings(s);
    if (open) this.sheetOpenedAt = performance.now();
    this.settingsEl.hidden = !open;
    this.handlers.onSettingsOpen(open);
  }
  get settingsOpen() { return !this.settingsEl.hidden; }

  renderSettings(s: Settings) {
    const seg = (key: keyof Settings, opts: [string | number, string][]) =>
      `<div class="seg" role="group" data-key="${key}">${opts.map(([v, l]) => `<button data-v="${v}" aria-pressed="${String(s[key]) === String(v)}">${l}</button>`).join('')}</div>`;
    const toggle = (key: keyof Settings, label: string) =>
      `<label class="row"><span>${label}</span><input type="checkbox" class="switch" data-key="${key}" ${s[key] ? 'checked' : ''}></label>`;
    const slider = (key: keyof Settings, label: string, min: number, max: number, step: number) =>
      `<label class="row col"><span>${label}</span><input type="range" data-key="${key}" min="${min}" max="${max}" step="${step}" value="${s[key]}"></label>`;
    this.sheetEl.innerHTML = `
      <div class="sheet-head"><h2>Settings</h2><button class="icon-btn small" data-act="close" aria-label="Close settings">${icon('x')}</button></div>
      <div class="sheet-body">
        <h3>Appearance</h3>
        ${seg('appearance', [['system', 'Auto'], ['dark', 'Dark'], ['light', 'Light']])}
        <div class="swatches" role="group" aria-label="Accent color">${Object.entries(ACCENTS).map(([k, a]) =>
          `<button class="swatch" data-accent="${k}" aria-pressed="${s.accent === k}" aria-label="${a.name}" style="--sw:${a.fill}"></button>`).join('')}</div>
        ${slider('hologram', 'Hologram Strength', 0.3, 1.6, 0.05)}
        ${toggle('labels', 'Subsystem Labels')}
        <h3>Sound</h3>
        ${toggle('sound', 'Interface Sounds')}
        ${slider('volume', 'Volume', 0, 1, 0.05)}
        <h3>Presentation</h3>
        ${toggle('autoSpin', 'Turntable Spin')}
        ${slider('spinSeconds', 'Spin Speed (slower →)', 25, 120, 1)}
        <span class="row-label">Pit Display After</span>
        ${seg('pitMode', [[0, 'Off'], [15, '15 s'], [30, '30 s'], [60, '1 min']])}
        <h3>Performance</h3>
        ${seg('quality', [['battery', 'Battery'], ['balanced', 'Balanced'], ['max', 'Max']])}
        ${toggle('stats', 'Show Frame Rate')}
        <div class="sheet-actions"><button class="act" data-act="reset">${icon('rotate-ccw')}<span>Reset Settings</span></button></div>
        <p class="tip">${esc(robot.name)} · ${esc(robot.team)}${robot.season ? ` · ${esc(robot.season)}` : ''}. Open these settings any time by pressing and holding the robot’s name.</p>
      </div>`;
    const emit = (patch: Partial<Settings>) => this.handlers.onSettings(patch);
    this.sheetEl.querySelector('[data-act=close]')!.addEventListener('click', () => this.openSettings(false));
    this.sheetEl.querySelector('[data-act=reset]')!.addEventListener('click', () => this.handlers.onResetSettings());
    this.sheetEl.querySelectorAll<HTMLElement>('.seg').forEach((g) => g.querySelectorAll<HTMLButtonElement>('button').forEach((b) => b.addEventListener('click', () => {
      g.querySelectorAll('button').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
      const key = g.dataset.key as keyof Settings;
      const raw = b.dataset.v!;
      emit({ [key]: isNaN(Number(raw)) ? raw : Number(raw) } as Partial<Settings>);
    })));
    this.sheetEl.querySelectorAll<HTMLButtonElement>('.swatch').forEach((b) => b.addEventListener('click', () => {
      this.sheetEl.querySelectorAll('.swatch').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
      emit({ accent: b.dataset.accent as Settings['accent'] });
    }));
    this.sheetEl.querySelectorAll<HTMLInputElement>('input.switch').forEach((i) => i.addEventListener('change', () => emit({ [i.dataset.key!]: i.checked } as Partial<Settings>)));
    this.sheetEl.querySelectorAll<HTMLInputElement>('input[type=range]').forEach((i) => i.addEventListener('input', () => emit({ [i.dataset.key!]: Number(i.value) } as Partial<Settings>)));
  }

  // ------------------------------------------------------------ pins & brackets
  updatePins(pins: PinInfo[], show: boolean) {
    // Near pins claim space first; a label that would collide hides (its dot stays), so nothing jumps around.
    const placed: { x: number; y: number; w: number }[] = [];
    const order = [...pins].sort((a, b) => Number(a.far) - Number(b.far) || a.y - b.y);
    for (const p of order) {
      const el = this.pins.get(p.id);
      if (!el) continue;
      const vis = show && p.visible;
      el.classList.toggle('hidden', !vis);
      el.classList.toggle('far', p.far);
      el.tabIndex = vis ? 0 : -1;
      if (!vis) continue;
      const w = (this.pinWidths.get(p.id) ?? this.pinWidths.set(p.id, el.offsetWidth || 110).get(p.id))!;
      const clash = placed.some((q) => Math.abs(q.y - p.y) < 40 && p.x - 22 < q.x - 22 + q.w + 4 && q.x - 22 < p.x - 22 + w + 4);
      el.classList.toggle('collapsed', clash);
      if (!clash) placed.push({ x: p.x, y: p.y, w });
      el.style.transform = `translate3d(${p.x.toFixed(1)}px, ${p.y.toFixed(1)}px, 0)`;
    }
  }

  updateBrackets(r: { x: number; y: number; w: number; h: number } | null, label: string | null, dims: string | null = null) {
    const svg = this.brackets;
    if (!r || !label) { svg.classList.remove('on'); this.bracketLabel.classList.remove('on'); return; }
    const pad = 14;
    const x = r.x - pad, y = r.y - pad, w = r.w + pad * 2, hh = r.h + pad * 2;
    const L = Math.max(12, Math.min(28, Math.min(w, hh) * 0.18));
    const W = this.app.clientWidth, H = this.app.clientHeight;
    svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    svg.innerHTML = `<path d="M${x} ${y + L}V${y}H${x + L} M${x + w - L} ${y}H${x + w}V${y + L} M${x + w} ${y + hh - L}V${y + hh}H${x + w - L} M${x + L} ${y + hh}H${x}V${y + hh - L}"/>`;
    svg.classList.add('on');
    this.bracketLabel.innerHTML = `<b>${esc(label)}</b>${dims ? `<span>${dims}</span>` : ''}`;
    const lx = Math.min(Math.max(x, 12), W - 260), ly = Math.max(y - (dims ? 46 : 30), 8);
    this.bracketLabel.style.transform = `translate3d(${lx}px, ${ly}px, 0)`;
    this.bracketLabel.classList.add('on');
  }

  /** Small floating label next to the cursor: what a click would select, or a drag readout. */
  hoverTip(t: { x: number; y: number; title: string; sub: string; hint: string | null } | null) {
    if (!t) { this.tip.classList.remove('on'); return; }
    this.tip.innerHTML = `<b>${esc(t.title)}</b><span>${esc(t.sub)}</span>${t.hint ? `<em>${esc(t.hint)}</em>` : ''}`;
    const W = this.app.clientWidth;
    const x = Math.min(t.x + 18, W - 260);
    this.tip.style.transform = `translate3d(${x}px, ${t.y + 18}px, 0)`;
    this.tip.classList.add('on');
  }

  // ------------------------------------------------------------ loader / toast / debug
  progress(f: number) { this.ring.style.strokeDashoffset = String(175.93 * (1 - f)); }
  loaded() {
    this.loader.classList.add('done');
    this.app.classList.remove('loading');
    setTimeout(() => this.loader.remove(), 700);
  }
  loadError(msg: string) { this.loader.querySelector('.loader-inner')!.insertAdjacentHTML('beforeend', `<p class="err">${esc(msg)}</p>`); }

  toast(text: string, opts: { ms?: number; icon?: string; action?: { label: string; fn: () => void } } = {}) {
    clearTimeout(this.toastTimer);
    this.toastEl.innerHTML = `${icon(opts.icon ?? 'circle-check')}<span>${esc(text)}</span>${opts.action ? `<button>${esc(opts.action.label)}</button>` : ''}`;
    if (opts.action) this.toastEl.querySelector('button')!.addEventListener('click', () => { opts.action!.fn(); this.toastEl.classList.add('hidden'); });
    this.toastEl.classList.remove('hidden');
    this.toastTimer = window.setTimeout(() => this.toastEl.classList.add('hidden'), opts.ms ?? 3200);
  }

  /** A short explanatory sheet (used when full screen isn't available in this browser). */
  sheet(title: string, body: string) {
    this.sheetEl.innerHTML = `<div class="sheet-head"><h2>${esc(title)}</h2><button class="icon-btn small" data-act="close" aria-label="Close">${icon('x')}</button></div><div class="sheet-body">${body}</div>`;
    this.sheetEl.querySelector('[data-act=close]')!.addEventListener('click', () => { this.settingsEl.hidden = true; });
    this.sheetOpenedAt = performance.now();
    this.settingsEl.hidden = false;
  }

  restoring(on: boolean) {
    let el = this.layer.querySelector('.restoring');
    if (on && !el) { el = h('div', 'restoring', 'Restoring…'); this.layer.append(el); }
    if (!on) el?.remove();
  }

  debug(text: string | null) {
    if (text === null) { this.debugEl?.remove(); this.debugEl = null; return; }
    if (!this.debugEl) { this.debugEl = h('div', 'debug'); this.layer.append(this.debugEl); }
    this.debugEl.textContent = text;
  }
}
