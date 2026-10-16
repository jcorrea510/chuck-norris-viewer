// The Controls page: every driving action with its key and controller button. Tap a binding, then
// press the new key or button (Esc clears it). Also: what "forward" means, the on-screen controls,
// and a reset. Saved on this device.
import { icon } from '../icons';
import { ACTIONS, DEFAULT_BINDINGS, keyName, padName, type Action, type DriverControls } from './input';

const esc = (s: string) => s.replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);

export interface ControlsPageOpts {
  touch: boolean;
  onTouch(on: boolean): void;
  padName: string | null;
}

export function controlsPage(root: HTMLElement, c: DriverControls, opts: ControlsPageOpts) {
  const render = () => {
    const b = c.bindings;
    const row = (a: (typeof ACTIONS)[number]) => {
      const keys = b.keys[a.id];
      return `<div class="kb-row" data-a="${a.id}">
        <div class="kb-label"><b>${esc(a.label)}</b>${a.hint ? `<span>${esc(a.hint)}</span>` : ''}</div>
        <button class="kb-key" data-kind="key" data-a="${a.id}" aria-label="Key for ${esc(a.label)}">${keys.map((k) => `<kbd>${esc(keyName(k))}</kbd>`).join('<i>or</i>') || '<em>none</em>'}</button>
        ${a.pad ? `<button class="kb-key pad" data-kind="pad" data-a="${a.id}" aria-label="Controller button for ${esc(a.label)}"><kbd>${esc(padName(b.pad[a.id]))}</kbd></button>` : `<span class="kb-key pad fixed"><kbd>${a.id === 'turnLeft' || a.id === 'turnRight' ? 'Right stick' : 'Left stick'}</kbd></span>`}
      </div>`;
    };
    root.innerHTML = `
      <p class="lead">Driving is field-relative, like a real FRC driver with a gyro: up always drives toward the opponent’s wall, which is up on screen in every camera, whichever way the robot faces. In Free Drive the robot drives itself again 15 seconds after you let go.</p>
      <div class="kb-pad">${icon('gamepad-2')}<span>${opts.padName ? `Connected: <b>${esc(opts.padName)}</b>` : 'No controller yet: connect one and press any button.'}</span></div>
      <h3><span>Action</span><span class="kb-cols"><span>${icon('keyboard')} Keyboard</span><span>${icon('gamepad-2')} Controller</span></span></h3>
      <div class="kb-list">${ACTIONS.map(row).join('')}</div>
      <p class="tip">Tap a key or button, then press the new one. Esc clears it.</p>
      <h3>Touch</h3>
      <label class="row"><span>On-screen driving controls</span><input type="checkbox" class="switch" data-k="touch" ${opts.touch ? 'checked' : ''}></label>
      <p class="tip">Left thumb drives (the stick appears where it lands), right thumb turns. Intake turns the rollers on and off (on drops the intake; off stows it, and a stowed robot is too tall for the TRENCH). Hold Shoot to feed the turrets.</p>
      <div class="sheet-actions"><button class="act" data-act="reset">${icon('rotate-ccw')}<span>Reset Controls</span></button></div>`;
    wire();
  };
  const wire = () => {
    root.querySelector<HTMLInputElement>('[data-k=touch]')!.addEventListener('change', (e) => opts.onTouch((e.target as HTMLInputElement).checked));
    root.querySelector('[data-act=reset]')!.addEventListener('click', () => { c.bindings = structuredClone(DEFAULT_BINDINGS); c.save(); render(); });
    root.querySelectorAll<HTMLButtonElement>('button.kb-key').forEach((btn) => btn.addEventListener('click', () => {
      const a = btn.dataset.a as Action, kind = btn.dataset.kind as 'key' | 'pad';
      root.querySelectorAll('.kb-key.listening').forEach((x) => x.classList.remove('listening'));
      btn.classList.add('listening');
      btn.innerHTML = `<kbd>${kind === 'key' ? 'Press a key…' : 'Press a button…'}</kbd>`;
      c.captureNext(kind, (code) => {
        const b = c.bindings;
        if (kind === 'key') {
          const k = String(code);
          // a key does one thing: take it from any other action first
          for (const other of Object.keys(b.keys) as Action[]) b.keys[other] = b.keys[other].filter((x) => x !== k);
          b.keys[a] = k ? [k] : [];
        } else {
          for (const other of Object.keys(b.pad) as Action[]) if (b.pad[other] === code) delete b.pad[other];
          if (typeof code === 'number') b.pad[a] = code; else delete b.pad[a];
        }
        c.save();
        render();
      });
    }));
  };
  render();
  return { refresh: (padNameNow: string | null) => { opts.padName = padNameNow; render(); }, close: () => c.cancelCapture() };
}
