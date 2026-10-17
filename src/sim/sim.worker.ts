// The simulation's Web Worker: Rapier, the world, the game and the robot's brain, off the render
// thread. Route planning gets its own nested worker where the browser supports it.
import { initRapier, type CollisionIndex } from './phys/engine';
import { SimWorld, NO_INPUT, type DriverInput, type FieldData, type SimMode } from './world';
import { Stepper } from './local';
import type { Frame } from './backend';
import { WorkerPlanner } from './planner';

let stepper: Stepper | null = null;

const send = (frame: Frame) => {
  const f = frame.fuel;
  const buffers = [f.st, f.pos, f.prev, f.rot, f.prevRot, f.squashT, f.squashA, f.squashN, f.vel, f.dragF, f.liftF].map((a) => a.buffer);
  buffers.push(frame.net.local.buffer, frame.net.prev.buffer, frame.panels.buffer);
  if (frame.debug) buffers.push(frame.debug.vertices.buffer, frame.debug.colors.buffer, frame.debug.contacts.buffer);
  (self as unknown as Worker).postMessage({ frame }, buffers as ArrayBuffer[]);
};

self.onmessage = async (e: MessageEvent<{ init?: { data: FieldData; collision: { bin: ArrayBuffer; index: CollisionIndex }; seed: number; mode: SimMode }; tick?: { dt: number; input: DriverInput; debug: boolean }; setMode?: SimMode; resumeAI?: boolean }>) => {
  const m = e.data;
  try {
    if (m.init) {
      await initRapier();
      let planner;
      try { planner = typeof Worker !== 'undefined' ? new WorkerPlanner(m.init.data.grid, m.init.data.hubs) : undefined; } catch { planner = undefined; }
      const w = new SimWorld(m.init.data, m.init.collision, m.init.seed, planner);
      if (m.init.mode !== 'free') w.setMode(m.init.mode);
      stepper = new Stepper(w);
      send(stepper.advance(0, NO_INPUT, true));
      return;
    }
    if (!stepper) return;
    if (m.setMode) stepper.world.setMode(m.setMode);
    if (m.resumeAI) stepper.world.resumeAI();
    if (m.tick) send(stepper.advance(m.tick.dt, m.tick.input, true, m.tick.debug));
  } catch (err) {
    (self as unknown as Worker).postMessage({ error: String((err as Error)?.stack ?? err) });
  }
};
