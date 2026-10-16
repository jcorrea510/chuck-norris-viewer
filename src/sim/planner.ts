// Route planning off the render thread. In the browser a Web Worker owns its own copy of the
// navigation grid and answers jobs; the robot keeps following its old path until the new one
// arrives, so planning never stalls a frame. In Node (tests, soak) jobs run inline.
import { FieldGrid, type GridData, type HubData } from './field';
import { runJob, type PlanJob, type PlanResult } from './route';

export interface Planner {
  request(job: PlanJob, cb: (r: PlanResult) => void): void;
  /** jobs sent but not answered yet */
  readonly pending: number;
  /** longest time a job took (ms), for the debug overlay */
  readonly worst: number;
  dispose(): void;
}

export class InlinePlanner implements Planner {
  pending = 0;
  worst = 0;
  constructor(private field: FieldGrid) {}
  request(job: PlanJob, cb: (r: PlanResult) => void) {
    const t0 = performance.now();
    const r = runJob(this.field, job);
    this.worst = Math.max(this.worst, performance.now() - t0);
    cb(r);
  }
  dispose() {}
}

export class WorkerPlanner implements Planner {
  private worker: Worker;
  private next = 1;
  private waiting = new Map<number, (r: PlanResult) => void>();
  worst = 0;
  get pending() { return this.waiting.size; }

  constructor(grid: GridData, hubs: HubData[]) {
    this.worker = new Worker(new URL('./planner.worker.ts', import.meta.url), { type: 'module' });
    this.worker.postMessage({ init: { grid, hubs } });
    this.worker.onmessage = (e: MessageEvent<{ id: number; result: PlanResult; ms: number }>) => {
      const cb = this.waiting.get(e.data.id);
      this.waiting.delete(e.data.id);
      this.worst = Math.max(this.worst, e.data.ms);
      cb?.(e.data.result);
    };
  }
  request(job: PlanJob, cb: (r: PlanResult) => void) {
    const id = this.next++;
    this.waiting.set(id, cb);
    this.worker.postMessage({ id, job });
  }
  /** forget answers still on their way (after a reset) */
  cancelAll() { this.waiting.clear(); }
  dispose() { this.worker.terminate(); this.waiting.clear(); }
}

export function makePlanner(grid: GridData, hubs: HubData[], field: FieldGrid): Planner {
  if (typeof Worker !== 'undefined' && typeof window !== 'undefined') {
    try { return new WorkerPlanner(grid, hubs); } catch { /* fall back */ }
  }
  return new InlinePlanner(field);
}
