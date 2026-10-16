// The route planner's Web Worker: A* on the navigation grid, off the render thread.
import { FieldGrid, type GridData, type HubData } from './field';
import { runJob, type PlanJob } from './route';

let field: FieldGrid | null = null;
self.onmessage = (e: MessageEvent<{ init?: { grid: GridData; hubs: HubData[] }; id?: number; job?: PlanJob }>) => {
  if (e.data.init) { field = new FieldGrid(e.data.init.grid, e.data.init.hubs); return; }
  if (!field || !e.data.job) return;
  const t0 = performance.now();
  const result = runJob(field, e.data.job);
  (self as unknown as Worker).postMessage({ id: e.data.id, result, ms: performance.now() - t0 });
};
