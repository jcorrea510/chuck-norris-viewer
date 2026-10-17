// Where the physics runs. The renderer never touches the world directly: it asks a backend to
// advance by the frame's dt with the driver's input and draws the latest Frame (a snapshot of
// everything it needs, with the previous step's poses for interpolation).
//   WorkerBackend — the world runs in a Web Worker (default): physics never costs a frame.
//   LocalBackend  — (local.ts) the world runs on this thread (tests that step the simulation themselves,
//                   ?physics=main, or browsers without module workers).
import type { DriverInput, FieldData, SimMode, SimStatus } from './world';
import type { NetHit } from './phys/hub';
import type { CollisionIndex } from './phys/engine';

export interface RobotFrame {
  x: number; y: number; z: number; th: number; vx: number; vz: number; speed: number; pitch: number; roll: number;
  p: number[]; prevP: number[]; q: number[]; prevQ: number[];
  intakeAngle: number; rollers: string; ferry: boolean;
  /** `a` is the CAD mesh's rotation (rad), `fly`/`flyGoal` rad/s; `aw` is the WPILib azimuth (deg, unwrapped), `hoodDeg` the hood's actual angle, `pulseUs` its servo position, `rps`/`rpsGoal` the flywheel */
  turrets: { id: string; a: number; fly: number; flyGoal: number; aw: number; hoodDeg: number; pulseUs: number; rps: number; rpsGoal: number }[];
  /** motor speeds (rps), the turret lock (D-pad down), and the pose estimate's heading (sim yaw, rad) */
  intakeRollerRps: number; hotdogRps: number; indexerRps: number; turretLocked: boolean; headingEstimate: number;
  modules: { steer: number; wheel: number; current: number; N: number }[];
  battery: number; totalCurrent: number; brownout: boolean; slipping: boolean;
}
export interface FuelFrame {
  n: number; awake: number;
  st: Uint8Array; pos: Float32Array; prev: Float32Array; rot: Float32Array; prevRot: Float32Array;
  squashT: Float32Array; squashA: Float32Array; squashN: Float32Array;
  vel: Float32Array; dragF: Float32Array; liftF: Float32Array;
}
export interface Frame {
  clock: number; alpha: number; steps: number; physMs: number;
  status: SimStatus;
  robot: RobotFrame;
  fuel: FuelFrame;
  netHits: NetHit[];
  /** the hopper's cargo net (knots, robot frame, now and one step ago) and the polycarbonate panels' bend amplitudes (q₁, q₂ per panel, m) */
  net: { local: Float32Array; prev: Float32Array };
  panels: Float32Array;
  /** impacts since the last frame: [speed, x, y, z] … */
  impacts: number[];
  respawned: boolean;
  plannerWorst: number;
  debug?: { vertices: Float32Array; colors: Float32Array; contacts: Float32Array };
}

export interface Backend {
  readonly frame: Frame | null;
  tick(dt: number, input: DriverInput, wantDebug?: boolean): void;
  setMode(mode: SimMode): void;
  resumeAI(): void;
  dispose(): void;
}

const NO_INPUT: DriverInput = { mx: 0, mz: 0, turn: 0, intake: false, shoot: false, unjam: false, reverseIntake: false, faceHub: false, retract: false, resetHeading: false, turretLock: false, handBack: false };
const PRESSES = ['retract', 'resetHeading', 'turretLock', 'handBack'] as const;
type Presses = Record<(typeof PRESSES)[number], boolean>;
const NO_PRESS = (): Presses => ({ retract: false, resetHeading: false, turretLock: false, handBack: false });

export class WorkerBackend implements Backend {
  frame: Frame | null = null;
  private worker: Worker;
  private busy = false;
  private dt = 0;
  private input: DriverInput = NO_INPUT;
  private pendingToggles: Presses = NO_PRESS();
  private debugWanted = false;
  readonly ready: Promise<void>;

  constructor(data: FieldData, collision: { bin: ArrayBuffer; index: CollisionIndex }, seed: number, mode: SimMode) {
    this.worker = new Worker(new URL('./sim.worker.ts', import.meta.url), { type: 'module' });
    this.ready = new Promise((resolve, reject) => {
      this.worker.onmessage = (e: MessageEvent<{ frame?: Frame; error?: string }>) => {
        if (e.data.error) { reject(new Error(e.data.error)); return; }
        if (e.data.frame) { this.frame = e.data.frame; this.busy = false; resolve(); }
      };
      this.worker.onerror = (e) => reject(e.error ?? new Error(e.message));
    });
    this.worker.postMessage({ init: { data, collision, seed, mode } }, [collision.bin]);
  }
  tick(dt: number, input: DriverInput, debug = false) {
    this.dt += dt;
    // a press must never be lost between worker replies
    for (const k of PRESSES) this.pendingToggles[k] ||= input[k];
    this.input = input;
    this.debugWanted = debug;
    if (this.busy) return;
    this.busy = true;
    this.worker.postMessage({ tick: { dt: this.dt, input: { ...this.input, ...this.pendingToggles }, debug: this.debugWanted } });
    this.dt = 0;
    this.pendingToggles = NO_PRESS();
  }
  setMode(mode: SimMode) { this.worker.postMessage({ setMode: mode }); }
  resumeAI() { this.worker.postMessage({ resumeAI: true }); }
  dispose() { this.worker.terminate(); }
}

