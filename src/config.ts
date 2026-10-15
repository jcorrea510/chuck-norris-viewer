import raw from './subsystems.json';

export interface Spec { label: string; value: string }
export interface Subsystem {
  id: string;
  name: string;
  /** Short label for the dock chip. Falls back to the first word of `name`. */
  short?: string;
  icon: string;
  summary: string;
  specs: Spec[];
  talkingPoints: string[];
  match: string[];
  /** Optional camera angle override, in degrees. */
  camera?: { azimuth?: number; elevation?: number; zoom?: number };
}
export interface RobotConfig {
  name: string;
  team: string;
  season?: string;
  front: string;
  idleResetSeconds: number;
  autoSpinResumeSeconds: number;
  secondsPerRevolution: number;
  exclude?: string[];
}

export const robot = raw.robot as RobotConfig;
export const subsystems = raw.subsystems as Subsystem[];
export const subsystemById = new Map(subsystems.map((s) => [s.id, s]));
export const shortName = (s: Subsystem) => s.short ?? s.name.split(/[\s&]+/)[0];

/** Where the build step writes the optimized model (relative to the site root). */
export const MODEL_URL = 'models/chuck-norris.glb';
