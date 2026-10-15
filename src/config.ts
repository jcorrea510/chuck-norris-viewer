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
  /** 3-4 very short bullets. */
  highlights: string[];
  match: string[];
  /** Parts (by name) hidden while this subsystem is selected, e.g. panels that block the view. */
  hideWhenFocused?: string[];
  /** Parts (by name) the camera centers on when the subsystem is selected. Default: all of them. */
  frameOn?: string[];
  /** Optional camera angle override, in degrees. */
  camera?: { azimuth?: number; elevation?: number; zoom?: number };
}
export interface RobotConfig {
  name: string;
  team: string;
  season?: string;
  front: string;
  /** Seconds of no input before the pit display takes over (UI hides, robot turns). */
  pitModeSeconds: number;
  autoSpinResumeSeconds: number;
  secondsPerRevolution: number;
  exclude?: string[];
}

export const robot = raw.robot as RobotConfig;
/** Which subsystem's panel shows each mechanism's slider. */
export const mechanismSubsystem = new Map((raw.mechanisms as { id: string; subsystem: string }[]).map((m) => [m.id, m.subsystem]));
export const subsystems = raw.subsystems as Subsystem[];
export const subsystemById = new Map(subsystems.map((s) => [s.id, s]));
export const shortName = (s: Subsystem) => s.short ?? s.name.split(/[\s&]+/)[0];

/** Where the build step writes the optimized model (relative to the site root). */
export const MODEL_URL = 'models/chuck-norris.glb';
export const PARTS_URL = 'models/chuck-norris.parts.json';
