// Per-device preferences (kept in this browser only).
export type Appearance = 'system' | 'dark' | 'light';
export type Accent = 'maroon' | 'blue' | 'graphite' | 'gold';

export interface Settings {
  appearance: Appearance;
  accent: Accent;
  hologram: number;        // ghost strength 0.3..1.6
  sound: boolean;
  volume: number;          // 0..1
  autoSpin: boolean;
  spinSeconds: number;     // seconds per revolution
  labels: boolean;
  idleReset: number;       // seconds, 0 = off
  quality: 'battery' | 'balanced' | 'max';
  stats: boolean;
}

export const ACCENTS: Record<Accent, { fill: string; ink: string; name: string }> = {
  maroon: { fill: '#9e1f40', ink: '#f08ca3', name: 'Team Maroon' },
  blue: { fill: '#0a64d8', ink: '#6fb4ff', name: 'Blue' },
  graphite: { fill: '#4a4f58', ink: '#c7ccd6', name: 'Graphite' },
  gold: { fill: '#9a6a12', ink: '#f2c56b', name: 'Gold' },
};

const KEY = 'chuck-norris-settings-v1';

export function defaults(robotSpin: number, robotIdle: number): Settings {
  return {
    appearance: 'dark', accent: 'maroon', hologram: 1, sound: true, volume: 0.6,
    autoSpin: true, spinSeconds: robotSpin, labels: true, idleReset: robotIdle,
    quality: 'balanced', stats: false,
  };
}

export function loadSettings(d: Settings): Settings {
  try {
    const raw = localStorage.getItem(KEY);
    if (raw) return { ...d, ...JSON.parse(raw) };
  } catch { /* storage unavailable: defaults */ }
  return { ...d };
}

export function saveSettings(s: Settings) {
  try { localStorage.setItem(KEY, JSON.stringify(s)); } catch { /* ignore */ }
}

export function isLight(s: Settings) {
  if (s.appearance === 'system') return matchMedia('(prefers-color-scheme: light)').matches;
  return s.appearance === 'light';
}
