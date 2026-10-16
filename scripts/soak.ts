// Runs the simulation (physics + brains, no rendering) for hours of simulated time in Node and
// reports anything a pit display must never do: get stuck, stop scoring, lose fuel, go NaN.
//   node scripts/soak.mjs endless 7200      (simulated seconds)
//   node scripts/soak.mjs auto
//   node scripts/soak.mjs drive 600         (random driver input bursts, AI takes back over)
import fs from 'node:fs';
import { SimWorld, STEP, NO_INPUT, type DriverInput } from '../src/sim/world';
import { S } from '../src/sim/balls';

const mode = process.argv[2] ?? 'endless';
const seconds = Number(process.argv[3] ?? (mode === 'auto' ? 20 : 3600));
const seed = Number(process.argv[4] ?? 1);
// deterministic Math.random for reproducible runs
let st = seed * 2654435761 >>> 0;
Math.random = () => { st = (st * 1664525 + 1013904223) >>> 0; return st / 4294967296; };

const data = JSON.parse(fs.readFileSync(new URL('../public/field/field-2026.json', import.meta.url), 'utf8'));
const w = new SimWorld(data);
w.setMode(mode === 'auto' ? 'auto' : 'endless');
if (mode === 'wtest') {
  // the AI drives for 5 s, then the driver asks for +z at full stick
  for (let k = 0; k < 5 / STEP; k++) w.step(STEP);
  const r0 = w.robot;
  console.log(`start pos=(${r0.x.toFixed(2)},${r0.z.toFixed(2)}) th=${(r0.th * 57.3).toFixed(0)} v=(${r0.vx.toFixed(2)},${r0.vz.toFixed(2)})`);
  for (let k = 0; k < 0.7 / STEP; k++) {
    w.step(STEP, { ...NO_INPUT, mz: 1 });
    const d = r0.drive;
    if (k % 6 === 0) console.log(`${(k * STEP).toFixed(2)} pos=(${r0.x.toFixed(2)},${r0.z.toFixed(2)}) v=(${r0.vx.toFixed(2)},${r0.vz.toFixed(2)}) w=${r0.w.toFixed(2)} th=${(r0.th * 57.3).toFixed(0)} steer=${Array.from(d.steer).map((x) => (x * 57.3).toFixed(0)).join('/')} wheel=${Array.from(d.wheel).map((x) => x.toFixed(2)).join('/')} slip=${d.slipping} eff=${d.effort.toFixed(2)} imp=${d.impact.toFixed(1)} clr=${(d.clearance(d.x, d.z, d.th) * 100).toFixed(1)}`);
  }
  process.exit(0);
}
if (mode === 'dyn') {
  // drivetrain check on open carpet: full stick, release, full spin, diagonal strafe
  const r0 = w.robot;
  w.robot.place(0, 0, 0); w.robot.setIntakeNow(true);
  w.takeOver();
  const run = (label: string, inp: Partial<DriverInput>, secs: number) => {
    const lines: string[] = [];
    let x0 = r0.x, z0 = r0.z;
    for (let k = 0; k < secs / STEP; k++) {
      w.step(STEP, { ...NO_INPUT, ...inp });
      if (k % 12 === 11) lines.push(`${((k + 1) * STEP).toFixed(1)}s v=${r0.speed.toFixed(2)} w=${r0.w.toFixed(2)} slip=${r0.drive.slipping ? 1 : 0}`);
    }
    console.log(`${label}: moved ${Math.hypot(r0.x - x0, r0.z - z0).toFixed(2)} m\n  ` + lines.join(' | '));
  };
  w.robot.place(-1.5, 0, Math.PI / 2); w.takeOver();
  run('full forward (+x)', { mx: 1 }, 1.2);
  run('release', { mx: 0.0001 }, 0.8);
  run('spin', { turn: 1 }, 1.0);
  run('release spin', { turn: 0.0001 }, 0.6);
  run('strafe while spinning', { mz: 1, turn: 1 }, 1.0);
  process.exit(0);
}
const t0 = Date.now();
const r = w.robot, b = w.balls;
let lastScore = 0, lastScoreT = 0, maxGap = 0, gapAt = [0, 0], nan = 0, respawns = 0;
const stuck: string[] = [];
let probe = { x: r.x, z: r.z, t: 0 };
const overlapLog: string[] = [];
let minClear = Infinity, hopperWorst = 0, overCap = 0;
w.onRespawn = () => { respawns++; stuck.push(`  RESPAWN at t=${w.clock.toFixed(0)}`); };
let input: DriverInput = NO_INPUT, burst = 0;
const steps = Math.round(seconds / STEP);
for (let k = 0; k < steps; k++) {
  if (mode === 'drive') {
    // random driving bursts, then hands off for a while (the AI takes back over after 15 s)
    if (burst <= 0 && Math.random() < STEP / 40) burst = 5 + Math.random() * 20;
    if (burst > 0) {
      burst -= STEP;
      if (Math.random() < STEP * 2) {
        const a = Math.random() * Math.PI * 2, m = Math.random();
        input = { ...NO_INPUT, mx: Math.cos(a) * m, mz: Math.sin(a) * m, turn: (Math.random() - 0.5) * 2, shoot: Math.random() < 0.4, intake: Math.random() < 0.5, eject: Math.random() < 0.08, lock: false, slow: Math.random() < 0.1, toggleArm: Math.random() < 0.1 };
      } else input = { ...input, toggleArm: false };
    } else input = NO_INPUT;
  }
  w.step(STEP, input);
  const t = w.clock;
  if (b.scored !== lastScore) { if (t - lastScoreT > maxGap) { maxGap = t - lastScoreT; gapAt = [r.x, r.z]; } lastScore = b.scored; lastScoreT = t; }
  const c = r.drive.clearance(r.x, r.z, r.th);
  if (c < minClear) minClear = c;
  if (!Number.isFinite(r.x + r.z + r.th)) nan++;
  if (r.load > r.capacity + 1) overCap++;
  // no movement for 20 s while the AI drives = stuck
  if (Math.hypot(r.x - probe.x, r.z - probe.z) > 0.3) probe = { x: r.x, z: r.z, t };
  else if (t - probe.t > 20 && w.control === 'ai' && !(w.brain.phase === 'Waiting for fuel') && !(w.brain.phase === 'Scoring' && r.stored > 0)) {
    const br = w.brain as unknown as { targetBall?: number; mode?: string };
    const tb = br.targetBall ?? -1;
    const end = r.order?.path.at(-1);
    stuck.push(`  STUCK ${(t - probe.t).toFixed(0)}s at t=${t.toFixed(0)} (${r.x.toFixed(2)},${r.z.toFixed(2)}) th=${(r.th * 57.3).toFixed(0)} load=${r.load} phase="${w.brain.phase}" intake=${r.intakeAngle.toFixed(0)}\n` +
      `      done=${r.done} brainMode=${br.mode} orderEnd=${end ? end.map((v) => v.toFixed(2)) : '-'} pts=${r.order?.path.length ?? 0} cmd=(${r.cmd.vx.toFixed(2)},${r.cmd.vz.toFixed(2)},${r.cmd.w.toFixed(2)}) v=(${r.vx.toFixed(2)},${r.vz.toFixed(2)}) clear=${(c * 100).toFixed(1)}cm` +
      ` target=${tb >= 0 ? `#${tb} st=${b.st[tb]} (${b.pos[tb * 3].toFixed(2)},${b.pos[tb * 3 + 2].toFixed(2)}) avail=${b.available(tb)} stranded=${w.isStranded(tb)}` : '-'} rollers=${r.rollers}`);
    probe.t = t;
  }
  if (k % Math.round(600 / STEP) === 0 || k === steps - 1) {
    const counts = [0, 0, 0, 0, 0, 0, 0];
    for (let i = 0; i < b.n; i++) { counts[b.st[i]]++; if (!Number.isFinite(b.pos[i * 3] + b.pos[i * 3 + 1] + b.pos[i * 3 + 2])) nan++; }
    console.log(`${t.toFixed(0).padStart(6)}s scored=${String(b.scored).padStart(5)} robot=(${r.x.toFixed(2)},${r.z.toFixed(2)}) load=${String(r.load).padStart(2)} ${w.control} "${w.brain.phase}" rest=${counts[S.REST]} moving=${counts[S.MOVING]} held=${counts[S.STORED] + counts[S.INTAKING]} flying=${counts[S.FLYING]} hub=${counts[S.IN_HUB]} parked=${counts[S.PARKED]} reachable=${w.reachableCount()}`);
  }
  if (k % 12 === 0) {
    const held: [number, number, number, string][] = [];
    r.forEachHeld((_i, x, y, z, wh) => held.push([x, y, z, wh]));
    for (let a = 0; a < held.length; a++) for (let q = a + 1; q < held.length; q++) {
      const ov = 0.15 - Math.hypot(held[a][0] - held[q][0], held[a][1] - held[q][1], held[a][2] - held[q][2]);
      if (ov > 0.05 && overlapLog.length < 8) overlapLog.push(`  OVERLAP ${(ov * 1000).toFixed(0)}mm t=${w.clock.toFixed(1)} ${held[a][3]} (${held[a].slice(0, 3).map((v) => (+v).toFixed(2))}) vs ${held[q][3]} (${held[q].slice(0, 3).map((v) => (+v).toFixed(2))}) intake=${r.intakeAngle.toFixed(0)}`);
      hopperWorst = Math.max(hopperWorst, ov);
    }
  }
}
const real = (Date.now() - t0) / 1000;
console.log(stuck.slice(0, 40).join('\n'));
console.log(overlapLog.join('\n'));
console.log(`\n${mode}: ${seconds}s simulated in ${real.toFixed(1)}s (${(seconds / real).toFixed(0)}× real time)`);
console.log(`shots=${r.shotsFired} scored=${b.scored} accuracy=${(b.scored / Math.max(1, r.shotsFired) * 100).toFixed(1)}% rate=${(b.scored / seconds * 60).toFixed(1)}/min longest gap=${maxGap.toFixed(1)}s at (${gapAt[0].toFixed(1)},${gapAt[1].toFixed(1)})`);
console.log(`stuck episodes=${stuck.length} respawns=${respawns} NaN=${nan} min footprint clearance=${(minClear * 100).toFixed(1)}cm worst hopper overlap=${(hopperWorst * 1000).toFixed(1)}mm over-capacity steps=${overCap}`);
