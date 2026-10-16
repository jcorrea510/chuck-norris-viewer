import { test } from 'vitest';
import { makeWorld } from './helpers';
import { BS } from '../../src/sim/phys/fuel';
test('shot accuracy debug', async () => {
  const w = await makeWorld(1);
  const r = w.robot, f = w.fuel;
  w.takeOver();
  r.place(-6.1, 0.0, Math.PI / 2);
  r.preload([0,1,2,3,4,5,6,7].map((k) => 300 + k));
  for (let k = 0; k < 240; k++) w.step();
  const tracked = new Map<number, { cross?: number[]; apex: number; p0?: number[]; v0?: number[] }>();
  r.onShot = (i) => { const o = i * 3; tracked.set(i, { apex: 0, p0: [f.pos[o], f.pos[o+1], f.pos[o+2]], v0: [f.vel[o], f.vel[o+1], f.vel[o+2]] }); };
  const inp = { mx: 0, mz: 0, turn: 0, shoot: true, intake: false, eject: false, lock: false, slow: false, toggleArm: false, toggleIntake: false, handBack: false };
  for (let k = 0; k < 240 * 5; k++) {
    w.step(inp);
    for (const [i, t] of tracked) {
      const o = i * 3;
      t.apex = Math.max(t.apex, f.pos[o + 1]);
      if (!t.cross && f.vel[o + 1] < 0 && f.pos[o + 1] < 1.83 && t.apex > 1.83) t.cross = [f.pos[o], f.pos[o + 2]];
    }
  }
  for (const [i, t] of tracked) console.log(`ball ${i} p0=${t.p0!.map(v=>v.toFixed(2))} v0=${t.v0!.map(v=>v.toFixed(2))} |v|=${Math.hypot(...t.v0!).toFixed(2)} apex=${t.apex.toFixed(2)} cross=${t.cross ? t.cross.map((v) => v.toFixed(2)) : '-'} st=${f.st[i]}`);
  console.log('scored', w.hubs.scoredBy, 'turret', r.turrets.map(t => `${(t.hood*57.3).toFixed(1)} ${t.flyGoal.toFixed(0)} tof ${t.tof.toFixed(2)}`));
});
