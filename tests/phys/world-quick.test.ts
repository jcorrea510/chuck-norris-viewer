import { test } from 'vitest';
import { makeWorld } from './helpers';

test('AI plays free drive for 60 s', async () => {
  const t0 = performance.now();
  const w = await makeWorld(1);
  console.log(`world built in ${(performance.now() - t0).toFixed(0)} ms, ${w.fuel.n} fuel, load ${w.robot.load}`);
  let worst = 0, sum = 0, n = 0;
  for (let k = 0; k < 240 * 60; k++) {
    w.step();
    worst = Math.max(worst, w.stepMs); sum += w.stepMs; n++;
    if (k % (240 * 5) === 0) console.log(`t=${(k / 240).toFixed(0)} pos=(${w.robot.x.toFixed(2)},${w.robot.z.toFixed(2)}) load=${w.robot.load} scored=${w.scored} shots=${w.robot.shotsFired} phase=${w.brain.phase} awake=${w.fuel.awake} step=${(sum / n).toFixed(3)}ms`);
  }
  console.log(`scored ${w.scored}, shots ${w.robot.shotsFired}, mean step ${(sum / n).toFixed(3)} ms, worst ${worst.toFixed(2)} ms`);
});
