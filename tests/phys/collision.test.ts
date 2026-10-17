// The robot must be stopped by what the field model shows and by nothing else around the HUBs and TOWERs:
// colliders sit where the visual mesh has its solids, and the robot slides past everything that is clear.
import { describe, test, expect } from 'vitest';
import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';
import { driverWorld, input, run, robotPenetration, fieldData } from './helpers';
import { FieldGrid } from '../../src/sim/field';

const log = (s: string) => console.log(s);

/** z of the two uprights of the TOWER at x = ±7.205, measured on the field model's own vertices (y 0.33–0.6 m). */
async function visualUprights(sx: number) {
  await MeshoptDecoder.ready;
  const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
  const doc = await io.read('public/field/field-2026.glb');
  const zs: number[] = [];
  for (const n of doc.getRoot().listNodes()) {
    const mesh = n.getMesh(); if (!mesh) continue;
    const W = n.getWorldMatrix(), el = [0, 0, 0];
    for (const p of mesh.listPrimitives()) {
      const A = p.getAttribute('POSITION');
      for (let i = 0; i < A.getCount(); i++) {
        A.getElement(i, el);
        const x = W[0] * el[0] + W[4] * el[1] + W[8] * el[2] + W[12], y = W[1] * el[0] + W[5] * el[1] + W[9] * el[2] + W[13], z = W[2] * el[0] + W[6] * el[1] + W[10] * el[2] + W[14];
        if (Math.abs(x - sx * 7.205) < 0.05 && y > 0.33 && y < 0.6 && Math.abs(z) < 1.5) zs.push(z);
      }
    }
  }
  zs.sort((a, b) => a - b);
  return [zs[0], zs[zs.length - 1]];
}

describe('TOWERs', () => {
  test('the solid uprights are where the field model draws them (both towers)', async () => {
    for (const sx of [-1, 1]) {
      const [za, zb] = await visualUprights(sx);
      log(`tower x=${sx * 7.205}: visual uprights at z ${za.toFixed(3)} and ${zb.toFixed(3)}`);
      // backing into the tower (rear first): centered on either upright and between them, the bumper stops at the
      // uprights' face; the robot (0.88 m wide) does not fit between them (0.82 m clear)
      for (const z of [za, (za + zb) / 2, zb]) {
        for (const down of [true, false]) {
          const w = await driverWorld(sx * 5.6, z, -sx * Math.PI / 2, { clearFuel: true, intakeDown: down });
          let worst = 0;
          run(w, 3, input({ mx: sx }), () => { worst = Math.max(worst, robotPenetration(w)); });
          const rear = Math.abs(w.robot.x) + 0.429;
          log(`  z=${z.toFixed(2)} intake ${down ? 'down' : 'up'}: stopped at x=${w.robot.x.toFixed(3)} (rear bumper ${rear.toFixed(3)}), overlap ${(worst * 1000).toFixed(1)} mm`);
          expect(rear).toBeGreaterThan(7.164 - 0.03);
          expect(rear).toBeLessThan(7.164 + 0.03);
          expect(worst).toBeLessThan(0.025);
        }
      }
      // beside the tower there is nothing: drives straight to the alliance wall
      for (const z of [za - 0.75, zb + 0.75]) {
        const w = await driverWorld(sx * 5.6, z, -sx * Math.PI / 2, { clearFuel: true });
        run(w, 3, input({ mx: sx }));
        log(`  beside the tower at z=${z.toFixed(2)}: reached x=${w.robot.x.toFixed(2)}`);
        expect(Math.abs(w.robot.x)).toBeGreaterThan(7.7);
      }
    }
  }, 120000);
});

describe('HUBs', () => {
  test('nothing invisible stands beside the HUB on the neutral-zone side: a robot drives straight past (both HUBs, intake up and down)', async () => {
    for (const sx of [-1, 1]) for (const down of [true, false]) {
      // the net frame's old floor-standing posts were at x = ±(2.775…3.055), z = ±0.73
      const x = sx * 2.55;
      for (const dz of [1, -1]) {
        const w = await driverWorld(x, -dz * 2, Math.PI / 2, { clearFuel: true, intakeDown: down });
        let slow = 0, t = 0, worst = 0;
        run(w, 3.5, input({ mz: dz }), () => { t += 1 / 120; if (t > 1 && w.robot.speed < 1.5 && Math.abs(w.robot.z) < 1.9) slow++; worst = Math.max(worst, robotPenetration(w)); });
        log(`hub ${sx}: along z=${dz} at x=${x}, intake ${down ? 'down' : 'up'}: end z=${w.robot.z.toFixed(2)}, slow steps ${slow}`);
        expect(w.robot.z * dz).toBeGreaterThan(1.9);
        expect(slow).toBe(0);
        expect(worst).toBeLessThan(0.012);
      }
    }
  }, 120000);

  test('shallow-angle slides along the HUB faces and past its corners keep moving (both HUBs, both faces, both ways)', async () => {
    for (const sx of [-1, 1]) for (const down of [true, false]) for (const [name, cx, into] of [['neutral face', 3.048 - 0.429 - 0.01, sx], ['alliance face', 4.242 + 0.429 + 0.01, -sx]] as [string, number, number][]) for (const dz of [1, -1]) {
      // already against the face, then pressed 4° into it while driving along it and off its corner (the robot is
      // done with the face once its trailing edge passes z = ±0.6, i.e. at z = ±1.04)
      const w = await driverWorld(sx * cx, -dz * 0.3, Math.PI / 2, { clearFuel: true, intakeDown: down });
      let slow = 0, worst = 0;
      const t0 = w.clock;
      run(w, 2, input({ mx: into * Math.sin(0.07), mz: dz * Math.cos(0.07) }), () => { if (w.clock - t0 > 0.9 && w.robot.speed < 1.5 && w.robot.z * dz < 1.0) slow++; worst = Math.max(worst, robotPenetration(w)); });
      log(`hub ${sx} ${name} dz ${dz} intake ${down ? 'down' : 'up'}: end (${w.robot.x.toFixed(2)}, ${w.robot.z.toFixed(2)}), slow steps ${slow}`);
      expect(w.robot.z * dz).toBeGreaterThan(1.1);
      expect(slow).toBe(0);
      expect(worst).toBeLessThan(0.012);
    }
  }, 120000);

  test('the HUB itself is solid: driven at full speed into every face the robot stops at the body', async () => {
    for (const sx of [-1, 1]) {
      for (const [name, x, z, mx] of [['from the neutral zone', sx * 2.3, 0.2, sx], ['from our side', sx * 5.2, -0.2, -sx]] as [string, number, number, number][]) {
        const w = await driverWorld(x, z, Math.PI / 2, { clearFuel: true });
        let worst = 0;
        run(w, 3, input({ mx }), () => { worst = Math.max(worst, robotPenetration(w)); });
        log(`hub ${sx} ${name}: stopped at x=${w.robot.x.toFixed(3)}, overlap ${(worst * 1000).toFixed(1)} mm`);
        expect(Math.abs(Math.abs(w.robot.x) - 3.645)).toBeGreaterThan(0.597 + 0.429 - 0.03);
        expect(worst).toBeLessThan(0.012);
      }
    }
  }, 120000);
});

describe('autopilot navigation grid', () => {
  test('marks the HUB body and the TOWERs solid, and the clear space beside the HUB free', () => {
    const d = fieldData();
    const g = new FieldGrid(d.grid, d.hubs);
    for (const sx of [-1, 1]) {
      expect(g.isSolid(sx * 3.645, 0)).toBe(true);
      expect(g.isSolid(sx * 3.645, 0.55)).toBe(true);
      expect(g.isSolid(sx * 2.9, 0.66)).toBe(false);          // where the net frame's feet used to be marked
      expect(g.isSolid(sx * 3.645, 0.65)).toBe(false);
      expect(g.isSolid(sx * 7.205, -sx * 0.289)).toBe(true);     // between and at the uprights
      expect(g.isSolid(sx * 6.5, 0.3)).toBe(false);
    }
  });
});
