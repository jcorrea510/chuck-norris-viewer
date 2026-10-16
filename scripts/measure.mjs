#!/usr/bin/env node
// Prints part boxes in the simulation's robot frame (stage frame: front = +z, ground = 0), for
// tuning the hopper / indexer / turret geometry in src/sim: node scripts/measure.mjs [pattern…]
import { preview } from 'vite';
import { chromium } from 'playwright';
const patterns = process.argv.slice(2);
const server = await preview({ preview: { port: 4177 }, logLevel: 'warn' });
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 640, height: 480 } });
page.on('pageerror', (e) => console.log('pageerror', String(e)));
await page.goto('http://localhost:4177/');
await page.waitForFunction(() => window.__viewer?.state.ready, null, { timeout: 120000 });
await page.evaluate(() => window.__viewer.api.enterSim('endless'));
await page.waitForFunction(() => window.__viewer.state.sim?.active, null, { timeout: 120000 });
const out = await page.evaluate((patterns) => {
  const v = window.__viewer.viewer, sim = window.__viewer.state.sim;
  v.hooks.clear();
  const P = v.parts;
  const lines = [];
  const f = (b) => `[${b.min.x.toFixed(3)}, ${b.min.y.toFixed(3)}, ${b.min.z.toFixed(3)}] → [${b.max.x.toFixed(3)}, ${b.max.y.toFixed(3)}, ${b.max.z.toFixed(3)}]`;
  const pose = (intake) => {
    v.stage.position.set(0, 0, 0); v.stage.quaternion.identity(); v.stage.updateMatrix(); v.stage.matrixWorld.copy(v.stage.matrix);
    v.setMechanismNow('intake', intake);
    for (const id of ['turret-left', 'turret-right']) v.setMechanismNow(id, 0);
    v.syncPose();
  };
  lines.push(`root rotation.y=${v.root.rotation.y.toFixed(4)} root pos=${v.root.position.toArray().map((x) => x.toFixed(4))}`);
  for (const intake of [0, -73]) {
    pose(intake);
    lines.push(`\n=== intake ${intake}° ===`);
    for (const sub of ['swerve', 'intake', 'hopper', 'indexer', 'turrets', 'electrical', 'frame']) lines.push(`subsystem ${sub.padEnd(10)} ${f(v.boxOfParts(v.subsystemPids(sub), false))}`);
    for (const [id, m] of v.mechanisms) { const p = m.node.getWorldPosition(v.camera.position.clone()); lines.push(`mech ${id.padEnd(12)} pivot (${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)})`); }
    for (const pat of patterns) {
      const pids = P.partsMatching([pat]);
      // group identical names, list each instance
      lines.push(`-- ${pat}: ${pids.length} parts, union ${pids.length ? f(v.boxOfParts(pids, false)) : '-'}`);
      const seen = new Map();
      for (const p of pids) {
        const nm = P.name(P.nodeOfPart(p));
        const k = (seen.get(nm) ?? 0) + 1; seen.set(nm, k);
        if (k > 12) continue;
        lines.push(`   ${nm.slice(0, 46).padEnd(46)} ${f(v.boxOfParts([p], false))}`);
      }
    }
  }
  return lines;
}, patterns).catch(async (e) => { console.log(String(e)); return []; });
console.log(out.join('\n'));
await browser.close(); server.httpServer.close();
