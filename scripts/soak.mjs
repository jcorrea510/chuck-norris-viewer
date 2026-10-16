#!/usr/bin/env node
// Bundles scripts/soak.ts with the simulation sources and runs it in Node: node scripts/soak.mjs [mode] [seconds] [seed]
import { build } from 'rolldown';
import { spawnSync } from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const out = path.join(os.tmpdir(), `chuck-soak-${process.pid}.mjs`);
await build({ input: path.join(root, 'scripts/soak.ts'), platform: 'node', output: { file: out, format: 'esm' }, logLevel: 'silent' });
// the bundle reads the field relative to scripts/
const fixed = out.replace(/\.mjs$/, '.run.mjs');
const fs = await import('node:fs');
fs.writeFileSync(fixed, fs.readFileSync(out, 'utf8').replaceAll("new URL('../public/field/field-2026.json', import.meta.url)", JSON.stringify(path.join(root, 'public/field/field-2026.json'))).replaceAll('new URL("../public/field/field-2026.json", import.meta.url)', JSON.stringify(path.join(root, 'public/field/field-2026.json'))));
const r = spawnSync(process.execPath, [fixed, ...process.argv.slice(2)], { stdio: 'inherit' });
fs.rmSync(out, { force: true }); fs.rmSync(fixed, { force: true });
process.exit(r.status ?? 1);
