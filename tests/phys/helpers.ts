import fs from 'node:fs';
import { Engine, initRapier, type CollisionIndex } from '../../src/sim/phys/engine';

export async function makeEngine(withField = true) {
  await initRapier();
  const eng = new Engine();
  if (withField) {
    const buf = fs.readFileSync('public/field/field-collision.bin');
    const bin = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
    const index = JSON.parse(fs.readFileSync('public/field/field-collision.json', 'utf8')) as CollisionIndex;
    eng.addField(bin, index);
  }
  return eng;
}
export const fieldData = () => JSON.parse(fs.readFileSync('public/field/field-2026.json', 'utf8'));

import { SimWorld } from '../../src/sim/world';
export async function makeWorld(seed = 1) {
  await initRapier();
  const buf = fs.readFileSync('public/field/field-collision.bin');
  const bin = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  const index = JSON.parse(fs.readFileSync('public/field/field-collision.json', 'utf8')) as CollisionIndex;
  return new SimWorld(fieldData(), { bin, index }, seed);
}
