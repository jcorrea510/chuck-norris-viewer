// Seeded random numbers: every bit of variability in the simulation comes from here, so the same
// seed always replays the same match (tests rely on it).
export class Rng {
  private s: number;
  constructor(seed = 1) { this.s = (seed * 2654435761) >>> 0 || 1; }
  /** uniform [0, 1) — mulberry32 */
  next() {
    let t = (this.s = (this.s + 0x6d2b79f5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  range(a: number, b: number) { return a + (b - a) * this.next(); }
  /** standard normal (Box–Muller) */
  gauss() {
    const u = Math.max(1e-12, this.next()), v = this.next();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
}
