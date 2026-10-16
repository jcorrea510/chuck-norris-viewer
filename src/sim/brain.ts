// The robot's own brain, for when nobody is driving (Free Drive left alone, the pit display):
// it keeps finding fuel (scored fuel pours out of the HUBs into the neutral zone), collects it and
// scores it, forever; each crossing picks the trench or the bump at random. Routes come from the
// planner (a Web Worker in the browser): while a new route computes, the robot keeps following the
// old one, so planning never stalls it. A watchdog notices when it stops making progress and
// escalates: a new target, backing out, a fresh spot on the field, and finally a reset.
// It drives through the same robot code as a human: it works the trigger (LT), the shoot trigger (RT)
// and RB, and the path follower turns its desired motion into stick positions.
import type { SimWorld } from './world';
import { CLEAR, footprintClearance, OUTLINE_DOWN, OUTLINE_UP } from './route';
import { BS } from './phys/fuel';
import { DRIVE } from './phys/config';

const VMAX = DRIVE.speedAt12V;

export class EndlessBrain {
  phase = 'Endless';
  private mode: 'collect' | 'score' = 'score';
  private target = -1;
  get targetBall() { return this.mode === 'collect' ? this.target : -1; }
  private replan = 0;
  private progress = { x: 0, z: 0, t: 0, strikes: 0 };
  private lastGain = 0;
  private lastLoad = 0;
  private lastScored = 0;
  private escalation = 0;
  private blacklist = new Map<number, number>();
  private clock = 0;
  private stowUntil = 0;
  private blockedTimes: number[] = [];
  private forced = false;
  /** while > 0 the trigger is released, so the next press extends the intake again (IntakeCommand extends on its first press only) */
  private ltOff = 0;
  /** a deploy press is under way: wait for the code to see it before judging that it failed */
  private deployWait = 0;
  /** a plan request is out; its answer is ignored if a newer one was sent */
  private ticket = 0;

  constructor(private sim: SimWorld, private idle = false) {
    this.progress.x = sim.robot.x; this.progress.z = sim.robot.z;
  }

  resume() {
    this.idle = false;
    const r = this.sim.robot;
    r.autopilot = true;
    r.releaseControls();
    this.lastGain = this.clock;
    this.escalation = 0;
    this.progress = { x: r.x, z: r.z, t: 0, strikes: 0 };
    if (r.load >= 8) this.toScore(); else this.toCollect();
  }

  /** the robot is collecting and the fuel it is after is within reach of the rollers' pull */
  private collecting() {
    const r = this.sim.robot, f = this.sim.fuel;
    if (this.mode !== 'collect' || r.load >= r.capacity || this.target < 0) return false;
    return Math.hypot(f.pos[this.target * 3] - r.x, f.pos[this.target * 3 + 2] - r.z) < 2.4;
  }

  private available(i: number) {
    const f = this.sim.fuel;
    return f.st[i] === BS.FREE && f.pos[i * 3 + 1] < 0.3 && Math.abs(f.pos[i * 3]) < 8.1 && Math.abs(f.pos[i * 3 + 2]) < 3.95;
  }

  update(dt: number) {
    const sim = this.sim, r = sim.robot;
    this.clock += dt;
    if (this.idle) { this.idle = false; this.resume(); }
    this.replan -= dt;
    this.ltOff = Math.max(0, this.ltOff - dt);
    this.deployWait = Math.max(0, this.deployWait - dt);
    // the intake comes down (LT) when it is stowed and there is room, or after a stuck recovery has waited long enough
    const needDeploy = !r.intakeDown && ((this.clock > this.stowUntil && footprintClearance(sim.field, r.x, r.z, r.th, OUTLINE_DOWN) > 0.03) || this.clock > this.stowUntil + 6);
    if (needDeploy && r.input.intake && this.ltOff <= 0 && this.deployWait <= 0) this.ltOff = 0.06;
    // the rollers run (LT held) only near the fuel it is after: holding LT caps the robot at 2.5 m/s
    r.input.intake = this.ltOff <= 0 && (needDeploy || this.collecting());
    if (needDeploy && r.input.intake) this.deployWait = 0.15;
    const zone = sim.canShoot();
    r.input.shoot = zone && r.stored > 0;
    if (r.load > this.lastLoad || sim.hubs.scoredBy[0] > this.lastScored) { this.lastGain = this.clock; this.escalation = 0; }
    this.lastLoad = r.load; this.lastScored = sim.hubs.scoredBy[0];

    if (this.mode === 'score') {
      if (r.load === 0 || (r.load <= 2 && r.done && zone && r.lastShotT > 3)) { this.toCollect(); return; }
      if (r.done) {
        this.phase = 'Scoring';
        if (!zone && this.replan <= 0) this.toScore();
      }
    } else {
      if (r.load >= r.capacity - 2) { this.toScore(); return; }
      const t = this.target;
      const gone = t < 0 || !this.available(t);
      if (gone && r.load >= 12) { this.toScore(); return; }
      if ((gone && this.replan < 1.6) || this.replan <= 0 || r.done && this.replan < 1.8) this.pickTarget();
    }
    this.checkStuck(dt);
  }

  private go(goals: [number, number][], speed: number, then?: (path: [number, number][], goal: number) => void, prefer?: 'trench' | 'bump') {
    const sim = this.sim, r = sim.robot;
    const ticket = ++this.ticket;
    sim.plan(goals, (res) => {
      if (ticket !== this.ticket || sim.control !== 'ai') return;
      if (!res.route) { if (this.target >= 0) this.blacklist.set(this.target, this.clock + 12); this.target = -1; return; }
      const path = res.route.path;
      then?.(path, res.goal);
      r.navigate({ path, speed, heading: 'travel', stopAtEnd: true, segHeading: res.route.segHeading });
      if (res.route.via) this.phase = `${this.mode === 'score' ? 'To the HUB' : 'Collecting fuel'}, ${res.route.via}`;
    }, prefer);
  }

  private toScore() {
    const sim = this.sim, r = sim.robot;
    this.mode = 'score';
    this.phase = sim.canShoot() ? 'Scoring' : 'Heading to the HUB';
    const h = sim.hubCenter();
    const spots: { p: [number, number]; d: number }[] = [];
    for (let x = -7.0; x <= -4.7; x += 0.35) for (let z = -3.3; z <= 3.3; z += 0.35) {
      const dh = Math.hypot(x - h.x, z - h.z);
      if (dh < 2.0 || dh > 4.0 || sim.field.distAt(x, z) < 0.86) continue;
      spots.push({ p: [x, z], d: Math.hypot(x - r.x, z - r.z) + Math.abs(z) * 0.15 + (this.escalation > 1 ? sim.rng.next() * 3 : 0) });
    }
    spots.sort((a, b) => a.d - b.d);
    this.go(spots.length ? spots.slice(0, 4).map((s) => s.p) : [[-5.6, 0.9]], VMAX * 0.85);
    this.replan = 6;
  }

  private toCollect() { this.mode = 'collect'; this.target = -1; this.pickTarget(); }

  private pickTarget() {
    const sim = this.sim, r = sim.robot, f = sim.fuel, g = sim.field;
    this.replan = 2.4;
    const now = this.clock;
    const cand: { i: number; d: number }[] = [];
    for (let i = 0; i < f.n; i++) {
      if (!this.available(i)) continue;
      const x = f.pos[i * 3], z = f.pos[i * 3 + 2];
      if (x > 7.7 || g.distAt(x, z) < 0.07) continue;
      if ((this.blacklist.get(i) ?? 0) > now) continue;
      cand.push({ i, d: Math.hypot(x - r.x, z - r.z) });
    }
    if (!cand.length) {
      this.phase = 'Waiting for fuel';
      if (r.load > 0) { this.toScore(); return; }
      if (r.done && Math.hypot(r.x + 6.4, r.z - 2.2) > 0.5) this.go([[-6.4, 2.2]], 2.5);
      return;
    }
    cand.sort((p, q) => p.d - q.d);
    const top = cand.slice(0, 90);
    let best = top[0], bestScore = -Infinity;
    for (const c of top) {
      const x = f.pos[c.i * 3], z = f.pos[c.i * 3 + 2];
      let dens = 0;
      for (const o of top) { const dx = f.pos[o.i * 3] - x, dz = f.pos[o.i * 3 + 2] - z; if (dx * dx + dz * dz < 0.36) dens++; }
      const score = Math.pow(dens, 0.8) / (1 + c.d / 2.0) * (this.escalation > 1 ? 0.5 + sim.rng.next() : 1);
      if (score > bestScore) { bestScore = score; best = c; }
    }
    this.target = best.i;
    if (!this.phase.startsWith('Collecting')) this.phase = 'Collecting fuel';
    const bx = f.pos[best.i * 3], bz = f.pos[best.i * 3 + 2];
    let dx = bx - r.x, dz = bz - r.z;
    const dl = Math.hypot(dx, dz) || 1;
    dx /= dl; dz /= dl;
    if (g.distAt(bx, bz) < 0.6) { const [ax, az] = g.awayFromWalls(bx, bz); if (ax || az) { dx = -ax; dz = -az; } }
    // approach points with the intake leading, best first
    const goals: [number, number][] = [], dirs: [number, number][] = [];
    for (const rot of [0, 0.5, -0.5, 1, -1, 1.6, -1.6]) {
      const c = Math.cos(rot), s = Math.sin(rot);
      const ux = dx * c - dz * s, uz = dx * s + dz * c;
      const ax = bx - ux * 0.95, az = bz - uz * 0.95;
      if (g.distAt(ax, az) < CLEAR) continue;
      goals.push([ax, az]); dirs.push([ux, uz]);
    }
    if (!goals.length) { this.blacklist.set(best.i, now + 12); this.target = -1; return; }
    this.go(goals, 3.4, (path, k) => {
      // sweep through the fuel, but only as far as the whole robot (intake out) fits
      const [ux, uz] = dirs[k], [ax, az] = goals[k];
      const th = Math.atan2(ux, uz);
      const fits = (s: number) => footprintClearance(g, bx + ux * s, bz + uz * s, th, OUTLINE_DOWN) > 0.02;
      let run = 0.25;
      while (run < 1.0 && fits(run + 0.1) && g.lineFree(ax, az, bx + ux * (run + 0.1), bz + uz * (run + 0.1), 0.43)) run += 0.1;
      while (run > -0.62 && !fits(run)) run -= 0.06;
      path.push([bx + ux * run, bz + uz * run]);
    });
  }

  private checkStuck(dt: number) {
    const sim = this.sim, r = sim.robot, p = this.progress;
    const quiet = this.clock - this.lastGain;
    const waiting = this.phase === 'Waiting for fuel' && r.done;
    if (!waiting && quiet > 25 + this.escalation * 12) {
      this.escalation++;
      this.lastGain = this.clock;
      if (this.escalation >= 4) { sim.respawn(); this.escalation = 0; return; }
      this.blacklist.clear();
      if (this.target >= 0) this.blacklist.set(this.target, this.clock + 30);
      const spots: [number, number][] = [[-6, 2], [-6, -2], [-1.5, 0], [0, 2.2], [0, -2.2], [1.8, 0.8]];
      this.mode = 'collect'; this.target = -1; this.replan = 4;
      this.go([spots[Math.floor(sim.rng.next() * spots.length)]], 3);
      return;
    }
    if (r.blocked) {
      r.blocked = false;
      this.blockedTimes = this.blockedTimes.filter((t) => this.clock - t < 8);
      this.blockedTimes.push(this.clock);
      if (this.blockedTimes.length >= 2) { this.blockedTimes = []; p.strikes = Math.max(p.strikes, 1); p.t = 1.2; this.forced = true; }
    }
    p.t += dt;
    if (p.t < 1.2) return;
    const moved = Math.hypot(r.x - p.x, r.z - p.z);
    p.t = 0; p.x = r.x; p.z = r.z;
    const holding = r.done && this.mode === 'score' && sim.canShoot();
    const forced = this.forced;
    this.forced = false;
    if (!forced && (moved > 0.15 || holding || r.shooting || waiting)) { p.strikes = 0; return; }
    p.strikes++;
    if (this.target >= 0) this.blacklist.set(this.target, this.clock + 10);
    if (p.strikes >= 2) {
      let best: [number, number] = [-Math.sin(r.th), -Math.cos(r.th)], bestC = -Infinity;
      for (let k = 0; k < 16; k++) {
        const a = (k / 16) * Math.PI * 2, dx = Math.sin(a), dz = Math.cos(a);
        const c = Math.min(footprintClearance(sim.field, r.x + dx * 0.3, r.z + dz * 0.3, r.th, OUTLINE_UP), footprintClearance(sim.field, r.x + dx * 0.7, r.z + dz * 0.7, r.th, OUTLINE_UP));
        if (c > bestC) { bestC = c; best = [dx, dz]; }
      }
      if (r.load <= 32) { r.pulse('retract'); this.stowUntil = this.clock + 2.5; }
      const step = p.strikes >= 4 ? 1.2 : 0.7;
      this.ticket++;
      r.navigate({ path: [[r.x, r.z], [r.x + best[0] * step, r.z + best[1] * step]], speed: 1.6, heading: r.th + (p.strikes >= 4 ? (sim.rng.next() - 0.5) * 1.5 : 0), stopAtEnd: true });
      if (p.strikes >= 6) p.strikes = 0;
      return;
    }
    if (this.mode === 'collect') this.pickTarget(); else this.toScore();
  }
}
