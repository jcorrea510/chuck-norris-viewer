// The robot's autonomous brains.
//   Auto    — the team's autonomous: start in the left TRENCH lane, intake up, facing the middle;
//             under the trench, drop the intake, sweep the neutral zone, back under the trench and
//             shoot, then again. It never touches a BUMP.
//   Endless — keeps finding fuel wherever it is (scored fuel pours out of the HUB into the neutral
//             zone), collects it and scores it, forever; each crossing picks the trench or the bump
//             at random. A watchdog notices when it stops making progress and escalates: a new
//             target, backing out, a fresh spot on the field, and finally a reset.
import type { SimWorld } from './world';
import { CLEAR } from './world';
import { VMAX } from './robot';

export const AUTO_SECONDS = 20;

type Step =
  | { kind: 'drive'; path: [number, number][]; speed: number; heading?: number; stop?: boolean; label: string; shoot?: boolean }
  | { kind: 'shoot'; max: number; label: string }
  | { kind: 'intake'; down: boolean };

export class AutoBrain {
  t = 0;
  ended = false;
  phase = 'Autonomous';
  private steps: Step[];
  private i = 0;
  private stepT = 0;

  constructor(private sim: SimWorld) {
    const start = sim.robot;
    const sx = start.x, sz = start.z;
    const tz = -sim.trenchZ;
    const shootAt: [number, number] = [-5.75, tz + 0.85];
    this.steps = [
      { kind: 'drive', label: 'Under the trench', path: [[sx, sz], [-2.45, tz]], speed: 3.4 },
      { kind: 'intake', down: true },
      { kind: 'drive', label: 'Into the fuel', path: [[-2.45, tz], [-1.4, -2.35], [-1.0, -1.88]], speed: 2.6 },
      { kind: 'drive', label: 'Sweeping the neutral zone', path: [[-1.0, -1.88], [1.0, -1.88]], speed: 1.25 },
      { kind: 'drive', label: 'Back under the trench', path: [[1.0, -1.88], [1.3, -2.6], [0.6, tz + 0.1], [-2.45, tz], [-4.9, tz], shootAt], speed: 3.6, stop: true, shoot: true },
      { kind: 'shoot', max: 3.5, label: 'Scoring' },
      { kind: 'drive', label: 'Under the trench', path: [shootAt, [-4.9, tz], [-2.45, tz]], speed: 3.6, shoot: true, heading: Math.PI / 2 },
      { kind: 'drive', label: 'Into the fuel', path: [[-2.45, tz], [-1.35, -2.35], [-1.0, -1.27]], speed: 2.6 },
      { kind: 'drive', label: 'Second sweep', path: [[-1.0, -1.27], [1.0, -1.27]], speed: 1.25 },
      { kind: 'drive', label: 'Back under the trench', path: [[1.0, -1.27], [1.35, -2.3], [0.6, tz + 0.1], [-2.45, tz], [-4.9, tz], shootAt], speed: 3.6, stop: true, shoot: true },
      { kind: 'shoot', max: 6, label: 'Scoring' },
    ];
    sim.robot.rollers = 'in';
    this.begin();
  }

  private begin() {
    const s = this.steps[this.i];
    this.stepT = 0;
    if (!s) return;
    if (s.kind === 'intake') { this.sim.robot.intakeDown = s.down; this.i++; this.begin(); return; }
    if (s.kind === 'drive') {
      this.phase = s.label;
      this.sim.robot.navigate({ path: s.path, speed: s.speed, heading: s.heading ?? 'travel', stopAtEnd: !!s.stop });
    } else this.phase = s.label;
  }

  update(dt: number) {
    const r = this.sim.robot;
    this.t += dt;
    if (this.ended) { r.shoot = false; return; }
    this.stepT += dt;
    const s = this.steps[this.i];
    const fire = this.sim.canShoot() && (!s || s.kind === 'shoot' || (s.kind === 'drive' && !!s.shoot));
    r.shoot = fire;
    // flywheels come up to speed on the way back so the first shot isn't late
    r.spinUp = fire || (s?.kind === 'drive' && !!s.shoot);
    if (this.t >= AUTO_SECONDS) {
      this.ended = true;
      r.navigate(null);
      r.shoot = false;
      this.phase = 'Auto complete';
      return;
    }
    if (!s) return;
    if (s.kind === 'drive' && r.done) { this.i++; this.begin(); }
    else if (s.kind === 'shoot' && (r.stored === 0 || this.stepT > s.max)) { this.i++; this.begin(); }
  }
}

export class EndlessBrain {
  phase = 'Endless';
  private mode: 'collect' | 'score' = 'score';
  private target = -1;
  get targetBall() { return this.mode === 'collect' ? this.target : -1; }
  private replan = 0;
  private progress = { x: 0, z: 0, t: 0, strikes: 0 };
  /** time of the last real progress (fuel collected or scored) */
  private lastGain = 0;
  private lastLoad = 0;
  private lastScored = 0;
  private escalation = 0;
  private blacklist = new Map<number, number>();
  private clock = 0;
  /** intake held up until then, to shrink the footprint and get unwedged */
  private stowUntil = 0;
  private blockedTimes: number[] = [];
  private forced = false;

  constructor(private sim: SimWorld, private idle = false) {
    this.progress.x = sim.robot.x; this.progress.z = sim.robot.z;
    if (!idle) this.toScore();
  }

  /** Called when the AI gets the robot back (from a driver, or after a reset). */
  resume() {
    this.idle = false;
    const r = this.sim.robot;
    r.manual = null;
    r.intakeDown = true;
    r.rollers = 'in';
    this.lastGain = this.clock;
    this.escalation = 0;
    this.progress = { x: r.x, z: r.z, t: 0, strikes: 0 };
    if (r.load >= 8) this.toScore(); else this.toCollect();
  }

  update(dt: number) {
    const sim = this.sim, r = sim.robot;
    this.clock += dt;
    if (this.idle) { this.resume(); }
    this.replan -= dt;
    r.rollers = 'in';
    // back down once the robot has room for it (the arm would shove the robot off a wall otherwise)
    if (!r.intakeDown && this.clock > this.stowUntil && r.drive.clearance(r.x, r.z, r.th, sim.deployedOutline) > 0.03) r.intakeDown = true;
    else if (this.clock > this.stowUntil + 6) r.intakeDown = true;
    const zone = sim.canShoot();
    r.shoot = zone;
    r.spinUp = zone || (this.mode === 'score' && sim.robot.x < -3);

    // progress bookkeeping for the watchdog
    if (r.load > this.lastLoad || sim.balls.scored > this.lastScored) { this.lastGain = this.clock; this.escalation = 0; }
    this.lastLoad = r.load; this.lastScored = sim.balls.scored;

    if (this.mode === 'score') {
      // empty, or a straggler the indexer can't reach yet: go get more (it'll feed on the way)
      if (r.load === 0 || r.load <= 2 && r.done && zone && r.lastShotT > 3) { this.toCollect(); return; }
      if (r.done) {
        this.phase = 'Scoring';
        if (!zone) this.toScore();
      }
    } else {
      const full = r.load >= r.capacity - 1;
      if (full) { this.toScore(); return; }
      const t = this.target;
      const gone = t < 0 || !sim.balls.available(t);
      if (gone && r.load >= 12) { this.toScore(); return; }
      if (gone || this.replan <= 0 || r.done) this.pickTarget();
    }
    this.checkStuck(dt);
  }

  private toScore() {
    const sim = this.sim, r = sim.robot;
    this.mode = 'score';
    this.phase = sim.canShoot() ? 'Scoring' : 'Heading to the HUB';
    // nearest open shooting spot in the alliance zone, 2–4 m from the HUB, with room to turn
    const h = sim.hub;
    let best: [number, number] | null = null, bestD = Infinity;
    for (let x = -7.0; x <= -4.7; x += 0.35) for (let z = -3.3; z <= 3.3; z += 0.35) {
      const dh = Math.hypot(x - h.cx, z - h.cz);
      if (dh < 2.0 || dh > 4.0 || sim.field.distAt(x, z) < 0.86) continue;
      const d = Math.hypot(x - r.x, z - r.z) + Math.abs(z) * 0.15 + (this.escalation > 1 ? Math.random() * 3 : 0);
      if (d < bestD) { bestD = d; best = [x, z]; }
    }
    const goal = best ?? [-5.6, 0.9];
    const route = sim.route(goal[0], goal[1]);
    if (route) {
      r.navigate({ path: route.path, speed: VMAX * 0.85, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading });
      if (route.via && !sim.canShoot()) this.phase = `To the HUB, ${route.via}`;
    }
    this.replan = 6;
  }

  private toCollect() {
    this.mode = 'collect';
    this.target = -1;
    this.pickTarget();
  }

  /** Best fuel to go get: lots of fuel nearby, not too far, reachable with the intake leading. */
  private pickTarget() {
    const sim = this.sim, r = sim.robot, b = sim.balls, g = sim.field;
    this.replan = 2.2;
    const now = this.clock;
    const cand: { i: number; d: number }[] = [];
    for (let i = 0; i < b.n; i++) {
      if (!b.available(i) || sim.isStranded(i)) continue;
      const x = b.pos[i * 3], z = b.pos[i * 3 + 2];
      if (x > 7.7 || g.distAt(x, z) < 0.07) continue;
      if ((this.blacklist.get(i) ?? 0) > now) continue;
      cand.push({ i, d: Math.hypot(x - r.x, z - r.z) });
    }
    if (!cand.length) {
      // nothing left out there: wait by the outpost, where the human player rolls fuel back in
      this.phase = 'Waiting for fuel';
      if (r.load > 0) { this.toScore(); return; }
      if (r.done && Math.hypot(r.x + 6.4, r.z - 2.2) > 0.5) {
        const route = sim.route(-6.4, 2.2);
        if (route) r.navigate({ path: route.path, speed: 2.5, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading });
      }
      return;
    }
    cand.sort((p, q) => p.d - q.d);
    const top = cand.slice(0, 90);
    let best = top[0], bestScore = -Infinity;
    for (const c of top) {
      const x = b.pos[c.i * 3], z = b.pos[c.i * 3 + 2];
      let dens = 0;
      for (const o of top) { const dx = b.pos[o.i * 3] - x, dz = b.pos[o.i * 3 + 2] - z; if (dx * dx + dz * dz < 0.36) dens++; }
      const score = Math.pow(dens, 0.8) / (1 + c.d / 2.0) * (this.escalation > 1 ? 0.5 + Math.random() : 1);
      if (score > bestScore) { bestScore = score; best = c; }
    }
    this.target = best.i;
    this.phase = 'Collecting fuel';
    const bx = b.pos[best.i * 3], bz = b.pos[best.i * 3 + 2];
    // approach with the intake leading; drive straight at walls when fuel is against one
    let dx = bx - r.x, dz = bz - r.z;
    const dl = Math.hypot(dx, dz) || 1;
    dx /= dl; dz /= dl;
    if (g.distAt(bx, bz) < 0.6) {
      const [ax, az] = g.awayFromWalls(bx, bz);
      if (ax || az) { dx = -ax; dz = -az; }
    }
    for (const rot of [0, 0.5, -0.5, 1, -1, 1.6, -1.6]) {
      const c = Math.cos(rot), s = Math.sin(rot);
      const ux = dx * c - dz * s, uz = dx * s + dz * c;
      const ax = bx - ux * 0.95, az = bz - uz * 0.95;
      if (g.distAt(ax, az) < CLEAR) continue;
      const route = sim.route(ax, az);
      if (!route) continue;
      const path = route.path;
      this.phase = route.via ? `Collecting fuel, ${route.via}` : 'Collecting fuel';
      // sweep through the fuel, but only as far as the whole robot (intake out) fits
      const th = Math.atan2(ux, uz);
      const fits = (k: number) => sim.robot.drive.clearance(bx + ux * k, bz + uz * k, th, sim.deployedOutline) > 0.02;
      let run = 0.25;
      while (run < 1.0 && fits(run + 0.1) && g.lineFree(ax, az, bx + ux * (run + 0.1), bz + uz * (run + 0.1), 0.43)) run += 0.1;
      while (run > -0.62 && !fits(run)) run -= 0.06;
      path.push([bx + ux * run, bz + uz * run]);
      r.navigate({ path, speed: 3.6, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading });
      return;
    }
    this.blacklist.set(best.i, now + 12);
    this.target = -1;
  }

  /**
   * Watchdog. Every 1.2 s: did it move? If not, try something else; twice in a row, back out into
   * open space. Separately, if nothing has been collected or scored for a long time, escalate: a
   * different part of the field, then a reset of the robot (it never sits stuck in the pit).
   */
  private checkStuck(dt: number) {
    const sim = this.sim, r = sim.robot, p = this.progress;
    const quiet = this.clock - this.lastGain;
    const waiting = this.phase === 'Waiting for fuel' && r.done;
    if (!waiting && quiet > 25 + this.escalation * 12) {
      this.escalation++;
      this.lastGain = this.clock;
      if (this.escalation >= 4) { sim.respawn(); this.escalation = 0; return; }
      // somewhere else entirely
      this.blacklist.clear();
      if (this.target >= 0) this.blacklist.set(this.target, this.clock + 30);
      const spots: [number, number][] = [[-6, 2], [-6, -2], [-1.5, 0], [0, 2.2], [0, -2.2], [1.8, 0.8]];
      const [gx, gz] = spots[Math.floor(Math.random() * spots.length)];
      const route = sim.route(gx, gz);
      if (route) { r.navigate({ path: route.path, speed: 3, heading: 'travel', stopAtEnd: true, segHeading: route.segHeading }); this.mode = 'collect'; this.target = -1; this.replan = 4; }
      return;
    }
    // orders that keep ending against an obstacle count as being stuck, however much it jiggles
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
      // wedged: raise the intake (the robot gets 22 cm shorter), back out toward whichever direction
      // opens up the most room and straighten up; it comes back down once there's space
      let best: [number, number] = [-Math.sin(r.th), -Math.cos(r.th)], bestC = -Infinity;
      for (let k = 0; k < 16; k++) {
        const a = (k / 16) * Math.PI * 2, dx = Math.sin(a), dz = Math.cos(a);
        const c = Math.min(r.clearance(r.x + dx * 0.3, r.z + dz * 0.3, r.th), r.clearance(r.x + dx * 0.7, r.z + dz * 0.7, r.th));
        if (c > bestC) { bestC = c; best = [dx, dz]; }
      }
      if (r.load <= 32) { r.intakeDown = false; this.stowUntil = this.clock + 2.5; }
      const step = p.strikes >= 4 ? 1.2 : 0.7;
      r.navigate({ path: [[r.x, r.z], [r.x + best[0] * step, r.z + best[1] * step]], speed: 1.6, heading: r.th + (p.strikes >= 4 ? (Math.random() - 0.5) * 1.5 : 0), stopAtEnd: true });
      if (p.strikes >= 6) p.strikes = 0;
      return;
    }
    if (this.mode === 'collect') this.pickTarget(); else this.toScore();
  }
}
