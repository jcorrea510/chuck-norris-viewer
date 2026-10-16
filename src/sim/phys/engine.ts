// The rigid-body engine (Rapier, WASM) behind a small wrapper, so gameplay code never touches
// Rapier directly. Owns the world, the carpet, the field's collision mesh (built offline from the
// real field model by scripts/build-collision.mjs) and the collision groups.
import RAPIER from '@dimforge/rapier3d-compat';
import { FUEL, STEP } from './config';

export type Rapier = typeof RAPIER;
export type Surface = 'carpet' | 'hdpe' | 'polycarb' | 'steel' | 'net' | 'bumper' | 'robot' | 'fuel';

// Collision groups: membership in the high 16 bits, what it collides with in the low 16.
export const GROUP = { FIELD: 1, BALL: 2, HULL: 4, ROBOT_BALL: 8, SENSOR: 16 };
export const groups = (member: number, filter: number) => (member << 16) | filter;
export const IG = {
  field: groups(GROUP.FIELD, GROUP.BALL | GROUP.HULL),
  ball: groups(GROUP.BALL, GROUP.FIELD | GROUP.BALL | GROUP.ROBOT_BALL),
  hull: groups(GROUP.HULL, GROUP.FIELD),
  robotBall: groups(GROUP.ROBOT_BALL, GROUP.BALL),
  /** bumpers: they hit the field and push fuel */
  bumper: groups(GROUP.HULL | GROUP.ROBOT_BALL, GROUP.FIELD | GROUP.BALL),
  /** what the wheels' rays see */
  ground: groups(0xffff, GROUP.FIELD),
};

/** Ball colliders carry √(ball–ball value) and every other surface value ÷ √(ball–ball value), so the
 *  Multiply combine rule gives exactly the table in config.ts for every pair. */
export const BALL_MU = Math.sqrt(FUEL.friction.fuel);
export const BALL_E = Math.sqrt(FUEL.restitution.fuel);
export function surfaceMu(s: Surface) { return FUEL.friction[s as keyof typeof FUEL.friction] / BALL_MU; }
export function surfaceE(s: Surface) { return FUEL.restitution[s as keyof typeof FUEL.restitution] / BALL_E; }

export interface CollisionIndex { surfaces: { kind: Surface; vertexOffset: number; vertexCount: number; indexOffset: number; indexCount: number }[] }

let ready: Promise<void> | null = null;
export function initRapier() { return ready ??= RAPIER.init(); }

export class Engine {
  readonly R = RAPIER;
  readonly world: RAPIER.World;
  readonly events: RAPIER.EventQueue;
  /** collider handle → surface, for sounds and per-surface behavior */
  readonly surfaceOf = new Map<number, Surface>();
  ground!: RAPIER.Collider;

  constructor() {
    const R = RAPIER;
    this.world = new R.World({ x: 0, y: -9.81, z: 0 });
    this.world.timestep = STEP;
    const ip = this.world.integrationParameters;
    ip.numSolverIterations = 4;
    ip.numAdditionalFrictionIterations = 2;
    ip.maxCcdSubsteps = 2;
    (ip as unknown as { contact_natural_frequency: number }).contact_natural_frequency = 60;
    this.events = new R.EventQueue(true);
    // the carpet: a slab whose top is y = 0 over the whole field and beyond
    const g = R.ColliderDesc.cuboid(14, 0.5, 8).setTranslation(0, -0.5, 0)
      .setFriction(surfaceMu('carpet')).setRestitution(surfaceE('carpet'))
      .setFrictionCombineRule(R.CoefficientCombineRule.Multiply).setRestitutionCombineRule(R.CoefficientCombineRule.Multiply)
      .setCollisionGroups(IG.field);
    this.ground = this.world.createCollider(g);
    this.surfaceOf.set(this.ground.handle, 'carpet');
    // a safety box just outside the field perimeter, so nothing can ever leave the field even
    // through a gap in the simplified mesh (the real guardrails and walls are in the field mesh)
    const wall = (x: number, z: number, hx: number, hz: number) => {
      const c = this.world.createCollider(R.ColliderDesc.cuboid(hx, 2.5, hz).setTranslation(x, 2.5, z)
        .setFriction(surfaceMu('polycarb')).setRestitution(surfaceE('polycarb'))
        .setFrictionCombineRule(R.CoefficientCombineRule.Multiply).setRestitutionCombineRule(R.CoefficientCombineRule.Multiply)
        .setCollisionGroups(IG.field));
      this.surfaceOf.set(c.handle, 'polycarb');
    };
    wall(-8.42, 0, 0.12, 4.6); wall(8.42, 0, 0.12, 4.6);
    wall(0, -4.18, 8.6, 0.12); wall(0, 4.18, 8.6, 0.12);
  }

  /** The field's walls, guardrails, TRENCHes, TOWERs, BUMPs, DEPOTs and OUTPOSTs (one trimesh per surface). */
  addField(bin: ArrayBuffer, index: CollisionIndex) {
    const R = this.R;
    for (const s of index.surfaces) {
      const verts = new Float32Array(bin, s.vertexOffset, s.vertexCount * 3);
      const idx = new Uint32Array(bin.slice(s.indexOffset, s.indexOffset + s.indexCount * 4));
      const desc = R.ColliderDesc.trimesh(new Float32Array(verts), idx, R.TriMeshFlags.FIX_INTERNAL_EDGES)
        .setFriction(surfaceMu(s.kind)).setRestitution(surfaceE(s.kind))
        .setFrictionCombineRule(R.CoefficientCombineRule.Multiply).setRestitutionCombineRule(R.CoefficientCombineRule.Multiply)
        .setCollisionGroups(IG.field);
      const c = this.world.createCollider(desc);
      this.surfaceOf.set(c.handle, s.kind);
    }
    this.addTrenchArms();
    this.addTowers();
  }

  /**
   * The four TRENCH arms as solid steel boxes, matched to the field mesh (underside 0.565 m, the
   * manual's 22.25 in; 0.10 m deep bar at x = ±3.63, spanning the lane from the support to the
   * guardrail). A thin triangle-mesh bar hit edge-on by a robot can resolve the overlap the wrong
   * way (pushing the robot up into a wheelie); a solid box always pushes it straight back.
   */
  addTrenchArms() {
    const R = this.R;
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
      this.fixed(R.ColliderDesc.cuboid(0.081, 0.049, 0.63).setTranslation(sx * 3.64, 0.6155, sz * 3.405), 'steel');
    }
  }

  /**
   * Each TOWER's UPRIGHTS and RUNGS as solid steel (manual 5.8: 72.1 in tall, 1.5 × 3.5 in box
   * uprights 32.25 in apart; 1.66 in OD rungs at 27, 45 and 63 in, reaching 5.875 in past each
   * upright), placed where the field model has them. As thin triangle shells a robot could drive
   * straight through them.
   */
  addTowers() {
    const R = this.R, IN = 0.0254;
    const x = 7.205, zc = 0.295;                       // our tower (internal −x); the other is rotated 180°
    const half = (32.25 / 2 + 1.5 / 2) * IN;          // upright centers from the tower's center line
    const rungHalf = (32.25 / 2 + 1.5 + 5.875) * IN;
    for (const s of [-1, 1]) {
      for (const dz of [-half, half]) {
        this.fixed(R.ColliderDesc.cuboid(1.75 * IN, 72.1 * IN / 2, 0.75 * IN).setTranslation(s * x, 72.1 * IN / 2, s * (zc + dz)), 'steel');
      }
      for (const h of [27, 45, 63]) {
        this.fixed(R.ColliderDesc.cylinder(rungHalf, 1.66 * IN / 2).setRotation({ x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 })
          .setTranslation(s * x, h * IN, s * zc), 'steel');
      }
    }
  }

  /** A fixed collider with the field's material rules (used for the HUB). */
  fixed(desc: RAPIER.ColliderDesc, surface: Surface, body?: RAPIER.RigidBody) {
    const R = this.R;
    desc.setFriction(surfaceMu(surface)).setRestitution(surfaceE(surface))
      .setFrictionCombineRule(R.CoefficientCombineRule.Multiply).setRestitutionCombineRule(R.CoefficientCombineRule.Multiply);
    if (!desc.collisionGroups || desc.collisionGroups === 0xffffffff) desc.setCollisionGroups(IG.field);
    const c = this.world.createCollider(desc, body);
    this.surfaceOf.set(c.handle, surface);
    return c;
  }

  /** Ray straight down onto the field (wheels), ignoring fuel and the robot. */
  castDown(x: number, y: number, z: number, max: number, dir = { x: 0, y: -1, z: 0 }) {
    const ray = new this.R.Ray({ x, y, z }, dir);
    return this.world.castRayAndGetNormal(ray, max, true, undefined, IG.ground);
  }

  step() { this.world.step(this.events); }

  dispose() { this.world.free(); this.events.free(); }
}

/** Collision data for Node (tests, soak) and the browser alike. */
export async function loadCollision(fetchBin: () => Promise<ArrayBuffer>, fetchIndex: () => Promise<CollisionIndex>) {
  const [bin, index] = await Promise.all([fetchBin(), fetchIndex()]);
  return { bin, index };
}
