// The intake's four-bar (CAD, robot frame: z forward, y up): where the coupler (hopper front wall, side
// plates, roller) sits for a crank angle. Shared by the physics, the net and the renderers.
const DEG = Math.PI / 180;
/** crank pivot and the coupler joint at the deployed pose (z, y) */
export const PIVOT: [number, number] = [0.2094, 0.2164];
export const JOINT: [number, number] = [0.4521, 0.2913];
const DZ = JOINT[0] - PIVOT[0], DY = JOINT[1] - PIVOT[1];
/** the coupler's displacement (z, y) from the deployed pose at a crank angle (deg; 0 deployed, −73 stowed) */
export function couplerOffset(deg: number): [number, number] {
  const a = deg * DEG, c = Math.cos(a), s = Math.sin(a);
  return [PIVOT[0] + DY * s + DZ * c - JOINT[0], PIVOT[1] + DY * c - DZ * s - JOINT[1]];
}
