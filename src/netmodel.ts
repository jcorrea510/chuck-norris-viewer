// The cargo net in the CAD view: the same cloth as the simulation (src/sim/phys/net.ts), settled at rest
// on a robot that isn't moving, drawn with the same mesh (src/sim/net-mesh.ts). It follows the intake:
// when the four-bar moves the front edge goes with it and the cloth re-settles.
import * as THREE from 'three';
import { Net, type NetRobot } from './sim/phys/net';
import { NetMesh } from './sim/net-mesh';

export class CadNet {
  private net: Net;
  private view = new NetMesh();
  private robot: NetRobot & { intakeAngle: number };
  private angle = NaN;
  private last = 0;
  /** `at`: where the simulation's robot-frame origin (the drivetrain center, floor level) is in the model's own space;
   *  `shown`: whether the hopper is currently solid (not hidden or ghosted by a focus) */
  constructor(parent: THREE.Object3D, at: THREE.Vector3, private shown: () => boolean) {
    const id = (x: number, y: number, z: number, out: number[] = [0, 0, 0]) => { out[0] = x; out[1] = y; out[2] = z; return out; };
    const body = { linvel: () => ({ x: 0, y: 0, z: 0 }), angvel: () => ({ x: 0, y: 0, z: 0 }), applyImpulseAtPoint: () => {} };
    this.robot = { p: [0, 0, 0], intakeAngle: 0, captured: [], body, toWorld: id, toLocal: id };
    this.net = new Net(this.robot);
    this.view.mesh.position.copy(at);
    parent.add(this.view.mesh);
    this.solve(0, 0);
  }

  private solve(deg: number, now: number) {
    this.angle = deg; this.last = now;
    this.robot.intakeAngle = deg;
    this.net.reset();
    this.view.update(this.net.local, null, 1);
  }

  /** Call every frame with the intake's angle (deg, 0 deployed); `sim`: the simulation is drawing its own net. */
  update(intakeDeg: number, sim: boolean, now: number) {
    const visible = !sim && this.shown();
    this.view.mesh.visible = visible;
    if (visible && Math.abs(intakeDeg - this.angle) >= 0.4 && now - this.last >= 50) this.solve(intakeDeg, now);
  }
}
