import * as THREE from 'three';

/**
 * A photographic studio for image-based lighting: a dark room with large softboxes, a cool rim
 * strip and a warm kicker. Metals pick up real contrast and color from it (a flat gray room makes
 * aluminum look like white plastic). Rendered once into a PMREM; nothing is downloaded.
 */
export function studioScene(light: boolean): THREE.Scene {
  const scene = new THREE.Scene();
  const wall = light ? 0.32 : 0.035;
  const room = new THREE.Mesh(
    new THREE.BoxGeometry(14, 8, 14),
    new THREE.MeshBasicMaterial({ color: new THREE.Color(wall, wall, wall * 1.06), side: THREE.BackSide }),
  );
  room.position.y = 3;
  scene.add(room);
  // floor: slightly brighter so undersides get bounce light
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(14, 14), new THREE.MeshBasicMaterial({ color: new THREE.Color(wall * 2.2, wall * 2.2, wall * 2.3) }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.98;
  scene.add(floor);

  const box = (w: number, h: number, color: THREE.Color, pos: [number, number, number], look: [number, number, number] = [0, 0.3, 0]) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide }));
    m.position.set(...pos);
    m.lookAt(...look);
    scene.add(m);
  };
  const hdr = (r: number, g: number, b: number, k: number) => new THREE.Color(r * k, g * k, b * k);
  box(5, 2.6, hdr(1, 0.98, 0.95, 9), [0, 5.2, 0.6]);           // big overhead key softbox
  box(2.2, 3.2, hdr(1, 0.97, 0.93, 6), [-4.6, 2.6, 3.2]);      // front-left softbox
  box(1.6, 2.4, hdr(1, 1, 1, 3.2), [4.4, 2.2, 3.8]);           // front-right fill
  box(0.5, 5, hdr(0.62, 0.8, 1, 10), [5.6, 2.4, -3.2]);        // cool rim strip
  box(0.5, 5, hdr(0.62, 0.8, 1, 7), [-5.6, 2.4, -3.6]);        // second rim strip
  box(4.5, 0.4, hdr(1, 0.82, 0.62, 5), [0, 1.2, -6.4]);        // warm back kicker
  box(3, 1, hdr(1, 1, 1, 1.8), [0, -0.9, 3.5], [0, 0.4, 0]);   // low front bounce card
  return scene;
}
