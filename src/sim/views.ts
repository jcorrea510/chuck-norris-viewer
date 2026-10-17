// The simulation's camera list (kept free of three.js so settings and the UI can import it without loading the sim).
export type SimView = 'driver' | 'follow' | 'overhead' | 'robot' | 'turret' | 'broadcast' | 'chase';
export const SIM_VIEWS: { id: SimView; label: string; title: string }[] = [
  { id: 'driver', label: 'Driver', title: 'From our driver station' },
  { id: 'follow', label: 'Follow', title: 'Behind the robot, facing the opponent' },
  { id: 'overhead', label: 'Overhead', title: 'The whole field from above' },
  { id: 'robot', label: 'Robot', title: 'Mounted on the robot above the intake, looking forward' },
  { id: 'turret', label: 'Turret', title: 'Over the left turret, looking along its aim' },
  { id: 'broadcast', label: 'Broadcast', title: 'TV camera on the side line, tracking the robot' },
  { id: 'chase', label: 'Chase Low', title: 'Low behind the robot, bumper height, wide angle' },
];
export const isSimView = (v: unknown): v is SimView => SIM_VIEWS.some((x) => x.id === v);
