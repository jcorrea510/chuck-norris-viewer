# Architecture (as found at 22:40, Oct 15, before tonight's work)

Vite + TypeScript + three.js single-page app. `index.html` → `src/main.ts`.

## Modules

| File | Role |
| --- | --- |
| `src/main.ts` | App state machine: CAD viewer, selection, pit display, settings, entering/leaving the sim (lazy `import('./sim/index')`). Exposes `window.__viewer` for tests. |
| `src/viewer.ts` | three.js renderer, camera-controls, the robot model (parts, mechanisms, explode, ghosting), render loop (`tick` → hooks → `render`). `hooks` is the per-frame callback set the sim plugs into. `setSimRendering` swaps in the light robot LOD. |
| `src/ui.ts`, `src/style.css` | All DOM UI: dock, chips, panels, settings sheet, sim HUD (`.sim-hud`). |
| `src/settings.ts` | Per-device settings in localStorage (`chuck-norris-settings-v2`). |
| `src/sound.ts` | Synthesized Web Audio: UI chimes + sim layers (drive whine, flywheels, rollers). |
| `src/materials.ts`, `effects.ts`, `studio.ts` | Shaders/materials (part-state texture), grid, contact shadow, outline, studio lighting. |
| `src/parts.ts`, `knownparts.ts`, `subsystems.json`, `config.ts` | Part index, subsystem config and mechanism definitions. |
| `src/sim/index.ts` | `Simulation`: loads field GLB + `field-2026.json`, draws fuel (one InstancedMesh), trails, locator ring (the red floor circle), path viz; cameras (TV/Chase/Field/Driver + pit-display director); maps driver input to field frame; sound updates. |
| `src/sim/world.ts` | `SimWorld`: fixed-step (120 Hz) world without rendering: field grid, balls, robot, brains, driver takeover/hand-back, human player, stranded-fuel sweeper, routing across trench/bump lines. |
| `src/sim/field.ts` | `FieldGrid`: 5 cm 2.5D height/solid grid from the field model, SDF, A* planner. |
| `src/sim/balls.ts` | Hand-rolled FUEL physics: drag + lift, floor bounce, per-axis grid collision, spatial-hash ball–ball, HUB box with a "teleport in" scoring rule (any ball near the opening scores), net as a plane, diffuser exit. |
| `src/sim/drive.ts` | 2D swerve rigid body (x, z, θ) with motor curve, friction circle, SDF wall impulses; cosmetic tilt. |
| `src/sim/robot.ts` | Intake (scripted path), hopper (separate robot-frame mini-physics), indexer lanes, turrets (servo + flywheel), shot solver, autopilot path follower. |
| `src/sim/shot.ts` | Ballistic solver (drag + lift) → launch velocity. |
| `src/sim/brain.ts` | AutoBrain (scripted 20 s auto), EndlessBrain (collect/score forever with a watchdog). Synchronous A* (blocks the frame: Jack's item 13). |
| `src/sim/input.ts`, `drivepad.ts`, `controls-page.ts` | Keyboard/gamepad/touch input, on-screen sticks, rebinding page. Default frame was "camera" (Jack's item 11). |
| `src/sim/geometry.ts` | Robot dimensions measured from the CAD. |
| `scripts/verify.mjs` | Playwright (Chromium only) e2e at three iPad sizes. `scripts/soak.ts` Node soak. |

## Data flow (sim)

```
rAF → Viewer.tick → hooks → Simulation.frame(dt)
   controls.read() → driverInput() (camera-relative) → SimWorld.step(1/120) × n (accumulator, no interpolation)
   → present(): robot pose → viewer.stage, mechanisms; held fuel positions
   → writeInstances(), trails, brain viz, camera (camera-controls setLookAt), sound
```

## Problems this structure causes (maps to Jack's list)

- Physics is 2.5D on a 5 cm grid: field elements are "solid cells", the Tower/Trench have no height, so the robot can't be blocked by a trench by height (items 1, 2).
- Scoring is a box test around the opening, not a physical opening (item 3).
- Rendering uses the last physics state, no interpolation → choppy at 60 Hz vs 120 Hz (item 10).
- Cameras: four views plus a pit director; Chase follows robot heading (item 10); stick "forward" follows camera (item 11).
- A* runs synchronously on the main thread (item 13).

## After tonight

See the "New architecture" section appended below as it lands.
