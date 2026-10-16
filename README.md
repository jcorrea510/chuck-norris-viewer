# Chuck Norris: 3D Robot Showcase

An interactive 3D model of **Chuck Norris**, FRC Team 3950 RoboGym Robotics' 2026 REBUILT robot, built for showing judges on an iPad.

**Live site:** https://jcorrea510.github.io/chuck-norris-viewer/

## What it does

| Do this | What happens |
| --- | --- |
| Drag one finger | Turn the robot. Drag low to see the **underside**. |
| Pinch / two-finger drag / scroll wheel | Zoom (toward your finger or cursor) / move |
| Tap a subsystem name (bottom bar) or a floating tag | Fly to it. Everything else becomes a faint hologram. **Isolate** hides the rest. |
| **Tap any part** | Select that exact part (a motor, a roller, a Limelight). Real specs appear for known FRC parts. |
| **Tap the same part again** | Select the assembly it belongs to. Keep tapping to climb: part → assembly → turret → subsystem. |
| **Hover with a mouse or trackpad** | Highlights exactly what a click would select, with a small label |
| **Double-tap a part** | Hide it (**Undo** and **Show All** appear). **Double-tap empty space** clears the selection. |
| **Press and hold a turret, the intake or the brain pan, then drag** | Move it directly: turrets turn on their turntables, the intake four-bar rises and drops, the brain pan swings open |
| **Search** (top-right) | Find any part by name: "kraken", "roller", "limelight", "PDH" |
| **Move** | Sliders for every mechanism. Opening the brain pan lifts the robot and swings the camera underneath. |
| **Explode** | Pulls the subsystems apart |
| **Simulate** | Drive the robot on the 2026 REBUILT field (**Free Drive**), or play **High Score**: score as much as you can in 60 seconds. Three cameras: **Driver** (from our driver station), **Follow** (behind and above, always facing the opponent's wall) and **Overhead**. |
| **Gear button** (top-right), or press and hold "Chuck Norris" | **Settings**: light/dark, accent color, hologram strength, **bumper color (red/blue)**, sounds, spin speed, labels, pit-display delay, quality. In the simulation the gear opens its own short sheet: alliance, bumper color, camera, robot sound, reset field, controls help. |

**Pit display:** in the simulation, the controls fade and the robot keeps driving itself, with a lower-third showing the score. Otherwise, after 15 seconds without a touch (adjustable), every control fades away and the robot turns slowly while each subsystem takes a turn in the spotlight with a caption. A QR code lets judges open the site on their phones. The first touch only wakes the screen.

Everything works **offline** after the first visit, including the simulation.

### The simulation

Everything moves by real physics (the Rapier engine, in a Web Worker so it never slows the picture down). Every number, where it came from, and the test that checks it is in `overnight/PHYSICS.md`; values marked **EDIT** are estimates the team should replace with measurements.

- **Field:** the official 2026 field model. Its walls, guardrails, TRENCHes, TOWERs, BUMPs, DEPOTs and OUTPOSTs are solid, built from the model itself. The TRENCH clearance is the manual's 22.25 in, so the robot fits under only with the intake down (0.549 m tall, from the CAD); stowed, it is 0.728 m and the TRENCH stops it.
- **FUEL (504 of them, the manual's pre-match layout):** 215 g foam balls with air drag, backspin lift, spin decay, rolling resistance on the carpet, and bounces that get deader the harder they hit. A ball scores only by falling through the HUB's opening; it rattles down the funnel, rides the HUB for about a second and comes out of one of the four exits into the NEUTRAL ZONE. The net behind the HUB catches balls and drops them.
- **Robot:** a 61 kg rigid body on four swerve wheels with suspension, so it pitches over the BUMPs and its weight shifts between wheels. Each wheel's Kraken X60 follows the motor curve and current limit, its grip is limited by the load on that wheel, and the battery sags under load.
- **Intake, hopper, indexer, shooter:** the roller grabs FUEL by friction and carries it up the kicker ramp into the hopper, where the balls are real bodies that jostle as the robot drives. The indexer feeds only while you hold Shoot. Each flywheel spins up with the motor curve and loses a little speed with every shot. The turrets aim from distance tables solved with the same physics, and lead the HUB when you shoot on the move.
- **Ferry:** from our ALLIANCE ZONE it shoots at our HUB; from anywhere else it lobs FUEL back into our zone (a small "Ferry" note shows).
- **Alliance:** Settings → Alliance. Our wall is always at the bottom of the screen, and the robot wears our color.
- **Free Drive:** touch any control and the robot is yours; let go for 15 seconds and it drives itself again (collecting and scoring forever, for the pit display).
- **High Score:** 3-2-1, then 60 seconds from the pre-match layout. At 0:00 the robot is disabled; FUEL still in the air counts if it goes in within 3 seconds (the manual's rule). Your best score is remembered on this device.
- **Controls:** always field-relative, like a real driver with a gyro: up drives toward the opponent's wall. **iPad:** the left thumb drives (the stick appears where you touch), the right thumb turns, **Intake** turns the rollers on/off, hold **Shoot**. **Keyboard:** W A S D drive, Q E turn, Space shoot, F intake on/off, R intake up/down, X eject, C X-lock, Shift precision, V camera, H give control back. **Game controller:** left stick drives, right stick turns, RT shoot, LT intake on/off, A intake up/down, B eject, X X-lock, LB precision, Y camera, Menu give control back. Settings → Controls Help lets you change any of them.
- **Sound:** off by default (Settings → Robot Sound). It's synthesized from the physics: motor whine from the drive motors' speed and current, the flywheels' hum at their real speed, soft foam thuds.
- **Debug:** add `?debug=physics` to the address for frame and physics times, awake bodies, every collider as a wireframe, and the velocity, drag and lift arrows of FUEL in flight.

---

## Opening it on the iPad

1. On the iPad, open **Safari** and go to the site address above.
2. Wait for the robot to appear. A "Ready offline" note means everything is saved on the iPad.
3. Tap **Share** (the square with an arrow), then **Add to Home Screen**, then **Add**.
4. Open it from the **Chuck Norris** icon. It runs full screen, like an app, and works without Wi-Fi.

**About the full-screen button:** Safari on iPad only allows true full screen for pages opened from the Home Screen. If the button can't go full screen, it shows these same steps.

Before a competition, open it once on good Wi-Fi so the newest version is saved on the iPad.

---

## Changing the words

All the text lives in **`src/subsystems.json`**.

1. On GitHub, open `src` → `subsystems.json`, then click the **pencil** icon.
2. Change the text between the quote marks. Keep every quote mark and comma.
3. Click **Commit changes**. The site updates in about two minutes. Open copies pick it up by themselves: they check every minute and reload once nobody has touched the screen for 10 seconds (or tap **Reload** on the "New version available" note).

| Field | What it is |
| --- | --- |
| `name` / `short` | Full name on the card / short name in the bottom bar |
| `summary` | One line under the name |
| `specs` | Label/value pairs, e.g. `{ "label": "Motors", "value": "2× Kraken X60" }` |
| `highlights` | 3–4 bullets of about five words each |
| `icon` | `move`, `arrow-down-to-line`, `layers`, `crosshair`, `zap`, `box`, `cog`, `cpu`, `gauge`, `target`, `wrench`, `camera` or `battery` |
| `match` | Words from Onshape part/assembly names that belong to this subsystem |
| `hideWhenFocused` | Parts hidden while that subsystem is selected (the turret wiring side panels) |
| `camera` | *(optional)* `{ "azimuth": 45, "elevation": 30, "zoom": 1.2 }` |

Specs for common parts (Kraken X60/X44, Limelight, PDH, MK4i, VH-109 radio and more) come from `src/knownparts.ts`.

### Mechanisms

The `mechanisms` section in `subsystems.json` defines what moves. These were worked out from the CAD:

- **Turrets:** rotate about the center of each AndyMark 200T turntable gear. Only the turntable gear and the shooter on it turn; the 1×1 turret frame tubes and gussets, wiring side panels, legs, base plate and Kraken X44 rotation motor stay fixed to the robot.
- **Intake & Hopper:** a **parallel four-bar**. The powered link turns on the hex hub driven by the 26T sprocket (12T on the VersaPlanetary output, #35 chain); the follower link pivots on a binding barrel 7.6 cm away. Both links are 25.4 cm, so the intake and the hopper's front wall stay upright while they rise and drop. The two hopper rollers ride the powered link, so their bearings stay in its holes. The **kicker ramp** and the three bars through it hang on the secondary roller shaft and swing forward to ride over the bumper as the intake lifts, then hang straight again (`swivel` in the config; the swing is solved once at load so it moves smoothly). Screws, nuts, rivets and belts follow whatever part they are fastened to. **Up** is −73°; change that number if the real stowed angle differs.
- **Brain Pan:** the polycarbonate cover swings down on its four hinges, exposing the PDH and MPM. The pan and cover count as Frame (`"ownsParts": false`), so selecting **Electrical** frames the camera on the electronics from underneath the robot (`frameOn`, `camera.elevation: -30`) while the brain pan cover stays highlighted too (`includeMechanisms`), so you can watch it open and close with the slider in the Electrical card.
- **Swerve modules:** the steering assembly of each MK4i turns about its center column (used by the simulation).

---

## Updating the 3D model from Onshape

You need a computer with Node.js (nodejs.org, "LTS") and this repository downloaded.

1. In Onshape, right-click the top-level assembly → **Export** → **GLB**. Make sure nothing important is hidden.
2. Put the file in the **`cad/`** folder. The newest file there is used. `cad/` is never uploaded.
3. In Terminal, in this folder:
   ```
   npm install
   npm run model
   ```
   This prints the assembly tree, a **coverage report** (parts per subsystem) and the **mechanisms** it found, then writes `public/models/chuck-norris.glb` (about 13 MB) and `chuck-norris.parts.json` (the part names used for tapping and search).
4. Check it with `npm run dev`, then commit and push.

**A part is in the wrong subsystem?** Add a word from its Onshape name to that subsystem's `match` list and run `npm run model` again. Unmatched parts go to **Frame & Structure**. `npm run model -- --inspect` shows the tree without building. The climber and the stray FUEL ball in the CAD are excluded with `robot.exclude`.

### The simulation field

The field comes from the AdvantageScope 2026 field model (see below). To rebuild it, put `model.glb`, `model_0.glb` and `config.json` from `Field3d_2026FRCFieldV2.zip` into `cad/field/` and run `npm run field`.

---

## Troubleshooting

- **The robot didn't update.** It needs Wi-Fi to see the new version and about two minutes after a push. Still old? Fully close the app (swipe it away) and open it again.
- **"Restoring…" appears.** iPadOS reclaimed graphics memory in the background. It recovers by itself.
- **The screen sleeps.** The page asks the iPad to stay awake. If it still dims, set Settings → Display & Brightness → Auto-Lock to **Never** for the event.
- **It feels slow or the iPad gets warm.** Settings (gear button, top right) → Performance → **Battery**.
- **No sounds.** Check the iPad's silent switch and volume, then Settings → Sound.
- **The site didn't rebuild.** On GitHub, open **Actions**. A red X usually means a typo in `subsystems.json`.
- **Frame-rate numbers:** Settings → Show Frame Rate, or add `?debug=1` to the address.

---

## For programmers

- Vite + TypeScript + three.js, with camera-controls, three-mesh-bvh (tap picking) and vite-plugin-pwa (offline).
- `scripts/optimize-model.mjs` uses gltf-transform. Each part gets a numeric id stored per vertex (`_PARTID`), so the robot draws in about 40 meshes but every part can still be picked, hidden, highlighted or searched. A 64×N state texture drives visibility, focus and the hologram in the shaders. Mechanism parts are merged into pivot nodes.
- `src/sim/` (loaded on demand): `phys/` is the physics (Rapier wrapper `engine.ts`, `fuel.ts`, `hub.ts`, `robot.ts`, the aim tables `aim.ts`, and every constant in `config.ts`); `world.ts` the game (Free Drive, High Score, who is driving); `brain.ts` the self-driving robot; `route.ts` + `planner.worker.ts` path planning; `sim.worker.ts` runs it all in a Web Worker and `backend.ts` / `local.ts` hand frames to `index.ts`, which draws them and points the cameras; `input.ts`, `drivepad.ts` and `controls-page.ts` driving it yourself; `audio.ts` the robot's sound; `debug.ts` the `?debug=physics` overlay.
- `npm run build`: type-check plus production build. `npm run test:phys`: the physics tests (Vitest, no browser). `npm run e2e`: Playwright at three iPad sizes and a laptop (`--browser=chromium|firefox|webkit`, screenshots go to `shots/<browser>/`). `npm test`: all of it. `npm run soak`: 10 simulated minutes of random driving with invariants checked every step. `npm run collision`: rebuilds the field collision mesh from the field model.
- `node scripts/simlook.mjs`: screenshots of the three cameras. `node scripts/measure.mjs <part name>`: part boxes in the robot frame, from the CAD.
- Keyboard: ← → subsystems · Esc back · R reset · / search · , settings · H hide selection.

## Credits

The 2026 field, HUB and FUEL models are from **AdvantageScope** by Littleton Robotics (FRC 6328). Copyright (c) 2021-2026 Littleton Robotics, BSD-3-Clause; the full notice is in `public/field/LICENSE-AdvantageScope.txt`. The field's collision mesh is built from the same model. Physics: Rapier by Dimforge (Apache-2.0). Icons: Lucide (ISC).
