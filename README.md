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
| **Simulate** | The robot on the 2026 REBUILT field. **Auto** replays the team's autonomous; **Endless** collects and scores forever. Cameras: **TV** (broadcast shot that keeps the robot and where it's going in frame), **Chase** (behind the robot), **Field** (the whole field) and **Driver** (red driver station). Drag or pinch any time; the camera takes over again 4 seconds later. |
| **Gear button** (top-right, also in the simulation bar), or press and hold "Chuck Norris" | **Settings**: light/dark, accent color, hologram strength, sounds, spin speed, labels, pit-display delay, quality |

**Pit display:** in the simulation, it turns into a broadcast: a slow orbit around the robot, the TV camera, a chase shot and a high crane shot, with a lower-third showing the score (Auto rolls into Endless so it never stops). Otherwise, after 15 seconds without a touch (adjustable), every control fades away and the robot turns slowly while each subsystem takes a turn in the spotlight with a caption. A QR code lets judges open the site on their phones. The first touch only wakes the screen.

Everything works **offline** after the first visit, including the simulation.

### The simulation

- **Field:** the official 2026 field model (HUBs, BUMPs, TRENCHes, towers, depots, 456 staged FUEL) with a 5 cm physics grid built from the CAD.
- **FUEL physics:** air drag and backspin lift in flight, bounces, rolling down bump slopes, collisions between every ball, walls, the HUB funnel and net, and the diffuser that pours scored fuel back into the neutral zone.
- **Drivetrain:** the swerve drive is a 61 kg rigid body. Each module steers at a real rate, its Kraken X60 follows the motor's torque curve (current-limited off the line, back-EMF near the 4.7 m/s top speed: about 0.6 s to full speed), and each tread holds the carpet only up to its share of the friction circle, so a hard reversal slides before it grips. Bumpers hit walls and field elements with real impulses (a clipped corner spins the robot), the BUMPs' slopes slow it going up, and the chassis rocks a little as weight shifts. The whole footprint, bumpers *and* the deployed intake, collides, so nothing passes through a wall.
- **Intake:** the four-bar deploys and stows with real motion. The full-width roller grabs any fuel it touches and carries it under the roller, up the polycarb kicker ramp and over the secondary roller into the hopper; fuel the bumpers or the intake frame touch is pushed with real momentum instead. A hopper too full to fit stowed stops the intake from raising.
- **Hopper and indexer:** inside the robot, fuel is its own physics world built on the CAD's inclined bed of rollers: it piles up, sloshes when the robot brakes or spins, and the powered floor carries it back to two indexer lanes, one per turret, which pull from their half of the hopper, stage fuel under each turntable and feed it up through the ring into the shooter.
- **Turrets:** both turrets track the HUB, spin their flywheels up to the speed the shot needs (and lose a little speed with every shot), and launch each ball from the hood. Shots on the move lead the target.
- **Auto:** starts in the left TRENCH lane, intake up, facing the middle. It goes under the trench, drops the intake, sweeps a lane of the neutral zone, comes back under the trench and shoots, then does it again (about 90 fuel in 20 s). It never touches a BUMP.
- **Endless:** always picks the densest reachable fuel, sweeps it up with the intake leading, scores from the alliance zone, and repeats. Every time it crosses into or out of the neutral zone it picks the trench or the bump at random (the status bar says which). The red human player rolls outpost fuel back in through the chute when it's needed, and fuel wedged where no robot could reach it is quietly carried back to the outpost (only off camera). A watchdog notices when it stops making progress: it re-plans, raises the intake and backs out, relocates, and as a last resort resets the robot. Soak-tested for 2 hours of play, plus hours of random driving with hand-backs, without getting stuck.
- **Drive it yourself:** in Auto or Endless, touch any control and the robot is yours (Auto ends early); 15 seconds after you let go, it drives itself again (or tap the chip in the status bar). Keyboard: **W A S D** drive, **Q E** turn, **Space** shoot, **F** intake (drops the intake and runs the rollers while held), **X** eject, **R** intake up/down, **C** X-lock, **Shift** precision, **V** next camera, **H** give control back. Game controllers (Xbox, PlayStation, Switch Pro…): left stick drives, right stick turns, **RT** shoot, **LT** intake, **A** intake up/down, **B** eject, **X** X-lock, **LB** precision, **Y** camera, **Menu** give control back. iPad and phones get on-screen sticks and buttons. **Controls** in the simulation bar shows every binding; tap one and press a new key or button to change it, and choose what "forward" means (the camera, the field like a real FRC driver, or the robot).
- **Sounds:** everything is synthesized live: the interface's chimes and whooshes, and the robot's drive motors, flywheels, rollers and belts following what the simulation is doing, panned to where the robot is on screen.
- **Smooth on laptops:** the simulation swaps in a lighter copy of the robot (`chuck-norris-sim.glb`, ~200k triangles instead of 1.1M) and lowers the shadow and pixel-ratio budget, so it stays smooth on an M1 MacBook Air.

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
- `src/sim/` (loaded on demand): `world.ts` is the whole simulation without rendering (field, fuel, robot, brains, who is driving); `drive.ts` the swerve rigid body; `robot.ts` the intake, hopper, indexer and turrets; `geometry.ts` the robot measured from the CAD (`node scripts/measure.mjs <part name>` prints part boxes in the robot frame); `brain.ts` Auto and Endless; `input.ts`, `drivepad.ts` and `controls-page.ts` driving it yourself; `index.ts` draws it and points the cameras.
- `npm run build`: type-check plus production build. `npm test`: Playwright at 1180×820, 820×1180 and 1366×1024 with touch emulation (screenshots go to `shots/`).
- `node scripts/soak.mjs endless 7200` (or `auto`, or `drive` for random driving with hand-backs, or `dyn` for drivetrain numbers): runs the simulation in Node, about 30× faster than real time, and reports scoring, stuck episodes, resets and physics errors.
- `node scripts/drivetest.mjs`: drives in the real page with a simulated controller and touch, checks the hand-back. `node scripts/simshots.mjs`: close-ups of the robot handling fuel.
- Keyboard: ← → subsystems · Esc back · R reset · / search · , settings · H hide selection.

## Credits

The 2026 field, HUB and FUEL models are from **AdvantageScope** by Littleton Robotics (FRC 6328). Copyright (c) 2021-2026 Littleton Robotics, BSD-3-Clause; the full notice is in `public/field/LICENSE-AdvantageScope.txt`. Icons: Lucide (ISC).
