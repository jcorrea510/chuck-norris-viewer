# Chuck Norris: 3D Robot Showcase

An interactive 3D model of **Chuck Norris**, FRC Team 3950 RoboGym Robotics' 2026 REBUILT robot, built for showing judges on an iPad.

**Live site:** https://jcorrea510.github.io/chuck-norris-viewer/

## What it does

| Do this | What happens |
| --- | --- |
| Drag one finger | Turn the robot. Drag low to see the **underside**. |
| Pinch / two-finger drag | Zoom / move |
| Tap a subsystem name (bottom bar) or a floating tag | Fly to it. Everything else becomes a faint hologram. |
| **Tap any part** | Select that exact part: a motor, a roller, a Limelight. Its real specs appear when it's a known FRC part. |
| **Tap the same part again** | Select the assembly it belongs to. Keep tapping to climb up: part → assembly → turret → subsystem. The path at the top of the card works too. |
| **Double-tap a part** | Hide it. A "hidden" pill appears with **Undo** and **Show All**. **Isolate** on the card hides everything else. |
| **Search** (top-right magnifying glass) | Find any part by name: "kraken", "roller", "limelight", "PDH". |
| **Move** | Rotate each turret on its turntable, raise or lower the intake and hopper, and open the brain pan. Opening the brain pan lifts the robot and swings the camera underneath. |
| **Explode** | Pulls the subsystems apart |
| **Simulate** | Chuck Norris on the 2026 REBUILT field: drives under the trench, collects FUEL, and shoots on the move into the HUB with both turrets tracking. |
| **Press and hold "Chuck Norris"** (top-left) | Hidden **Settings**: light or dark mode, accent color, hologram strength, sounds, spin speed, labels, idle reset and quality |

If nobody touches it for 90 seconds (adjustable in Settings), it resets itself, so it's ready for the next judge. Everything works **offline** after the first visit, including the simulation.

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
3. Click **Commit changes**. The site updates in about two minutes.

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

The `mechanisms` section in `subsystems.json` defines what moves. These values were worked out from the CAD:

- **Turrets:** rotate about the center of each AndyMark 200T turntable gear. The shooter, the turret tubes and the side panels turn; the legs, base plate and Kraken X44 rotation motor stay fixed.
- **Intake & Hopper:** pivot on the 1/2" hex shaft driven through the 26T sprocket (12T on the VersaPlanetary output, #35 chain). **Up** (−110°) is where the whole assembly fits back inside the frame perimeter. Adjust the `Up` number if the real stowed angle differs.
- **Brain Pan:** the polycarbonate cover swings down on its four hinges, exposing the PDH and MPM mounted under the brain pan.

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

- **The robot didn't update.** Fully close the app (swipe it away) and open it on Wi-Fi.
- **"Restoring…" appears.** iPadOS reclaimed graphics memory in the background. It recovers by itself.
- **The screen sleeps.** The page asks the iPad to stay awake. If it still dims, set Settings → Display & Brightness → Auto-Lock to **Never** for the event.
- **It feels slow or the iPad gets warm.** Settings (press and hold the name) → Performance → **Battery**.
- **No sounds.** Check the iPad's silent switch and volume, then Settings → Sound.
- **The site didn't rebuild.** On GitHub, open **Actions**. A red X usually means a typo in `subsystems.json`.
- **Frame-rate numbers:** Settings → Show Frame Rate, or add `?debug=1` to the address.

---

## For programmers

- Vite + TypeScript + three.js, with camera-controls, three-mesh-bvh (tap picking) and vite-plugin-pwa (offline).
- `scripts/optimize-model.mjs` uses gltf-transform. Each part gets a numeric id stored per vertex (`_PARTID`), so the robot draws in about 40 meshes but every part can still be picked, hidden, highlighted or searched. A 64×N state texture drives visibility, focus and the hologram in the shaders. Mechanism parts are merged into pivot nodes.
- `src/sim.ts` (loaded on demand): scripted swerve path, intake collection, ballistic shots solved for the HUB opening, and FUEL returning through the HUB exit.
- `npm run build`: type-check plus production build. `npm test`: Playwright at 1180×820, 820×1180 and 1366×1024 with touch emulation (screenshots go to `shots/`).
- Keyboard: ← → subsystems · Esc back · R reset · / search · , settings · H hide selection.

## Credits

The 2026 field, HUB and FUEL models are from **AdvantageScope** by Littleton Robotics (FRC 6328). Copyright (c) 2021-2026 Littleton Robotics, BSD-3-Clause; the full notice is in `public/field/LICENSE-AdvantageScope.txt`. Icons: Lucide (ISC).
