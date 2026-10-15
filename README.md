# Chuck Norris: 3D Robot Showcase

An interactive 3D model of **Chuck Norris**, FRC Team 3950 RoboGym Robotics' robot, built for showing judges on an iPad.

- **Turn it:** drag with one finger.
- **Zoom:** pinch.
- **Move it:** drag with two fingers.
- **Learn about a part:** tap it, tap its floating name tag, or tap its name in the bar at the bottom.
- **Overview** resets the view. **Tour** steps through every subsystem in order. **Explode** pulls the subsystems apart.
- If nobody touches it for 90 seconds, it resets itself and starts spinning, ready for the next judge.

After the first time it's opened, it works with **no Wi-Fi**.

---

## Opening it on the iPad

1. On the iPad, open **Safari** and go to the site address (see "Where the site lives" below).
2. Wait for the robot to appear. A small "Ready offline" note means everything is saved on the iPad.
3. Tap the **Share** button (the square with an arrow), then **Add to Home Screen**, then **Add**.
4. From now on, open it from the **Chuck Norris** icon on the Home Screen. It opens full screen, like an app, and works without Wi-Fi.

Before a competition, open it once on venue-independent Wi-Fi (home or school) so the newest version is saved on the iPad.

**Tip:** In Safari (not the Home Screen app) there is a full-screen button in the top-right corner.

## Where the site lives

`https://jcorrea510.github.io/chuck-norris-viewer/`

Every time a change is pushed to GitHub, the site rebuilds by itself in about two minutes.

---

## Changing the words (subsystem names, specs, talking points)

All the text lives in one file: **`src/subsystems.json`**.

1. On GitHub, open the repository, click `src` → `subsystems.json`, then the **pencil** icon (Edit).
2. Change the text between the quote marks. Keep the quote marks and commas exactly where they are.
3. Anything that starts with **`EDIT:`** is a placeholder. Replace it with the real info, and delete the `EDIT:` part.
4. Click **Commit changes**. The site updates in about two minutes.

What each part means:

| Field | What it is |
| --- | --- |
| `name` | Full name shown on the info card |
| `short` | Short name for the bottom bar and the floating tags |
| `summary` | One line under the name |
| `specs` | The grid of label/value pairs, e.g. `{ "label": "Motors", "value": "2× Kraken X60" }` |
| `talkingPoints` | 2–4 bullet points for judges |
| `icon` | Icon name: `move`, `arrow-down-to-line`, `layers`, `crosshair`, `zap`, `box`, `cog`, `cpu`, `gauge`, `target`, `wrench`, `camera` or `battery` |
| `match` | Words from Onshape part/assembly names that belong to this subsystem (see below) |
| `camera` | *(optional)* Fixed viewing angle, e.g. `"camera": { "azimuth": 45, "elevation": 30, "zoom": 1.2 }` |

At the top, the `robot` section sets the robot name, team line, which side is the front (`"auto"` points the intake at the viewer), the idle reset time (90 seconds), and the spin speed.

If the site shows a blank screen after an edit, there is probably a missing comma or quote mark. GitHub's editor underlines mistakes in red.

---

## Updating the 3D model from Onshape

You need a computer with Node.js installed (nodejs.org, the "LTS" version) and this repository downloaded.

1. In Onshape, right-click the top-level assembly tab → **Export** → format **GLTF / GLB** (binary). Draco compression is fine.
2. Save the file as **`cad/chuck-norris.glb`** in this folder, replacing the old one. The `cad/` folder is never uploaded to GitHub because the raw file is too big.
3. Open Terminal in this folder and run:
   ```
   npm install
   npm run model
   ```
   This prints the assembly tree and a **coverage report** showing how many parts went to each subsystem, then writes the small, optimized model to `public/models/chuck-norris.glb`.
4. Check it locally with `npm run dev` and open the address it prints.
5. Commit and push. Only the optimized file gets uploaded.

**If a part shows up under the wrong subsystem:** add a word from its Onshape name to that subsystem's `match` list in `src/subsystems.json`, then run `npm run model` again. Anything that matches nothing goes to **Frame & Structure**. To see the tree and coverage without building, run `npm run model -- --inspect`.

The build keeps the CAD colors, merges thousands of parts into a few dozen pieces so the iPad stays smooth, and shrinks the file from about 123 MB to about 12 MB. It never changes the file in `cad/`.

---

## Troubleshooting

- **The robot didn't update after I changed something.** Close the app fully (swipe it away in the app switcher) and open it again while on Wi-Fi. It checks for the new version on launch.
- **"Restoring…" appears.** iPadOS reclaimed graphics memory while the app was in the background. It recovers on its own in a second.
- **The screen goes to sleep.** The site asks the iPad to stay awake after the first touch. If it still dims, go to Settings → Display & Brightness → Auto-Lock and set it to **Never** for the event.
- **It spins too fast or too slow.** Change `secondsPerRevolution` in `src/subsystems.json`.
- **The robot faces the wrong way at the start.** Change `"front"` to `"+x"`, `"-x"`, `"+z"` or `"-z"` in `src/subsystems.json`.
- **The site didn't rebuild.** On GitHub, open the **Actions** tab. A red X means the build failed; click it to see why (usually a typo in `subsystems.json`).
- **Debug numbers:** add `?debug=1` to the end of the address to see frame rate, draw calls and triangle counts.

---

## For programmers

- Vite + TypeScript + three.js, with camera-controls for orbit and flights, three-mesh-bvh for tap picking, and vite-plugin-pwa for offline use.
- `scripts/optimize-model.mjs`: gltf-transform pipeline. It tags each mesh with its subsystem, bakes transforms, normalizes units/up-axis/ground, merges per subsystem per finish, welds, simplifies only if over budget, and compresses with meshopt.
- `npm run build`: type-check plus a production build into `dist/`.
- `npm test`: Playwright check at 1180×820, 820×1180 and 1366×1024 with touch emulation. Screenshots go to `shots/`. Run `npm run build` first.
- If `public/models/chuck-norris.glb` is missing, the site falls back to a procedural stand-in robot with the same subsystem structure.
- Keyboard: ← → move between subsystems, Esc goes back, R resets.
