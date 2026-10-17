# Overnight log

Newest at the bottom. Times are Eastern.

## 22:38 Start
- Read OVERNIGHT.md. `~/.claude/settings.json` has no `autoContinueAtUsageLimit` (unset = on). No project `.claude/` dir. Nothing to change.
- Branch `overnight` created from main @ 90efd08.
- Installed `@dimforge/rapier3d-compat@0.14.0`, `vitest@3.2.4`; Playwright firefox + webkit downloaded.
- WebKit native launch fails on Fedora (wants Ubuntu ICU 74 sonames). Firefox works with WebGL2. Pulling `mcr.microsoft.com/playwright:v1.64.1-noble` with podman for WebKit.
- Fetched the 2026 manual (TU22) and the FUEL whitepaper into the scratchpad. Key facts: HUB 47×47 in, 41.7 in hex opening, front edge 72 in; BUMP 73×44.4×6.513 in, 15° HDPE ramps; TRENCH clearance 22.25 in, 50.34 in wide; TOWER 49.25×45×78.25 in, rungs at 27/45/63 in; DEPOT 1.125 in steel barriers; FUEL 0.203–0.227 kg, production ~216 g, spring rate ~20 (lbf/in) nominal, range 15–22.5; robot ≤135 lb with bumpers; HUB scoring assessed up to 3 s after 0:00.
- Wrote ARCHITECTURE.md (as found), DECISIONS.md.
- Next: measure robot heights per intake pose from the CAD, baseline screenshots, BACKLOG.md, then start the Rapier core.

## 23:15 Checkpoint before usage limit
Done (on branch `overnight`, NOT merged to main):
- Rapier physics core in `src/sim/phys/` (config, engine, fuel, hub, robot, aim, rng). Field collision from the real field model: `scripts/build-collision.mjs` → `public/field/field-collision.{bin,json}`.
- Measured: drop 1 m → 24.3 cm rebound (old 13.2); roll 2 m/s → 1.80 m (old 1.94); 0→90% speed 0.75 s (old 0.50); 8/8 preloads score from 2.5 m.
- Robot height from CAD: 0.549 m intake down, 0.728 m up (Trench 0.565 m).
- New world.ts (Free Drive + High Score), brain.ts (async planning), route.ts, planner.ts + planner.worker.ts, index.ts renderer (3 cameras, interpolation, alliance flip, net mesh, seam + squash, blob shadows), audio.ts, debug.ts, drivepad (stick/turn/Intake toggle/Shoot hold), input.ts (field-relative only), ui.ts minimal HUD, main.ts sim settings sheet, viewer.setBumperColor, settings (alliance, bumper, simView, simSound).
- `npx tsc --noEmit` clean; `vite build` OK.
- CAD bumper numbers look clean from every side (overnight/media/before/). The glitch is probably in the sim LOD robot: check in the Driver view.
Fixed along the way: WASM panic from switching balls to kinematic (now weightless, no-collision bodies); flywheel friction; launched balls clip the robot (now ignore robot colliders for 0.25 s).

NEXT on resume:
1. CSS for the new HUD (.sim-exit, .sim-score, .sim-chip, .sim-tr, .sim-ai, .sim-count, .sim-result-grid) and the drivepad (.dp-intake); remove old .sim-status/.sim-controls rules.
2. Check `node scripts/simlook.mjs` screenshots (was running at the limit); fix the cameras and the bumper LOD.
3. Perf: step ~0.7–1.2 ms × 4 steps per frame. Over budget when many balls are awake. Try fewer solver iterations or 120 Hz with substeps.
4. Update scripts/verify.mjs for the new sim UI (old selectors .sim-view/.sim-mode are gone); delete or update scripts/soak.ts, simtrace, simtest, drivetest (they import removed modules).
5. Vitest suite for §3.7, plus the Trench/penetration/net/determinism tests. Then merge to main once green.
6. Remove tests/phys/debug.test.ts and world-quick.test.ts or turn them into real tests.
- 23:15 simlook finished: the new sim loads and runs in Chromium with no console errors; screenshots in shots/sim-look/ (driver, follow, overhead). Review them on resume.

## 07:24 Resumed (Jack: "keep going until it's actually done well")
- Camera/HUD review from screenshots: driver cam was inside the tower → moved to driver station 2; HUD styled; portrait trophy icon fix.
- "3950" glitch root cause: the sim's low-poly electrical mesh poked through the back bumper (raycast found `electricalsteel` 6 mm in front of the fabric), plus decimated lettering. Sim now keeps full-detail bumpers + electrical. Verified close-up.
- Physics test suite written (tests/phys/realism.test.ts, 20 tests). Bugs it found and fixed:
  - AI "spinning": takeOver() didn't record input time → AI immediately resumed. Fixed.
  - Thin TRENCH arm as triangle mesh: bogus contacts / wheelie-wedge. Replaced by solid boxes matched to the mesh (x −3.72…−3.56, underside 0.565). Realistic hull friction/restitution so a wheelie falls back.
  - TOWER uprights/rungs were thin shells: the robot drove straight through (Jack's item 1). Now solid from the manual + mesh position.
  - Net: both HUBs shared the "side" state → reset every step → 20 m/s balls passed. Per-hub state + hysteresis: caught within 1 cm, 0.4 m/s rebound.
  - AI high-centered on the BUMP peak (belly 4.8 cm vs 0.16 m peak) → crosses with momentum.
  - Balls stuck between the crank bed and front wall, or dead-center between lane mouths → kicker-ramp floor + lane nudge.
  - Robot could ride a bumper over a ball → ball-only skirt under the robot; squeezed balls popped back onto the carpet.
- Performance: Rapier ≈ 5 µs/ball/step + my code; ~300 awake balls → 2–3 ms/step. Moved physics to a Web Worker, 120 Hz (benchmarked), early sleep. Main thread ≈ 1–1.5 ms/frame of sim work (debug overlay, Chromium/Firefox).
- WebKit: native launch fails on Fedora (ICU 74); the official Playwright container under podman works. Sim e2e passes in WebKit 27.2 (two camera checks had a too-tight absolute threshold → made relative).
- Soak (600 s random driving, previous build): no invariant violations; memory flat 24–59 MB.

## 10:22 Green on the final build → merging
- Physics (Vitest) 21/21; soak 600 s random driving: no violations, memory 18–65 MB (no trend).
- WebKit 27.2 (podman container): full suite 110/110, and the final-build sim suite 44/44.
- Chromium: CAD regression passes at all three iPad sizes (77 checks); final-build sim suite 44/44 (flicker: all three cameras identical frames).
- e2e fixes: wait for the robot's reveal animation and UI fades before flicker frames (they aren't flicker); relative camera-blend criterion; longer reload/clock waits for software rendering.
- Fast-forwarding main to overnight and pushing (deploys the site).

## 11:42 Done
- main deployed at 10:23 (run 38915279646) with the tested build; live site smoke-tested in Firefox (sim loads, worker physics, scores, no console errors). Docs/clips deployed at 11:41 (run 38925993698).
- Physics clips in overnight/media/*.jpg (scripts/media.mjs).
- Open items (BACKLOG "Later"): match-style Hub timing, climbing, a real-iPad test, replacing EDIT values with team measurements.

## Round 3 (Oct 16–17)
- Six branches merged into `overnight`: collision (towers mirrored, HUB posts, planner grid), team-code port (Rebuilt26 at 50 Hz, Talon-level motors), controls/scoreboard/Move removal, brain-pan wiring (three passes plus the lead's underside fill light), soft bodies (XPBD net, modal panels), and sim visuals (hood/servo/flywheel, the motor-thing fix, four new POVs, render perf).
- Lead fixes: the realism leak check needs persistence (one-step landing dips are contact transients); the long tests yield every 10 s (a vitest worker RPC timeout); the net stress test used the removed `toggleArm` field, now `retract`; and BUMP beaching is documented.
- Usage limits hit three times. An hourly cron job resumed the work after each reset.
