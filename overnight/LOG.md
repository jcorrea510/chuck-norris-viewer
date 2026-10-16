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
