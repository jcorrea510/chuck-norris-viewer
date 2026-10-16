# Morning report: Chuck Norris simulator

*Updated Fri Oct 16, ~09:00. Status at the bottom.*

## 1. What changed (your list)

| # | Your feedback | Status |
|---|---|---|
| 1 | Robot phased through the Tower | **Fixed, tested.** The field is now solid, built from the real field model. The Tower's uprights and rungs (they were thin shells you could drive through) and the Trench arms are solid steel. Tests drive into the Hub, Tower (centered and off-center), alliance wall, both guardrails, Trench support and Depot at full speed: worst overlap 1–21 mm for one step, never through. |
| 2 | Intake top phased through things | **Fixed, tested.** The intake's collider moves with the four-bar. From the CAD: 0.549 m tall with the intake down, 0.728 m up. Trench clearance is 0.565 m, so intake down drives under, intake up hits the arm and stops (a hard hit pops a brief wheelie, then it settles; you can back out). |
| 3 | Ferry mode | **Done, tested.** From our Alliance Zone it shoots at our Hub; anywhere else it lobs back to our zone (8/8 landed in our zone in the test), and a small "Ferry" note appears. A ball only scores by falling through the opening: shots from the neutral zone score 0. |
| 4 | Bumper color + alliance settings | **Done, tested.** CAD settings have Bumper Color; the sim's gear sheet has Alliance (sets our Hub, zones, ferry target, robot bumpers, and puts our wall at the bottom). Both persist. |
| 5 | Back bumper "3950" glitch | **Fixed.** It was the sim's low-poly robot: its electrical parts poked through the back bumper and the decimated lettering broke up. The sim now uses full-detail bumpers and electrical (close-ups in `shots/sim-look/`). |
| 6 | Cartoon sounds | **Replaced and off by default.** The chimes are gone from the sim. New sound is driven by the physics (motor whine from real motor speed/current, flywheel hum at real RPM including each shot's dip, soft foam thuds). I couldn't listen to it, so it ships **off**: Settings → Robot Sound. Your call. |
| 7 | Indexer behavior | **Done, tested.** Balls sit physically in the hopper. Nothing feeds until Shoot is held; release and it stops (test: 0 shots in 3 s without Shoot, 8 in 2.5 s with it, none after release). |
| 8 | Net does nothing | **Fixed, tested.** A soft, stretchy net: a 20 m/s ball is caught within a few cm and drops out at 0.4 m/s. It bulges visibly. No tunneling at any speed. |
| 9 | Crowded UI | **Redesigned.** One small score pill (FUEL count, plus FUEL/s while shooting), an × to exit, and three small buttons: High Score, Camera, Settings. Everything else lives in the settings sheet. |
| 10 | Too many / awkward cameras | **Fixed.** Exactly three: Driver (our driver station 2, eye height, gently pans and zooms), Follow (behind and above, always facing the opponent's wall, never rotates with the robot), Overhead (whole field, our alliance at the bottom). All are smooth-damped springs; switching blends. |
| 11 | "Forward" kept changing | **Fixed.** Always field-relative: up = toward the opponent's wall = up on screen in every camera. |
| 12 | High Score mode | **Done, tested.** 3-2-1, 60 s, robot disabled at 0:00, 3 s more for balls in the air (manual 6.5), results: score, shots, accuracy, best FUEL/s, personal best. Play Again / Exit. |
| 13 | Stutter while computer drives | **Fixed.** Path planning runs in its own Web Worker, and all the physics runs in another one; the robot keeps following its old path while a new one computes. |
| 14 | Top bar shows too much | **Fixed.** FUEL count, plus FUEL/s only while shooting. |
| 15 | Hub side panels flicker | **Fixed, tested.** Overlapping see-through panels were re-sorted every frame. They now draw in a fixed order (back faces first). The flicker test renders the paused scene repeatedly from all three cameras and requires identical frames. |
| 16 | Red floor circle | **Removed.** |

## 2. Physics, before → after

Before = last night's hand-made engine. After = Rapier, with every value sourced in [PHYSICS.md](PHYSICS.md).

| Check | Before | After |
|---|---|---|
| 1 m drop on carpet, rebound | 13.2 cm | 24.0 cm (and 31 % from 0.3 m, 14 % from 4 m: harder hits are deader) |
| 2 m/s roll-out on carpet | 1.94 m | 1.80 m in 2.0 s |
| Shot range with drag (12 m/s, 50°) | — | 11.7 m vs 14.9 m in vacuum; comes down steeper than it went up |
| Backspin lift | fixed constant | Nathan (2008) model: +16 cm apex on a steep shot, +7 % range on a flat one |
| Flywheel | instant | 1.1 s from rest to shooting speed; each shot costs ~3 %, back in ~25 ms |
| 0 → 90 % of top speed (4.7 m/s) | 0.50 s | 0.71 s (front wheels slip as weight shifts back) |
| Stop from top speed | 1.29 m | 1.70 m in 0.74 s |
| Full reversal (4.7 → −2 m/s) | — | 0.99 s |
| Over a Bump at speed | fake tilt | real pitch 10–17° (the chassis rides on four wheels) |
| Net catch at 20 m/s | — | stopped within 1 cm of the net plane, 0.4 m/s rebound |
| 30 shots from one spot (2.5 m) | — | 30/30 scored, 13 cm (1σ) spread at the rim |
| Robot left alone, 60 s | — | scores ~100–110 FUEL by itself |

Screenshots: `shots/chromium/` and `shots/sim-look/`.

## 3. Try this first (on the iPad)

1. **Simulate → hold Shoot from our zone, then drive past the Hub and hold Shoot again.** Watch it switch to Ferry and lob over the Hub, with the arcs dropping steeply because of drag.
2. **Tap Intake off, then drive at a Trench.** It stops at the arm (it's too tall stowed). Tap Intake on and it drives under with 1.6 cm to spare.
3. **Drive over a Bump fast**, then try it slowly.
4. **Shoot over the Hub into the net** (from far back in our zone). It catches and drops the balls.
5. **High Score.** 60 seconds; try to beat your best.

## 4. Decisions I made

All 17 are in [DECISIONS.md](DECISIONS.md). The top five:
1. **Physics runs in a Web Worker at 120 Hz** (Rapier). The main thread spends ~1 ms per frame on the sim.
2. **Thin field parts are solid.** Tower uprights/rungs and Trench arms are solid steel from the manual's dimensions; the rest comes straight from the field model.
3. **Our alliance is always internally at one end;** playing blue rotates the whole scene and swaps the colors.
4. **The Intake button is on/off.** On deploys and runs the rollers; off stops and stows.
5. **Sound ships off by default** until you've listened to it.

## 5. Needs real numbers from the team (`EDIT:` in `src/sim/phys/config.ts`)

| What | Current guess | How to measure |
|---|---|---|
| Robot mass with battery + bumpers | 61 kg | scale |
| Center of mass height | 0.17 m | tilt test or CAD mass properties |
| Belly clearance | 4.8 cm (CAD brain pan) | ruler; it decides whether a slow crawl over a Bump beaches the robot |
| Drive gear ratio | 6.12:1 (MK4i L3) | check the modules |
| Drive current limit | 60 A | robot code |
| Tread friction | 1.1 | pull test on carpet |
| Flywheel inertia | 0.004 kg·m² | CAD mass properties of wheel + shaft |
| Flywheel slip (exit speed ÷ half the wheel speed) | 0.82 | film a shot at 240 fps |
| Hood range | 45–72°, adjustable | is it adjustable? |
| Turret speed / accel / limits | 9 rad/s, 45 rad/s², ±180° | robot code |
| Indexer belt speed | 2.4 m/s | robot code / gearing |
| Idle flywheel speed while holding fuel | 35 % | robot code |
| FUEL drop rebound on our carpet | 25 cm from 1 m | drop test |
| FUEL roll-out from 2 m/s | 1.8 m | roll test |
| Net stiffness and give | k 600 N/m, 8 % rebound | film a ball into a real net |
| Battery resistance | 0.018 Ω | battery beak |
| Spin decay time | 2.2 s | slow-motion video |

## 6. Known issues (honest)

- **Untested in Safari/WebKit:** see Status below for whether the container run worked.
- **Sound unreviewed by ear**, so it's off by default.
- **Robot shoving a ball into a wall:** the engine can squeeze it into the carpet for a moment; a guard pops it back up. Rare, hard to see.
- **A full-speed hit with the intake up** makes a big wheelie. I believe that's real, but have a driver confirm.
- **Shots are very accurate from close range** (30/30 at 2.5 m), because the opening is big. Misses come from distance, moving shots, turret not settled and RPM dips. If the real robot misses more, raise the scatter values.
- **Performance measured only on this laptop** (no iPad tonight): ~1 ms per frame of sim work on the main thread; the physics worker takes 2–6 ms per frame when the robot plows into the big pile, on its own core.

## 7. Status

- Branch `overnight`: all work, pushed.
- Physics tests (Vitest): **20/20 pass**.
- E2E (Playwright): see the latest LOG.md entry.
- `main`: see the latest LOG.md entry. Site: https://jcorrea510.github.io/chuck-norris-viewer/
