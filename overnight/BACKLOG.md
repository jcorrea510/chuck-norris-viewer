# Backlog

Ordered by impact. `[x]` only after verification (test or recorded measurement). J# = Jack's item number (OVERNIGHT.md §2).

## A. Physics core (unblocks J1, J2, J3, J7, J8)
- [x] A1 Rapier engine wrapper, fixed step + accumulator, render interpolation, seeded RNG, single physics config
- [x] A2 Field collision mesh from the real field model, per surface (carpet plane, HDPE bumps, polycarb, steel, net handled separately)
- [x] A3 FUEL bodies: mass/size/inertia, e(v) per surface, friction, rolling resistance, drag, Magnus, spin decay, sleep, CCD
- [x] A4 Robot rigid body: CAD-derived compound colliders, 4 wheel contacts with suspension, swerve modules (steer rate, Kraken X60 curve, current limit, friction circle per wheel load), battery sag
- [x] A5 Moving colliders: intake four-bar (stowed/deployed), turrets; robot height 0.549 m down / 0.728 m up (CAD) vs Trench 0.565 m
- [x] A6 Intake rollers grab via friction at surface speed; hopper balls are real bodies; capacity from volume
- [x] A7 Indexer feeds only while shoot held, at belt speed, physical queue
- [x] A8 Flywheels: motor curve, inertia, closed loop, per-shot energy loss, exit speed/spin from contact mechanics
- [x] A9 Turrets: limits, rate/accel, settle; aim tables (distance→RPM/hood) solved offline with drag+Magnus; lead on the move
- [x] A10 Hub: real geometry collision, sensor plane scoring (exactly once), exits to the neutral zone, net = spring-damper surface
- [x] A11 Ferry mode: Hub from own Alliance Zone, lob to own zone otherwise; "Ferry" indicator
- [x] A12 Headless tests (Vitest): §3.7 checks, penetration/tunneling, trench per config, determinism, scoring once, ferry by zone, indexer gating

## B. Jack's list (UI/rendering/controls)
- [x] J4 Settings: alliance (sim) + bumper color (CAD), persisted
- [x] J5 Bumper "3950" numbers: every face, not mirrored, no z-fighting
- [x] J6 Sound: physically driven; off by default (not judged by ear: see MORNING)
- [x] J9 Minimal UI redesign
- [x] J10 Exactly three cameras (Driver default, Follow field-locked, Overhead), smooth-damp, smooth blend
- [x] J11 Field-relative controls, red presented with red wall at bottom
- [x] J12 High Score mode (3-2-1, 60 s, disable at 0:00, 3 s assessment, results + personal best)
- [x] J13 Path planning in a Web Worker; robot keeps following old path while replanning
- [x] J14 Top bar: FUEL scored + FUEL/s while shooting only
- [x] J15 Hub side panel flicker: root cause + fix + flicker test
- [x] J16 Remove the red floor circle

## C. Tests, perf, review
- [x] C1 Playwright e2e in Chromium + WebKit (podman) or Firefox; touch drive/intake/shoot/ferry, High Score round, cameras, settings persist, CAD regression, no console errors
- [x] C2 Soak test 10 min with invariants; memory stable
- [x] C3 Input fuzzing
- [x] C4 Flicker detection (frame diff)
- [x] C5 Perf: physics ≤ 4 ms/frame, no frame > 33 ms, FUEL one draw call; `?debug=physics` overlay
- [x] C6 Visual review at 1180×820, 820×1180, 1366×1024, laptop; clips in overnight/media/

## D. Docs
- [x] PHYSICS.md with every config number, source, confidence, test
- [x] CREDITS.md
- [x] MORNING.md current

## Found and fixed along the way
- [x] Tower uprights/rungs were thin mesh shells: robot drove through (solid colliders now)
- [x] Trench arm thin mesh: bogus contacts / wedge (solid boxes)
- [x] Net side state shared between hubs (fast balls passed)
- [x] AI high-centered on the Bump peak (momentum crossing)
- [x] Balls stuck in the hopper front pocket / between lane mouths
- [x] Robot could ride over a ball pinned to a wall (ball skirt, squeeze guard)
- [x] Low-poly electrical mesh poking through the back bumper (the "3950" glitch)
- [x] takeOver() didn't record input time (AI grabbed the robot back)
- [x] Physics too slow on the main thread → Web Worker, 120 Hz, early sleep

## Later (not blocking)
- [ ] Match-style Hub active/inactive timing (no match mode exists yet)
- [ ] Climbing
- [ ] Test on a real iPad (performance headroom, Safari touch)
- [ ] Replace every EDIT value with team measurements (MORNING.md §5)
