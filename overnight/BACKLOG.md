# Backlog

Ordered by impact. `[x]` only after verification (test or recorded measurement). J# = Jack's item number (OVERNIGHT.md §2).

## A. Physics core (unblocks J1, J2, J3, J7, J8)
- [ ] A1 Rapier engine wrapper, fixed step + accumulator, render interpolation, seeded RNG, single physics config
- [ ] A2 Field collision mesh from the real field model, per surface (carpet plane, HDPE bumps, polycarb, steel, net handled separately)
- [ ] A3 FUEL bodies: mass/size/inertia, e(v) per surface, friction, rolling resistance, drag, Magnus, spin decay, sleep, CCD
- [ ] A4 Robot rigid body: CAD-derived compound colliders, 4 wheel contacts with suspension, swerve modules (steer rate, Kraken X60 curve, current limit, friction circle per wheel load), battery sag
- [ ] A5 Moving colliders: intake four-bar (stowed/deployed), turrets; robot height 0.549 m down / 0.728 m up (CAD) vs Trench 0.565 m
- [ ] A6 Intake rollers grab via friction at surface speed; hopper balls are real bodies; capacity from volume
- [ ] A7 Indexer feeds only while shoot held, at belt speed, physical queue
- [ ] A8 Flywheels: motor curve, inertia, closed loop, per-shot energy loss, exit speed/spin from contact mechanics
- [ ] A9 Turrets: limits, rate/accel, settle; aim tables (distance→RPM/hood) solved offline with drag+Magnus; lead on the move
- [ ] A10 Hub: real geometry collision, sensor plane scoring (exactly once), exits to the neutral zone, net = spring-damper surface
- [ ] A11 Ferry mode: Hub from own Alliance Zone, lob to own zone otherwise; "Ferry" indicator
- [ ] A12 Headless tests (Vitest): §3.7 checks, penetration/tunneling, trench per config, determinism, scoring once, ferry by zone, indexer gating

## B. Jack's list (UI/rendering/controls)
- [ ] J4 Settings: alliance (sim) + bumper color (CAD), persisted
- [ ] J5 Bumper "3950" numbers: every face, not mirrored, no z-fighting
- [ ] J6 Sound: physically driven or removed
- [ ] J9 Minimal UI redesign
- [ ] J10 Exactly three cameras (Driver default, Follow field-locked, Overhead), smooth-damp, smooth blend
- [ ] J11 Field-relative controls, red presented with red wall at bottom
- [ ] J12 High Score mode (3-2-1, 60 s, disable at 0:00, 3 s assessment, results + personal best)
- [ ] J13 Path planning in a Web Worker; robot keeps following old path while replanning
- [ ] J14 Top bar: FUEL scored + FUEL/s while shooting only
- [ ] J15 Hub side panel flicker: root cause + fix + flicker test
- [ ] J16 Remove the red floor circle

## C. Tests, perf, review
- [ ] C1 Playwright e2e in Chromium + WebKit (podman) or Firefox; touch drive/intake/shoot/ferry, High Score round, cameras, settings persist, CAD regression, no console errors
- [ ] C2 Soak test 10 min with invariants; memory stable
- [ ] C3 Input fuzzing
- [ ] C4 Flicker detection (frame diff)
- [ ] C5 Perf: physics ≤ 4 ms/frame, no frame > 33 ms, FUEL one draw call; `?debug=physics` overlay
- [ ] C6 Visual review at 1180×820, 820×1180, 1366×1024, laptop; clips in overnight/media/

## D. Docs
- [ ] PHYSICS.md with every config number, source, confidence, test
- [ ] CREDITS.md
- [ ] MORNING.md current
