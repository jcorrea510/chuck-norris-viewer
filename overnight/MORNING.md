# Morning report (in progress; last updated 23:15)

**Status: work in progress on branch `overnight`. `main` has NOT changed. The public site is still last night's version.**

## What's done so far
- The simulation's physics was rebuilt on a real rigid-body engine (Rapier). Field collisions come from the real 2026 field model. The HUB has a physical funnel, a scoring sensor, exits and a spring-damper net. The robot is a 61 kg rigid body on four suspended swerve wheels, with motor curves, a battery and traction limits.
- Robot height from the CAD: 0.549 m with the intake down (fits under the 0.565 m Trench), 0.728 m with it up (blocked).
- New UI pieces are written: three cameras, field-relative controls, a minimal HUD, High Score mode, and settings for alliance and bumper color. Not yet styled or visually checked.

## Physics, before → after (measured)
| Check | Before | After |
|---|---|---|
| 1 m drop, first rebound | 13.2 cm | 24.3 cm |
| 2 m/s roll-out on carpet | 1.94 m | 1.80 m |
| 0 → 90% top speed | 0.50 s | 0.75 s (front wheels slip under weight transfer) |
| Stop from top speed | 1.29 m | not measured yet |

## Known issues right now
- The new sim UI has no CSS yet, and the e2e tests still target the old UI. That's why nothing is merged to main.
- The physics step is near or over the 4 ms/frame budget when many balls are awake.

See LOG.md for the details and the next steps.
