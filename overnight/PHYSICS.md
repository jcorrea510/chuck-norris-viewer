# Physics reference

Every number in `src/sim/phys/config.ts`, where it came from, how sure I am, and the test that checks it (`tests/phys/realism.test.ts` unless noted). **EDIT** = an estimate the team should replace with a measurement; each is also marked `EDIT:` in the config.

Confidence: **High** = from the manual, a datasheet or the CAD. **Med** = a published model or typical value, calibrated. **Low** = an engineering estimate.

## Engine and timing

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Engine | Rapier 0.14 (`@dimforge/rapier3d-compat`), WASM | spec | — | all |
| Step | 120 Hz fixed, accumulator, render interpolation | benchmark: 240 Hz cost ~2× for no visible difference; every spring has ω·dt < 0.6 | High | determinism, all |
| Where it runs | a Web Worker (`src/sim/sim.worker.ts`); route planning in its own worker | measured: ~1–3 ms/step with 300 awake balls (Rapier ≈ 5 µs/ball/step) would not fit a 4 ms frame budget on the main thread | High | e2e "physics runs in its worker" |
| Solver | 4 iterations + 2 friction, contact natural frequency 60 Hz | Rapier defaults, stiffer contacts | Med | collision tests |
| CCD | every ball; robot also gets 6 cm soft-CCD prediction | one 120 Hz step at full speed | High | "no tunneling", "no overlap" |
| g, ρ | 9.81 m/s², 1.2 kg/m³ | standard | High | — |
| Randomness | seeded mulberry32 (`phys/rng.ts`) | — | — | "same seed, same match" |

## Field

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Field collision | the AdvantageScope 2026 field model's own triangles (walls, guardrails, TRENCHes, TOWERs, BUMPs, DEPOTs, OUTPOSTs), per surface, simplified to ~1 cm | `scripts/build-collision.mjs` | High | collision tests |
| TRENCH arms | solid steel boxes, underside 0.565 m (22.25 in), x = ±3.64 (−3.72…−3.56), lane z 2.78–4.03 | manual 5.6 + measured from the mesh | High | TRENCH test |
| TOWER uprights / rungs | solid: 72.1 in tall, 1.5 × 3.5 in, 32.25 in apart; rungs 1.66 in OD at 27/45/63 in, reaching 5.875 in past each upright; at x = ±7.205 | manual 5.8 + measured from the mesh | High | TOWER tests |
| HUB | 47 in square body to 1.43 m, hexagonal funnel 41.7 in at 72 in (1.829 m), throat r 0.26 m | manual 5.4 + field model | High | scoring tests |
| HUB sensor | ball center enters the throat after passing the opening; counts once; ~1 s inside; leaves by one of 4 exits toward the NEUTRAL ZONE at 1.6–3.0 m/s | manual 5.4, 6.5.1, figure 5-8 | Med (exit speeds estimated) | "counts exactly once", "shots from the neutral zone never score" |
| Net | spring k = 600 N/m, damping 21 N·s/m, hysteresis (8 % stiffness on the way out), max stretch 0.3 m; plane 0.85 m behind the HUB center, y 1.70–3.02 m, z ±0.70 | field model for position; **EDIT** values to match video | Low | "20 m/s ball into the net": stopped within 1 cm of the plane, 0.4 m/s rebound |
| Carpet | a flat plane at y = 0 | — | High | — |

## FUEL

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Diameter | 0.150 m | manual 5.10 | High | — |
| Mass | 0.2155 kg ± 0.0035 (1σ), clamped 0.203–0.227 | whitepaper Table 4 (215–218 g averages); manual range | High | — |
| Inertia | 2/5 m r² (solid foam) | spec | High | — |
| Drag | C_d = 0.49, A = π(0.075)² → terminal 20 m/s | smooth sphere at Re ≈ 1e5 | Med | "terminal ~20 m/s, steeper descent" (range 11.7 m vs 14.9 m in vacuum at 12 m/s, 50°) |
| Magnus | C_L = 0.85 / (2.32 + 0.4/S), S = rω/v | Nathan (2008), *Am. J. Phys.* 76(2); 0.85 **EDIT** (smoother than a baseball) | Med | "backspin floats a shot" (55°: apex +16 cm; 30°: range 8.15 vs 7.62 m) |
| Spin decay | τ = 2.2 s in air; 0.6 s for non-rolling spin on carpet | **EDIT** estimate | Low | — |
| Restitution at 4.43 m/s | carpet 0.50, HDPE 0.56, polycarb 0.60, steel 0.58, FUEL–FUEL 0.45, bumper 0.30, robot 0.40, net 0.08 | "well under half on carpet, livelier on panels, dead in the net" (spec); **EDIT** drop tests | Med | "1 m drop rebounds 18–32 cm" (24.0 cm) |
| Speed dependence | 1 − e ∝ v^(1/5) | Kuwabara & Kono (1987) viscoelastic contact | Med | "harder hits lose more energy" (31 % of a 0.3 m drop, 14 % of a 4 m drop); "never gains energy" |
| Friction | carpet 0.85, HDPE 0.35, polycarb 0.30, steel 0.35, FUEL–FUEL 0.75, bumper 0.70, robot 0.45, net 0.6 | typical foam/plastic values; whitepaper "tackiness" ramp test confirms foam slides on panels | Med | — |
| Rolling resistance | 0.72 m/s² + 0.28 v (carpet), 0.18 m/s² (HDPE) | **EDIT** calibrated to a ~2 m roll-out from 2 m/s | Med | "2 m/s roll stops in 1.6–2.4 m" (1.80 m) |
| Compliance | rigid contacts + speed-dependent restitution; visual squash for one contact time | 20 lbf/in ≈ 3.5 kN/m spring rate (whitepaper fig. 7) → contact ≈ π√(m/k) = 25 ms | Med | — |
| Sleep | asleep after 0.4 s still on the floor; woken by any moving contact | performance | — | soak |
| Squeezed ball | a slow ball pushed below the carpet is put back on top | engine limit with a 300:1 mass ratio; real foam squishes | — | soak, "60 s of the robot playing itself" |

## Robot (Chuck Norris)

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Mass | 61 kg with battery and bumpers | **EDIT** weigh it (R408: ≤ 61.2 kg with bumpers, plus battery) | Med | — |
| Center of mass | (0, 0.17, −0.03) m | **EDIT** estimate from the CAD layout | Low | — |
| Inertia | pitch 4.6, yaw 6.2, roll 4.4 kg·m² | box of the frame, mass low and central | Low | — |
| Footprint | bumpers ±0.441 × ±0.429 m, 0.037–0.164 m high | CAD (`scripts/measure.mjs`) | High | collision tests |
| Height | 0.549 m intake down, 0.728 m stowed | CAD | High | TRENCH test |
| Belly clearance | 4.8 cm (brain pan) | CAD | Med | BUMP test; **ask the team**: it scrapes the 0.16 m BUMP peak, so it must cross with momentum |
| Modules | (±0.289, ±0.276) m, 4 in wheels | CAD | High | — |
| Suspension | 38 kN/m, 1500 N·s/m per wheel (tread + frame flex, ~4 mm sag) | estimate | Low | BUMP test (pitch 10–17°) |
| Tread μ | 1.1 static / 0.95 sliding on carpet, 0.75 on HDPE | **EDIT** vendor blue nitrile ≈ 1.1–1.2 | Med | 0→top speed |
| Drive motor | Kraken X60: 628 rad/s free, 7.09 N·m and 366 A stall, 2 A free | WCP datasheet | High | — |
| Drive gear | 6.12:1 (SDS MK4i L3), 90 % efficient | **EDIT** confirm | Med | 0→90 % of 4.7 m/s in 0.71 s; stop from top speed 1.70 m in 0.74 s; 180° reversal 0.99 s |
| Current limit | 60 A stator | **EDIT** from the team's code | Med | — |
| Steering | 18 rad/s max, P = 30 | typical MK4i | Med | — |
| Driver | 4.7 m/s, 7.5 rad/s full stick; deadband 0.08; slew 9 m/s², 30 rad/s² | typical robot code | Med | e2e touch drive |
| Battery | 12.6 V rest, 0.018 Ω (battery + breaker + wiring), brownout 6.75 V | **EDIT**; roboRIO 2 threshold | Med | debug overlay shows sag (~9–10 V under full acceleration) |

## Mechanisms

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Intake four-bar | 0° deployed, −73° stowed, 230°/s, 1700°/s² | CAD + existing mechanism config | Med | e2e intake toggle |
| Roller | 5 m/s surface; pull ≤ μ 0.9 × 50 N pinch; a ball moving away faster than 1.25 m/s is missed | FUEL spring rate × ~1.4 cm compression | Med | AI collection rate |
| Ramp feed | 2.4 m/s up the kicker ramp | estimate | Low | — |
| Hopper capacity | 48 deployed / 32 stowed (from its volume); stowing stalls if it wouldn't fit | CAD volume estimate | Med | — |
| Indexer | runs only while Shoot is held; belt 2.4 m/s (ball at 1.2 m/s, 3× in the turret); floor rollers 0.9 m/s | **EDIT** | Low | "the indexer only feeds while shoot is held" |
| Flywheel | 4 in wheel, J = 0.004 kg·m², 1:1 Kraken X60, 60 A | **EDIT** (heavier wheels lose less per shot, Chief Delphi) | Low | "spin-up from rest 1.13 s, dip ≈ 3 % per shot, recovery ≈ 25 ms" |
| Exit speed and spin | ½ × 0.82 × wheel surface speed; backspin 0.82 u / 2r | hooded single wheel; 0.82 slip **EDIT** | Med | aim tables (exit 6–12.5 m/s) |
| Energy per shot | ball KE ÷ 0.55 taken from the wheel | estimate | Low | dip test |
| Scatter | speed 1.2 %, yaw 0.6°, pitch 0.6° (1σ) | estimate | Low | "30 shots: 13 cm (1σ) spread at the rim" |
| Hood | 45–72°, 120°/s | **EDIT** (does the robot have an adjustable hood?) | Low | — |
| Turret | ±180°, 9 rad/s, 45 rad/s², ω_n 22, ζ 0.85; feeds only within 2.5° | **EDIT** | Low | — |
| Aim tables | distance → (ω, hood, time of flight), solved through the same drag + Magnus model; lead = robot velocity × time of flight | `phys/aim.ts` | High | "from our zone most shots score" (30/30) |
| Ferry | outside our ALLIANCE ZONE (x > −4.24 m) the turrets lob to (−6.3 m, robot z × 0.8) | manual zones; landing spot is a choice | Med | "ferried FUEL lands in our zone" (8/8) |

## Team code (Rebuilt26, branch insert-name-here)

The robot runs `src/sim/phys/teamcode.ts` (a port at 50 Hz) over the hardware in `robot.ts`, `mechanisms.ts`, `motors.ts`. Every constant lives in `config.ts` with its Java name; the main ones:

| Value | Number | Java source |
|---|---|---|
| Mass, yaw MOI, wheel COF | 74.088 kg, 6.883 kg·m², 1.2 | `Drive.java` ROBOT_MASS_KG, ROBOT_MOI, WHEEL_COF |
| Drive | MK4i L2 6.746:1, wheel 1.94 in, 4.58 m/s at 12 V, kP 0.1 kS 0.2225 kV 0.84381, stator 80 A, supply 30 A | `TunerConstants` |
| Steer | 21.43:1, kP 100 kD 0.5 kS 0.1, 40 A / 20 A, J 0.01 | `TunerConstants` |
| Modules | ±10.875 in × ±11.375 in | `TunerConstants` |
| Joystick | deadband 0.1, squared; 2.5 m/s while intaking/feeding | `DriveCommands`, `Constants.Drive.reducedSpeed` |
| Face HUB | PID 5/0.4, 8 rad/s, 20 rad/s², back to the HUB | `DriveCommands.joystickDriveAtAngle` |
| Turret | CANcoder 10 rot/rot, kP 8 kD 0.35 V/rot, 60/25 A, limits L [−380, 1] R [−365, 10], soft ±10°, start −180° | `Constants.Turret` |
| Flywheel | kP 1.2 kD 0.25 kV 0.0925, ramp 0.1 s, 60 A, r 2 in | `Constants.Turret`, `SimConstants` |
| Hood | 13…29.85°, servo 500…2500 µs, left inverted | `Hood.java` |
| Shot table, lookahead | 17 rows, 10 iterations, +0.05 s | `shot_table.json`, `GetAdjustedShot` |
| Targets | HUB (4.6256, 4.0346), ferry (hub.x−3, hub.y±2) in the neutral zone x 5.18…11.30 | `FieldConstants` |
| Intake | pivot 0↔12.75 rot, MM 60 rps / 120 rps², kP 15 kS 0.5 kV 0.12 kG 1; rollers 70 + 3v rps | `Constants.Intake`, `IntakeCommand` |
| Indexer, hotdog | 55 and 70 rps, kV 0.1285 / 0.11 | `Constants.Indexer` |

Estimates (EDIT): X44/X60 rotor inertia, turret inertia 0.05 and motor ratio 20:1, flywheel inertia 0.004, intake mass 4.5 kg and 62.9:1, hood servo travel 0.6 s, indexer belt and floor speeds, exit fit (slip 0.91, elevation 82° − hood). The older sections above predate this port where they list drive gear 6.12, 61 kg, 4.7 m/s, slew limits, the aim tables, the readiness gate, the old turret servo and the old ferry rule: all are replaced by the table above.

Tests: `tests/phys/teamcode.test.ts` (table, wrapping, buttons, ferry, intake, compliance, feed rate) and `realism.test.ts`.

## Soft bodies (`phys/net.ts`, `phys/panels.ts`, `tests/phys/softbody.test.ts`)

### Cargo net over the hopper

A diamond-mesh cord net as an XPBD cloth in the world frame (small steps: Macklin et al., "Detailed rigid body simulation with extended position based dynamics", 2019). World frame, so gravity and the robot's accelerations act on it with no fictitious forces; the pins follow the robot's pose. Runs before the engine step with the robot's pose extrapolated by its velocity, so a ball's path is swept through the cloth (nothing tunnels).

| Value | Number | Source | Conf. | Test |
|---|---|---|---|---|
| Lattice | 18 × 18 cells, 181 knots, 324 cords of 5.2 cm (the ~5 cm mesh in the photos); boundary knots = the webbing, pinned | photos, CAD | Med | — |
| Pins | back edge at the top of the rear panel (z −0.02, y 0.546), side edges straight to the front wall's top (z 0.646, y 0.549 deployed), half width 0.33 m; the front edge rides the four-bar (`fourbar.ts`) | CAD (hopper plates to 0.546 m, coupler to 0.549 m) | High | — |
| Mass | 0.5 kg over the free knots (3 g each) | spec; **EDIT** weigh the net | Low | — |
| Rest length | the deployed lattice + 0.33 % slack | designed for the sag below | — | sag |
| Sag at rest (deployed) | 4.7 cm at the middle (design 4.5 cm ± 1) | photos (a few cm); **EDIT** measure | Med | "rests with the designed sag" |
| Cords | tension only, k = 6000 N/m per cord; weak knot links two cells apart k = 120 N/m (a pure diamond lattice scissors freely); knot friction 70 s⁻¹ on the relative cord velocity, air 1.2 s⁻¹ | **EDIT** pull-test a panel; damping tuned so a 1 m drop rebounds 14 cm (carpet 24 cm) | Low | drop, pluck |
| Stowed | the front edge rises 0.18 m and comes back 0.24 m: the cloth hangs in a bag, kept above 0.31 m (hopper floor) and between the side walls | geometry | Med | stress |
| Solver | 6 substeps × 1 pass at 120 Hz (h = 1.4 ms); 4 substeps sag 1.4 cm more | cost vs stiffness | — | — |

FUEL contact: each ball near the net is a particle carried through the substeps (gravity on it, so a resting ball loads the net); sphere-vs-triangle (Ericson 5.1.5) with barycentric weights on the three knots; sequential impulses with restitution 0.04 above 0.5 m/s, a gentle push-out (≤ 0.6 m/s) and Coulomb friction μ 0.6 with the ball's spin. The knots get −j, the ball's Rapier body +j (and the torque), the robot gets the net's reaction at the ball (the pins carry the load). A sleeping ball the cloth only holds up stays asleep. Balls in the intake's capture path are skipped. Not modeled: the net against robot parts (it is clamped above 0.31 m and between the walls instead), the cloth's own inertial reaction on the robot.

### Polycarbonate panels

Six panels: hopper left/right walls, rear wall, the intake's front wall and its two side plates (the ones balls and walls actually reach). 1/8 in (3.175 mm) polycarbonate, E 2.3 GPa, ρ 1200, ν 0.37, 40 % open hex pattern (mass × 0.6, bending stiffness × 0.45: ω × 0.87; **EDIT**). The frame holds a panel at ~20 cm pitch (bolts and braces behind it; **EDIT**), so its modes are those of a 20 cm clamped bay: ω₁ = 35.99/L² √(D/ρh) = 1065 rad/s (169 Hz), second mode (odd along the panel) at 2.04 ω₁ (clamped square). Shapes φ₁ = sin²πu sin²πv, φ₂ = sin 2πu sin²πu sin²πv. Generalized mass M = ρ_eff h A ⟨φ²⟩ (68 g for a hopper wall) → modal stiffness 77 kN/m; ζ = 0.05 (spec 0.03–0.08).

`q̈ + 2ζω q̇ + ω² q = F φ(u₀,v₀)/M − Γ a_n`, Γ = ⟨φ⟩/⟨φ²⟩ = 1.78, advanced exactly per step (ω dt ≈ 9: an explicit scheme would blow up).

- **FUEL**: a ball striking a panel (relative normal speed > 1.2 m/s going in) is integrated through the contact: the foam's 3.5 kN/m spring with the damping that gives the surface's restitution when the panel is rigid, coupled to the modes. That gives the deflection under the ball (4.0 mm for a 10 m/s ball at the middle of a hopper wall), the ringing it leaves, and the energy the panel took, which comes off the ball's rebound (Rapier's rigid bounce is only ever reduced, never raised: 3.18 vs 3.23 m/s from the panel's edge). The contact lasts ~25 ms and the panel is stiff next to the foam, so the deflection follows the force (it is played back over the 3 steps) and little rings afterwards.
- **Robot**: the panels ride the frame. The frame's velocity change normal to a panel over a step is a half-sine acceleration pulse of 4 ms (bumper foam and frame; **EDIT**), or a constant acceleration when it is small. A wall hit with the intake out (the front wall leads the bumper) rings the front panel 2.2 mm at 2.8 m/s, more than the 10 m/s ball (1.0 mm there).
- Not modeled: the panels' effect on the ball other than lost energy; rotational acceleration of the frame; panels hitting each other.

Tests: sag 4.7 cm and still (0.01 mm/s drift); 1 m drop onto the net rebounds 14 cm and rests on it; 4 m/s throw from inside is caught 9 cm above the wall tops and returns; a plucked net only loses energy (3.4 J → 2.7 J, never up); 60 s of hard driving, shooting and intake toggles stays finite and bounded; panel modes 169 Hz / 346 Hz; a 10 m/s ball flexes a hopper wall 4.0 mm; ringing decays at the log decrement 0.316 (theory 0.315) and is 0.5 % after 0.1 s; resting drive does not drift the panels.

### Rendering

The net is one dynamic mesh (`src/sim/net-mesh.ts`: every cord two crossed 4.4 mm ribbons, flat 3 cm webbing along the edges; 2.6 k vertices written per frame), under the robot's stage, interpolated between the worker's last two steps (robot-frame knots are sent as Float32Arrays, transferred like the FUEL arrays). The panels bend in the vertex shader of the robot's hopper and intake meshes (`src/sim/softbody-render.ts`, `onBeforeCompile` on per-mesh clones of the part materials): the two mode shapes times the amplitudes the worker sends, selected by position in the robot frame, so merged meshes need no per-panel geometry. The CAD view shows the same cloth at rest (`src/netmodel.ts`), re-settled when the intake moves.
