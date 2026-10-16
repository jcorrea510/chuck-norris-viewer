# Decisions made overnight (no one awake to ask)

Format: what, why, alternatives.

1. **Rapier 0.14 (`@dimforge/rapier3d-compat`) as the rigid-body engine.** The spec asks for it; 0.14.0 is a stable release well over two weeks old. Wrapped in `src/sim/phys/`, so gameplay code never touches Rapier directly.
2. **FUEL compliance = rigid contacts + speed-dependent restitution, not a per-contact spring.** Rapier's contact softness is global (it's for penetration recovery, not material compliance). The whitepaper puts FUEL at ≈20 lbf/in (≈3.5 kN/m) → a 0.215 kg ball has a ~25 ms contact, which a 240 Hz rigid solver can't resolve with a spring anyway. So: e(v) curve per surface (fit to "rebounds well under half its drop height", livelier on hard panels, dead on the net) + a visual squash lasting the real contact time.
3. **Field collision comes from the real field model** (AdvantageScope 2026 field, BSD-3, already in the repo), extracted offline into a collision mesh per surface type, not hand-made boxes. The carpet is a flat ground plane.
4. **Robot-frame hopper physics is replaced by real bodies inside the robot's colliders**, so the hopper jostles from the real chassis motion (no pseudo-forces).
5. **High Score mode: the Hub is always active** (spec section 3.6). Balls in flight at 0:00 count if they score within 3 s — the manual's own rule (6.5: "assessment of FUEL scored in the HUB continues for up to 3 seconds after the ARENA timer displays 0:00").
