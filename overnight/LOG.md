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
