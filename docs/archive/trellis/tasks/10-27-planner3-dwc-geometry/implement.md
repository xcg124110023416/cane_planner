# Execution and verification

- [x] Shared human-cane geometry and directional pedestrian hull; body/heading tests.
- [x] Footprint-protected FIRI, adaptive path coverage, bounded progress; snapshot v3.
- [x] Pose-aware SAT and LFPC yaw replay/final checks; rotation and cane-tip tests.
- [x] Manager ranges/preview/markers, launch/RViz and operator usage documentation.
- [x] Build path_searching and plan_manage. Run relevant registered tests, including
   unchanged static geometry, snapshot and transaction suites; isolated ROS smoke
   verifies actual topic output. Do not restart the user's live ROS processes.
- [x] Update runtime specs, report actual results and limitations. No automatic commit
   or push requested for this implementation turn.

Rollback: switches disable full footprint/adaptive sampling and preview for comparison;
the approved implementation remains the planner-3 production default. Do not relax
source freshness, obstacle exclusion or execution authorization to make tests pass.

## Initial implementation verification (2026-10-27)

- Production and test builds: both packages passed, no build warnings in final runs.
- 13 C++ targets / 159 enabled tests passed: pedestrian polygon 11, FIRI 16,
  MVIE whitebox 15, snapshot 6, timed SAT/footprint 7, MPPI 23, manager 30,
  A* edges/recovery 17, historical DWC 18, kinematic MPPI 1, timed trajectory 3,
  timed walking corridor 10, smoother 2. Opt-in benchmark targets are excluded.
- Same-budget N10/K256/I2, 50 measured repetitions: old model median 2.28569 ms;
  full footprint median 14.6402 ms, P95 14.9145 ms. Includes intermediate turn
  checks, excludes map/FIRI/ROS. Cached local polygon avoids repeated hull building.
- Headless production smoke on owned master 11339: synthetic open map,
  dwcg_crossing source, start (0,0), goal (3,0), arrival (2.928469,0); all eight
  recorded decisions allow motion. Human/preview topic output verified. No full
  office-map navigation or RViz/Ogre visual inspection claim.
- Launch XML, duplicate args, RViz YAML/topic bindings and actual roslaunch
  parameter override dump passed. README/usage links and git diff --check passed.
- Full-scope manual review covered runtime/visualization specs, shared geometry,
  source-age checks, root yaw, sample/time/yaw alignment, complete common-region
  containment and async preview isolation. Added immediate invalid/stale-preview
  cleanup and regression during final review. User master 11311 was untouched.

Implementation and follow-up fixes are complete. The user explicitly authorized a
local commit and push to origin/mpc-lfpc-prototype on 2026-10-27. Task archival and
journal recording remain separate from this authorized commit/push step.

## User-reported office-map regression follow-up

User log: A* succeeded (162 / 107 tracking points), followed by repeatable
NO_PROGRESS at s=3.449001710975486 / 2.7350318460423422. Replay of both exact
snapshots reproduces without ROS or pedestrians. A minimized one-edge/two-side-cell
map reproduces at s=3.0671364726820975. New regressions fail before the fix.

Ranked probes: (1) point-seed tangent planes; (2) heading discontinuity; (3) clipping
roundoff; (4) real footprint blockage. The one-edge repro excludes heading switches;
trace shows geometric shrinking of progress from .19 m to 7.38e-7 m, excluding a
single numerical-boundary comparison. Protecting the forward original path prefix
with unchanged dimensions/clearance lets both complete recorded routes advance.
Root cause is point-only inflation failing to protect forward progress.

Fix: protect both endpoint footprints on all original edges in a bounded forward
prefix (max_segment_length, capped at local_radius/2), reduce to convex-hull extreme
vertices for separator cost, then retain analytic coverage/overlap certificates.
No shortened planning horizon, weakened collision constraints, retry or larger
region budget. Original runtime snapshots are frozen as regression fixtures.

Follow-up verification passed:

- Both production packages and test targets built without warnings.
- Both ORIGINAL user snapshots replay feasible with six regions each (~3 ms).
- 7 affected targets / 110 tests passed: FIRI 17, snapshot 7, MVIE whitebox 15,
  pedestrian 11, timed SAT 7, MPPI 23, manager 30. New regressions were red before
  the fix, green after it. No repeated unrelated test sweep was needed.
- Original office2_modify.pcd, default planner-3 parameters, use_pedestrians=true,
  dwcg_crossing and remembered editor settings, owned master 11339:
  south start (-2.798765,5.823184), goal (-2.84,-4.73), final (-2.724073,-4.538132),
  27 motion decisions / 0 STOP; diagonal start (-3.331193,8.373373), goal (3.77,-4.99),
  final (3.737395,-5.081231), 45 motion decisions / 6 DYNAMIC_REFERENCE_BLOCKED STOP.
  BOTH reached the actual goal radius and emitted zero NO_PROGRESS. Recorded
  pedestrian counts include 0 and 1. No synthetic map or relaxed gait/geometry.
- The smoke recorder requires new corridor/motion decisions after each goal;
  an initial after-arrival heartbeat from an older run cannot count as arrival.
- No production debug instrumentation was added. Full RViz/Ogre view remains
  uninspected. Only owned test processes were cleaned up; the user then authorized
  committing and pushing the verified changes.
