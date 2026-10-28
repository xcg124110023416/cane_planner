# Current-CoM corridor connector correction (2026-09-09)

Approved current slice runs directly in the dirty primary, preserving beam/suffix work. Prepend actual LFPC CoM to the projected forward authoritative A* reference without reducing its existing forward horizon. Sub-nanometre duplicate projection is replaced by actual CoM; all remaining edges still pass the same FIRI/static/current-pedestrian certificates. Require controller-tolerance root containment before publishing/committing successful geometry and before recovery route replacement. Blocked connectors retain existing static/dynamic reasons and only existing dynamic recovery; root postcondition failures use OUTPUT_CERTIFICATE with the root position. Add manager corner/offset, near-zero and blocked-connector regressions; build both packages and run registered tests. No navigation, process restart, dependency/configuration changes or commits. Earlier isolated-work restrictions below describe the historical beam slice, not this approved primary correction.

# Local-target suffix scoring correction

Current dirty-primary beam baseline verified and captured /tmp/beam-suffix-base. Charge original steering magnitude and consecutive-action variation on every simulated walking edge, even after local target arrival. Keep local-arrival terminal-distance convention and actual final-goal idle termination unchanged. Independent replay recomputes matching costs, first action has no previous-action penalty. No weights/actions/horizon/beam/safety/bridge change. Targeted suffix regression and existing arrival/full tests required before incremental delivery.

# Beam implementation execution

Baseline4fff814 verified; historical isolated edits backed up at /tmp/pre-lfpc-beam before checkout. No primary writes; user RViz excluded.

- Requirements/design persisted before code. Actual primary controller and LFPC inspected: launch horizon10, API heading increment, full-step checks and complete copyState identified.
- Implement deterministic root-diverse bounded tree, shared transition validator and final reroll; remove random/weighted runtime path.
- Add honest diagnostics and focused tree/collision/dynamics regressions; replace obsolete MPPI-specific assertions rather than claim unchanged sampler tests.
- Isolated catkin production/test builds and relevant manager/other-algorithm regressions; report timings/failures honestly. No navigation/screenshot reproduction claim.
- Deliver incremental4fff814 patch with exact baseline/result SHA and logs, no unrelated pedsim or RViz files; main independent review before integration.

No commit,push,install or live ROS restart/launch. Current step: implementation pending.

Verification: isolated production/test catkin builds passed without compiler warnings. All 130 registered tests passed; controller suite13 includes two new tree regressions and replaces four obsolete averaging assertions with discrete selection/cost/foot/union assertions. Other registered manager transaction,goal,heartbeat,recovery and algorithm tests passed. Local horizon5,width2/root fixture evaluated180 edges at0.127ms; not hard realtime or live-navigation result. No runtime launch/restart. Existing sample parameters outside controller may remain for legacy planner4 launch compatibility but are not alternative planner3 runtime.

Blocking arrival review corrected: true final goal/radius explicitly passed by manager; safe terminal leaves model idle instead of forced post-arrival walking. Local subgoals cannot terminate, full incoming certificates and final replay retained. Added controller narrow-corridor final/local/unsafe regression and actual manager last step->arrival->idle>.5s regression. analyze_mpc_eval filters unavailable ratio -1; related corridor analysis and event monitor report NaN/NA, summary receives no fabricated mean. Full132 C++ tests pass after correction. No primary/runtime changes.

Suffix correction verified: all 133 registered C++ tests pass (132 previous +1). New local-target-before-horizon test checks every node motion increment, penalized suffix turns, zero-turn selected ten-step safe continuation and replay cost equality; true final terminal still one step. Initial test map was too small for straight ten-step motion, corrected open fixture bounds rather than safety checks. Builds no compiler warnings. Logs /tmp/beam-suffix-build.log,beam-suffix-test-build.log,beam-suffix-tests.log and test-logs. No navigation or bag replay claim.
