# Configurable pedsim single-scene slice

Current dirty primary baseline captured at delivery, live nodes untouched. YAML holds headon/overtake/crossing start/goal/initial0/desired-max speed. CLI --scenario defaults headon, invalid names fail in argparse before master. Exporter checks actual world projection endpoint bounds/free/connectivity and writes selected XML. No unknown XML speed attribute: optional upstream single_pedestrian_speed invokes existing setVmax after instantiation for exactly one ordinary pedestrian in guarded mode; default-1 preserves original upstream speed. Candidate built separately /tmp/pedsim-scenes-candidate/ws, permanent dependency not overwritten. Four scene/forwarding/record tests pass, all three actual PCD exports pass, private11573 actual speed0.3 cap/motion and guard-loss tests pass. No full navigation launch/safety guarantee. User controls encounter via RViz own start/goal, no planner changes. Current main integration and user-authorized dependency update/restart pending.

# Workflow ownership/record review corrections verified

Added persistent per-port flock supervisor with explicitly spawned/verified rosmaster PID+token; lock held before preparation until complete cleanup, no exec-lost lock. External port race rejects without navigation attachment or external kill. Added record subcommand using same setup/host/port with identity proof and safe label argv; no11311 fallback. SIGINT/TERM cleanup limited to owned subprocess groups.

Private11563 tests pass: competing owners, stale lock-file nonownership, preexisting master rejection, external master stealing port during preparation remains alive and launch is refused, normal child identity and lock/port release. Recorder unit test verifies11564 environment/label and no spawn for unowned master, no actual bag recorded. Existing6 lifecycle/pairing tests pass. No navigation UI/current11311 operations or repeat dependency deployment. USAGE includes single start/record/reset commands. Primary integration/review pending.

# Workflow execution slice

Candidate copied from current dirty primary in /tmp/pedsim-workflow-candidate. Dedicated /home/xcg/cane-pedsim-ws source hashes verified against current guard-v2 manifest, existing deps build only. Add launcher/composition/startup readiness/tests/recorder docs; private tests, no navigation UI or live11311 writes. Preserve underlying contracts below.

# After-arrival authoritative plant heartbeat follow-up

Baseline is current dirty integrated primary including pedsim guardV2, not an older
commit patch. Isolated manager copy/build in /tmp/cane-idle-heartbeat; live11311 and
all running upstream executables untouched. Confirmed LFPC belongs to manager;
simulation_generator odom is not the executed plant and must not replace sim_odom.
Minimal change: stopped serialization at actual arrival plus mutex-protected existing
WAIT_TARGET tick for reached/initialized lightweight planner3 with no pending goal.
No new timer, integration change, cached bridge timestamp rewrite, tolerance change
or guard unlock. Idle/newgoal/hardware/uninitialized scope in ROS interface contract.

Verification: isolated production build and22 actual manager tests pass (20 prior+2).
Private master11541 cleaned up only owned processes. Actual FSM reaches goal, emits
arrival heartbeat, waits1.2s across12 fresh heartbeats with unchanged pose/yaw/LFPC
control/gait and zero twist, then accepts/searches/activates new goal without idle
motion. Uninitialized/hardware/Gazebo idle do not publish. Prior moving,STOP,
source-expiry,diagnostic and goal-race tests pass. Bridge not modified; prior true
source-loss latch remains contract; no new full pedsim+manager lifecycle launch or
navigation safety claim. Logs build.log,build-tests.log,test-results.log and
ws/test-logs under /tmp/cane-idle-heartbeat. Independent review/main integration and
user-authorized restart required before live effect. No commit/install/main devel
write or user rviz modification.

Baseline history preserved below:

# Guard V2 isolated correction

Candidate /tmp/pedsim-guard-v2/upstream built into /tmp/pedsim-guard-v2/ws only; original active upstream binary untouched. Fixed now-before-lookup race and premature robot_seen latch: use post-lookup now and set robot_started only immediately before actual integration. True running stale/future/rollback loss remains latched, including re-pause. Initial paused readiness/loss waits. Added transition-only actual internal guard_status with timing evidence. Bridge starts only on first exact paired output; normal empty initial wait does not latch, no bridge scope expansion.

Candidate Noetic build, extracted production guard deterministic race/future/stale/rollback tests, private11539 moving paired pause-loss-recovery test and clock rollback pass. Prior message reorder unit suite retained. No main source/devel or live11311 changes, restart/unpause/kill/goal publication; no install/commit. Updated original-revision full patch and hashes; deployment requires user permission to restart into candidate. Independent review pending.

# Optional pedsim execution and review corrections

Baseline1d16286 copied read-only. P1 replaced latest/latest position comparison with explicit upstream step input evidence and bounded matching; P2 restored three distinct documents as additions to baseline. Patched upstream Noetic build passed. Five offline tests passed, including actual bridge callbacks at10Hz0.12m steps with agent/evidence reordering and true stale latch. Private ROS master11531 end-to-end simulator+bridge moving-input test produced24 paired observations, then stale loss prevented integration/output and fresh TF could not resume. Simulated clock rollback guard test passed. Only owned processes cleaned. Map overlay inspected:228217 points,100299 selected,6149 cells,348 segments. Logs paired-build.log,paired-unit-tests.log,paired-motion-tests.log,guard-clock-tests.log in temporary verification directories. Existing planning tests are preserved baseline history, not rerun claim. No primary/devel writes,install/navigation/commit. Independent review and user RViz remain pending.

Baseline execution history preserved:

# Soft pruning verified delivery

Increment against fresh actual dirty-primary125 profiling baseline /tmp/astar-soft-pruning-base/manifest.json, not e2b7945. Only post-hard-check finite-neighbor length bound, query saved counter and tests/spec notes. No33-field diagnostic changes, cache, clearance/sampling/goal/reopen policy changes.

126 registered tests pass (125 baseline + one deterministic saved-query/float ties regression); production/test builds no warnings. Seven alternating baseline/optimized runs use identical generated compiler/link flags, same updated test executable and a baseline astar.cpp rebuilt against compatible current profiling header. Baseline fails only saved-query>0 assertion (expected ordinary exit1); optimized passes. All routes, exact accumulated costs, expansions and hard-call counts match for every run and weight. No tie differences observed.

80x40 synthetic grid via actual selected static classifier, immutable hull: weight1 baseline median70.271ms range70.055–71.053; optimized16.207ms range16.120–16.467 (~76.9% reduction,4.34x). Soft5012->973, skipped4039, hard5167 unchanged, expanded639 unchanged. Weight0 baseline2.842ms range2.767–3.035; optimized2.713ms range2.677–2.736; soft5044->971, hard5199,expanded643 unchanged. Frozen historical failure-9ziyMV classifier-backed fixture (no dynamic overlay) route bytes unchanged: baseline median2.176ms range1.640–2.398; optimized2.034ms range1.962–2.343; overlapping ranges, no robust gain claim there. These are local fixtures, not bag replay/navigation/real-time guarantees; deadline/freshness failures remain unchanged in semantics and baseline successes were measured without timeout. No source bag reconstruction claimed.

Commands: catkin build --workspace /tmp/cane-dwcg-a24e path_searching plan_manage --no-deps --no-status -j2 -p1; repeat --make-args tests; python3 /tmp/cane-dwcg-a24e/run_tests.py; python3 /tmp/cane-dwcg-a24e/compare_soft_pruning.py. Logs soft-pruning-build.log,soft-pruning-test-build.log,soft-pruning-tests.log,soft-pruning-comparison.log and soft-pruning-comparison/ include exact compile flags, runs, result routes/costs and timings. Private masters11519/11523 cleaned up. No main writes,commit,install,simulation; independent review pending.

# Minimal A* soft-query pruning follow-up

Baseline is actual dirty-primary125-test profiling implementation, preserved /tmp/astar-soft-pruning-base/manifest.json. Only prune nonimproving known finite neighbors AFTER existing hard certificate and deadline checks in corridor mode. Use exactly (pro_pos-cur_pos).norm() as corridorEdgeCost does, not d_pos.norm(): rounded coordinates can differ. Validated nonnegative weight and t*t imply computed cost >= computed length; IEEE monotone addition with the same current_g means computed lower sum cannot exceed actual candidate sum. Finite lower >= finite existing g cannot satisfy strict candidate<existing update, including equality/reopen cases. Never prune unseen/nonfinite or improving neighbors; connectors/legacy untouched. Profile saved calls without altering33 diagnostic indices. Repeated identical-flag baseline/optimized classifier fixtures, exact route/cost/hard-count equality, full125+targeted regressions required. No cache, map/sampling/safety changes.

# Completed recovery profiling and diagnostics follow-up

Baseline: actual dirty primary containing reviewed124-test recovery, saved /tmp/recovery-performance-baseline/manifest.json. Deliver ONLY incremental recovery-performance patch against these bytes; no full e2b7945 reapplication.

No performance optimization/cache added. One local pure-compute inspection found no demonstrated safe improvement without changing sampling/formula; do not claim speedup. Adapter lacks consistent immutable map version/read lock, so repeated-grid caching is deferred rather than expanding synchronization architecture.

Profile fixture (synthetic 80x40 native grid through actual static classifier): original uninstrumented recorded run 68.773 ms; instrumented 70.297 ms and repeat70.265 ms (~2.2% observed difference, NOT isolated controlled overhead estimate: scheduling/load varies). Same 52 edges; profile 639 expansions,5167 hard calls,5012 soft calls,10078 grid reads; hard2.30ms,soft67.33ms,grid32.64ms nested within hard/soft. No measured speed gain. Overhead includes one clock at scope entry and one at destruction, only recovery query scopes; transaction tracing also adds clocks/publisher. No claim exact overhead or hard realtime.

125 tests pass: original124 plus diagnostic regression for schema33 fields, source frame/stamp/receipt/generation, ROS/steady intervals, successful integration, invalid STOP, disabled and skipped stage resets. Existing moving/no-path/recovery/goal freshness regressions unchanged. Production/test build passed. Logs /tmp/cane-dwcg-a24e/performance-final-build.log,performance-final-test-build.log,performance-final-tests.log,test-logs/. Search baseline profile-build.log/profile-tests.log preserved. Recorder AST/topics checked offline, no capture/navigation. TSAN still unavailable; no install.

Dedicated /mpc/transaction_diagnostics uses documented v1 schema and NaN skipped fields; existing metric positions unchanged. All-topic2.1s gap cause remains UNKNOWN. Recorder now includes raw source,clock,goals,initialpose,rosout,odoms,path/corridor/sample/debug/stop. Future command: rosrun plan_manage record_lightweight_mpc_eval.sh recovery_profile. Source enable and scenario coordinates untouched.

Independent review pending. No main source/devel writes,commit,push,install or user simulation.
