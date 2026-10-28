# Time-triggered local reference repair (planner 3)

> Historical contract. Superseded by [periodic-global-reference.md](periodic-global-reference.md).

Approved 2026-10-28. Supersedes the default current-frame pedestrian FIRI gate
and current-CoM-to-final-goal recovery in the older corridor contract.

## Spatial domain and temporal safety

Default active and preview FIRI use the selected STATIC grid and configured full
human-cane envelope. They are static free-space constraints, not pedestrian-free
space at every future time. Build and validate the accepted directional pedestrian
hull under the same observation transaction, but bind it to timed MPPI/SAT instead
of blocking the whole active reference with its current position. No invalid,
missing, expired, rollback or multi-pedestrian observation bypasses the source gate.
Disabling timed checks retains current-frame whole-polygon FIRI exclusion as a
spatial ablation; it does not silently remove pedestrian constraints.

Try timed MPPI first. Only an invalid plan with a genuine collision witness may
attempt one bounded local route repair. Valid plans never trigger recovery solely
because the current pedestrian hull crosses the spatial reference. Keep root,
all LFPC sample/yaw/time, weighted-mean, selected replay and execution-age checks.
Do not add speed/wait actions, partial-horizon success or execution-collision retry.
Cold-start MPPI receives a geometric steering proposal from a copied LFPC following
the active reference; its projection advances across segments. Preserve retained
warm starts. Slot 1 keeps a checked zero-steering alternative until this cycle has
a verified candidate. Neither proposal bypasses any trajectory checks.

## Local A* contract

`Astar::repairLocalReference` takes the exact-root remaining authoritative polyline,
same-observation hull translated to a witnessed prediction time, footprint config,
root yaw and cooperative deadline. The hull is spatial guidance only: a new route
must still pass matching-time MPPI checks against the original hull/velocity.

Locate affected original edges/turn poses with the full footprint and select entry
and rejoin vertices with >=max(1 m, envelope radius + one search bin) flank when
available. Search a nearby bounded rectangle, splice only that interval and retain
the exact untouched suffix/final goal. States are position plus incoming one-of-eight
lattice heading (and exact root heading), so different arrival headings are not
merged. Euclidean heuristic; no claim of global optimality or completeness.

Fetch the selected static grid ONCE, with footprint/comfort padding. Do not persist
pedestrians in maps. For each edge certify the entire fixed-heading footprint
translation sweep against whole blocked cells and the complete predicted hull
using support intervals. Check turns at endpoints and intervening global <=5-degree
angles. This is sampled rotational geometry, not continuous swept-turn safety.
Routing clearance is existing clearance plus one search-resolution bin; physical
shape is unchanged. Per-search exact-value sweep/pose/heading caches cannot survive
map/source changes. Comfort distance is built from the same grid using the existing
unsigned field and whole-edge midpoint integration, never used as a hard certificate.

Limit search to min(0.2 s, half original header/receipt remaining freshness).
Check deadlines in reference scanning, expansion, cell/comfort loops and after
bounded non-preemptible grid/field work. Failure/timeout returns no candidate.

## Candidate transaction

The manager holds dynObsMutex throughout first MPPI, optional search, static FIRI
rebuild, second MPPI, original-frame authorization and execution revalidation.
No SFM recomputation, timestamp refresh, freshness widening or queued-frame
substitution. Rebuild the active-length corridor for a candidate. Seed steering
from a copied current LFPC following that new reference; this proposal receives
the same full evaluator as every other MPPI sample and is not presumed safe.
Rebind source age at retry time. Only after selected-plan execution revalidation
may the pending route replace global_waypoints_/index/lookahead. Integrate before
visual publication. Failed FIRI, MPPI, freshness or execution checks retain the old
authority and frozen LFPC STOP behavior. Fresh empty frames still allow resumption.

`READY` is a temporary candidate, `SUCCESS` is an execution-authorized commit.
Other statuses: NOT_NEEDED, RECOVERY_UNAVAILABLE, ATTEMPTED, NO_PATH, TIMEOUT,
EXACT_GOAL_UNAVAILABLE, EXPIRED, REBUILD_FAILED, MPPI_REJECTED, EXECUTION_REJECTED.
Preserve metric indices; MPPI timing includes both attempts when recovery runs.
Recovery hard/soft/grid counters now describe local snapshot search, with one
grid acquisition; its grid time is separate from the hard/soft loops. Static
snapshot captures are valid for static FIRI failures even when timed pedestrians
are enabled, but do not replay the complete dynamic transaction.

## Visualization and regression

Debug text declares domain=STATIC_BASE and dynamic_validation=TIMED_SAT. Spatial
ablation reports CURRENT_FRAME/SPATIAL_ABLATION. Preview is explicitly static-only
space by default, retaining original observation freshness/revision cleanup.
Timed matching hulls/path and source body/social markers remain authoritative for
their respective geometry, with short lifetimes and deletion on invalid/reset/STOP.

Regress the real manager: future-vacated crossing with zero recovery, direct timed
MPPI avoidance, forced insufficient-proposal local repair with suffix retention,
full-footprint FIRI and temporal success, closed passage STOP, search/rebuild/MPPI/
execution-authorization failure without route commit, queued callback serialization
and unchanged 0.5 s source/receipt expiry. Also retain full geometry, static selected
backend, timed SAT, final-goal and LFPC replay tests. Benchmark and office replay
outcomes must separate successful repair, direct avoidance, STOP and arrival.
