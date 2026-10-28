# Design

## Remove the premature spatial gate

Active/preview FIRI describes static free space. The manager still builds the
accepted directional pedestrian hull, validates its original observation, and
binds that exact hull/velocity to MPPI's timed SAT. Every rollout, weighted update,
selected replay and execution-epoch check remains mandatory. A successful timed
MPPI needs no route recovery even if current pedestrian geometry crosses the
reference. Seed missing warm starts from a copied LFPC following the forward
reference projection; preserve retained warm starts and a checked straight anchor. This supersedes the current-frame whole-reference pedestrian gate.

## Recovery trigger and geometry

First run timed MPPI in the static corridor. Only a failed plan with an actual
timed collision witness may request one bounded local repair in this cycle.
Use the same accepted pedestrian hull translated to the witness prediction time
as a spatial detour guide. This overlay is not a temporal safety certificate;
the repaired candidate must pass timed MPPI and execution checks again.

Reuse Astar with a local heading-aware recovery entry point. Locate the obstructed
reference interval using the whole footprint. Select entry/rejoin vertices with
space for turns, search a bounded nearby window, splice the detour into the
remaining original polyline and preserve its exact suffix. Search states include
incoming lattice heading so a different arrival heading is not incorrectly
discarded. Certify translated footprint sweeps against whole static cells and the
overlay; sample reference heading changes at <=5 degrees, matching the existing
pose-check convention without claiming continuous rotational swept safety.

The local grid is copied once from the selected backend, padded for the footprint
and comfort distance. It is query-scoped and never rasterizes pedestrians into
persistent maps. Reuse the distance-field comfort objective and bounded deadlines.

## Manager transaction

Keep the accepted-observation mutex through geometry, first MPPI, optional local
repair/rebuild/second MPPI, source authorization and execution revalidation.
Candidate route/corridor/target remain temporary until all checks pass. Commit
the new authoritative route only with a valid fresh timed plan. Keep the old
route on failures, but never execute an invalid trajectory. Retain <=0.2 s / half
remaining source-freshness A* budget. Do not retry execution-time collision.

## Outputs

Label corridor debug/preview as static free space plus timed dynamic validation.
Preserve existing timed collision/path displays, source hull markers, accepted
A* path, debug field positions, STOP odometry and cleanup. Append recovery scope
and conflict information to text diagnostics where useful.

## Review and rollback

The user's explicit approval to start applies to this design derived from the
accepted recommendation. No unrelated changes or new dependency. Revert this
task's diff to restore current-frame recovery; do not weaken geometric checks.
