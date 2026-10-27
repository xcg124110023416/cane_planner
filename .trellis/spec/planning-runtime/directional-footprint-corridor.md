# Directional pedestrian / human-cane / adaptive corridor contract

Approved 2026-10-27; supersedes older safety-octagon, COM-only and fixed-edge
production requirements. Scope is planner 3; no SFM expansion or multi-pedestrian
support. Practical configuration is documented in docs/PLANNER3_CORRIDOR_USAGE.md.

PedestrianPolygon returns physical body, raw social triangle and used collision
hull. Pad the complete world AABB independently by body_margin=.03, then hull it
with heading +/-110-degree Ds=.4 points and force point. Ds is not raised to body
radius, and no full social-radius disk masks direction. Zero force uses moving
heading; missing stationary heading uses symmetric padded body. Force computation
and bounded observations remain unchanged. Both FIRI and CV/SAT use this exact hull.

HumanCaneFootprint supplies one rigid convex envelope shared by FIRI, controller
and markers. Circumscribed disk polygons cover full body/front disks with margin,
including between vertices. Production parameters enable it; standalone legacy
fixtures and old snapshots retain explicit point/circular mode. Use LFPC torso yaw,
never spatial Z or lateral sway velocity direction. No articulated cane is inferred.

Extend the finite angular separator to ALL protected footprint vertices and both
edge endpoints. Enumerate every seed-clearance-active direction, preserve unshifted
objective and independent whole-cell/polygon/seed certificates. Current root includes
actual pose. Original edges are never shortcut. Default adaptive mode analytically
clips the continuous original path against footprint-eroded halfspaces and advances
to before the first coverage exit (up to .25 m overlap). Every region must make
progress; whole-path footprint coverage, joins, deadlines and raw region cap hold.
Inflation must protect a FORWARD polyline prefix, not only the current pose:
point-seeded planes can become tangent to the envelope and cause asymptotic
NO_PROGRESS in otherwise free side-obstacle passages. The prefix length is capped
by min(max_segment_length, local_radius/2), truncated to the remaining route.
Include both endpoints of every intersected ORIGINAL edge with its reference yaw;
retain intermediate corners rather than a chord shortcut. Convex-hull extreme
vertices suffice for the same linear protected-seed constraints. Coverage-based
region placement remains independent of input vertex density. Preserve the frozen
adaptive-office snapshots and minimized two-side-cell forward-progress regression.
Legacy edge mode remains for comparisons and v1/v2 snapshot replay. Snapshot v3
records sampling mode, dimensions, enable and root yaw; never lose those on capture.

Each rollout sample's WHOLE envelope must fit ONE common region, not separate
vertices in unrelated regions. This conservative join rule can reject union-contained
poses. Keep static COM and foot checks. SAT binds the same local envelope and caches
axes/supports per torso yaw. Plan caches its local polygon once, rotating vertices
without reconstructing a hull in the hot loop. Retain matching path/yaw/time arrays
through selection/replay and actual execution-age recheck. Step-boundary rotations
are sampled <=5 degrees at the root of that step and stored at that same model time;
they are not a new continuous-time turn model or a swept collision guarantee.

Planning reference length defaults 8 m, exact truncation includes root-projection
connector; preview length defaults 20 m, zero selects remaining route. Preview is
an independent bounded async value-copy job; no manager/controller access, route
recovery or STOP authority. Current/receipt age, clock rollback, goal/route revision,
reset/invalid cleanup and short marker lifetimes apply. Dynamic preview uses the
already accepted hull, never reruns SFM with a larger cropped-grid boundary. Copying
the selected grid and CPU contention are still finite costs, not zero-cost parallelism.

Required tests: both complete physical disks at rotated poses, heading and weak-force
visibility with body coverage, cane-tip-only collision, repeated yaw cache changes,
execution latency rejection, dense/sparse adaptive generation counts, corners and
whole original path coverage, budgets, historical static snapshots, v3 exact roundtrip,
actual manager distant preview failure without route/model/STOP mutation and marker
geometry matching production. Source freshness and accepted-observation transaction
regressions remain mandatory. No claim of continuous safety or real-device calibration.
