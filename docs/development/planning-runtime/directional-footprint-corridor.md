# Directional pedestrian / human-cane / adaptive corridor contract

Updated 2026-10-28: the user explicitly limits spatial human-cane protection to
the actual current pose. This supersedes the earlier predicted full-envelope
static/corridor hard constraints. Scope is planner 3; no SFM expansion or multi-pedestrian
support. Practical configuration is documented in docs/PLANNER3_CORRIDOR_USAGE.md.

PedestrianPolygon returns physical body, raw social triangle and used collision
hull. Pad the complete world AABB independently by body_margin=.03, then hull it
with heading +/-110-degree Ds=.4 points and force point. Ds is not raised to body
radius, and no full social-radius disk masks direction. Zero force uses moving
heading; missing stationary heading uses symmetric padded body. Force computation
and bounded observations remain unchanged. CV/SAT binds this exact source-frame hull. FIRI always separates it after
translation to its current generation epoch, independently of timed checking.

HumanCaneFootprint supplies one rigid convex envelope shared by FIRI, controller
and markers. Circumscribed disk polygons cover full body/front disks with margin,
including between vertices. Production parameters enable it; standalone legacy
fixtures and old snapshots retain explicit point/circular mode. Use LFPC torso yaw,
never spatial Z or lateral sway velocity direction. No articulated cane is inferred.

FIRI protects the actual current human-cane hull in its first region, using
root_yaw rather than the heading of a future reference edge. That hull must fit
one region. Occupied cells still have a common separating plane and independent
whole-cell/seed/ellipse certificates. Do not translate the hull down the reference,
protect future reference-heading hulls, or erode future coverage by its support.
The spatial corridor has no time dimension.

Adaptive inflation protects a bounded forward portion of the original COM
polyline. Retain both endpoints of each intersected original edge and intermediate
corners, never a chord shortcut. The prefix length is capped by
min(max_segment_length, local_radius/2), truncated to the remaining route. If the
whole protected interval cannot be geometrically separated, halve it down to the
remaining current original straight edge before rejecting. A free bent polyline
does not imply its convex hull is free. Never shorten away the actual root hull,
 change clearance, or recover invalid/budget failures this way. Include
the actual hull only in the first region's protected set. Analytically clip the
point polyline against the resulting halfspaces and advance to before its first
coverage exit (up to .25 m overlap). Require progress, original-path coverage,
positive-interior joins, deadlines and raw region cap. Legacy per-edge mode also
protects the actual hull only at the first seed. Snapshot v3 continues to record
sampling mode, physical dimensions, enable and root yaw; old snapshots are replayed
with current algorithm semantics, not a frozen historical generator.

`buildStatic` requires complete coverage. `buildPrefix` in adaptive mode may
retain an already generated connected prefix on a geometric suffix failure
(reference occupied, geometric output certificate, no progress or no overlap).
This only retains regions generated before the failed suffix; if the first region
fails, no prefix is returned. Re-certify original-path
coverage from zero to `covered_length` and all positive-interior joins. Input,
Input, scale, numerical and deadline failures remain unusable. A feasible prefix
reports `complete=false`, `failure_reason=NONE`, and `truncation_reason`;
its retained failure location/diagnostics describe the omitted suffix. MPPI must
still validate its complete configured horizon in the returned corridor; never
shorten the optimization horizon or admit unchecked remaining states.

MPPI checks COM containment in the corridor and static traversability of COM and
support foot. Future full envelopes do NOT have to fit one common region and are
NOT tested against static cells, including during timed lightweight execution.
The current actual hull is certified when building each new planning corridor.
This does not guarantee future cane clearance from static obstacles.

Prediction remains meaningful for pedestrians: SAT binds the complete configured
local envelope and caches axes/supports per torso yaw. Keep matching path/yaw/time
arrays through selection, independent replay and execution-age revalidation.
Step-boundary rotations are sampled <=5 degrees and stored at that same model
time for pedestrian checks. These samples do not enlarge FIRI's protected root
hull or add static turn-sweep constraints. The envelope remains shared by actual
root containment, timed pedestrian collision geometry and RViz markers.

Planning reference length defaults 8 m, exact truncation includes root-projection
connector; preview length defaults 20 m, zero selects remaining route. FIRI separates static cells AND current pedestrian hulls; future pedestrian
collision is checked independently at matching trajectory times (see periodic-global-reference.md). Preview is
an independent bounded async value-copy job with a default .2s phase aligned
to global search starts (period .1..5s); no manager/controller access, route
recovery or STOP authority. Current/receipt age, clock rollback, goal/route revision,
reset/invalid cleanup and short marker lifetimes apply. Preview always separates the already accepted current hull without rerunning SFM
with a larger grid boundary. Before publication, recheck the completed regions
against current hulls; clear an overlapping stale preview. Timed checking does not
change spatial corridor generation. Copying
the selected grid and CPU contention are still finite costs, not zero-cost parallelism.

Required tests: complete physical disks at rotated poses; directional pedestrian
hulls; cane-tip-only timed collision; yaw/time/latency revalidation; dense/sparse
point-reference coverage and forward progress; actual-root hull containment and
rejection when obstructed; future static cane overlap / corridor escape no longer
rejecting a free COM; original static COM/foot rejection; historical snapshots and
v3 exact roundtrip; preview failure without STOP/route/model mutation; source
freshness and accepted-observation transactions. No continuous safety or
real-device calibration claim.
