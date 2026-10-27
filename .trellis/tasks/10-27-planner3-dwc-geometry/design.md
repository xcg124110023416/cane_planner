# Design

PedestrianPolygon returns body, raw social triangle and collision hull; physical
rectangle margin is independent of social Ds. Zero-force uses heading, zero-speed
without heading uses physical core. Keep bounded force and fresh observation contract.

HumanCaneFootprint is pure shared geometry, using circumscribed disk polygons and
a convex hull; configs supply dimensions, margin, enable. Production planner 3
enables it, while historical standalone fixtures may explicitly use point/circle
mode. Initial model is rigid with LFPC torso yaw, not independently articulated cane.
Reference seeds protect footprint vertices; controller checks whole footprint in one
region (conservative at joins), dynamic SAT uses the same rotated footprint. Sample
yaw and time travel together through rollout/replay/execution checks and markers.
Check intermediate rotation poses; no continuous-time safety claim.

ConvexCorridor retains its separator/MVIE and whole obstacle certificates. Generalize
protected seeds to include footprint vertices. Adaptive sampling advances via analytic
original-edge clipping of footprint-eroded halfspaces with a backward overlap offset.
Each adaptive inflation protects a bounded forward original-polyline prefix and
both endpoint footprints of every intersected edge; point-only protection can
produce tangent planes and asymptotic NO_PROGRESS in free side-obstacle passages.
The prefix cap is min(max_segment_length, local_radius/2), with convex-hull extreme
vertices reducing redundant linear constraints. No original corners are shortcut.
Retain edge-based mode for comparison/legacy snapshot replay. Snapshot v3 records all
new geometry/sampling config. Point-mode historical snapshots stay reproducible.

Planning length defaults 8 m; preview 20 m (0 means remaining route), region radius
3 m. Preview uses immutable copied inputs in an async job and a bounded independent
FIRI budget, never calls recovery or changes active controller geometry. Only fresh,
same-goal results are drawn; reset/invalid/expired results clear. Planning transaction
and STOP logic stay authoritative. Far geometry is explicitly labelled preview.

Runtime publishes raw social/body/used hull, FOV/force, current and predicted human-
cane shapes and a separate pale preview topic. Shared shape calculations avoid
visual/collision mismatch. Existing map inflation remains an additional margin.
