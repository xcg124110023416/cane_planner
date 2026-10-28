# Design

`cane_planner::CorridorDistanceField` in plan_env is a value built from an aligned
selected 2D blocked-cell snapshot. Rasterize entire closed cells onto a
half-native-resolution lattice, include the snapshot boundary, and run Euclidean
distance transform. Bilinear lookup approximates distance between lattice samples.
This is a comfort field only; it cannot authorize edges or footprint containment.

A* lazily caches fields on native-aligned spatial tiles (up to 64 bins per core).
Each tile has a halo exceeding the finite comfort influence range by a native
bin, so artificial crop sides cannot affect the truncated penalty in its core.
Tile dimensions respect the existing snapshot adapter budget. Keep complete-edge
midpoint quadrature and metre costs. Clear tiles at EVERY new search, reset,
parameter reload and collision-source replacement: search-local snapshots avoid
requiring a new revision contract for every local-map producer. This trades a few
tile rebuilds for explicit freshness and bounded work; no cross-search cache.

Existing continuous hard whole-cell and whole-polygon checks and final FIRI remain
unchanged. Check recovery deadline before and after a bounded tile build. Do not
include pedestrian polygons in the static field, increase the deadline/freshness,
shorten the planning reference, or redesign global recovery in this change.
