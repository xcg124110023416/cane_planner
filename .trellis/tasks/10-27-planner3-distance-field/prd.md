# Planner 3 distance-field comfort cost

User approved implementation on 2026-10-27: replace repeated whole-grid wall
distance scans in A* comfort cost with fast distance-field queries derived from
the SAME selected corridor obstacle grid. Preserve hard certificates, pedestrian
geometry, observation freshness, recovery deadline and STOP behavior.

Acceptance:
- Soft-cost map extraction scales with visited regions, not candidate edges.
- New searches observe updated maps; a pedestrian overlay never pollutes static
  distances. Invalid map input cannot reuse an old field.
- Distances approximate whole blocked cells and the map boundary, including
  selected backend/known-region classification; soft approximation is documented.
- Existing exact-edge, exact-goal, heuristic and manager safety tests pass.
- Production build and isolated replay of the reported office/pedestrian route
  quantify recovery time and distinguish search success from FIRI acceptance.
- Commit/push require explicit user instruction; granted in the subsequent
  request on 2026-10-27. The discussed dynamic corridor redesign is not included.
