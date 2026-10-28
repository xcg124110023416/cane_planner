# Implementation and validation

- [x] Add a counter-based regression at actual A* recovery entry: soft queries
      must reuse selected-grid tiles rather than extract a grid for every edge.
      Run it red against the old implementation.
- [x] Add the map-owned distance-field value and search-local A* cache, with
      field geometry/invalid input and map-change regressions.
- [x] Build with catkin build; run affected A*, field, static controller and
      manager transaction/STOP regression targets on owned ROS master 11339.
- [x] Replay the previously recorded office scene, measure recovery time, inspect
      TIMEOUT / REBUILD_FAILED / SUCCESS separately, and clean up owned processes.
- [x] Update runtime contract/usage and persist measured findings. Review complete
      diff and git diff --check. User reviewed the result and subsequently authorized commit/push.

Rollback: restore previous comfort calculation only; hard certificates and motion
authorization are not part of this change. Do not restart user ROS on 11311.

Completed: 98 tests across five affected targets passed; production build passed.
Recovery search in the reported scene now completes in 38–43 ms; all 15 candidates
were safely rejected by subsequent footprint corridor rebuild. No successful
bypass claim. Detailed before/after evidence and remaining limitation are in
research/validation.md. No production debug probes or extra ROS interfaces added.
The initial implementation scope excluded automatic commit/push. On 2026-10-27,
the user explicitly requested committing the reviewed current changes and pushing
them to origin/mpc-lfpc-prototype. The proposed time-aware corridor redesign has
not been implemented and is outside this commit.
