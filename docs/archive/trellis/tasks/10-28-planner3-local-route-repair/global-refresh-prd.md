# Approved follow-up: periodic full global A*

User explicitly approved implementation after requesting full global refreshing.
Default cadence is 0.5 seconds, current CoM to the exact final goal, independent
of MPPI success. Use the latest selected static backend and same-observation
current pedestrian hull as a temporary overlay. Rebuild the active FIRI corridor
and validate timed MPPI plus original source/receipt freshness and execution age
before authoritative route replacement. Remove the local-repair runtime entry.
Preserve LFPC state on failed candidates, physical footprint, map semantics and
planner 4. Do not change LFPC timing, add speed/wait actions or time-expanded A*.

Acceptance: proactive cadence, new static occupancy rerouting, current-pedestrian
avoidance, exact goal, transactional rejection, existing safety/heartbeat tests,
production build and isolated office/pedestrian replay.

Current tuning after user feedback: see refresh-cadence-design.md. Default period is now .2s, with fixed-phase scheduling instead of start-relative drift. Original .5s remains configurable; all candidate and execution gates remain.
