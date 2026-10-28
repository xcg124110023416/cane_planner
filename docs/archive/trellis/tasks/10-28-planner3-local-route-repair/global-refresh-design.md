# Design

Replace local-window search with heading-aware full-map footprint A*. Reuse the
whole-cell translation certificates, sampled turn checks and selected-grid comfort
field. Search full selected map bounds; no suffix splicing or conflict witness.
One accepted-observation transaction builds pedestrian geometry once, attempts
periodic search, rebuilds active FIRI for the candidate, runs timed MPPI and final
execution authorization. Current hull is translated only by source age to the
search epoch; it is spatial guidance, not future occupancy certification.

A candidate remains pending until successful integration authorization. Search
failure may use the retained route only through fresh corridor/MPPI/execution
checks. Candidate rebuild/MPPI rejection stops without route replacement. Cadence
uses ROS time, resets on goal and rollback, and counts attempted searches including
failed ones. Search deadline remains <=0.2s and half remaining observation age.
A* publications show only accepted routes. Keep diagnostic array positions; rename
text status to global_replan and state scope=FULL_GLOBAL. Default FIRI remains
static free space; timed SAT supplies pedestrian safety at sampled model times.

Current tuning after user feedback: see refresh-cadence-design.md. Default period is now .2s, with fixed-phase scheduling instead of start-relative drift. Original .5s remains configurable; all candidate and execution gates remain.
