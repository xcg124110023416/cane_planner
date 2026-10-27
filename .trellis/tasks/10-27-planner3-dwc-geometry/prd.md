# Approved scope

The user approved the four-part implementation described in the conversation:
directional pedestrian geometry with physical coverage, full human-cane geometry,
coverage-driven FIRI sampling, and separately configurable planning/display ranges.
SFM expansion and multiple pedestrians are excluded. MPPI and accepted-observation
freshness/execution authorization remain.

Acceptance: heading affects the social hull without losing the body; reverse/zero
force is defined; a cane-tip collision rejects a trajectory although COM is clear;
full footprint fits a certified region at every checked pose; adaptive regions cover
all original edges with overlaps and finite budgets; distant preview cannot authorize
motion or trigger STOP. Configurable launch and RViz displays use production geometry.
Build and meaningful geometry/controller/manager tests, plus isolated ROS smoke checks.

User approval: “我同意，开始吧” (2026-10-27).
