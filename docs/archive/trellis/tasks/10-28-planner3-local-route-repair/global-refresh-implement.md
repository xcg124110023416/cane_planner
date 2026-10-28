# Implementation checklist

- [x] Add proactive manager regression and confirm RED.
- [x] Replace footprint local-search entry with full-map current-root/exact-goal API.
- [x] Integrate cadence, pending corridor/MPPI targets and authorized commit.
- [x] Expose default 0.5s parameter through launch entry points.
- [x] Adapt regression cases and run affected suites.
- [x] Build production and run owned-master office and pedestrian replays.
- [x] Update docs/specs with measurements and practical limitations.

Use catkin build only. Never touch user master/nodes on 11311. Implementation
authorized; new commit/push awaits user instruction after concrete review.

Validation: [109 passing tests and two office replays](../../../docs/PLANNER3_GLOBAL_REFRESH_VALIDATION.md). Final manual full-scope review covered runtime and operator launch contracts. Candidate changes await the project-required local commit confirmation; no new commit or push was requested in this implementation turn.

## User-reported slow updates

User reports slow visible refreshing after testing. Continue diagnosis/implementation
under the same approved feature. Reproduce a 100ms-cadence failure against the real
manager; measure actual A* publication gaps separately from throttled logs, search
time, failed candidate validation and static root blockage. Then tune default to
100ms if measured workload supports it, retain slower configurable periods and
all original freshness/geometry/MPPI/execution checks. Preserve scheduler phase
under jitter and skip missed deadlines without catch-up bursts. Record actual
throughput under a private master; never restart user-owned nodes.

Slow-update follow-up complete: default global/preview .2s, fixed phase and shared per-transaction schedule epoch. Manager 40/40, production build and both real topic replays pass. Final office mean interval .192s; pedestrian preview mean .225s, max .303s. See refresh-cadence evidence and validation append.
