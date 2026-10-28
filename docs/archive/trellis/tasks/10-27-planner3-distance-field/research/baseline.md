# Baseline evidence

Before this change, replay of reported route (-6.393,4.190,.089) -> (3.633,-4.828)
on office2_modify.pcd with remembered dwcg_crossing pedestrian yielded 7 recovery
TIMEOUTs and 7 STOP decisions; eventual arrival followed pedestrian disappearance.
Saved recorder: /tmp/cane_pedestrian_analysis.py; captured result:
/tmp/cane-pedestrian-analysis-reported-result.json (not committed).

Across seven attempts: mean total 200.003 ms; inclusive soft cost 183.533 ms
(91.76%); inclusive hard check 12.342 ms; other 4.128 ms. Nested grid query time
85.820 ms must NOT be added again. About 3,546 expanded nodes, 7,712 soft calls,
27,911 hard calls and 35,572 grid extractions per attempt. Release / -O3 build.

Root cause: repeated per-edge selected-grid classification plus nearest blocked
cell scan at every <=2 cm midpoint. Recovery also re-searches to final goal with
a 200 ms deadline; local bypass policy is outside the approved optimization.
