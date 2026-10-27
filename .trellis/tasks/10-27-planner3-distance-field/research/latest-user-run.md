# Latest user run: spatial corridor vs timed collision

User requested read-only verification after distance-field changes. Examined the
user's 18:59 launch (2026-10-27), not the agent's 18:51 replay. ROS log run UUID
2855ef8a-d1b1-11f1-8fba-516ca33c0506; latest goal starts at rosout.log line 1999.
No October bag was found under the configured /home/xcg/ws/records directory.

Latest goal: (-5.14,4.66) -> (3.51,-4.52), A* materializes 139 points in ~19.5 ms.
18:59:34.302: DYNAMIC_REFERENCE_BLOCKED, segment 62, s=7.153,
position (-1.929,-1.854), OBSTACLE_SEPARATION_SOLVE. STOP valid=0.00.
18:59:34.804: same STOP reason. 18:59:35.306: same failure at segment 42,
s=5.070, position (-2.129,.146), STOP valid=0.00.
18:59:39.562: final goal reached at (3.60,-4.51), 56 steps.
These are throttled console entries, not counts of all decision cycles.

All recorded STOPs for this latest goal are spatial corridor blockage. Code binds
current-observation pedestrian hulls to FIRI, protecting human-cane footprint
vertices on original reference edges. Invalid corridor clears the controller's
constraint set; plan returns before rollout optimization. This precheck does not
time-match the reference pose's arrival with the pedestrian's future position.
It can invalidate the entire 8 m active reference for a far-ahead obstruction.
The console does not identify a cane-tip-only witness or the recovery A* status;
do not claim either from this trace. No new production code changes were made.

The PREVIOUS 18:58 launch separately contains TIMED_EXECUTION_COLLISION at
18:58:02.800 (valid=.55) and 18:58:21.219 (valid=.18). Those are actual selected
trajectory execution-epoch revalidation failures; distinguish them from the
latest goal's DYNAMIC_REFERENCE_BLOCKED. Even those messages identify the
combined footprint rather than a particular body/cane component.
