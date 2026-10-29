uint32 id
geometry_msgs/Pose2D start
geometry_msgs/Point goal
# Desired/max speed; actual social-force motion may be slower or deflected.
float64 desired_speed
# stop | return | repeat (arrival-based, not a fixed travel time)
string end_behavior
