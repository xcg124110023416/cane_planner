# Authoritative snapshot for the single-pedestrian RViz editor.
std_msgs/Header header
bool editable
bool placed
geometry_msgs/Pose start_pose
geometry_msgs/Vector3 size
float64 speed
float64 duration
# stop | disappear | return | repeat
string end_behavior
bool start_on_goal
# waiting | walking | paused | finished | cleared | legacy_<mode>
string phase
bool visible
bool remember_last
string persistence_error
