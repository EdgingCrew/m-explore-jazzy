# Current frontier goal of an explore_lite instance, shared between robots
# so they can spread out (see the coordination_scale parameter).

# header.frame_id: frame of goal, the explore node's map frame
std_msgs/Header header
# fully qualified name of the publishing explore node
string robot
geometry_msgs/Point goal
# false: the robot has no frontier goal (paused, finished, returning)
bool active
