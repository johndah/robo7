# One point of the trajectory

# Data of the points
int64 id_number
geometry_msgs/Vector3 point_coord
geometry_msgs/Twist pose
float64 speed
float64 distance
std_msgs/Bool line_or_turn  #if it is a line -> true otherwise -> false
float64 radius
