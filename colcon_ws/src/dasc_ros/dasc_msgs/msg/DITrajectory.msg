std_msgs/Header header

# from the start time in the header, construct a trajectory
float64 dt # timestep between these messages, in seconds

geometry_msgs/Pose[] poses # each contains both linear and angular components
geometry_msgs/Twist[] twists
geometry_msgs/Accel[] accelerations
