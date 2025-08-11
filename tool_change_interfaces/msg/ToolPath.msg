# Cartesian path used to couple or detach a tool

# Tip link that should follow defined pose waypoints
string tip

# Frame relative to which the path is defined
string path_frame

# Waypoints of cartesian path
geometry_msgs/Pose[] path

# Cartesian limits to use during path execution
# If left unchanged, default limits will be used.
manipulation_pipeline_interfaces/CartesianLimits motion_limits
