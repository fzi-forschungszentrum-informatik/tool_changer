# Kinematic and semantic definition of tool components

# The name of the tool
string name

# The name of the root link of the tool kinematic model
string root_link

# Names of all links that are part of the tool
# This includes root_link.
string[] links
# Names of all joints that are part of the tool
string[] joints

# Action to call before executing couple_before_lock trajectory
# No action will be called if left empty.
string pre_lock_action

# Cartesian path used to move the robot to the tool before locking it
# As the tip link depends on the robot that couples, its entry is ignored and the value from the Couple action is used.
ToolPath couple_before_lock

# Action to call to lock tool to robot
# No action will be called if left empty.
string lock_action

# Cartesian path used to remove the tool from its storage location after locking it
ToolPath couple_after_lock

# Action to call before executing decouple_before_unlock trajectory
# No action will be called if left empty
string pre_unlock_action

# Cartesian path used to move the robot before unlocking the tool
# To make the definition easier, the path frame is evaluated at the end of the trajectory. This allows a trajectory definition relative to the final position of a tool link.
ToolPath decouple_before_unlock

# Action to call to unlock tool from robot
# No action will be called if left empty.
string unlock_action

# Cartesian path used to move the robot after unlocking the tool
# As the tip link depends on the robot that couples, its entry is ignored and the value from the Decouple action is used.
ToolPath decouple_after_unlock
