# The state of a single tool

# The name of the tool as defined in the tool_change_manager configuration
string name

# The link this tool is currently attached to
string parent

# Whether the tool is currently active, i.e. attached to the tip of a single-chain group
bool active

# If active, the group name of the single-chain group this tool is attached to
string active_parent_group
