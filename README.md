Tool Changer for ROS 2
======================

This repository contains tooling to implement tool changing in ROS 2.

---

This package is still work in progress and neither feature complete nor tested in its entirety.

---

Packages in the Repository
--------------------------

- `tool_change_manager`: Handles the parsing and re-assembling of URDFs and SRDFs to dynamically change the robot description.
- `tool_change_interfaces`: ROS 2 interfaces for interacting with tool changer nodes.
- `tool_change_executor`: Manipulation pipeline based node that performs coupling and decoupling motions.

Architecture
------------

This package implements tool changing by updating the current URDF `robot_description` and SRDF `robot_description_semantic` based on a specified tool state.
All tools are always present in both descriptions, but their attachment joints to the static base description are adjusted as needed.

The `tool_change_manager` handles most of this logic.
On startup, it retrieves the initial description and parses them to extract tool descriptions based on its configuration.
It then continuously publishes the state of each tool and provides a service to reattach a tool to a different link, leading to an update of both URDF and SRDF.
The new descriptions are published by setting the parameters of a `robot_state_publisher` and `srdf_publisher` node.

As tool changing typically involves some coupling motions and might require additional locking or unlocking tasks, `tool_change_executor` provides a higher-level interface for complete coupling and decoupling sequences.
All cartesian coupling and decoupling paths as well as additional actions are defined by configuration, and the `tool_change_manager` service is used to perform the required description updates.
High-level `Couple` and `Decouple` actions are provided that perform all required operations and try to prevent common problems (e.g. coupling a tool when another is still attached).

While `tool_change_manager` is a standalone component, `tool_change_executor` requires the `manipulation_pipeline` package for the execution of cartesian paths.

Tool Definition
---------------

The tool change manager node requires a list of tools as a parameter:

```yaml
tool_change_manager:
  ros__parameters:
    tools:
      - pneumatic_gripper
      - vacuum_gripper
      - screwdriver
      - [...]
```

Each tool is defined by a set of parameters:

```yaml
tool_change_manager:
  ros__parameters:
    pneumatic_gripper:
      root_link: tool_gripper_adapter
      tip_links:  # Optional
        - tool_gripper_grasp_center

    [...]
```

On startup, the tool change manager obtains the current URDF by reading the `robot_description` parameter of a [robot_state_publisher](https://index.ros.org/p/robot_state_publisher/).
It considers a tool to consist of all the links and joints that are descendants of `root_link` in the kinematic tree.

Attached Tools
--------------

All tools are always part of the URDF and attached to some parent links using fixed joints.
A distinction is made between active tools and passive tools: A tool is considered active if it is attached to the tip of a groups that consists of a single chain.

For active tools, the `tool_change_manager` will:
- Automatically create groups for each link specified in `tip_links` with names `<parent chain group name>_<tip link name>`.
  These groups consist of one chain from the base link of the parent group to the tip link.
- Automatically create an `end_effector` tag for each group in the tool.

Known Issues
------------

There are some known issues that might limit the applicability of this package to specific use cases.
If you know of a solution to them, please don't hesitate to open an issue:
- Non-URDF tags: The robot description commonly contains additional tags (`<gazebo>`, `<ros2_control>` etc.).
  As this package uses urdfdom for parsing, these tags are discarded and are not present in newly assembled descriptions.
  In some use-cases, you might need to make sure that e.g. `ros2_control` starts up before the tool changing node to work around this.
- `tool_change_manager` does note recognize new descriptions when they are published to the `robot_description` or `robot_description_semantic` topics.
  To work around this, you should make sure that you restart all tool changing nodes after loading a new robot description.
  It might be possible to listen to description parameter changes and handle these correctly, but some care needs to be taken to ensure that this does not lead to continuous description updates.

Acknowledgements
----------------

Development of this driver was in parts supported by the project "GANResilRob - Generative Adversarial Networks and Semantics for Resilient, Flexible Production Robots", funded by the German Federal Ministry for Economic Affairs and Energy under grant agreement 01MJ22003A.
