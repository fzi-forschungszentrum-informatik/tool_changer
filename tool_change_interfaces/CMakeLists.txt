cmake_minimum_required(VERSION 3.8)
project(tool_change_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_pipeline_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    action/Couple.action
    action/Decouple.action
    action/Lock.action
    action/Unlock.action
    srv/SetTool.srv
    msg/DisableCollision.msg
    msg/State.msg
    msg/ToolDefinition.msg
    msg/ToolLibrary.msg
    msg/ToolPath.msg
    msg/ToolState.msg
  DEPENDENCIES
    geometry_msgs
    manipulation_pipeline_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
