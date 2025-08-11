cmake_minimum_required(VERSION 3.8)
project(tool_change_executor)

find_package(ament_cmake REQUIRED)

#
# Find dependencies
#
set(ROS_DEPENDENCIES
  manipulation_pipeline_interfaces
  rclcpp
  rclcpp_action
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tool_change_interfaces
)
foreach(DEP ${ROS_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()

find_package(fmt REQUIRED)
#
# Create executable
#
add_executable(tool_change_executor)
target_sources(tool_change_executor
  PRIVATE
    src/tool_change_executor.cpp
    src/tool_library.cpp
)
target_include_directories(tool_change_executor
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(tool_change_executor
  PUBLIC
    c_std_11
    cxx_std_20
)
target_link_libraries(tool_change_executor
  fmt::fmt
)
ament_target_dependencies(tool_change_executor
  ${ROS_DEPENDENCIES}
)

#
# Installation
#
install(
  TARGETS
    tool_change_executor
  DESTINATION lib/${PROJECT_NAME}
)

#
# Testing
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
