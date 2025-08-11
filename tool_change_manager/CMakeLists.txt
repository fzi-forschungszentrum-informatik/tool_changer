cmake_minimum_required(VERSION 3.8)
project(tool_change_manager)

find_package(ament_cmake REQUIRED)

#
# Find dependencies
#
set(ROS_DEPENDENCIES
  rclcpp
  srdfdom
  tf2
  tool_change_interfaces
  urdfdom
)
foreach(DEP ${ROS_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()

find_package(fmt REQUIRED)
find_package(TinyXML2 REQUIRED)

#
# Create executable
#
add_executable(tool_change_manager)
target_sources(tool_change_manager
  PRIVATE
    src/description_builder.cpp
    src/param_interface.cpp
    src/robot_description.cpp
    src/robot_model.cpp
    src/srdf_builder.cpp
    src/tool.cpp
    src/tool_change_manager.cpp
    src/urdf_builder.cpp
)
target_include_directories(tool_change_manager
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(tool_change_manager
  PUBLIC
    c_std_11
    cxx_std_20
)
target_link_libraries(tool_change_manager
  fmt::fmt
  tinyxml2::tinyxml2
)
ament_target_dependencies(tool_change_manager
  ${ROS_DEPENDENCIES}
)

#
# Installation
#
install(
  TARGETS
    tool_change_manager
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
