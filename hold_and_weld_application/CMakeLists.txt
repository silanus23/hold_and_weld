cmake_minimum_required(VERSION 3.8)
project(hold_and_weld_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteGripper.action"
  "action/ExecuteWeld.action"
  "action/TriggerGripper.action"
  "action/TriggerWelder.action"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  tf2_geometry_msgs
  ament_index_cpp
)

add_library(gripper_action_server SHARED
  src/action_servers/gripper_action_server.cpp
)
ament_target_dependencies(gripper_action_server ${DEPENDENCIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_action_server "${cpp_typesupport_target}" yaml-cpp)

add_library(welder_action_server SHARED
  src/action_servers/welder_action_server.cpp
)
ament_target_dependencies(welder_action_server ${DEPENDENCIES})
target_link_libraries(welder_action_server "${cpp_typesupport_target}" nlohmann_json::nlohmann_json yaml-cpp)

add_library(dual_robot_coordinator SHARED
  src/coordinator/dual_robot_coordinator.cpp
)

add_library(hold_and_weld_kinematics SHARED
  src/kinematics/urdf_parser.cpp
)
ament_target_dependencies(hold_and_weld_kinematics
  rclcpp
  ament_index_cpp
  urdf
)
target_link_libraries(hold_and_weld_kinematics
  Eigen3::Eigen
)

ament_target_dependencies(dual_robot_coordinator ${DEPENDENCIES})
target_link_libraries(dual_robot_coordinator
  "${cpp_typesupport_target}"
  yaml-cpp
)

# Executables
add_executable(gripper_server_node
  src/action_servers/gripper_server_main.cpp
)
ament_target_dependencies(gripper_server_node ${DEPENDENCIES})
target_link_libraries(gripper_server_node gripper_action_server)

add_executable(welder_server_node
  src/action_servers/welder_server_main.cpp
)
ament_target_dependencies(welder_server_node ${DEPENDENCIES})
target_link_libraries(welder_server_node welder_action_server)

add_executable(coordinator_node
  src/coordinator/coordinator_main.cpp
)
ament_target_dependencies(coordinator_node ${DEPENDENCIES})
target_link_libraries(coordinator_node dual_robot_coordinator yaml-cpp)

install(TARGETS
  gripper_action_server
  welder_action_server
  dual_robot_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gripper_server_node
  welder_server_node
  coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/utils/add_collision_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # URDF Parser Test
  find_package(hold_and_weld_description REQUIRED)

  ament_add_gtest(urdf_parser_test
    test/urdf_parser_test.cpp
  )
  target_include_directories(urdf_parser_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(urdf_parser_test
    rclcpp
    ament_index_cpp
    urdf
    hold_and_weld_description
  )
  target_link_libraries(urdf_parser_test
    hold_and_weld_kinematics
    Eigen3::Eigen
  )
endif()

ament_package()
