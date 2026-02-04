cmake_minimum_required(VERSION 3.8)
project(hold_and_weld_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteGripper.action"
  "action/ExecuteWeld.action"
  "action/TriggerGripper.action"
  "action/TriggerWelder.action"
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(hold_and_weld_kinematics SHARED
  src/kinematics/urdf_parser.cpp
  src/kinematics/kinematics_solver.cpp
  src/kinematics/ceres_ik_solver.cpp
)

ament_target_dependencies(hold_and_weld_kinematics
  rclcpp
  geometry_msgs
  urdf
  kdl_parser
  ament_index_cpp
)

target_link_libraries(hold_and_weld_kinematics
  Eigen3::Eigen
  Ceres::ceres
)

add_library(hold_and_weld_action_servers SHARED
  src/action_servers/gripper_action_server.cpp
  src/action_servers/welder_action_server.cpp
  src/action_servers/move_to_pose_action_server.cpp
)

ament_target_dependencies(hold_and_weld_action_servers
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  control_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(hold_and_weld_action_servers
  "${cpp_typesupport_target}"
  yaml-cpp
  nlohmann_json::nlohmann_json
)

add_library(hold_and_weld_coordinator SHARED
  src/coordinator/dual_robot_coordinator.cpp
)

ament_target_dependencies(hold_and_weld_coordinator
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)

target_link_libraries(hold_and_weld_coordinator
  "${cpp_typesupport_target}"
)

add_executable(gripper_server
  src/action_servers/gripper_server_main.cpp
)

target_link_libraries(gripper_server
  hold_and_weld_action_servers
)

ament_target_dependencies(gripper_server
  rclcpp
)

add_executable(welder_server
  src/action_servers/welder_server_main.cpp
)

target_link_libraries(welder_server
  hold_and_weld_action_servers
)

ament_target_dependencies(welder_server
  rclcpp
)

add_executable(move_to_pose_server
  src/action_servers/move_to_pose_server_main.cpp
)

target_link_libraries(move_to_pose_server
  hold_and_weld_action_servers
)

ament_target_dependencies(move_to_pose_server
  rclcpp
)

add_executable(dual_robot_coordinator
  src/coordinator/coordinator_main.cpp
)

target_link_libraries(dual_robot_coordinator
  hold_and_weld_coordinator
)

ament_target_dependencies(dual_robot_coordinator
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(kinematics_solver_test
    test/kinematics_solver_test.cpp
  )
  target_link_libraries(kinematics_solver_test
    hold_and_weld_kinematics
  )
  ament_target_dependencies(kinematics_solver_test
    rclcpp
    hold_and_weld_description
  )

  ament_add_gtest(ceres_ik_solver_test
    test/ceres_ik_solver_test.cpp
  )
  target_link_libraries(ceres_ik_solver_test
    hold_and_weld_kinematics
  )
  ament_target_dependencies(ceres_ik_solver_test
    rclcpp
    ament_index_cpp
    hold_and_weld_description
  )

endif()

install(TARGETS
  hold_and_weld_kinematics
  hold_and_weld_action_servers
  hold_and_weld_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gripper_server
  welder_server
  move_to_pose_server
  dual_robot_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

install(PROGRAMS
  scripts/utils/add_collision_objects.py
  scripts/magic_wand.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trajectories/
  DESTINATION share/${PROJECT_NAME}/trajectories
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

ament_export_include_directories(include)
ament_export_libraries(
  hold_and_weld_kinematics
  hold_and_weld_action_servers
  hold_and_weld_coordinator
)

ament_package()
