cmake_minimum_required(VERSION 3.8)
project(hold_and_weld_gripper_sampler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tinyxml2 REQUIRED)

# Find FCL (Flexible Collision Library)
find_package(fcl REQUIRED)
if(fcl_FOUND)
  message(STATUS "Found FCL version ${fcl_VERSION}")
endif()

# Find OpenCASCADE
find_package(OpenCASCADE REQUIRED)
if(OpenCASCADE_FOUND)
  message(STATUS "Found OpenCASCADE version ${OpenCASCADE_VERSION}")
  message(STATUS "OpenCASCADE include dir: ${OpenCASCADE_INCLUDE_DIR}")
  message(STATUS "OpenCASCADE libraries: ${OpenCASCADE_LIBRARIES}")
endif()

add_library(${PROJECT_NAME}_core SHARED
  src/core/gripper_kinematics.cpp
  src/geometry/geometry_mapper.cpp
  src/geometry/topology.cpp
  src/geometry/occt_utils.cpp
  src/geometry/shape_refiner.cpp
  src/geometry/fcl_collision_checker.cpp
  src/filters/surface_dimension_filter.cpp
  src/filters/surface_geometry_filter.cpp
  src/filters/edge_avoidance_filter.cpp
  src/constraints/exclusion_zone_constraint.cpp
  src/constraints/kissing_surface_constraint.cpp
  src/sampling/contact_point_sampler.cpp
  src/angle_finding/grasp_orientation_finder.cpp
  src/core/grasp_finder.cpp
  src/io/config_parser.cpp
  src/io/result_writer.cpp
  src/io/shape_loader.cpp
  src/io/gripper_parser.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    Eigen3::Eigen
    ${OpenCASCADE_LIBRARIES}
    fcl
    yaml-cpp
    tinyxml2::tinyxml2
)

ament_target_dependencies(${PROJECT_NAME}_core
  PUBLIC
    rclcpp
    ament_index_cpp
)

add_executable(grasp_finder_node
  src/grasp_finder_node.cpp
)

target_include_directories(grasp_finder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(grasp_finder_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(grasp_finder_node
  rclcpp
  ament_index_cpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  grasp_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  Eigen3
  yaml-cpp
  pluginlib
  tinyxml2
  fcl
)

ament_export_include_directories(
  include
  ${OpenCASCADE_INCLUDE_DIR}
)

ament_export_libraries(
  ${PROJECT_NAME}_core
  ${OpenCASCADE_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Helper macro to keep test definitions extremely clean
  macro(add_sampler_test target_name source_file)
    ament_add_gtest(${target_name} ${source_file})
    if(TARGET ${target_name})
      target_include_directories(${target_name} PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCASCADE_INCLUDE_DIR}
      )
      target_link_libraries(${target_name}
        ${PROJECT_NAME}_core
      )
      ament_target_dependencies(${target_name}
        rclcpp
        Eigen3
      )
    endif()
  endmacro()

  # Unit tests (Just add one line here when you create a new test file)
  add_sampler_test(topology_extraction_test test/topology_extraction_test.cpp)
  add_sampler_test(shape_refiner_test test/shape_refiner_test.cpp)
  add_sampler_test(exclusion_zone_constraint_test test/exclusion_zone_constraint_test.cpp)
  add_sampler_test(gripper_parser_test test/gripper_parser_test.cpp)
  add_sampler_test(kissing_surface_constraint_test test/kissing_surface_constraint_test.cpp)
  add_sampler_test(contact_point_sampler_test test/contact_point_sampler_test.cpp)
  add_sampler_test(grasp_orientation_finder_test test/grasp_orientation_finder_test.cpp)
  add_sampler_test(shape_loader_test test/shape_loader_test.cpp)
  add_sampler_test(fcl_collision_checker_test test/fcl_collision_checker_test.cpp)

endif()

ament_package()
