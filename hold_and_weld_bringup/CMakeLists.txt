cmake_minimum_required(VERSION 3.8)
project(hold_and_weld_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/workcell_configurator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_utils test/test_launch_utils.py)
  ament_add_pytest_test(test_workcell_configurator test/test_workcell_configurator.py)
endif()

ament_package()
