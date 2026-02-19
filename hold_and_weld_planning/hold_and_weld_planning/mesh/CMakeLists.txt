cmake_minimum_required(VERSION 3.14)
project(mesh_intersection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release build for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Find pybind11
find_package(pybind11 REQUIRED)

# Find CGAL
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)

# Find Boost (required by CGAL)
find_package(Boost REQUIRED)

# Build the Python extension module
pybind11_add_module(mesh_intersection mesh_intersection.cpp)

# Link CGAL
target_link_libraries(mesh_intersection PRIVATE CGAL::CGAL)

# Include dirs
target_include_directories(mesh_intersection PRIVATE
    ${CGAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# CGAL requires these definitions
target_compile_definitions(mesh_intersection PRIVATE
    CGAL_EIGEN3_ENABLED
)