cmake_minimum_required(VERSION 3.14)
project(mesh_intersection)

# C++17 required for CGAL and modern pybind11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release build for performance (CGAL corefinement is compute-intensive)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Dependencies: pybind11 for Python binding
find_package(pybind11 REQUIRED)

# CGAL for robust geometric mesh intersection
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)

# Boost required by CGAL
find_package(Boost REQUIRED)

# Build Python extension module from C++ source
pybind11_add_module(mesh_intersection mesh_intersection.cpp)

# Link CGAL libraries
target_link_libraries(mesh_intersection PRIVATE CGAL::CGAL)

# Include directories for CGAL, Boost, and Eigen3
target_include_directories(mesh_intersection PRIVATE
    ${CGAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Enable Eigen3 support in CGAL
target_compile_definitions(mesh_intersection PRIVATE
    CGAL_EIGEN3_ENABLED
)