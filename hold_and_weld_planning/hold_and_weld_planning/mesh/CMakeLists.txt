cmake_minimum_required(VERSION 3.14)
project(mesh_intersection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(pybind11 REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

pybind11_add_module(mesh_intersection mesh_intersection.cpp)

target_link_libraries(mesh_intersection PRIVATE
    CGAL::CGAL
    pybind11::module
    ${Boost_LIBRARIES}
)

target_include_directories(mesh_intersection PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(mesh_intersection PRIVATE
    CGAL_EIGEN3_ENABLED
)
