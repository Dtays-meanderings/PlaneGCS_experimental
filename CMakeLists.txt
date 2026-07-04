cmake_minimum_required(VERSION 3.15)
project(PlaneGCS CXX)
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/x64-windows")

# Dependency resolution
find_package(Eigen3 REQUIRED CONFIG)
find_package(GTest REQUIRED CONFIG)

set(PlaneGCS_SRCS
    src/GCS.cpp
    src/GCS.h
    src/Util.h
    src/Geo.cpp
    src/Geo.h
    src/Constraints.cpp
    src/Constraints.h
    src/SubSystem.cpp
    src/SubSystem.h
    src/qp_eq.cpp
    src/qp_eq.h
)

# Solver library
add_library(PlaneGcsStatic STATIC ${PlaneGCS_SRCS})
set_target_properties(PlaneGcsStatic PROPERTIES OUTPUT_NAME planegcs_static)
target_include_directories(PlaneGcsStatic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(PlaneGcsStatic PUBLIC Eigen3::Eigen)

# Unit tests
enable_testing()

add_executable(PlaneGcsTests tests/PlaneGcsTests.cpp)
target_link_libraries(PlaneGcsTests PRIVATE PlaneGcsStatic GTest::gtest GTest::gtest_main)
add_test(NAME PlaneGcsTests COMMAND PlaneGcsTests)

# Python Bindings
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED CONFIG)
    pybind11_add_module(planegcs_py python/bindings.cpp)
    set_target_properties(planegcs_py PROPERTIES OUTPUT_NAME planegcs)
    target_link_libraries(planegcs_py PRIVATE PlaneGcsStatic)
endif()
