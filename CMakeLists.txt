cmake_minimum_required(VERSION 3.15)
project(PlaneGCS CXX)
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/x64-windows")

# Dependency resolution
find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(Eigen3)
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        target_include_directories(Eigen3::Eigen INTERFACE ${eigen3_SOURCE_DIR})
    endif()
endif()

find_package(Threads REQUIRED)

option(BUILD_TESTING "Build testing" ON)
if(BUILD_TESTING)
    find_package(GTest REQUIRED CONFIG)
endif()

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
target_link_libraries(PlaneGcsStatic PUBLIC Eigen3::Eigen Threads::Threads)

# Unit tests
if(BUILD_TESTING)
    enable_testing()
    add_executable(PlaneGcsTests tests/PlaneGcsTests.cpp)
    target_link_libraries(PlaneGcsTests PRIVATE PlaneGcsStatic GTest::gtest GTest::gtest_main)
    add_test(NAME PlaneGcsTests COMMAND PlaneGcsTests)
endif()

# Python Bindings
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED CONFIG)
    pybind11_add_module(planegcs_py python/bindings.cpp)
    set_target_properties(planegcs_py PROPERTIES OUTPUT_NAME planegcs)
    target_link_libraries(planegcs_py PRIVATE PlaneGcsStatic)
endif()
