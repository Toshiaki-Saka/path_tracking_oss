# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

project(path_tracking_oss
    VERSION 1.0.0
    DESCRIPTION "Open-source vehicle path-tracking algorithm comparison"
    LANGUAGES CXX)

# ---- Build type default ------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---- Dependencies ------------------------------------------------------------
# Eigen3 is used by the MPC controller for matrix algebra (header-only).
# On most systems: apt install libeigen3-dev / brew install eigen / vcpkg.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${Eigen3_VERSION}")

# ---- Header-only path-tracking library --------------------------------------
# All algorithms (Pure Pursuit / Stanley / MPC / MPPI) are header-only.
# Only the MPC controller depends on Eigen; everything else is plain C++17.
add_library(path_tracking INTERFACE)

target_include_directories(path_tracking INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(path_tracking INTERFACE Eigen3::Eigen)

target_compile_features(path_tracking INTERFACE cxx_std_17)

# ---- Comparison benchmark executable ----------------------------------------
add_executable(path_tracking_compare src/main.cpp)

target_link_libraries(path_tracking_compare PRIVATE path_tracking)

set_target_properties(path_tracking_compare PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_compile_options(path_tracking_compare PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -O2>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2 /utf-8>)

# ---- Convenience: copy the sample route next to the binary ------------------
add_custom_command(TARGET path_tracking_compare POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/data
            $<TARGET_FILE_DIR:path_tracking_compare>/data
    COMMENT "Copying sample route data next to the executable")

# ---- Cut-in following control demo ------------------------------------------
add_executable(cutin_demo src/cutin_demo.cpp)

target_link_libraries(cutin_demo PRIVATE path_tracking)

set_target_properties(cutin_demo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_compile_options(cutin_demo PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -O2>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2 /utf-8>)

add_custom_command(TARGET cutin_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/data
            $<TARGET_FILE_DIR:cutin_demo>/data
    COMMENT "Copying sample route data next to cutin_demo")

# ---- Tests ------------------------------------------------------------------
enable_testing()

add_executable(path_tracking_tests tests/path_tracking_tests.cpp)

target_link_libraries(path_tracking_tests PRIVATE path_tracking)

set_target_properties(path_tracking_tests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_compile_options(path_tracking_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -O2>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2 /utf-8>)

add_test(NAME path_tracking_tests COMMAND path_tracking_tests)
