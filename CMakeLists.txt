cmake_minimum_required (VERSION 2.8)

project(ExtendedKF)

add_definitions(-std=c++0x)

set(sources
    src/fusion_ekf.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/tools.cpp)

add_executable(ExtendedKF ${sources})
