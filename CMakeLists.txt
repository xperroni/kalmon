cmake_minimum_required (VERSION 2.8)

project(ExtendedKF)

add_definitions(-std=c++0x)

add_executable(kalmon
    "src/kalman_filter.cpp"
    "src/main.cpp"
    "src/measurement.cpp"
    "src/settings.cpp"
    "src/state.cpp"
    "src/tools.cpp"
)
