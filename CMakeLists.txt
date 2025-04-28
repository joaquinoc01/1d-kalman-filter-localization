cmake_minimum_required(VERSION 3.10)
project(kalman_filter)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add the include directory
include_directories(include)

# Add the executable
add_executable(kalman_filter
    src/kalman_filter.cpp
)