cmake_minimum_required(VERSION 2.8)
project(ssc_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)

add_subdirectory(common)
add_subdirectory(vehicle_model)
add_subdirectory(forward_simulator)
add_subdirectory(motion_predictor)
add_subdirectory(semantic_map_manager)
add_subdirectory(ssc_planner)

