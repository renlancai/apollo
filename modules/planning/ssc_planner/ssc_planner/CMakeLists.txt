cmake_minimum_required(VERSION 2.8)
project(ssc_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)

# PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
#     ssc_planner/proto/ssc_config.proto
# )


include_directories(
    inc
    ${EIGEN3_INCLUDE_DIR}

    "../common/inc/"
    "../common/thirdparty/"
    "../semantic_map_manager/inc/"
    "../motion_predictor/inc/"
    "../forward_simulator/inc/"
    "../vehicle_model/inc/"

    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(hkust_pl_ssc
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
    src/ssc_planner/ssc_config.pb.cc
    ${PROTO_SRCS}
)
target_link_libraries(hkust_pl_ssc PRIVATE
    hkust_pl_common
    hkust_pl_smm
    "${OpenMP_CXX_FLAGS}"
)

# target_compile_options(hkust_pl_ssc PRIVATE "${OpenMP_CXX_FLAGS}")

# add_library(ssc_server_ros
#     src/ssc_planner/ssc_server_ros.cc
#     src/ssc_planner/ssc_visualizer.cc
# )
# target_link_libraries(ssc_server_ros PRIVATE
#     hkust_pl_ssc
#     hkust_smm_ros
#     ${OpenMP_CXX_FLAGS}
# )
