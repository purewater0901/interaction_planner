cmake_minimum_required(VERSION 3.0.2)
project(common)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
)

find_package(Protobuf REQUIRED)
set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)
set(THIRD_PARTY_INCLUDE_DIRS  ${THIRD_PARTY_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})
set(ooqp_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/ooqp/include/ooqp)
set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE PARENT_SCOPE)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OOQP REQUIRED)
if(NOT OOQP_FOUND)
    message(FATAL_ERROR "OOQP not found.")
endif()

find_package(Nlopt REQUIRED)
if(NOT NLOPT_FOUND)
    message(FATAL_ERROR "NLOPT not found.")
endif()

catkin_package(
        INCLUDE_DIRS include ${THIRD_PARTY_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${ooqp_INCLUDE_DIRS}
        LIBRARIES hkust_pl_common
        CATKIN_DEPENDS roscpp visualization_msgs sensor_msgs tf
)
set(THIRD_PARTY_LIBRARIES
    dw
    ${PROTOBUF_LIBRARY}
    ${NLOPT_LIBRARIES}
    )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${THIRD_PARTY_INCLUDE_DIRS}
        ${ooqp_INCLUDE_DIRS}
        ${NLOPT_INCLUDE_DIRS}
)

add_library(hkust_pl_ooqp_itf
            src/solver/ooqp_interface.cpp
            src/solver/qp_solver.cpp
           )
target_link_libraries(hkust_pl_ooqp_itf
                      ${OOQP_LIBRARIES}
                     )

set(COMMON_SRC
    src/basics/tool_func.cpp
    src/basics/shapes.cpp
    src/basics/colormap.cpp
    src/math/calculations.cpp
    src/state/free_state.cpp
    src/idm/intelligent_driver_model.cpp
    src/circle_arc/circle_arc.cpp
    src/circle_arc/circle_arc_branch.cpp
    )
add_library(hkust_pl_common
            ${COMMON_SRC}
           )
target_link_libraries(hkust_pl_common
        ${THIRD_PARTY_LIBRARIES}
        opencv_core
        opencv_highgui
        hkust_pl_ooqp_itf
        )
