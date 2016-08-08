cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(lighthouse SHARED src/lighthouse_sim.cpp)
target_link_libraries(lighthouse ${GAZEBO_LIBRARIES})

add_library(RayPlugin SHARED src/RayPlugin.cc)
target_link_libraries(RayPlugin ${GAZEBO_LIBRARIES})

add_library(scan SHARED src/TargetScan.cpp)
target_link_libraries(scan ${GAZEBO_LIBRARIES})

add_library(BlinkVisualPlugin SHARED src/BlinkVisualPlugin.cc)
target_link_libraries(BlinkVisualPlugin ${GAZEBO_LIBRARIES})
