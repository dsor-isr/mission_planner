cmake_minimum_required(VERSION 3.1)
project(parallel_port)

find_package(catkin REQUIRED COMPONENTS
  farol_msgs
  nmea_msgs
  diagnostic_msgs
  roscpp
  farol_gimmicks_library
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(parallel_port src/parallel_port.cpp)
add_dependencies(parallel_port ${catkin_EXPORTED_TARGETS})
target_link_libraries(parallel_port ${catkin_LIBRARIES})

add_executable(ds_emergency_stop src/ds_emergency_stop.cpp)
add_dependencies(ds_emergency_stop ${catkin_EXPORTED_TARGETS})
target_link_libraries(ds_emergency_stop ${catkin_LIBRARIES})
