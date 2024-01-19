cmake_minimum_required(VERSION 3.1.0)
project(glider_bsd_interface)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs
 dsor_msgs 
 farol_msgs
 roscpp
 dsor_utils

 # farol_gimmicks_library
 farol_gimmicks_library

 # slocum glider extctl package
 slocum_glider_msgs
)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIR})

catkin_package(
 CATKIN_DEPENDS
 DEPENDS GeographicLib
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/glider_bsd_interface_ros
 include/glider_bsd_interface_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/glider_bsd_interface_ros/GliderBsdInterfaceNode.cpp src/glider_bsd_interface_algorithms/GliderBsdInterfaceAlgorithm.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
