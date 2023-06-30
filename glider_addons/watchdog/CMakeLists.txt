cmake_minimum_required(VERSION 3.1.0)
project(watchdog)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 auv_msgs
 roscpp

 # farol_gimmicks_library
 farol_gimmicks_library
)

catkin_package(
 CATKIN_DEPENDS 
 roscpp
 farol_msgs
 auv_msgs
 farol_gimmicks_library
 std_msgs
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/watchdog_ros
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/watchdog_ros/WatchdogNode.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
