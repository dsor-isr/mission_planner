cmake_minimum_required(VERSION 3.1)
project(leak)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 roscpp
 diagnostic_msgs
 farol_gimmicks_library
 glider_diagnostics_library
)

catkin_package(
 CATKIN_DEPENDS farol_gimmicks_library glider_diagnostics_library
)

add_compile_options(-std=c++11) 

include_directories(
 include/leak_ros
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/leak_ros/LeakNode.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
