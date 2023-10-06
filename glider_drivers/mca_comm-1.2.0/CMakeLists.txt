cmake_minimum_required(VERSION 3.5)

project(mca_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED)
find_package(Ftd2xx REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mca_comm mca_data
  DEPENDS
    LIBFTD2XX
)

add_library(mca_comm
  src/gbs_mca_comm.cpp)
target_include_directories(mca_comm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mca_comm ${LIBFTD2XX_LIBRARIES})

add_library(mca_data
  src/gbs_mca_data.cpp
  src/gbs_mca_spefile.cpp)
target_include_directories(mca_data
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(mca_cmd
  src/cmd_processor.cpp
  src/main.cpp)
target_include_directories(mca_cmd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mca_cmd mca_comm mca_data)

add_executable(mca_shutdown
  src/mca_shutdown.cpp)
target_include_directories(mca_shutdown
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(mca_shutdown mca_comm)

install(
  TARGETS mca_comm mca_data
  EXPORT mca_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/
  DESTINATION include)
