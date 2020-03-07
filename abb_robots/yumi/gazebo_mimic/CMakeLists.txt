cmake_minimum_required(VERSION 2.8.3)
project(gazebo_mimic)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES gazebo_mimic_plugin
   CATKIN_DEPENDS roscpp
)


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_mimic_plugin src/gazebo_mimic_plugin/mimic_plugin.cpp)

target_link_libraries(gazebo_mimic_plugin ${catkin_LIBRARIES})
