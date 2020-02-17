cmake_minimum_required(VERSION 2.8.3)
project(navigation_stage)

# Find catkin
find_package(catkin REQUIRED roscpp rospy std_msgs genmsg navi_msgs costmap_2d)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

pkg_check_modules(playerc++ REQUIRED playerc++)
pkg_check_modules(playerc REQUIRED playerc)
SET (CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")

INCLUDE (UsePlayerC++)
INCLUDE (UsePlayerPlugin)

# Install launch files
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install move_base_config files
#install(DIRECTORY move_base_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install stage_config files
#install(DIRECTORY stage_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install the rviz files
#install(FILES multi_robot.rviz single_robot.rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_package(
CATKIN_DEPENDS
    roscpp
DEPENDS
    eigen3
)

add_compile_options(-std=c++11)
#include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include/navigation_stage ${catkin_INCLUDE_DIRS})
include_directories(${pkg-config_INCLUDE_DIRS}) 
include_directories(${player-3.0.2_INCLUDE_DIRS}) 
#include_directories(/home/jendav/player-3.0.2/libplayercommon)
#include_directories(/home/jendav/player-3.0.2/client_libs)
#include_directories(/home/jendav/player-3.0.2/build)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

#file(GLOB SOURCES "src/*.cpp")
#PLAYER_ADD_PLAYERCPP_CLIENT (robot SOURCES src/robot.cpp src/RefreshItemList.cpp LINKFLAGS)

PLAYER_ADD_PLAYERCPP_CLIENT (stage_robots_node SOURCES src/stage_robots_node.cpp LINKFLAGS)
TARGET_LINK_LIBRARIES (stage_robots_node ${catkin_LIBRARIES})
 
add_executable(path_cost_node src/path_cost_node.cpp)
target_link_libraries(path_cost_node ${catkin_LIBRARIES})

add_executable(euclidean_cost_node src/euclidean_cost_node.cpp)
target_link_libraries(euclidean_cost_node ${catkin_LIBRARIES})

add_executable(send_goals_node src/send_goals_node.cpp)
target_link_libraries(send_goals_node ${catkin_LIBRARIES})

add_executable(task_allocation_node src/task_allocation_node.cpp src/heuristic.cpp src/hungarian.cpp)
target_link_libraries(task_allocation_node ${catkin_LIBRARIES})

#if it is your library
#add_library(mylib src/mylib.cpp)
#FIND_LIBRARY (playerc++_LIBRARIES playerc++ /home/jendav/player-3.0.2)
#add_executable(robot src/task.cpp src/robot.cpp src/RefreshItemList.cpp)
#target_link_libraries(robot ${playerc++_LIBS})
#add_dependencies(path_cost_node ${cost_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

