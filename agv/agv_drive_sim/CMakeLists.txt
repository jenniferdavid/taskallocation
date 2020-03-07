cmake_minimum_required(VERSION 2.8.3)
project(agv_drive_sim)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package()

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
    cargo_ants_msgs
    roscpp
    rospy
    std_msgs)

#SRCS files. ROS node
SET(SRCS_ROS_NODE 
      src/agv_drive_sim_node.cpp
)

#dynamic reconfigure cfg file
# generate_dynamic_reconfigure_options(cfg/my_config_file.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package (CATKIN_DEPENDS 
       cargo_ants_msgs
       roscpp 
       rospy 
       std_msgs)

# add to include path
include_directories(${catkin_INCLUDE_DIRS})

# build executable
add_executable(agv_drive_sim src/agv_drive_sim_main.cpp ${SRCS_ROS_NODE})
target_link_libraries(agv_drive_sim ${catkin_LIBRARIES})
add_dependencies(agv_drive_sim $(cargo_ants_msgs_EXPORTED_TARGETS))
add_dependencies(agv_drive_sim ${catkin_EXPORTED_TARGETS})
