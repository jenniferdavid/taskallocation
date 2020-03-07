cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_yumi_launch)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/gazebo_simulation.launch)
roslaunch_add_file_check(launch/live_robots.launch)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
