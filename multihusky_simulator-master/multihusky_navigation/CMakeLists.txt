cmake_minimum_required(VERSION 2.8.3)
project(multihusky_navigation)

find_package(catkin REQUIRED roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
  scripts/flex_smach.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

