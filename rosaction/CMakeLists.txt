cmake_minimum_required(VERSION 2.8.3)
project(rosaction)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
)

catkin_install_python(PROGRAMS scripts/rosaction
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
