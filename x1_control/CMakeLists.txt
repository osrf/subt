cmake_minimum_required(VERSION 2.8.3)
project(x1_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
