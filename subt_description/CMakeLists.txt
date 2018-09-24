cmake_minimum_required(VERSION 2.8.3)
project(subt_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY meshes materials urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
