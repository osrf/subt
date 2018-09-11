cmake_minimum_required(VERSION 2.8.3)
project(x2_navigation)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/include/amcl.launch)
roslaunch_add_file_check(launch/amcl_demo.launch)
roslaunch_add_file_check(launch/include/gmapping.launch)
roslaunch_add_file_check(launch/gmapping_demo.launch)
roslaunch_add_file_check(launch/include/move_base.launch)
roslaunch_add_file_check(launch/odom_navigation_demo.launch)

install(
DIRECTORY launch maps params
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
