cmake_minimum_required(VERSION 3.10)
project(ctu_cras_norlab_trailer_sensor_config_1)

find_package(catkin REQUIRED roscpp subt_communication_broker subt_msgs subt_ros)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_comms nodes/test_comms.cc)
target_link_libraries(test_comms ${catkin_LIBRARIES})

install(DIRECTORY launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/print_robot_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
