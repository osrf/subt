cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
find_package(ignition-cmake2 REQUIRED)
project(cerberus_kolibri_sensor_config_1)

find_package(catkin REQUIRED)

catkin_package()

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport9 QUIET REQUIRED OPTIONAL_COMPONENTS log)

set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo4 REQUIRED)
add_library(KolibriController SHARED voliro_controller_plugin/kolibri_controller.cc )
set_property(TARGET KolibriController PROPERTY CXX_STANDARD 17)
target_link_libraries(KolibriController
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo4::ignition-gazebo4)

install(TARGETS KolibriController
	ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch materials meshes rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


