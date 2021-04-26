cmake_minimum_required(VERSION 3.10.0)

project(ctu_cras_norlab_lily_sensor_config_1)

## make params
set(IGN_GAZEBO_VER 4)

set(DEPS roscpp ros_ign_bridge nodelet)
set(MSG_DEPS std_msgs geometry_msgs)


##############################
## Dependencies
##############################

## Ignition dependency
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)


##############################
## catkin/ROS setup
##############################

find_package(catkin REQUIRED COMPONENTS ${DEPS} ${MSG_DEPS} message_generation)

add_message_files(FILES Contact.msg ContactsStamped.msg LogicalContact.msg)
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${DEPS} ${MSG_DEPS} message_runtime)


##############################
## Building libs
##############################

add_library(lily_foot_contact_bridge src/foot_contact_bridge.cpp)
target_include_directories(lily_foot_contact_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(lily_foot_contact_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(lily_foot_contact_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(lily_logical_contact_bridge src/logical_contact_bridge.cpp)
target_include_directories(lily_logical_contact_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(lily_logical_contact_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(lily_logical_contact_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(lily-logical-contact-system src/logical_contact_system.cpp)
target_link_libraries(lily-logical-contact-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

##############################
## installation
##############################

install(TARGETS
    lily_foot_contact_bridge
    lily_logical_contact_bridge
    lily-logical-contact-system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch meshes textures urdf worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES lily.sdf model.config nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
